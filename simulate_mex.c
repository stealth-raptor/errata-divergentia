/*
 * SIMULATE_MEX  Compiled closed-loop simulation; the same model as the .m code.
 *
 *   [q, u, nlog, diverged, kstop] = simulate_mex(pstar, rc, m, Idiag, ca, sa, g, fc, b, ...
 *                                                r, dt, every, Kp, Ki, Kd, ...
 *                                                FiK, Fir, FiA, FiB, FdK, Fdr, FdA, FdB, ...
 *                                                use_int, use_diff, abort_err)
 *
 * A line-for-line port of the loop in SIMULATE_CLOSED_LOOP together with
 * FOPID_UPDATE and ROBOT_DYNAMICS, so that the FOPSO-GWO tuner runs at
 * compiled speed under Octave as well as MATLAB (about 400x faster than the
 * .m loop in Octave).  SIMULATE_CLOSED_LOOP calls it automatically once it
 * has been built with BUILD_MEX; tools/check_mex.m verifies that both give
 * the same results.
 *
 * Plant: M(q) qdd + C(q,qd) qd + G(q) + fc sgn(qd) + b qd = tau (paper
 * Eq. 21), recursive Newton-Euler in standard DH (Corke's formulation) with
 * the fields of ROBOT_PARAMS: pstar (3x6), rc (3x6), m (6), Idiag (6x3),
 * ca = cos(alpha), sa = sin(alpha) (6), g, fc (6), b (6).
 *
 * Controller: the fields of FOPID_CONTROLLER (Oustaloup filter banks with
 * NF = 11 sections, integer parts as integrator / backward difference).
 *
 * r is the 6xNt reference at the controller rate dt; q and u are logged every
 * `every` samples (6 x nlog).  nlog is the number of samples logged before a
 * divergence; diverged is true if the run stopped early (|e| > abort_err, or
 * a non-finite state or |q| > 1e3); kstop is the 1-based sample index at
 * which the loop ended.
 *
 * Build:  build_mex  (mkoctfile --mex in Octave, mex in MATLAB)
 */
#include "mex.h"
#include <math.h>
#include <string.h>

#define NJ 6
#define NF 11      /* Oustaloup sections, 2N+1 with N = 5 */

typedef struct {
    const double *pstar;   /* 3x6: origin of frame i seen from frame i-1, in frame i */
    const double *rc;      /* 3x6: centre of mass of link i, in frame i */
    const double *m;       /* 6 */
    const double *Idiag;   /* 6x3: principal inertias about the COM */
    const double *ca, *sa; /* cos/sin of the DH twist */
    const double *fc, *b;  /* Coulomb / viscous friction */
    double g;
} Model;

/* rotation of DH link i (frame i -> frame i-1), row-major R[3][3] */
static void link_rotation(const Model *P, int i, double qi, double R[3][3])
{
    double ct = cos(qi), st = sin(qi), ca = P->ca[i], sa = P->sa[i];
    R[0][0] = ct; R[0][1] = -st*ca; R[0][2] =  st*sa;
    R[1][0] = st; R[1][1] =  ct*ca; R[1][2] = -ct*sa;
    R[2][0] = 0;  R[2][1] =  sa;    R[2][2] =  ca;
}

static void cross3(const double *a, const double *b, double *c)
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/* recursive Newton-Euler, standard DH; R holds the six link rotations */
static void rnea(const Model *P, double R[NJ][3][3], const double *qd,
                 const double *qdd, double g, double *tau)
{
    double w[3] = {0, 0, 0}, wd[3] = {0, 0, 0}, vd[3] = {0, 0, g};
    double F[NJ][3], N[NJ][3];
    int i, k;

    for (i = 0; i < NJ; i++) {
        const double *p = P->pstar + 3*i, *r = P->rc + 3*i;
        double x[3], y[3], wn[3], c1[3], c2[3], c3[3], Iw[3], vdn[3];

        /* w = R' (w + z qd) ;  wd = R' (wd + z qdd + w x z qd) */
        x[0] = w[0]; x[1] = w[1]; x[2] = w[2] + qd[i];
        y[0] = wd[0] + w[1]*qd[i]; y[1] = wd[1] - w[0]*qd[i]; y[2] = wd[2] + qdd[i];
        for (k = 0; k < 3; k++) {
            wn[k] = R[i][0][k]*x[0] + R[i][1][k]*x[1] + R[i][2][k]*x[2];
            wd[k] = R[i][0][k]*y[0] + R[i][1][k]*y[1] + R[i][2][k]*y[2];
        }
        for (k = 0; k < 3; k++) w[k] = wn[k];

        /* vd = wd x p + w x (w x p) + R' vd */
        cross3(wd, p, c1); cross3(w, p, c2); cross3(w, c2, c3);
        for (k = 0; k < 3; k++)
            vdn[k] = c1[k] + c3[k] + R[i][0][k]*vd[0] + R[i][1][k]*vd[1] + R[i][2][k]*vd[2];
        for (k = 0; k < 3; k++) vd[k] = vdn[k];

        /* COM acceleration, Newton and Euler */
        cross3(wd, r, c1); cross3(w, r, c2); cross3(w, c2, c3);
        for (k = 0; k < 3; k++) F[i][k] = P->m[i] * (c1[k] + c3[k] + vd[k]);
        for (k = 0; k < 3; k++) Iw[k] = P->Idiag[i + NJ*k] * w[k];
        cross3(w, Iw, c1);
        for (k = 0; k < 3; k++) N[i][k] = P->Idiag[i + NJ*k] * wd[k] + c1[k];
    }

    {
        double f[3] = {0, 0, 0}, n[3] = {0, 0, 0};
        for (i = NJ - 1; i >= 0; i--) {
            const double *p = P->pstar + 3*i, *r = P->rc + 3*i;
            double Rf[3] = {0, 0, 0}, Rn[3] = {0, 0, 0}, pr[3], c1[3], c2[3];
            if (i < NJ - 1) {
                for (k = 0; k < 3; k++) {
                    Rf[k] = R[i+1][k][0]*f[0] + R[i+1][k][1]*f[1] + R[i+1][k][2]*f[2];
                    Rn[k] = R[i+1][k][0]*n[0] + R[i+1][k][1]*n[1] + R[i+1][k][2]*n[2];
                }
            }
            for (k = 0; k < 3; k++) pr[k] = p[k] + r[k];
            cross3(p, Rf, c1);
            cross3(pr, F[i], c2);
            for (k = 0; k < 3; k++) {
                n[k] = Rn[k] + c1[k] + c2[k] + N[i][k];
                f[k] = Rf[k] + F[i][k];
            }
            /* joint axis z_{i-1} in frame i = third row of R_i */
            tau[i] = n[0]*R[i][2][0] + n[1]*R[i][2][1] + n[2]*R[i][2][2];
        }
    }
}

/* forward dynamics: qdd = M(q) \ (u - h(q, qd)), Cholesky solve */
static void accel(const Model *P, const double *q, const double *qd,
                  const double *u, double *qdd)
{
    double R[NJ][3][3], h[NJ], M[NJ][NJ], L[NJ][NJ], col[NJ], e[NJ], zero[NJ] = {0, 0, 0, 0, 0, 0};
    double rhs[NJ], y[NJ];
    int i, j, k;

    for (i = 0; i < NJ; i++) link_rotation(P, i, q[i], R[i]);
    rnea(P, R, qd, zero, P->g, h);
    for (i = 0; i < NJ; i++)
        h[i] += P->fc[i] * ((qd[i] > 0) - (qd[i] < 0)) + P->b[i] * qd[i];
    for (j = 0; j < NJ; j++) {
        for (i = 0; i < NJ; i++) e[i] = (i == j);
        rnea(P, R, zero, e, 0.0, col);
        for (i = 0; i < NJ; i++) M[i][j] = col[i];
    }
    for (i = 0; i < NJ; i++)
        for (j = 0; j <= i; j++) {
            double s = 0.5 * (M[i][j] + M[j][i]);
            for (k = 0; k < j; k++) s -= L[i][k] * L[j][k];
            if (i == j) L[i][i] = sqrt(s);
            else        L[i][j] = s / L[j][j];
        }
    for (i = 0; i < NJ; i++) rhs[i] = u[i] - h[i];
    for (i = 0; i < NJ; i++) {
        double s = rhs[i];
        for (k = 0; k < i; k++) s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    for (i = NJ - 1; i >= 0; i--) {
        double s = y[i];
        for (k = i + 1; k < NJ; k++) s -= L[k][i] * qdd[k];
        qdd[i] = s / L[i][i];
    }
}

/* FILTER_STEP of fopid_update.m for joint j: one zero-order-hold step */
static double filter_step(double K, const double *r, const double *A,
                          const double *B, double *x, int j, double e)
{
    double y = K * e;
    int s;
    for (s = 0; s < NF; s++) y += r[j + NJ*s] * x[j + NJ*s];
    for (s = 0; s < NF; s++) x[j + NJ*s] = A[j + NJ*s] * x[j + NJ*s] + B[j + NJ*s] * e;
    return y;
}

static const double *get(const mxArray *a, int nel, const char *name)
{
    if (!mxIsDouble(a) || mxIsComplex(a) || (int) mxGetNumberOfElements(a) != nel)
        mexErrMsgIdAndTxt("simulate_mex:input", "argument %s must be real double with %d elements", name, nel);
    return mxGetPr(a);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    Model P;
    const double *r, *Kp, *Ki, *Kd, *FiK, *Fir, *FiA, *FiB, *FdK, *Fdr, *FdA, *FdB, *uint_, *udif;
    double dt, abort_err, *qo, *uo;
    double xi[NJ*NF], xd[NJ*NF], integral[NJ], prev_d[NJ];
    double q[NJ], qd[NJ], e[NJ], u[NJ];
    int Nt, every, nlog, k, j, nlogged = 0, diverged = 0, first = 1, kstop = 0;

    if (nrhs != 26)
        mexErrMsgIdAndTxt("simulate_mex:nargin", "26 inputs expected");

    P.pstar = get(prhs[0], 18, "pstar");
    P.rc    = get(prhs[1], 18, "rc");
    P.m     = get(prhs[2], NJ, "m");
    P.Idiag = get(prhs[3], 18, "Idiag");
    P.ca    = get(prhs[4], NJ, "ca");
    P.sa    = get(prhs[5], NJ, "sa");
    P.g     = *get(prhs[6], 1, "g");
    P.fc    = get(prhs[7], NJ, "fc");
    P.b     = get(prhs[8], NJ, "b");
    if (mxGetM(prhs[9]) != NJ)
        mexErrMsgIdAndTxt("simulate_mex:input", "r must be 6xNt");
    Nt    = (int) mxGetN(prhs[9]);
    r     = get(prhs[9], NJ*Nt, "r");
    dt    = *get(prhs[10], 1, "dt");
    every = (int) *get(prhs[11], 1, "every");
    Kp  = get(prhs[12], NJ, "Kp");
    Ki  = get(prhs[13], NJ, "Ki");
    Kd  = get(prhs[14], NJ, "Kd");
    FiK = get(prhs[15], NJ, "Fi.K");
    Fir = get(prhs[16], NJ*NF, "Fi.r");
    FiA = get(prhs[17], NJ*NF, "Fi.A");
    FiB = get(prhs[18], NJ*NF, "Fi.B");
    FdK = get(prhs[19], NJ, "Fd.K");
    Fdr = get(prhs[20], NJ*NF, "Fd.r");
    FdA = get(prhs[21], NJ*NF, "Fd.A");
    FdB = get(prhs[22], NJ*NF, "Fd.B");
    uint_ = get(prhs[23], NJ, "use_integrator");
    udif  = get(prhs[24], NJ, "use_difference");
    abort_err = *get(prhs[25], 1, "abort_err");
    if (every < 1)
        mexErrMsgIdAndTxt("simulate_mex:input", "every must be >= 1");

    nlog = (Nt - 1) / every + 1;
    plhs[0] = mxCreateDoubleMatrix(NJ, nlog, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(NJ, nlog, mxREAL);
    qo = mxGetPr(plhs[0]);
    uo = mxGetPr(plhs[1]);

    memset(xi, 0, sizeof xi);
    memset(xd, 0, sizeof xd);
    for (j = 0; j < NJ; j++) { integral[j] = 0; q[j] = 0; qd[j] = 0; prev_d[j] = 0; }

    for (k = 0; k < Nt; k++) {
        double emax = 0;
        kstop = k + 1;
        for (j = 0; j < NJ; j++) {
            e[j] = r[j + NJ*k] - q[j];
            if (fabs(e[j]) > emax) emax = fabs(e[j]);
        }
        if (emax > abort_err) { diverged = 1; break; }

        /* ---- fopid_update ---- */
        for (j = 0; j < NJ; j++) {
            double yi = filter_step(FiK[j], Fir, FiA, FiB, xi, j, e[j]);
            double yd, diff;
            integral[j] += yi * dt;
            yd = filter_step(FdK[j], Fdr, FdA, FdB, xd, j, e[j]);
            if (first) prev_d[j] = yd;               /* no kick on the first sample */
            diff = (yd - prev_d[j]) / dt;
            prev_d[j] = yd;
            u[j] = Kp[j]*e[j] + Ki[j]*((uint_[j] != 0) ? integral[j] : yi)
                              + Kd[j]*((udif[j] != 0) ? diff : yd);
        }
        first = 0;

        if (k % every == 0) {
            int c = k / every;
            for (j = 0; j < NJ; j++) { qo[j + NJ*c] = q[j]; uo[j + NJ*c] = u[j]; }
            nlogged = c + 1;
        }
        if (k == Nt - 1) break;

        /* ---- fourth-order Runge-Kutta step with the torque held constant ---- */
        {
            double k1v[NJ], k2v[NJ], k3v[NJ], k4v[NJ], k2q[NJ], k3q[NJ], k4q[NJ], qa[NJ];
            int bad = 0;
            accel(&P, q, qd, u, k1v);
            for (j = 0; j < NJ; j++) { k2q[j] = qd[j] + dt/2*k1v[j]; qa[j] = q[j] + dt/2*qd[j]; }
            accel(&P, qa, k2q, u, k2v);
            for (j = 0; j < NJ; j++) { k3q[j] = qd[j] + dt/2*k2v[j]; qa[j] = q[j] + dt/2*k2q[j]; }
            accel(&P, qa, k3q, u, k3v);
            for (j = 0; j < NJ; j++) { k4q[j] = qd[j] + dt*k3v[j];   qa[j] = q[j] + dt*k3q[j]; }
            accel(&P, qa, k4q, u, k4v);
            for (j = 0; j < NJ; j++) {
                q[j]  += dt/6 * (qd[j] + 2*k2q[j] + 2*k3q[j] + k4q[j]);
                qd[j] += dt/6 * (k1v[j] + 2*k2v[j] + 2*k3v[j] + k4v[j]);
                if (!isfinite(q[j]) || !isfinite(qd[j]) || fabs(q[j]) > 1e3) bad = 1;
            }
            if (bad) { diverged = 1; break; }
        }
    }

    if (nlhs > 2) plhs[2] = mxCreateDoubleScalar((double) nlogged);
    if (nlhs > 3) plhs[3] = mxCreateLogicalScalar(diverged != 0);
    if (nlhs > 4) plhs[4] = mxCreateDoubleScalar((double) kstop);
}
