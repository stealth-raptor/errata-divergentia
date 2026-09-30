/*
 * SIMULATE_MEX  Compiled closed-loop simulation; same model as the .m code.
 *
 *   [q, u, n, diverged] = simulate_mex(R_fixed, p_fixed, I, g, r, dt, ...
 *                                      Kp, Ki, Kd, FiK, Fir, FiA, FiB, ...
 *                                      FdK, Fdr, FdA, FdB, use_int, use_diff, ...
 *                                      abort_err)
 *
 * A line-for-line port of the loop in SIMULATE_CLOSED_LOOP together with
 * FOPID_UPDATE and ROBOT_DYNAMICS (recursive Newton-Euler), so that the tuner
 * runs at compiled speed under Octave as well as MATLAB.  It is called by
 * SIMULATE_CLOSED_LOOP when it has been built (see BUILD_MEX); without it the
 * .m implementation is used and gives the same results.
 *
 * Inputs are the fields of ROBOT_PARAMS and FOPID_CONTROLLER; r is the 6xNt
 * reference.  Outputs: q and u are 6xNt (columns after n are zero), n is the
 * number of samples actually simulated and diverged is true if the run was
 * stopped by abort_err or a non-finite state.
 *
 * Build:  mex simulate_mex.c            (MATLAB)
 *         mkoctfile --mex simulate_mex.c (Octave)
 */
#include "mex.h"
#include <math.h>
#include <string.h>

#define NJ 6
#define NF 11      /* Oustaloup sections, 2N+1 with N = 5 */

typedef struct {
    const double *R;   /* 3x3x6, column-major */
    const double *p;   /* 3x6 */
    const double *I;   /* 6x6x6 */
    double g;
} Model;

static void cross3(const double *a, const double *b, double *c)
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

/* y = R' * x for the 3x3 block i of R_fixed (column-major) */
static void mulRt(const double *R, const double *x, double *y)
{
    int r;
    for (r = 0; r < 3; r++)
        y[r] = R[0 + 3*r]*x[0] + R[1 + 3*r]*x[1] + R[2 + 3*r]*x[2];
}

/* y = R * x */
static void mulR(const double *R, const double *x, double *y)
{
    int r;
    for (r = 0; r < 3; r++)
        y[r] = R[r + 0]*x[0] + R[r + 3]*x[1] + R[r + 6]*x[2];
}

/* ROTATE of robot_dynamics.m: Rz(q)' * (R_fixed' * x), with R = R_fixed' */
static void rotate_in(const double *Rf, double c, double s, const double *x, double *y)
{
    double t[3];
    mulRt(Rf, x, t);
    y[0] =  c*t[0] + s*t[1];
    y[1] = -s*t[0] + c*t[1];
    y[2] =  t[2];
}

/* y = I6 * x for a 6x6 column-major matrix */
static void mul6(const double *I, const double *x, double *y)
{
    int r, c;
    for (r = 0; r < 6; r++) {
        double s = 0;
        for (c = 0; c < 6; c++) s += I[r + 6*c] * x[c];
        y[r] = s;
    }
}

/* RNEA of robot_dynamics.m */
static void rnea(const Model *P, const double *q, const double *qd,
                 const double *qdd, double g, double *tau)
{
    double w[3] = {0,0,0}, v[3] = {0,0,0}, aw[3] = {0,0,0}, av[3] = {0,0,g};
    double n[3*NJ], f[3*NJ];
    int i, k;

    for (i = 0; i < NJ; i++) {
        const double *Rf = P->R + 9*i;
        const double *p  = P->p + 3*i;
        double c = cos(q[i]), s = sin(q[i]);
        double tmp[3], x[3], w_p[3], v_p[3], aw_p[3], av_p[3], z[3];
        double wv[6], awav[6], Iv[6], Ia[6], t1[3], t2[3];

        rotate_in(Rf, c, s, w, w_p);
        cross3(p, w, tmp);
        for (k = 0; k < 3; k++) x[k] = v[k] - tmp[k];
        rotate_in(Rf, c, s, x, v_p);
        rotate_in(Rf, c, s, aw, aw_p);
        cross3(p, aw, tmp);
        for (k = 0; k < 3; k++) x[k] = av[k] - tmp[k];
        rotate_in(Rf, c, s, x, av_p);

        z[0] = 0; z[1] = 0; z[2] = qd[i];
        for (k = 0; k < 3; k++) w[k] = w_p[k] + z[k];
        for (k = 0; k < 3; k++) v[k] = v_p[k];
        cross3(w_p, z, tmp);
        for (k = 0; k < 3; k++) aw[k] = aw_p[k] + tmp[k];
        aw[2] += qdd[i];
        cross3(v_p, z, tmp);
        for (k = 0; k < 3; k++) av[k] = av_p[k] + tmp[k];

        for (k = 0; k < 3; k++) { wv[k] = w[k];  wv[k+3] = v[k]; }
        for (k = 0; k < 3; k++) { awav[k] = aw[k]; awav[k+3] = av[k]; }
        mul6(P->I + 36*i, wv, Iv);
        mul6(P->I + 36*i, awav, Ia);

        cross3(w, Iv, t1);
        cross3(v, Iv + 3, t2);
        for (k = 0; k < 3; k++) n[3*i + k] = Ia[k] + t1[k] + t2[k];
        cross3(w, Iv + 3, t1);
        for (k = 0; k < 3; k++) f[3*i + k] = Ia[k+3] + t1[k];
    }

    {
        double nn[3], ff[3];
        for (k = 0; k < 3; k++) { nn[k] = n[3*5 + k]; ff[k] = f[3*5 + k]; }
        for (i = NJ - 1; i >= 0; i--) {
            tau[i] = nn[2];
            if (i > 0) {
                const double *Rf = P->R + 9*i;
                const double *p  = P->p + 3*i;
                double c = cos(q[i]), s = sin(q[i]);
                double rn[3], rf[3], nn_p[3], ff_p[3], t[3];
                rn[0] = c*nn[0] - s*nn[1]; rn[1] = s*nn[0] + c*nn[1]; rn[2] = nn[2];
                rf[0] = c*ff[0] - s*ff[1]; rf[1] = s*ff[0] + c*ff[1]; rf[2] = ff[2];
                mulR(Rf, rn, nn_p);
                mulR(Rf, rf, ff_p);
                cross3(p, ff_p, t);
                for (k = 0; k < 3; k++) {
                    nn[k] = n[3*(i-1) + k] + nn_p[k] + t[k];
                    ff[k] = f[3*(i-1) + k] + ff_p[k];
                }
            }
        }
    }
}

/* Solve A x = b (6x6, column-major, destroyed) by Gaussian elimination with
 * partial pivoting. */
static void solve6(double *A, double *b)
{
    int col, r, c;
    for (col = 0; col < NJ; col++) {
        int piv = col;
        double best = fabs(A[col + NJ*col]);
        for (r = col + 1; r < NJ; r++)
            if (fabs(A[r + NJ*col]) > best) { best = fabs(A[r + NJ*col]); piv = r; }
        if (piv != col) {
            double t;
            for (c = 0; c < NJ; c++) {
                t = A[col + NJ*c]; A[col + NJ*c] = A[piv + NJ*c]; A[piv + NJ*c] = t;
            }
            t = b[col]; b[col] = b[piv]; b[piv] = t;
        }
        for (r = col + 1; r < NJ; r++) {
            double m = A[r + NJ*col] / A[col + NJ*col];
            for (c = col; c < NJ; c++) A[r + NJ*c] -= m * A[col + NJ*c];
            b[r] -= m * b[col];
        }
    }
    for (r = NJ - 1; r >= 0; r--) {
        double s = b[r];
        for (c = r + 1; c < NJ; c++) s -= A[r + NJ*c] * b[c];
        b[r] = s / A[r + NJ*r];
    }
}

/* Forward dynamics: qdd = M(q) \ (u - h(q, qd)) */
static void derivative(const Model *P, const double *q, const double *qd,
                       const double *u, double *dq, double *dqd)
{
    double h[NJ], M[NJ*NJ], Ms[NJ*NJ], zero[NJ] = {0,0,0,0,0,0}, e[NJ], col[NJ];
    int i, j;

    rnea(P, q, qd, zero, P->g, h);
    for (j = 0; j < NJ; j++) {
        for (i = 0; i < NJ; i++) e[i] = (i == j);
        rnea(P, q, zero, e, 0.0, col);
        for (i = 0; i < NJ; i++) M[i + NJ*j] = col[i];
    }
    for (i = 0; i < NJ; i++)
        for (j = 0; j < NJ; j++)
            Ms[i + NJ*j] = 0.5 * (M[i + NJ*j] + M[j + NJ*i]);

    for (i = 0; i < NJ; i++) { dq[i] = qd[i]; dqd[i] = u[i] - h[i]; }
    solve6(Ms, dqd);
}

/* FILTER_STEP of fopid_update.m for one joint */
static double filter_step(double K, const double *r, const double *A,
                          const double *B, double *x, int j, double e)
{
    double y = K * e;
    int m;
    for (m = 0; m < NF; m++) y += r[j + NJ*m] * x[j + NJ*m];
    for (m = 0; m < NF; m++) x[j + NJ*m] = A[j + NJ*m] * x[j + NJ*m] + B[j + NJ*m] * e;
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
    double dt, abort_err;
    double *qo, *uo;
    double xi[NJ*NF], xd[NJ*NF], integral[NJ], prev_d[NJ];
    double q[NJ], qd[NJ], e[NJ], u[NJ];
    int Nt, k, j, n_done = 0, diverged = 0, first = 1;

    if (nrhs != 20)
        mexErrMsgIdAndTxt("simulate_mex:nargin", "20 inputs expected");

    P.R = get(prhs[0], 54, "R_fixed");
    P.p = get(prhs[1], 18, "p_fixed");
    P.I = get(prhs[2], 216, "I");
    P.g = *get(prhs[3], 1, "g");
    if (mxGetM(prhs[4]) != NJ)
        mexErrMsgIdAndTxt("simulate_mex:input", "r must be 6xNt");
    Nt = (int) mxGetN(prhs[4]);
    r   = get(prhs[4], NJ*Nt, "r");
    dt  = *get(prhs[5], 1, "dt");
    Kp  = get(prhs[6],  NJ, "Kp");
    Ki  = get(prhs[7],  NJ, "Ki");
    Kd  = get(prhs[8],  NJ, "Kd");
    FiK = get(prhs[9],  NJ, "Fi.K");
    Fir = get(prhs[10], NJ*NF, "Fi.r");
    FiA = get(prhs[11], NJ*NF, "Fi.A");
    FiB = get(prhs[12], NJ*NF, "Fi.B");
    FdK = get(prhs[13], NJ, "Fd.K");
    Fdr = get(prhs[14], NJ*NF, "Fd.r");
    FdA = get(prhs[15], NJ*NF, "Fd.A");
    FdB = get(prhs[16], NJ*NF, "Fd.B");
    uint_ = get(prhs[17], NJ, "use_integrator");
    udif  = get(prhs[18], NJ, "use_difference");
    abort_err = *get(prhs[19], 1, "abort_err");

    plhs[0] = mxCreateDoubleMatrix(NJ, Nt, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(NJ, Nt, mxREAL);
    qo = mxGetPr(plhs[0]);
    uo = mxGetPr(plhs[1]);

    memset(xi, 0, sizeof xi);
    memset(xd, 0, sizeof xd);
    for (j = 0; j < NJ; j++) { integral[j] = 0; q[j] = 0; qd[j] = 0; prev_d[j] = 0; }

    for (k = 0; k < Nt; k++) {
        double emax = 0;
        int finite = 1;
        for (j = 0; j < NJ; j++) {
            e[j] = r[j + NJ*k] - q[j];
            if (!isfinite(q[j]) || !isfinite(qd[j])) finite = 0;
            if (fabs(e[j]) > emax) emax = fabs(e[j]);
        }
        if (!finite || emax > abort_err) { diverged = 1; break; }

        /* ---- fopid_update ---- */
        for (j = 0; j < NJ; j++) {
            double yi = filter_step(FiK[j], Fir, FiA, FiB, xi, j, e[j]);
            double yd = filter_step(FdK[j], Fdr, FdA, FdB, xd, j, e[j]);
            double term_i, term_d, diff;
            integral[j] += yi * dt;
            term_i = (uint_[j] != 0) ? integral[j] : yi;
            if (first) prev_d[j] = yd;
            diff = (yd - prev_d[j]) / dt;
            prev_d[j] = yd;
            term_d = (udif[j] != 0) ? diff : yd;
            u[j] = Kp[j]*e[j] + Ki[j]*term_i + Kd[j]*term_d;
        }
        first = 0;

        for (j = 0; j < NJ; j++) { qo[j + NJ*k] = q[j]; uo[j + NJ*k] = u[j]; }
        n_done = k + 1;
        if (k == Nt - 1) break;

        /* ---- RK4 with the torque held constant ---- */
        {
            double k1q[NJ], k1v[NJ], k2q[NJ], k2v[NJ], k3q[NJ], k3v[NJ], k4q[NJ], k4v[NJ];
            double qa[NJ], va[NJ];
            derivative(&P, q, qd, u, k1q, k1v);
            for (j = 0; j < NJ; j++) { qa[j] = q[j] + dt/2*k1q[j]; va[j] = qd[j] + dt/2*k1v[j]; }
            derivative(&P, qa, va, u, k2q, k2v);
            for (j = 0; j < NJ; j++) { qa[j] = q[j] + dt/2*k2q[j]; va[j] = qd[j] + dt/2*k2v[j]; }
            derivative(&P, qa, va, u, k3q, k3v);
            for (j = 0; j < NJ; j++) { qa[j] = q[j] + dt*k3q[j]; va[j] = qd[j] + dt*k3v[j]; }
            derivative(&P, qa, va, u, k4q, k4v);
            for (j = 0; j < NJ; j++) {
                q[j]  += dt/6 * (k1q[j] + 2*k2q[j] + 2*k3q[j] + k4q[j]);
                qd[j] += dt/6 * (k1v[j] + 2*k2v[j] + 2*k3v[j] + k4v[j]);
            }
        }
    }

    if (nlhs > 2) plhs[2] = mxCreateDoubleScalar((double) n_done);
    if (nlhs > 3) plhs[3] = mxCreateLogicalScalar(diverged != 0);
}
