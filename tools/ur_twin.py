"""Fast Python twin of the Octave model, used only to identify the gains.

Identifying 48 unpublished controller parameters from the published curves
needs thousands of closed-loop simulations; the Octave code is the reference
implementation but too slow for that.  This module implements *the same*
model with numba:

  plant       M(q) qdd + C(q, qd) qd + G(q) + tau_f(qd) = tau      (Eq. 21)
              standard DH, UR structure, Table-2 masses/lengths/inertias
  controller  u = Kp e + Ki D^-lambda e + Kd D^mu e                  (Eq. 9)
              Oustaloup N = 5 on [1e-3, 1e3] rad/s, exact ZOH per section
  simulation  controller at 1 kHz (ZOH), RK4 plant integration at 1 ms,
              signals logged every 0.01 s as in the paper's figures

tools/check_twin.m verifies that the Octave code and this twin agree.
"""
import numpy as np
from numba import njit

# ---------------------------------------------------------------- plant ----
TABLE2_L = np.array([0.128, 0.612, 0.571, 0.164, 0.115, 0.092])
UR5_L = np.array([0.089159, 0.425, 0.39225, 0.10915, 0.09465, 0.0823])
MASS = np.array([2.0, 2.5, 5.7, 3.9, 2.5, 2.5])
IDIAG = np.array([[1.0, 1.0, 1.0], [4.0, 4.0, 4.0], [6.0, 6.0, 4.0],
                  [5.5, 5.5, 4.0], [4.0, 4.0, 4.0], [4.0, 4.0, 4.0]]) \
    * np.array([1e-3, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2])[:, None]
ALPHA = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])


def make_plant(L=TABLE2_L, com='distal', g=0.0, fc=None, b=None, coriolis=True):
    """Plant parameter arrays.  com: 'distal' | 'middle' | 'proximal'.
    coriolis=False drops C(q, qd) qd (a diagnostic hypothesis only)."""
    d = np.array([L[0], 0, 0, L[3], L[4], L[5]])
    a = np.array([0, -L[1], -L[2], 0, 0, 0])
    pstar = np.stack([a, d * np.sin(ALPHA), d * np.cos(ALPHA)], axis=1)  # O_{i-1}->O_i in frame i
    frac = {'distal': 0.0, 'middle': 0.5, 'proximal': 1.0}[com]
    rc = -frac * pstar                                                     # COM in frame i
    return dict(d=d, a=a, alpha=ALPHA.copy(), pstar=pstar, rc=rc, m=MASS.copy(),
                I=IDIAG.copy(), g=float(g),
                fc=np.zeros(6) if fc is None else np.asarray(fc, float),
                b=np.zeros(6) if b is None else np.asarray(b, float),
                vscale=1.0 if coriolis else 0.0)


@njit(cache=True, inline='always')
def _cr(a0, a1, a2, b0, b1, b2):
    return a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0


@njit(cache=True)
def _rots(q, alpha, R):
    for i in range(6):
        ct, st, ca, sa = np.cos(q[i]), np.sin(q[i]), np.cos(alpha[i]), np.sin(alpha[i])
        R[i, 0, 0] = ct; R[i, 0, 1] = -st * ca; R[i, 0, 2] = st * sa
        R[i, 1, 0] = st; R[i, 1, 1] = ct * ca;  R[i, 1, 2] = -ct * sa
        R[i, 2, 0] = 0;  R[i, 2, 1] = sa;       R[i, 2, 2] = ca


@njit(cache=True)
def _rnea_R(R, qd, qdd, pstar, rc, m, I, g, tau, F, N):
    """Recursive Newton-Euler, standard DH (Corke's formulation), scalar code.
    R[i] rotates frame i -> frame i-1."""
    w0 = 0.0; w1 = 0.0; w2 = 0.0
    a0 = 0.0; a1 = 0.0; a2 = 0.0
    v0 = 0.0; v1 = 0.0; v2 = g
    for i in range(6):
        # Rt @ (w + z qd)
        x0 = w0; x1 = w1; x2 = w2 + qd[i]
        nw0 = R[i, 0, 0] * x0 + R[i, 1, 0] * x1 + R[i, 2, 0] * x2
        nw1 = R[i, 0, 1] * x0 + R[i, 1, 1] * x1 + R[i, 2, 1] * x2
        nw2 = R[i, 0, 2] * x0 + R[i, 1, 2] * x1 + R[i, 2, 2] * x2
        # Rt @ (wd + z qdd + w x z qd)   with  w x (0,0,qd) = (w1 qd, -w0 qd, 0)
        x0 = a0 + w1 * qd[i]; x1 = a1 - w0 * qd[i]; x2 = a2 + qdd[i]
        a0 = R[i, 0, 0] * x0 + R[i, 1, 0] * x1 + R[i, 2, 0] * x2
        a1 = R[i, 0, 1] * x0 + R[i, 1, 1] * x1 + R[i, 2, 1] * x2
        a2 = R[i, 0, 2] * x0 + R[i, 1, 2] * x1 + R[i, 2, 2] * x2
        w0 = nw0; w1 = nw1; w2 = nw2
        # vd = wd x p + w x (w x p) + Rt vd
        p0 = pstar[i, 0]; p1 = pstar[i, 1]; p2 = pstar[i, 2]
        c0, c1, c2 = _cr(a0, a1, a2, p0, p1, p2)
        e0, e1, e2 = _cr(w0, w1, w2, p0, p1, p2)
        f0, f1, f2 = _cr(w0, w1, w2, e0, e1, e2)
        y0 = R[i, 0, 0] * v0 + R[i, 1, 0] * v1 + R[i, 2, 0] * v2
        y1 = R[i, 0, 1] * v0 + R[i, 1, 1] * v1 + R[i, 2, 1] * v2
        y2 = R[i, 0, 2] * v0 + R[i, 1, 2] * v1 + R[i, 2, 2] * v2
        v0 = c0 + f0 + y0; v1 = c1 + f1 + y1; v2 = c2 + f2 + y2
        # COM acceleration and body force / moment
        s0 = rc[i, 0]; s1 = rc[i, 1]; s2 = rc[i, 2]
        c0, c1, c2 = _cr(a0, a1, a2, s0, s1, s2)
        e0, e1, e2 = _cr(w0, w1, w2, s0, s1, s2)
        f0, f1, f2 = _cr(w0, w1, w2, e0, e1, e2)
        F[i, 0] = m[i] * (c0 + f0 + v0); F[i, 1] = m[i] * (c1 + f1 + v1); F[i, 2] = m[i] * (c2 + f2 + v2)
        Iw0 = I[i, 0] * w0; Iw1 = I[i, 1] * w1; Iw2 = I[i, 2] * w2
        g0, g1, g2 = _cr(w0, w1, w2, Iw0, Iw1, Iw2)
        N[i, 0] = I[i, 0] * a0 + g0; N[i, 1] = I[i, 1] * a1 + g1; N[i, 2] = I[i, 2] * a2 + g2
    fx = 0.0; fy = 0.0; fz = 0.0; nx = 0.0; ny = 0.0; nz = 0.0
    for i in range(5, -1, -1):
        p0 = pstar[i, 0]; p1 = pstar[i, 1]; p2 = pstar[i, 2]
        if i == 5:
            Rf0 = 0.0; Rf1 = 0.0; Rf2 = 0.0; Rn0 = 0.0; Rn1 = 0.0; Rn2 = 0.0
        else:
            Rf0 = R[i + 1, 0, 0] * fx + R[i + 1, 0, 1] * fy + R[i + 1, 0, 2] * fz
            Rf1 = R[i + 1, 1, 0] * fx + R[i + 1, 1, 1] * fy + R[i + 1, 1, 2] * fz
            Rf2 = R[i + 1, 2, 0] * fx + R[i + 1, 2, 1] * fy + R[i + 1, 2, 2] * fz
            Rn0 = R[i + 1, 0, 0] * nx + R[i + 1, 0, 1] * ny + R[i + 1, 0, 2] * nz
            Rn1 = R[i + 1, 1, 0] * nx + R[i + 1, 1, 1] * ny + R[i + 1, 1, 2] * nz
            Rn2 = R[i + 1, 2, 0] * nx + R[i + 1, 2, 1] * ny + R[i + 1, 2, 2] * nz
        c0, c1, c2 = _cr(p0, p1, p2, Rf0, Rf1, Rf2)
        e0, e1, e2 = _cr(p0 + rc[i, 0], p1 + rc[i, 1], p2 + rc[i, 2], F[i, 0], F[i, 1], F[i, 2])
        nx = Rn0 + c0 + e0 + N[i, 0]; ny = Rn1 + c1 + e1 + N[i, 1]; nz = Rn2 + c2 + e2 + N[i, 2]
        fx = Rf0 + F[i, 0]; fy = Rf1 + F[i, 1]; fz = Rf2 + F[i, 2]
        # joint axis z_{i-1} in frame i = third row of R_i
        tau[i] = nx * R[i, 2, 0] + ny * R[i, 2, 1] + nz * R[i, 2, 2]


@njit(cache=True)
def rnea(q, qd, qdd, d, alpha, pstar, rc, m, I, g):
    R = np.empty((6, 3, 3)); _rots(q, alpha, R)
    tau = np.empty(6); F = np.empty((6, 3)); N = np.empty((6, 3))
    _rnea_R(R, qd, qdd, pstar, rc, m, I, g, tau, F, N)
    return tau


@njit(cache=True)
def mass_matrix(q, d, alpha, pstar, rc, m, I):
    R = np.empty((6, 3, 3)); _rots(q, alpha, R)
    M = np.empty((6, 6)); col = np.empty(6); F = np.empty((6, 3)); N = np.empty((6, 3))
    zero = np.zeros(6); e = np.zeros(6)
    for j in range(6):
        e[:] = 0.0; e[j] = 1.0
        _rnea_R(R, zero, e, pstar, rc, m, I, 0.0, col, F, N)
        M[:, j] = col
    return 0.5 * (M + M.T)


@njit(cache=True)
def forward_dyn(q, qd, u, d, alpha, pstar, rc, m, I, g, fc, b):
    R = np.empty((6, 3, 3)); _rots(q, alpha, R)
    F = np.empty((6, 3)); N = np.empty((6, 3))
    h = np.empty(6); zero = np.zeros(6)
    _rnea_R(R, qd, zero, pstar, rc, m, I, g, h, F, N)
    M = np.empty((6, 6)); col = np.empty(6); e = np.zeros(6)
    for j in range(6):
        e[:] = 0.0; e[j] = 1.0
        _rnea_R(R, zero, e, pstar, rc, m, I, 0.0, col, F, N)
        M[:, j] = col
    rhs = u - h - (fc * np.sign(qd) + b * qd)
    # Cholesky solve (M is symmetric positive definite)
    L = np.zeros((6, 6))
    for i in range(6):
        for j in range(i + 1):
            s = 0.5 * (M[i, j] + M[j, i])
            for k in range(j):
                s -= L[i, k] * L[j, k]
            if i == j:
                L[i, i] = np.sqrt(s)
            else:
                L[i, j] = s / L[j, j]
    y = np.empty(6)
    for i in range(6):
        s = rhs[i]
        for k in range(i):
            s -= L[i, k] * y[k]
        y[i] = s / L[i, i]
    x = np.empty(6)
    for i in range(5, -1, -1):
        s = y[i]
        for k in range(i + 1, 6):
            s -= L[k, i] * x[k]
        x[i] = s / L[i, i]
    return x


# ----------------------------------------------------------- controller ----
def oustaloup(order, dt, wb=1e-3, wh=1e3, N=5):
    """Parallel-form Oustaloup approximation of s^order, order in (-1, 1),
    each first-order section discretised exactly with a zero-order hold."""
    a = np.atleast_1d(np.asarray(order, float))
    Mn = 2 * N + 1
    k = np.arange(-N, N + 1)
    zeros = wb * (wh / wb) ** ((k[None, :] + N + 0.5 * (1 - a[:, None])) / Mn)
    poles = wb * (wh / wb) ** ((k[None, :] + N + 0.5 * (1 + a[:, None])) / Mn)
    K = wh ** a
    r = np.empty_like(poles)
    for j in range(Mn):
        num = np.prod(zeros - poles[:, [j]], axis=1)
        others = np.delete(poles, j, axis=1)
        den = np.prod(others - poles[:, [j]], axis=1)
        r[:, j] = K * num / den
    A = np.exp(-poles * dt)
    B = (1 - A) / poles
    return K, r, A, B


def controller(gains, dt):
    lam = np.asarray(gains['lambda'], float); mu = np.asarray(gains['mu'], float)
    ni = np.floor(lam + 1e-12); nd = np.floor(mu + 1e-12)
    if np.any(ni > 1) or np.any(nd > 1):
        raise ValueError('lambda and mu must be below 2')
    Ki_, ri, Ai, Bi = oustaloup(-(lam - ni), dt)
    Kd_, rd, Ad, Bd = oustaloup(mu - nd, dt)
    return (np.asarray(gains['Kp'], float), np.asarray(gains['Ki'], float), np.asarray(gains['Kd'], float),
            Ki_, ri, Ai, Bi, Kd_, rd, Ad, Bd, ni >= 1, nd >= 1)


# ----------------------------------------------------------- simulation ----
@njit(cache=True)
def _simulate(r, dt, Kp, Ki, Kd, KFi, rFi, AFi, BFi, KFd, rFd, AFd, BFd, use_int, use_diff,
              d, alpha, pstar, rc, m, I, g, fc, b, log_every):
    Nt = r.shape[1]
    q = np.zeros(6); qd = np.zeros(6)
    xi = np.zeros(rFi.shape); xd = np.zeros(rFd.shape)
    integ = np.zeros(6); prev = np.zeros(6); first = True
    nlog = (Nt - 1) // log_every + 1
    Q = np.empty((6, nlog)); U = np.empty((6, nlog))
    blown = False
    for k in range(Nt):
        e = r[:, k] - q
        yi = KFi * e
        for j in range(rFi.shape[1]):
            yi += rFi[:, j] * xi[:, j]
            xi[:, j] = AFi[:, j] * xi[:, j] + BFi[:, j] * e
        integ += yi * dt
        yd = KFd * e
        for j in range(rFd.shape[1]):
            yd += rFd[:, j] * xd[:, j]
            xd[:, j] = AFd[:, j] * xd[:, j] + BFd[:, j] * e
        if first:
            prev = yd.copy(); first = False
        diff = (yd - prev) / dt
        prev = yd.copy()
        ti = np.where(use_int, integ, yi)
        td = np.where(use_diff, diff, yd)
        u = Kp * e + Ki * ti + Kd * td
        if k % log_every == 0:
            Q[:, k // log_every] = q
            U[:, k // log_every] = u
        if k == Nt - 1:
            break
        k1q = qd; k1v = forward_dyn(q, qd, u, d, alpha, pstar, rc, m, I, g, fc, b)
        k2q = qd + dt / 2 * k1v; k2v = forward_dyn(q + dt / 2 * k1q, k2q, u, d, alpha, pstar, rc, m, I, g, fc, b)
        k3q = qd + dt / 2 * k2v; k3v = forward_dyn(q + dt / 2 * k2q, k3q, u, d, alpha, pstar, rc, m, I, g, fc, b)
        k4q = qd + dt * k3v;     k4v = forward_dyn(q + dt * k3q, k4q, u, d, alpha, pstar, rc, m, I, g, fc, b)
        q = q + dt / 6 * (k1q + 2 * k2q + 2 * k3q + k4q)
        qd = qd + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        if not (np.all(np.isfinite(q)) and np.max(np.abs(q)) < 1e3):
            blown = True
            break
    if blown:
        Q[:, :] = np.nan
    return Q, U


def reference(kind, t, t_step=1.0, omega=1.5):
    if kind == 'step':
        return np.tile((t >= t_step - 1e-12).astype(float), (6, 1))
    if kind == 'sine':
        return np.tile(np.sin(omega * t), (6, 1))
    raise ValueError(kind)


def simulate(plant, gains, kind, dt=1e-3, T=5.0, log_dt=0.01):
    t = np.arange(0, round(T / dt) + 1) * dt
    r = reference(kind, t)
    C = controller(gains, dt)
    every = int(round(log_dt / dt))
    Q, U = _simulate(r, dt, *C, plant['d'], plant['alpha'], plant['pstar'], plant['rc'], plant['m'],
                     plant['I'], plant['g'], plant['fc'], plant['b'], every)
    tl = t[::every]
    return tl, r[:, ::every], Q, U
