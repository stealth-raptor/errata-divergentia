"""Joint-by-joint consistency test with the other joints prescribed.

For joint j, the other five joints are forced to follow the paper's own
published trajectories exactly; only joint j is simulated, under its own
(FO)PID, with the coupling torques of the real arm:

    M_jj(q) qdd_j = u_j - h_j(q, qd) - sum_{k != j} M_jk(q) qdd_k(paper)

and only joint j's three (PID) or five (FOPID) gains are identified, on the
step and sine experiments together.  This removes every interaction with
the fit of the other joints, so a remaining mismatch on joint j points at
the plant model (or the controller structure), not at the optimiser.

Usage: python3 tools/prescribed_joint_id.py PID table2 distal
"""
import os, sys, json
import numpy as np
from numba import njit
from scipy.signal import savgol_filter
import cma

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_twin as T
import paper_data as D

DT = 1e-3
TH = np.round(np.arange(0, 10001) * DT / 2, 10)        # half-step grid for RK4


def prescribed(kind, series):
    """Paper trajectories and their smoothed derivatives on the 0.5 ms grid."""
    first = 6 if kind == 'step' else 13
    Q = np.vstack([D.load(f'fig{first + j:02d}', 'a', series, grid=TH) for j in range(6)])
    h = DT / 2
    if kind == 'step':
        Q[:, TH < 0.995] = 0.0
        k1 = int(round(1.0 / h))
        segs = [np.arange(0, k1), np.arange(k1, Q.shape[1])]
    else:
        segs = [np.arange(Q.shape[1])]
    q = np.zeros_like(Q); qd = np.zeros_like(Q); qdd = np.zeros_like(Q)
    for s in segs:
        q[:, s] = Q[:, s]
        qd[:, s] = savgol_filter(Q[:, s], 81, 3, deriv=1, delta=h, axis=1, mode='interp')
        qdd[:, s] = savgol_filter(Q[:, s], 81, 3, deriv=2, delta=h, axis=1, mode='interp')
    return q, qd, qdd


@njit(cache=True)
def _acc_j(j, qj, qdj, u, Qh, Qdh, Qddh, kh, d, alpha, pstar, rc, m, I, g, vscale=1.0):
    q = Qh[:, kh].copy(); qd = Qdh[:, kh].copy() * vscale; qdd = Qddh[:, kh].copy()
    q[j] = qj; qd[j] = qdj * vscale; qdd[j] = 0.0
    # torque needed with qdd_j = 0, then add M_jj qdd_j
    tau0 = T.rnea(q, qd, qdd, d, alpha, pstar, rc, m, I, g)[j]
    e = np.zeros(6); e[j] = 1.0
    Mjj = T.rnea(q, np.zeros(6), e, d, alpha, pstar, rc, m, I, 0.0)[j]
    return (u - tau0) / Mjj


@njit(cache=True)
def _sim_j(j, r, Kp, Ki, Kd, KFi, rFi, AFi, BFi, KFd, rFd, AFd, BFd, use_int, use_diff,
           Qh, Qdh, Qddh, d, alpha, pstar, rc, m, I, g, vscale=1.0):
    Nt = r.shape[0]
    qj = 0.0; qdj = 0.0
    xi = np.zeros(rFi.shape[0]); xd = np.zeros(rFd.shape[0])
    integ = 0.0; prev = 0.0; first = True
    out = np.empty(Nt)
    for k in range(Nt):
        e = r[k] - qj
        yi = KFi * e
        for i in range(rFi.shape[0]):
            yi += rFi[i] * xi[i]; xi[i] = AFi[i] * xi[i] + BFi[i] * e
        integ += yi * DT
        yd = KFd * e
        for i in range(rFd.shape[0]):
            yd += rFd[i] * xd[i]; xd[i] = AFd[i] * xd[i] + BFd[i] * e
        if first:
            prev = yd; first = False
        diff = (yd - prev) / DT; prev = yd
        u = Kp * e + Ki * (integ if use_int else yi) + Kd * (diff if use_diff else yd)
        out[k] = qj
        if k == Nt - 1:
            break
        kh = 2 * k
        a1 = _acc_j(j, qj, qdj, u, Qh, Qdh, Qddh, kh, d, alpha, pstar, rc, m, I, g, vscale)
        q2 = qj + DT / 2 * qdj; v2 = qdj + DT / 2 * a1
        a2 = _acc_j(j, q2, v2, u, Qh, Qdh, Qddh, kh + 1, d, alpha, pstar, rc, m, I, g, vscale)
        q3 = qj + DT / 2 * v2; v3 = qdj + DT / 2 * a2
        a3 = _acc_j(j, q3, v3, u, Qh, Qdh, Qddh, kh + 1, d, alpha, pstar, rc, m, I, g, vscale)
        q4 = qj + DT * v3; v4 = qdj + DT * a3
        a4 = _acc_j(j, q4, v4, u, Qh, Qdh, Qddh, kh + 2, d, alpha, pstar, rc, m, I, g, vscale)
        qj = qj + DT / 6 * (qdj + 2 * v2 + 2 * v3 + v4)
        qdj = qdj + DT / 6 * (a1 + 2 * a2 + 2 * a3 + a4)
        if not np.isfinite(qj) or abs(qj) > 1e3:
            out[:] = np.nan
            break
    return out


class JointProblem:
    def __init__(self, series, plant, j):
        self.j = j; self.plant = plant; self.frac = series == 'FOPID'
        self.pre = {k: prescribed(k, series) for k in ('step', 'sine')}
        tf = np.arange(0, 5001) * DT
        self.r = dict(step=(tf >= 1 - 1e-12).astype(float), sine=np.sin(1.5 * tf))
        self.data = {k: D.positions(k, series)[j] for k in ('step', 'sine')}

    def gains(self, x):
        g = dict(Kp=np.full(6, 10 ** x[0]), Ki=np.full(6, 10 ** x[1]), Kd=np.full(6, 10 ** x[2]))
        g['lambda'] = np.full(6, x[3] if self.frac else 1.0)
        g['mu'] = np.full(6, x[4] if self.frac else 1.0)
        return g

    def sim(self, x, kind):
        C = T.controller(self.gains(x), DT)
        Kp, Ki, Kd, KFi, rFi, AFi, BFi, KFd, rFd, AFd, BFd, ui, ud = C
        P = self.plant
        q, qd, qdd = self.pre[kind]
        y = _sim_j(self.j, self.r[kind], Kp[0], Ki[0], Kd[0], KFi[0], rFi[0], AFi[0], BFi[0],
                   KFd[0], rFd[0], AFd[0], BFd[0], bool(ui[0]), bool(ud[0]), q, qd, qdd,
                   P['d'], P['alpha'], P['pstar'], P['rc'], P['m'], P['I'], P['g'], P.get('vscale', 1.0))
        return y[::10]

    def cost(self, x):
        c = 0.0
        for kind in ('step', 'sine'):
            y = self.sim(x, kind)
            if not np.all(np.isfinite(y)):
                return 100.0
            c += np.mean((y - self.data[kind]) ** 2)
        return c / 2


def fit_joint(series, plant, j, restarts=4, seed=0):
    prob = JointProblem(series, plant, j)
    n = 5 if prob.frac else 3
    lo = np.array([-3, -4, -4] + ([0.05, 0.05] if prob.frac else []))
    hi = np.array([7, 7, 5] + ([1.95, 1.95] if prob.frac else []))
    best = (np.inf, None)
    rng = np.random.default_rng(seed)
    for rs in range(restarts):
        x0 = lo + (hi - lo) * rng.uniform(0.25, 0.75, n)
        es = cma.CMAEvolutionStrategy(x0.tolist(), 1.5, dict(bounds=[lo.tolist(), hi.tolist()], seed=seed + rs,
                                                              verbose=-9, maxfevals=1500, popsize=12))
        es.optimize(prob.cost)
        if es.result.fbest < best[0]:
            best = (es.result.fbest, np.array(es.result.xbest))
    return prob, best[1], np.sqrt(best[0])


if __name__ == '__main__':
    series = sys.argv[1] if len(sys.argv) > 1 else 'PID'
    lengths = sys.argv[2] if len(sys.argv) > 2 else 'table2'
    com = sys.argv[3] if len(sys.argv) > 3 else 'distal'
    joints = [int(s) - 1 for s in sys.argv[4].split(',')] if len(sys.argv) > 4 else range(6)
    L = T.TABLE2_L if lengths == 'table2' else T.UR5_L
    plant = T.make_plant(L=L, com=com)
    res = {}
    for j in joints:
        prob, x, rms = fit_joint(series, plant, j)
        rs = {k: float(np.sqrt(np.mean((prob.sim(x, k) - prob.data[k]) ** 2))) for k in ('step', 'sine')}
        print(f'{series} {lengths} {com} joint {j+1}: rms {rms:.4f} (step {rs["step"]:.4f}, sine {rs["sine"]:.4f})  '
              f'Kp={10**x[0]:.4g} Ki={10**x[1]:.4g} Kd={10**x[2]:.4g}' + (f' lam={x[3]:.3f} mu={x[4]:.3f}' if len(x) > 3 else ''),
              flush=True)
        res[j + 1] = dict(x=[float(v) for v in x], rms=rms, **rs)
    out = sys.argv[5] if len(sys.argv) > 5 else None
    if out:
        json.dump(res, open(out, 'w'), indent=1)
