"""Identify gains and test plant hypotheses by inverse dynamics.

If the paper's arm is  M(q) qdd + C(q, qd) qd + G(q) = tau  and each joint is
driven by  tau_j = Kp e_j + Ki D^-lambda e_j + Kd D^mu e_j,  then the published
trajectories q(t) of all six joints fix the torque every joint must have
received:  tau(t) = RNEA(q, qd, qdd).  For a given (lambda, mu) that torque is
*linear* in (Kp, Ki, Kd), so the gains follow from ordinary least squares on
the paper's own curves -- step and sine together, since both experiments use
the same controllers -- with no closed-loop simulation at all.

How well the regression explains tau (R^2) measures how consistent a plant
hypothesis (link lengths, centre of mass, gravity) is with the published
figures, independently of any optimiser.

The derivative kick right after the step (1.00 <= t < 1.04 s) is excluded:
there qdd contains the impulse, which the 0.01 s published curves cannot
resolve.

Usage: python3 tools/inverse_dynamics_id.py
"""
import os, sys, json, itertools
import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_twin as T
import paper_data as D

DT = 1e-3
TF = np.round(np.arange(0, 5001) * DT, 10)


def dense_positions(kind, series):
    first = 6 if kind == 'step' else 13
    Q = np.vstack([D.load(f'fig{first + j:02d}', 'a', series, grid=TF) for j in range(6)])
    if kind == 'step':
        Q[:, TF < 0.995] = 0.0
    return Q


def derivatives(Q, kind, win=41):
    """Savitzky-Golay derivatives; for the step test the pre-step and
    post-step parts are smoothed separately so the kick is not smeared."""
    if kind == 'sine':
        segs = [np.arange(Q.shape[1])]
    else:
        k1 = int(round(1.0 / DT))
        segs = [np.arange(0, k1), np.arange(k1, Q.shape[1])]
    qs = np.zeros_like(Q); qd = np.zeros_like(Q); qdd = np.zeros_like(Q)
    for s in segs:
        qs[:, s] = savgol_filter(Q[:, s], win, 3, axis=1, mode='interp')
        qd[:, s] = savgol_filter(Q[:, s], win, 3, deriv=1, delta=DT, axis=1, mode='interp')
        qdd[:, s] = savgol_filter(Q[:, s], win, 3, deriv=2, delta=DT, axis=1, mode='interp')
    return qs, qd, qdd


def inverse_dynamics(plant, q, qd, qdd):
    a = (plant['d'], plant['alpha'], plant['pstar'], plant['rc'], plant['m'], plant['I'], plant['g'])
    return np.column_stack([T.rnea(q[:, k], qd[:, k], qdd[:, k], *a) for k in range(q.shape[1])])


def fractional_signals(e, lam, mu):
    """D^-lambda e and D^mu e with the same discrete operators as the controller."""
    g = dict(Kp=np.zeros(6), Ki=np.ones(6), Kd=np.zeros(6), **{'lambda': np.full(6, lam), 'mu': np.full(6, mu)})
    C = T.controller(g, DT)
    _, _, _, KFi, rFi, AFi, BFi, KFd, rFd, AFd, BFd, use_int, use_diff = C
    n = e.shape[1]
    I = np.zeros_like(e); Dd = np.zeros_like(e)
    xi = np.zeros(rFi.shape); xd = np.zeros(rFd.shape); integ = np.zeros(6); prev = None
    for k in range(n):
        ek = e[:, k]
        yi = KFi * ek + np.sum(rFi * xi, axis=1); xi = AFi * xi + BFi * ek[:, None]
        integ = integ + yi * DT
        yd = KFd * ek + np.sum(rFd * xd, axis=1); xd = AFd * xd + BFd * ek[:, None]
        if prev is None: prev = yd
        diff = (yd - prev) / DT; prev = yd
        I[:, k] = np.where(use_int, integ, yi)
        Dd[:, k] = np.where(use_diff, diff, yd)
    return I, Dd


def build(plant, series):
    rows = {}
    for kind in ('step', 'sine'):
        Q = dense_positions(kind, series)
        q, qd, qdd = derivatives(Q, kind)
        tau = inverse_dynamics(plant, q, qd, qdd)
        r = np.tile((TF >= 1.0 - 1e-9).astype(float), (6, 1)) if kind == 'step' else np.tile(np.sin(1.5 * TF), (6, 1))
        e = r - q
        if kind == 'step':
            keep = (TF < 0.99) | (TF >= 1.04)
        else:
            keep = TF >= 0.02
        rows[kind] = dict(tau=tau, e=e, keep=keep, t=TF)
    return rows


def regress(rows, lam=1.0, mu=1.0, positive=True):
    """Least squares tau_j = Kp e + Ki D^-lam e + Kd D^mu e per joint, both experiments."""
    from scipy.optimize import nnls
    out = []
    for j in range(6):
        A = []; b = []
        for kind in ('step', 'sine'):
            R = rows[kind]
            I, Dd = fractional_signals(R['e'], lam if np.isscalar(lam) else lam[j], mu if np.isscalar(mu) else mu[j])
            m = R['keep']
            A.append(np.column_stack([R['e'][j, m], I[j, m], Dd[j, m]]))
            b.append(R['tau'][j, m])
        A = np.vstack(A); b = np.concatenate(b)
        s = np.abs(A).max(axis=0) + 1e-12
        if positive:
            x, _ = nnls(A / s, b)
        else:
            x = np.linalg.lstsq(A / s, b, rcond=None)[0]
        x = x / s
        res = b - A @ x
        r2 = 1 - np.sum(res ** 2) / np.sum((b - b.mean()) ** 2)
        out.append(dict(K=x, r2=r2, rms_tau=np.sqrt(np.mean(b ** 2)), rms_res=np.sqrt(np.mean(res ** 2))))
    return out


if __name__ == '__main__':
    for lengths, com, g in itertools.product(['table2', 'ur5'], ['distal', 'middle', 'proximal'], [0.0]):
        L = T.TABLE2_L if lengths == 'table2' else T.UR5_L
        plant = T.make_plant(L=L, com=com, g=g)
        rows = build(plant, 'PID')
        res = regress(rows)
        print(f'{lengths:6s} {com:8s} g={g:4.2f}  PID  R2 per joint:', ' '.join('%.3f' % r['r2'] for r in res),
              '  mean %.3f' % np.mean([r['r2'] for r in res]))
        for j, r in enumerate(res):
            print('      j%d Kp=%9.4g Ki=%9.4g Kd=%9.4g   rms tau %.3g  rms res %.3g' % (j + 1, *r['K'], r['rms_tau'], r['rms_res']))
