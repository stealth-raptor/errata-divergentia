"""Exact consistency test for joint 1 (vertical axis) by angular momentum.

Nothing in the arm's inertia matrix depends on q1, so joint 1's equation of
motion is exactly the balance of angular momentum about the base axis,

    d/dt p1 = tau1,      p1 = sum_k M_1k(q) qd_k.

Integrating once, the controller law gives

    PID:    p1(t) = Kd e1(t) + Kp int e1 + Ki int int e1
    FOPID:  p1(t) = Kp int e1 + Ki int D^-lambda e1 + Kd int D^mu e1

which holds straight through the step (the derivative kick is the jump of
Kd e1).  p1 is computed from the paper's published trajectories of all six
joints (first derivatives only), so this is a linear least-squares test of
whether *any* gains can explain the published joint-1 curves under a given
plant model -- no simulation, no optimiser, almost no noise amplification.

Usage: python3 tools/joint1_momentum_test.py
"""
import os, sys
import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_twin as T
import paper_data as D
import inverse_dynamics_id as ID

DT = ID.DT
TF = ID.TF


def momentum(plant, kind, series):
    Q = ID.dense_positions(kind, series)
    q, qd, _ = ID.derivatives(Q, kind)
    a = (plant['d'], plant['alpha'], plant['pstar'], plant['rc'], plant['m'], plant['I'])
    p1 = np.empty(Q.shape[1])
    for k in range(Q.shape[1]):
        M = T.mass_matrix(q[:, k], *a)
        p1[k] = M[0] @ qd[:, k]
    r = (TF >= 1.0 - 1e-9).astype(float) if kind == 'step' else np.sin(1.5 * TF)
    return p1, r - q[0]


def regressors(e, lam=1.0, mu=1.0):
    E = np.tile(e, (6, 1))
    I, Dd = ID.fractional_signals(E, lam, mu)
    ie = np.cumsum(e) * DT
    if lam == 1.0 and mu == 1.0:
        return np.column_stack([ie, np.cumsum(ie) * DT, e])          # Kp, Ki, Kd
    return np.column_stack([ie, np.cumsum(I[0]) * DT, np.cumsum(Dd[0]) * DT])


def test(plant, series, kinds=('step', 'sine'), lam=1.0, mu=1.0):
    A = []; b = []
    for kind in kinds:
        p1, e = momentum(plant, kind, series)
        keep = np.ones_like(e, bool)
        if kind == 'step':
            keep = (TF < 0.99) | (TF > 1.01)     # the kick itself is a jump, keep both sides
        A.append(regressors(e, lam, mu)[keep]); b.append(p1[keep])
    A = np.vstack(A); b = np.concatenate(b)
    s = np.abs(A).max(axis=0)
    x = np.linalg.lstsq(A / s, b, rcond=None)[0] / s
    res = b - A @ x
    r2 = 1 - np.sum(res ** 2) / np.sum((b - b.mean()) ** 2)
    return x, r2


if __name__ == '__main__':
    for lengths in ('table2', 'ur5'):
        for com in ('distal', 'middle', 'proximal'):
            L = T.TABLE2_L if lengths == 'table2' else T.UR5_L
            P = T.make_plant(L=L, com=com)
            line = []
            for kinds in (('step',), ('sine',), ('step', 'sine')):
                x, r2 = test(P, 'PID', kinds)
                line.append(f'{"+".join(kinds):9s} R2={r2:.4f} (Kp={x[0]:.3g} Ki={x[1]:.3g} Kd={x[2]:.3g})')
            print(f'PID  {lengths:6s} {com:8s} ' + ' | '.join(line), flush=True)
