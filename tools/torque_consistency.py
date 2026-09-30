"""Are the paper's torque figures the output of its control law?

For each controller and joint, fit  tau = Kp e + Ki int e + Kd de/dt  by least
squares, with tau from Fig. 12 (step) or Fig. 19 (sine) and e from the error
panels of Figs 6-11 / 13-18, all on the paper's 0.01 s grid (the derivative
as a backward difference, as a fixed-step derivative block would compute it).
For the sine test the largest 20 % of |tau| (the spikes) are left out.

A torque curve produced by the stated controller would give R^2 close to 1
and positive gains; the published ones do not (docs/audit_report.md, 3.4).

Usage: python3 tools/torque_consistency.py
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import paper_data as D

h = 0.01
for kind in ('step', 'sine'):
    print('=====', kind)
    for s in ('PID', 'FOPID'):
        E = D.errors(kind, s)
        U = D.torques(kind, s)
        for j in range(6):
            e, u = E[j], U[j]
            ie = np.cumsum(e) * h
            de = np.r_[0, np.diff(e)] / h
            m = D.TG > (1.2 if kind == 'step' else 0.0)      # the step kick itself is left out
            A = np.c_[e, ie, de][m]
            w = np.ones(m.sum(), bool)
            if kind == 'sine':
                w = np.abs(u[m]) < np.quantile(np.abs(u[m]), 0.8)
            x, *_ = np.linalg.lstsq(A[w], u[m][w], rcond=None)
            res = u[m][w] - A[w] @ x
            r2 = 1 - res @ res / np.sum((u[m][w] - u[m][w].mean()) ** 2)
            print(f'  {s:5s} joint {j+1}: Kp={x[0]:+.3g} Ki={x[1]:+.3g} Kd={x[2]:+.3g}   R2={r2:.2f}'
                  f'   max|tau| {np.abs(u[m]).max():.3g} Nm')
