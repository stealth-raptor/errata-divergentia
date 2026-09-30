"""Load the published curves extracted by extract_paper_curves.py onto the
paper's own 0.01 s logging grid."""
import os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, '..', 'data', 'paper_curves')
TG = np.round(np.arange(0, 501) * 0.01, 10)          # t = 0, 0.01, ..., 5 s


def load(fig, panel, series, grid=TG):
    d = np.loadtxt(os.path.join(DATA, f'{fig}_{panel}_{series}.csv'), delimiter=',', skiprows=1)
    o = np.argsort(d[:, 0], kind='stable')
    return np.interp(grid, d[o, 0], d[o, 1])


def positions(kind, series):
    """6 x 501 joint positions from Figs 6-11 (step) or 13-18 (sine), panel (a)."""
    first = 6 if kind == 'step' else 13
    Q = np.vstack([load(f'fig{first + j:02d}', 'a', series) for j in range(6)])
    if kind == 'step':
        Q[:, TG < 0.995] = 0.0        # flat before the step (drawn as one segment)
    return Q


def errors(kind, series):
    """6 x 501 tracking errors from panel (b)."""
    first = 6 if kind == 'step' else 13
    E = np.vstack([load(f'fig{first + j:02d}', 'b', series) for j in range(6)])
    if kind == 'step':
        E[:, TG < 0.995] = 0.0
    return E


def reference(kind):
    if kind == 'step':
        return np.tile((TG >= 1.0 - 1e-9).astype(float), (6, 1))
    return np.tile(np.sin(1.5 * TG), (6, 1))


def torques(kind, series):
    """6 x 501 joint torques from Fig 12 (step) or Fig 19 (sine)."""
    fig = 'fig12' if kind == 'step' else 'fig19'
    return np.vstack([load(fig, f'tau{j + 1}', series) for j in range(6)])


def write_grid(outdir=os.path.join(HERE, '..', 'data', 'paper_grid')):
    """Write the published curves on the paper's 0.01 s grid for Octave:
    <kind>_<series>_<q|e|tau>.csv with columns t, joint1..joint6."""
    os.makedirs(outdir, exist_ok=True)
    for kind in ('step', 'sine'):
        for series in ('PID', 'FOPID', 'FBPA'):
            for name, fn in (('q', positions), ('e', errors), ('tau', torques)):
                Y = fn(kind, series)
                A = np.column_stack([TG, Y.T])
                np.savetxt(os.path.join(outdir, f'{kind}_{series}_{name}.csv'), A, delimiter=',',
                           header='t,joint1,joint2,joint3,joint4,joint5,joint6', comments='', fmt='%.8g')
