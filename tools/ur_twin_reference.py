"""Write data/twin_reference.csv for tools/check_twin.m.

Parses the gains from controller_gains.m, simulates the three identified
controllers through both experiments with the Python twin (tools/ur_twin.py)
and stores q on the 0.01 s grid.  Columns: run, t, q1..q6, with runs
  1..6   PID step, PID sine, FOPID step, FOPID sine, FBPA step, FBPA sine
         with the per-experiment gains
  7..12  the same with the shared gains.
"""
import os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ur_twin as T


def read_gains(path=os.path.join(HERE, '..', 'controller_gains.m')):
    src = open(path).read()
    out = {}
    for block in re.split(r"\n\s*case\s+", src)[1:]:
        m = re.match(r"'([\w/]+)'", block)
        if not m or 'gains.Kp' not in block:      # e.g. a case that loads gains from a file
            continue
        name = m.group(1)
        g = {}
        for key in ('Kp', 'Ki', 'Kd', 'lambda', 'mu'):
            m = re.search(r"gains\.%s\s*=\s*\[([^\]]*)\]" % key, block)
            g[key] = np.array([float(v) for v in m.group(1).replace('...', ' ').split()])
        out[name] = g
    return out


if __name__ == '__main__':
    gains = read_gains()
    plant = T.make_plant()          # must match robot_params() defaults
    rows = []
    k = 0
    for mode in ('separate', 'shared'):
        for c in ('PID', 'FOPID', 'FBPA'):
            for kind in ('step', 'sine'):
                k += 1
                g = gains[f'{c}/{kind if mode == "separate" else "shared"}']
                t, r, Q, U = T.simulate(plant, g, kind)
                rows.append(np.column_stack([np.full(t.size, k), t, Q.T]))
    np.savetxt(os.path.join(HERE, '..', 'data', 'twin_reference.csv'), np.vstack(rows), delimiter=',',
               header='run,t,q1,q2,q3,q4,q5,q6', comments='', fmt='%.15g')
    print('wrote data/twin_reference.csv')
