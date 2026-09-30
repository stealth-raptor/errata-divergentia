"""Sweep: can each joint's published step AND sine curves be explained by one
set of gains, for each plant hypothesis?  Uses prescribed_joint_id (the other
five joints follow the paper's trajectories exactly).

For every (hypothesis, controller, joint) three fits are made: step+sine
together, step only, sine only.  Output: data/consistency_sweep.json

Usage: python3 tools/consistency_sweep.py [PID|FOPID ...]
"""
import os, sys, json, itertools
from multiprocessing import Pool
import numpy as np
import cma

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_twin as T
import prescribed_joint_id as PJ

HYP = [(l, c) for l in ('table2', 'ur5') for c in ('distal', 'middle', 'proximal')]


def task(args):
    series, lengths, com, j = args
    L = T.TABLE2_L if lengths == 'table2' else T.UR5_L
    prob = PJ.JointProblem(series, T.make_plant(L=L, com=com), j)
    frac = series == 'FOPID'
    lo = [-3, -4, -4] + ([0.05, 0.05] if frac else []); hi = [7, 7, 5] + ([1.95, 1.95] if frac else [])
    x0 = [1.5, 1.0, 0.5] + ([1.0, 1.0] if frac else [])
    rms = {}
    sims = {}

    def err(x, k):
        y = prob.sim(x, k)
        return np.mean((y - prob.data[k]) ** 2) if np.all(np.isfinite(y)) else 100.0

    for obj, kinds in (('both', ('step', 'sine')), ('step', ('step',)), ('sine', ('sine',))):
        best = (np.inf, None)
        for rs in range(3):
            es = cma.CMAEvolutionStrategy(x0, 1.5, dict(bounds=[lo, hi], seed=rs + 1, verbose=-9,
                                                        maxfevals=800 if not frac else 1200, popsize=10))
            es.optimize(lambda x: np.mean([err(x, k) for k in kinds]))
            if es.result.fbest < best[0]:
                best = (es.result.fbest, np.array(es.result.xbest))
        x = best[1]
        rms[obj] = dict(x=[float(v) for v in x],
                        step=float(np.sqrt(err(x, 'step'))), sine=float(np.sqrt(err(x, 'sine'))))
    return (series, lengths, com, j + 1, rms)


if __name__ == '__main__':
    series_list = sys.argv[1:] or ['PID']
    tasks = [(s, l, c, j) for s in series_list for (l, c) in HYP for j in range(6)]
    out_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'consistency_sweep.json')
    results = json.load(open(out_file)) if os.path.exists(out_file) else {}
    with Pool(4) as pool:
        for (s, l, c, j, rms) in pool.imap_unordered(task, tasks):
            results.setdefault(s, {}).setdefault(f'{l}/{c}', {})[str(j)] = rms
            print(f'{s:5s} {l:6s} {c:8s} j{j}: both step/sine {rms["both"]["step"]:.3f}/{rms["both"]["sine"]:.3f}'
                  f' | step-only {rms["step"]["step"]:.3f} | sine-only {rms["sine"]["sine"]:.3f}', flush=True)
            json.dump(results, open(out_file, 'w'), indent=1)
