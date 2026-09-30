"""Identify the unpublished PID / FOPID gains from the paper's own curves.

The paper publishes no gains.  What it does publish, as vector graphics, is
every joint trajectory of both experiments (Figs 6-11 step, Figs 13-18 sine).
Those curves were extracted exactly (tools/extract_paper_curves.py), so the
gains can be identified by least squares: simulate the closed loop and match
all 12 published trajectories of a controller at the paper's 0.01 s logging
instants.  The same gain set has to reproduce the step AND the sine figures.

  PID    18 unknowns  Kp, Ki, Kd per joint            (lambda = mu = 1)
  FOPID  30 unknowns  Kp, Ki, Kd, lambda, mu per joint (paper, Sect. 3)

Search: block-coordinate CMA-ES (one joint at a time, coupled simulation),
then CMA-ES over all gains, then trust-region least squares.

Usage:
  python3 tools/identify_gains.py PID   --lengths table2 --com distal --out fit_PID.json
  python3 tools/identify_gains.py FOPID --lengths table2 --com distal --out fit_FOPID.json
"""
import argparse, json, os, sys, time
from multiprocessing import Pool
import numpy as np
import cma
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ur_twin as T
import paper_data as D

LOG_BOUNDS = {'Kp': (-3, 7), 'Ki': (-4, 7), 'Kd': (-4, 5)}
ORDER_BOUNDS = (0.05, 1.95)


class Problem:
    def __init__(self, controller, plant, weights=(1.0, 1.0), t_from=(0.0, 0.0)):
        self.controller = controller
        self.plant = plant
        self.data = {k: D.positions(k, controller) for k in ('step', 'sine')}
        self.w = dict(step=weights[0], sine=weights[1])
        self.mask = dict(step=D.TG >= t_from[0], sine=D.TG >= t_from[1])
        self.frac = controller == 'FOPID'
        self.n = 30 if self.frac else 18

    # parameter vector <-> gains
    def gains(self, x):
        x = np.asarray(x, float)
        g = dict(Kp=10 ** x[0:6], Ki=10 ** x[6:12], Kd=10 ** x[12:18])
        if self.frac:
            g['lambda'] = x[18:24]; g['mu'] = x[24:30]
        else:
            g['lambda'] = np.ones(6); g['mu'] = np.ones(6)
        return g

    def bounds(self):
        lo = [LOG_BOUNDS['Kp'][0]] * 6 + [LOG_BOUNDS['Ki'][0]] * 6 + [LOG_BOUNDS['Kd'][0]] * 6
        hi = [LOG_BOUNDS['Kp'][1]] * 6 + [LOG_BOUNDS['Ki'][1]] * 6 + [LOG_BOUNDS['Kd'][1]] * 6
        if self.frac:
            lo += [ORDER_BOUNDS[0]] * 12; hi += [ORDER_BOUNDS[1]] * 12
        return np.array(lo), np.array(hi)

    def residuals(self, x):
        g = self.gains(x)
        out = []
        for kind in ('step', 'sine'):
            try:
                _, _, Q, _ = T.simulate(self.plant, g, kind)
            except Exception:
                Q = np.full((6, D.TG.size), np.nan)
            res = (Q - self.data[kind])[:, self.mask[kind]] * self.w[kind]
            res = np.where(np.isfinite(res), res, 10.0)
            out.append(np.clip(res, -10, 10).ravel())
        return np.concatenate(out)

    def cost(self, x):
        r = self.residuals(x)
        return float(np.mean(r ** 2))

    def per_joint_rms(self, x):
        g = self.gains(x)
        out = {}
        for kind in ('step', 'sine'):
            _, _, Q, _ = T.simulate(self.plant, g, kind)
            out[kind] = np.sqrt(np.mean((Q - self.data[kind]) ** 2, axis=1))
        return out


_P = None


def _init(controller, plant, weights=(1.0, 1.0)):
    global _P
    _P = Problem(controller, plant, weights=weights)


def _cost(x):
    return _P.cost(x)


def initial_guess(prob):
    """Crude decoupled start: J_eff from M(0), natural frequency ~6 rad/s."""
    P = prob.plant
    M0 = T.mass_matrix(np.zeros(6), P['d'], P['alpha'], P['pstar'], P['rc'], P['m'], P['I'])
    J = np.diag(M0)
    wn, zeta = 6.0, 0.35
    Kp = J * wn ** 2; Kd = 2 * zeta * wn * J; Ki = 0.5 * Kp
    x = list(np.log10(Kp)) + list(np.log10(Ki)) + list(np.log10(Kd))
    if prob.frac:
        x += [1.0] * 6 + [1.0] * 6
    return np.array(x)


def _cma(pool, prob, x, idx, sigma, maxiter, popsize, seed, lo, hi):
    """CMA-ES over the coordinates idx of x, the rest held fixed."""
    idx = np.asarray(idx)
    x = np.array(x, float)
    z0 = np.clip(x[idx], lo[idx] + 1e-6, hi[idx] - 1e-6)
    opts = dict(bounds=[lo[idx].tolist(), hi[idx].tolist()], maxiter=maxiter, seed=seed, verbose=-9,
                popsize=popsize, tolfun=1e-10, tolx=1e-5)
    es = cma.CMAEvolutionStrategy(z0.tolist(), sigma, opts)
    best_f = np.inf; best = x.copy()
    while not es.stop():
        Z = es.ask()
        X = []
        for z in Z:
            xx = x.copy(); xx[idx] = z; X.append(xx)
        F = pool.map(_cost, X)
        es.tell(Z, F)
        k = int(np.argmin(F))
        if F[k] < best_f:
            best_f = F[k]; best = X[k].copy()
    return best, best_f


def joint_indices(prob, j):
    idx = [j, 6 + j, 12 + j]
    if prob.frac:
        idx += [18 + j, 24 + j]
    return idx


def identify(controller, plant, x0=None, cycles=4, block_iter=40, full_iter=150, workers=4,
             seed=1, lsq=True, verbose=True, weights=(1.0, 1.0)):
    """Staged identification:
      1. block-coordinate CMA-ES, one joint's gains at a time (fully coupled
         simulation, total cost), cycled over the joints;
      2. CMA-ES over all gains with a small step size;
      3. trust-region least squares."""
    prob = Problem(controller, plant, weights=weights)
    lo, hi = prob.bounds()
    x = initial_guess(prob) if x0 is None else np.array(x0, float)
    x = np.clip(x, lo + 1e-6, hi - 1e-6)
    t0 = time.time()
    with Pool(workers, initializer=_init, initargs=(controller, plant, weights)) as pool:
        f = pool.map(_cost, [x])[0]
        if verbose:
            print(f'  [{controller}] start rms {np.sqrt(f):.4f} rad', flush=True)
        for c in range(cycles):
            sigma = 0.8 if c == 0 else 0.4 / c
            for j in range(6):
                xb, fb = _cma(pool, prob, x, joint_indices(prob, j), sigma, block_iter, 10, seed + 17 * c + j, lo, hi)
                if fb < f:
                    x, f = xb, fb
            if verbose:
                print(f'  [{controller}] cycle {c + 1}: rms {np.sqrt(f):.4f} rad  ({time.time() - t0:.0f}s)', flush=True)
        if full_iter > 0:
            xb, fb = _cma(pool, prob, x, np.arange(prob.n), 0.15, full_iter, 16, seed + 999, lo, hi)
            if fb < f:
                x, f = xb, fb
            if verbose:
                print(f'  [{controller}] full CMA: rms {np.sqrt(f):.4f} rad  ({time.time() - t0:.0f}s)', flush=True)
    if lsq:
        ls = least_squares(prob.residuals, x, bounds=(lo, hi), x_scale='jac', diff_step=1e-4, max_nfev=60 * prob.n)
        if prob.cost(ls.x) < f:
            x = ls.x; f = prob.cost(x)
        if verbose:
            print(f'  [{controller}] least squares: rms {np.sqrt(f):.4f} rad  ({time.time() - t0:.0f}s)', flush=True)
    return prob, x


def to_json(prob, x, meta):
    g = prob.gains(x)
    rms = prob.per_joint_rms(x)
    return dict(controller=prob.controller, meta=meta,
                gains={k: [float(v) for v in np.asarray(g[k])] for k in ('Kp', 'Ki', 'Kd', 'lambda', 'mu')},
                x=[float(v) for v in x],
                rms_step=[float(v) for v in rms['step']], rms_sine=[float(v) for v in rms['sine']],
                rms_total=float(np.sqrt(prob.cost(x))))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('controller', choices=['PID', 'FOPID'])
    ap.add_argument('--lengths', default='table2', choices=['table2', 'ur5'])
    ap.add_argument('--com', default='distal', choices=['distal', 'middle', 'proximal'])
    ap.add_argument('--g', type=float, default=0.0)
    ap.add_argument('--cycles', type=int, default=4)
    ap.add_argument('--full-iter', type=int, default=150)
    ap.add_argument('--no-lsq', action='store_true')
    ap.add_argument('--weights', default='1,1', help='weights of the step and sine residuals, e.g. 1,0')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--x0', default=None, help='json file with a previous fit to start from')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    L = T.TABLE2_L if a.lengths == 'table2' else T.UR5_L
    plant = T.make_plant(L=L, com=a.com, g=a.g)
    x0 = np.array(json.load(open(a.x0))['x']) if a.x0 else None
    w = tuple(float(v) for v in a.weights.split(','))
    prob, x = identify(a.controller, plant, x0=x0, cycles=a.cycles, full_iter=a.full_iter, seed=a.seed,
                       lsq=not a.no_lsq, weights=w)
    res = to_json(Problem(a.controller, plant), x, dict(lengths=a.lengths, com=a.com, g=a.g, seed=a.seed, weights=w))
    json.dump(res, open(a.out, 'w'), indent=1)
    print(json.dumps({k: res[k] for k in ('rms_step', 'rms_sine', 'rms_total')}))
    print(json.dumps(res['gains']))
