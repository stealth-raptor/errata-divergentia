"""Figures of the paper draft (paper/main.tex), as vector PDFs.

Reads what the MATLAB code stored and simulates nothing:
  results/table_gains/simulation_results.mat   written by main('table'):
      the trajectories of PID and FOPID (gains fitted to the reference
      paper's Tables 3-4) and of the FOPSO-GWO-tuned FOPID
  results/optimizer_runs/fopso_gwo_whole_*.mat  written by tools/compare_optimizers.m:
      the best-fitness history of the eight tuning runs

Run from the repository root:  python3 paper/figures/make_figures.py
Needs numpy, scipy and matplotlib.
"""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter
import numpy as np
import scipy.io as sio

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT = os.path.join(ROOT, 'paper', 'figures')

# Okabe-Ito colours (colour-blind safe)
CONTROLLERS = [('PID', 'PID', '#CC79A7', (0, (1, 1))),
               ('FOPID', 'FOPID', '#0072B2', (0, (4, 1.5))),
               ('FOPSO_GWO', 'FOPSO-GWO-FOPID', '#D55E00', '-')]
GWO = '#D55E00'

plt.rcParams.update({'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8,
                     'legend.fontsize': 7, 'xtick.labelsize': 7, 'ytick.labelsize': 7,
                     'axes.linewidth': 0.6, 'lines.linewidth': 1.0,
                     'pdf.fonttype': 42})

PLAIN = FuncFormatter(lambda v, _: ('%g' % v))


def plain_log_axis(ax):
    """Log y axis labelled 0.5, 1, 2 ... instead of 5x10^-1."""
    ax.yaxis.set_major_formatter(PLAIN)
    ax.yaxis.set_minor_formatter(NullFormatter())


def load(path):
    return sio.loadmat(path, squeeze_me=True, struct_as_record=False)


def joint_grid(sim, experiment, signal, ylabel, filename):
    """2 x 3 panels, one per joint, the three controllers overlaid."""
    fig, axes = plt.subplots(2, 3, figsize=(6.6, 3.6), sharex=True)
    for j, ax in enumerate(axes.flat):
        if signal == 'q':
            run = getattr(sim.FOPSO_GWO, experiment)
            ax.plot(run.t, run.r[j], color='#BBBBBB', lw=2.2, label='reference', zorder=0)
        for key, label, colour, style in CONTROLLERS:
            run = getattr(getattr(sim, key), experiment)
            y = run.q[j] if signal == 'q' else run.r[j] - run.q[j]
            ax.plot(run.t, y, color=colour, ls=style, label=label,
                    lw=1.3 if key == 'FOPSO_GWO' else 0.9)
        ax.set_title('joint %d' % (j + 1), fontsize=8, pad=2)
        ax.grid(True, lw=0.3, color='#dddddd')
        ax.set_xlim(0, 5)
        if j >= 3:
            ax.set_xlabel('time (s)')
        if j % 3 == 0:
            ax.set_ylabel(ylabel)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=len(labels), frameon=False,
               bbox_to_anchor=(0.5, -0.01))
    fig.tight_layout(rect=(0, 0.06, 1, 1), h_pad=0.6, w_pad=0.6)
    fig.savefig(os.path.join(OUT, filename))
    plt.close(fig)


def runs(prefix, cost, seeds):
    out = []
    for s in seeds:
        f = os.path.join(ROOT, 'results', '%s_%s_seed%d.mat' % (prefix, cost, s))
        if os.path.exists(f):
            m = load(f)
            out.append({'cost': float(m['cost']), 'history': np.atleast_1d(m['info'].history),
                        'evaluations': float(m['info'].evaluations)})
    return out


def convergence(filename):
    """Fitness against cost evaluations: the eight runs and the reported (best) one."""
    fig, ax = plt.subplots(figsize=(4.4, 2.6))
    rr = runs('optimizer_runs/fopso_gwo', 'whole', range(1, 9))
    K = len(rr[0]['history'])
    evals = 30 + (rr[0]['evaluations'] - 30) / K * np.arange(1, K + 1)
    best = min(range(len(rr)), key=lambda i: rr[i]['cost'])
    for i, r in enumerate(rr):
        if i != best:
            ax.plot(evals, r['history'], color='#999999', lw=0.6,
                    label='other runs' if i == 0 else None)
    ax.plot(evals, rr[best]['history'], color=GWO, lw=1.6, label='best run')
    ax.set_yscale('log')
    ax.set_yticks([1, 2, 5, 10, 20, 40])
    plain_log_axis(ax)
    ax.set_xlim(0, 3030)
    ax.set_xlabel('fitness evaluations')
    ax.set_ylabel('best fitness $J$')
    ax.grid(True, which='both', lw=0.3, color='#dddddd')
    ax.legend(frameon=False, loc='upper right')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, filename))
    plt.close(fig)


def main():
    sim = load(os.path.join(ROOT, 'results', 'table_gains', 'simulation_results.mat'))['results']
    joint_grid(sim, 'step', 'q', 'angle (rad)', 'fig_step.pdf')
    joint_grid(sim, 'sine', 'e', 'error (rad)', 'fig_sine_error.pdf')
    convergence('fig_convergence.pdf')
    print('figures written to', OUT)


if __name__ == '__main__':
    main()
