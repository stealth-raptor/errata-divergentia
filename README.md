# Trajectory tracking of a 6-DOF robotic arm: PID, FOPID and FOPSO-GWO

Octave reproduction of the simulation results in

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025). DOI 10.1007/s40435-025-01620-x

The arm model, both controllers and both experiments of the paper are built from scratch. The
controller gains, which the paper does not publish, are **identified from the paper's own
published curves**. These were read exactly from the vector graphics of the PDF.

On top of the reproduction, the FOPID is re-tuned with a **fractional-order PSO / grey-wolf
hybrid (FOPSO-GWO)**, this work's counterpart of the paper's FBPA optimiser.

This branch (`brand-new-day`) is the final code. It combines
* the audited model and the PID / FOPID reproduction of branch `audit`: a line-by-line audit
  against the paper, why the published graphs could not be matched before, and several
  inconsistencies found in the paper itself, all in [`docs/audit_report.md`](docs/audit_report.md);
* the FOPSO-GWO tuner of branch `claude/pensive-ramanujan-mmu564`, ported onto that model (only
  the optimiser, the tuning driver and its fitness function).

## Run it

```
octave --eval build_mex             # once: compiles the C simulation (about 1000x faster)
octave --eval "tune_fopid_hybrid(struct('Target', 'FBPA', 'RandomSeed', 4))"   # FOPSO-GWO, ~6 min
octave --eval main                  # PID, FOPID and FOPSO-GWO -> results/
octave --eval "main('shared')"      # one PID / FOPID gain set for both experiments -> results/shared_gains/
```

Without `build_mex` everything still runs on the plain .m simulation, with identical results,
but about 40 s per simulation instead of 0.03 s; tuning is then only practical with a small
budget. No packages are needed, and the code also runs in MATLAB. `main` simulates the
controllers through both experiments and compares them with the paper in three ways:
1. the published Tables 3 and 4;
2. the same metrics recomputed from the paper's own figures, which do not fully agree with its
   tables;
3. every published curve, joint by joint.

Outputs: `summary.md`, the figure set, and `simulation_results.mat`. `results/` in this branch
holds the output of the tuning command above followed by `main`.

**Gain sets.** The paper publishes no gains and does not say whether its step and sine
experiments used the same ones. `controller_gains(controller, experiment)` holds three
identified sets per controller: `'step'`, `'sine'` and `'shared'`. The default `main` uses the
per-experiment sets, which match the figures most closely; `main('shared')` uses one set for
both experiments, as a single controller would.

Checks:

```
octave --eval "addpath tools; verify_dynamics"   # dynamics vs Jacobian formula, energy conservation
octave --eval "addpath tools; check_twin"        # Octave model == Python identification twin
octave --eval "addpath tools; check_twin('shared')"
octave --eval "addpath tools; check_mex"         # compiled simulation == .m loop (~10 min)
```

### Figures

| File | Corresponds to | Content |
|---|---|---|
| `fig06…fig11_jointN_step.png` | Figs 6–11 | step response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig12_step_torque.png` | Fig 12 | joint driving torques, step response (3×2 grid) |
| `fig13…fig18_jointN_sine.png` | Figs 13–18 | sine response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig19_sine_torque.png` | Fig 19 | joint driving torques, sine response (3×2 grid) |
| `compare_step.png`, `compare_sine.png` | Figs 6–11, 13–18 | every published curve (thick black) overlaid on this work (thin colour), with the rms difference per joint |

Colours are the paper's own: reference ("dir") red, PID green, FOPID blue. In the sine error
panels and in Fig 19 the paper draws FOPID in red, which is kept. FOPSO-GWO, in place of the
paper's FBPA-FOPID curve, is magenta. All signals are logged every 0.01 s, as in the paper.

## Result: PID and FOPID

Every published curve of Figs 6–11 and 13–18 was read from the PDF, and the gains were
identified so that the simulation matches them.

**Match to the published curves** (rms of q_this work − q_paper, rad):

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean, separate gains (`main`) | mean, shared gains (`main('shared')`) | previous gains |
|---|---|---|---|---|---|---|---|---|---|---|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | **0.092** | 0.108 | 0.297 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | **0.080** | 0.099 | 0.188 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | **0.058** | 0.070 | 0.171 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | **0.051** | 0.062 | 0.099 |

The per-joint columns are for the separate gains. Separate gains lower every mean by 15–19 %,
mostly on joints 3–5. Joint 1's step does not improve (audit report 4.3).

**Tables 3 and 4** (separate gains). "Paper figures" is the same metric recomputed from the
paper's own published curves. The paper's tables and figures do not fully agree with each other.

| Metric | PID: paper table | PID: paper figures | PID: this work | FOPID: paper table | FOPID: paper figures | FOPID: this work |
|---|---:|---:|---:|---:|---:|---:|
| Step overshoot | 54.6 % | 59.1 % | 44.0 % | 31.2 % | 34.4 % | 33.2 % |
| Step adjustment time (5 %) | 2.44 s | 2.32 s | 2.98 s | 1.89 s | 1.95 s | 1.95 s |
| Step peak time | 1.39 s | 1.44 s | 1.47 s | 1.33 s | 1.46 s | 1.42 s |
| Sine MSE | 2.27e-2 | 2.26e-2 | 1.54e-2 | 8.8e-3 | 8.8e-3 | 5.5e-3 |
| Sine Σ\|τ\| | 3.74e4 | 2.32e4 | 1.05e4 | 2.57e4 | 3.74e4 | 1.03e4 |

`main('shared')` writes the same tables for the shared gains to `results/shared_gains/summary.md`.

Why an exact match of every panel is not possible (details in the audit report):

* **The paper contradicts itself.** Its joint-1 step and sine curves cannot come from the same arm
  and controllers. This is shown exactly from the angular-momentum balance about the base axis,
  for any link lengths, centre of mass or friction (audit 4.1).
* **The torque figures are numerical artefacts.** Figs 12 and 19 are not outputs of the control
  law and differ from each other by 4–5 orders of magnitude, and Table 4's torque column is a
  *permutation* of Fig. 19 (audit 3.3–3.4). The torque metric therefore cannot be reproduced.
  Ours is the physically consistent value.
* The paper publishes no gains, no solver settings and no fractional-operator settings. The
  identified gains are not unique; they show the curves are attainable, not that they are the
  authors' gains.

## FOPSO-GWO: beating the paper's FBPA-FOPID

The paper improves its FOPID with FBPA, a fractional-order PSO fused with beetle antennae search.
FOPSO-GWO is this work's counterpart: a **fractional-order PSO hybridised with the grey wolf
optimiser**. It re-tunes all 30 FOPID parameters (Kp, Ki, Kd, λ, μ for each of the six joints),
starting from the identified FOPID, with the paper's own FBPA budget of 30 particles × 100
iterations.

### Result

`tune_fopid_hybrid(struct('Target', 'FBPA', 'RandomSeed', 4))`, then `main`, on the paper's
plant (Table 2) and with the paper's metrics:

| Metric | FOPID (identified, the seed) | Paper FBPA-FOPID (Tables 3–4) | **FOPSO-GWO** | vs FBPA |
|---|---:|---:|---:|---:|
| Step overshoot (%) | 35.9 | 22.1 | **10.3** | −53 % |
| Step adjustment time (s) | 2.10 | 1.43 | **1.162** | −19 % |
| Step peak time (s) | 1.32 | 1.09 | **1.077** | −1.2 % |
| Sine MSE (rad²) | 1.05e-2 | 3.7e-3 | **2.47e-4** | −93 % |
| Sine Σ\|τ\| (Nm) | 9961 | 2.32e4 | **9427** | −59 % |

The two ITAE terms of the cost fall by 76 % (step) and 88 % (sine) against the FOPID. The price
is a 24 % higher peak torque at the step instant, a metric the paper does not report. The full
comparison, per joint and against the FBPA values recomputed from the paper's figures, is in
[`results/summary.md`](results/summary.md), and the curves are in the magenta traces of Figs 6–19.

**FOPSO-GWO beats the paper's FBPA-FOPID on all five of the paper's metrics.** This is not a
lucky draw. Eight runs with different random seeds and settings all beat FBPA on all five
(overshoot 6.8–14.7 %, adjustment time 1.11–1.22 s, peak time 1.075–1.087 s, MSE 2.2–3.6e-4,
Σ|τ| 9.3–9.7e3).
* **Peak time** is the tightest metric. Every run lands at 1.075–1.087 s against FBPA's 1.09 s,
  and weighting it 3× does not lower it further, so it is close to what this arm and controller
  structure allow.
* **Torque.** The paper's torque values are dominated by numerical artefacts (audit report
  3.3–3.4), so that comparison holds but means little.

### The algorithm (`hybrid_fopso_gwo.m`)

It keeps the velocity equation of FBPA (paper Eq. 26) and replaces the beetle term with a
grey-wolf term:

```
v(k+1) = (w-1+a) v(k) + a(1-a)/2 v(k-1) + a(1-a)(2-a)/6 v(k-2) + a(1-a)(2-a)(3-a)/24 v(k-3)   fractional memory, Eq. 25
         + c1 r1 (pbest - x) + c2 r2 (gbest - x)                                               PSO
         + c3 r3 (x_gwo - x)                                                                    GWO
x_gwo  = mean over L in {alpha, beta, delta} of  L - A .* |C .* L - x|,   A = 2 a_g r - a_g,  C = 2 r'
```

* Alpha, beta and delta are the three best personal bests.
* The inertia w falls linearly from 0.9 to 0.4, and the fractional order a follows Eq. 27
  (0.9 → 0.4).
* The GWO coefficient falls as a_g = 2(1 − k/K)², so the GWO term first explores around the three
  leaders and then refines around them.

Two settings differ from the paper's FBPA, both chosen on its own four 30-D test functions when
the optimiser was developed: c1 = c2 = c3 = 1 instead of 2, and the quadratic a_g decay instead
of GWO's usual linear one.

### The tuning (`tune_fopid_hybrid.m`, `fopid_fitness.m`)

* **Target.** With `Target = 'FBPA'`, the five paper metrics are scored against the paper's
  FBPA-FOPID values (Tables 3 and 4), and any metric not yet better than FBPA's is penalised
  extra. The two ITAE terms (paper Eq. 29) are scored against the FOPID. Adjustment and peak
  time are scored after the step instant: measured from t = 0, as the paper reports them, every
  peak time is ≥ 1 s and the ratio would hardly move. The reported metrics are unchanged. The
  default `Target = 'baseline'` scores everything against the FOPID instead.
* **Search space.** Kp, Ki and Kd are searched on a log10 scale, and λ and μ linearly. The bounds
  are log10 Kp ∈ [−2, 5], log10 Ki ∈ [−4, 5], log10 Kd ∈ [−1, 3] and λ, μ ∈ [0.05, 1.95]. They
  contain every gain of the identified FOPID, which seeds the swarm (one particle plus 30 %
  jittered copies), so the result is never worse than the FOPID under the chosen cost.
* **Robust.** Each finished candidate is simulated again with all gains × (1 + 1e-10), and a
  closed loop that then moves by more than 1e-6 rad is penalised. Without this check, one of the
  test runs returned gains whose response changed by 2e-2 rad for a 1e-14 gain change: a loop
  chattering in a round-off-sensitive regime, which would not reproduce on another machine. The
  check is on by default when the compiled simulation is built.
* **Unstable candidates** stop as soon as any joint error exceeds 5 rad.

### Speed: the compiled simulation (`simulate_mex.c`, `build_mex.m`)

`simulate_mex.c` is the closed-loop simulation in C: the same Newton–Euler dynamics, FOPID
update and RK4 loop as the .m code. `tools/check_mex.m` verifies that both agree, to round-off
(1e-14 rad, and 6e-9 rad on the least well-conditioned gain set). One 5 s simulation takes
0.03 s instead of ~40 s in Octave, and the paper's tuning budget (≈ 3000 evaluations) runs in
about 6 minutes instead of days. `simulate_closed_loop` uses it automatically once
`build_mex` has compiled it.

### Why the earlier FOPSO-GWO results (branch `claude/pensive-ramanujan-mmu564`) differ

That branch reported, for example, an overshoot of 13.4 % and an MSE of 3.2e-4. Those numbers
came from its **pre-audit plant**: real UR5 link lengths with each link's mass placed on its
joint axis, about 3× less inertia and far less coupling than the paper's Table-2 arm. Evaluated
on the paper's plant, its tuned gains overshoot by 66–112 % (peak time 1.71–1.73 s). Its torque
sums (3.1e4) also used 5001 samples instead of the paper's 501; on the paper's grid the same run
gives ~3.0e3. Even there, FBPA was not beaten on torque, and in the latest run peak time only
tied. The compiled simulation and the optimiser come from that branch, ported to the audited
plant. Its `benchmark_optimizer.m` (an optimiser test on mathematical test functions) is not
needed to generate FOPSO-GWO and was not carried over.

## Architecture

```
main.m                      runner: both controllers x both experiments, tables and figures
│
├── robot_params.m          arm parameters: Table 2 of the paper, UR DH, COM / gravity / friction options
├── robot_dynamics.m        M(q) qdd + C(q,qd) qd + G(q) + tau_f = tau  (Eq. 21), batched Newton-Euler
│
├── controller_gains.m      gains identified from the published curves (+ FOPSO-GWO from results/)
├── fopid_controller.m      builds the six joint controllers
├── fopid_update.m          one control step: u = Kp e + Ki D^-lambda e + Kd D^mu e  (Eq. 9)
├── fractional_operator.m   Oustaloup approximation of s^alpha, discretised
│
├── simulate_closed_loop.m  controller at 1 kHz + RK4 integration of the plant, logged at 0.01 s
│                           (runs simulate_mex when built)
├── performance_metrics.m   the metrics of the paper's Tables 3 and 4
├── paper_curves.m          the paper's published curves, in the same format as a simulation
└── plot_paper_figures.m    the figure set, in the paper's own layout, plus overlays

tune_fopid_hybrid.m         tunes the 30 FOPID parameters -> results/fopso_gwo_gains.mat
├── hybrid_fopso_gwo.m      FO-PSO / grey-wolf hybrid optimiser
└── fopid_fitness.m         cost of one gain set (ITAE + the paper's metrics)

simulate_mex.c              the closed-loop simulation in C, used automatically once built
build_mex.m                 compiles it (mkoctfile in Octave, mex in MATLAB)

data/paper_curves/          published curves of Figs 6-19, per panel and series (from the PDF)
data/paper_grid/            the same on the paper's 0.01 s grid
tools/                      curve extraction, gain identification (Python twin), consistency tests
docs/audit_report.md        the audit
```

## The two experiments

* **Step response.** All six joints are commanded 1 rad at t = 1 s, from rest. Measured: overshoot,
  adjustment (settling) time and peak time, averaged over the joints (paper Table 3).
* **Sine tracking.** All six joints follow sin(1.5 t) rad for 5 s. Measured: mean squared error and
  summed absolute torque, averaged over the joints (paper Table 4).

## Modelling choices

| Quantity | Choice | Reason |
|---|---|---|
| Masses, inertias, link lengths | Table 2 of the paper, as printed | published; the earlier argument for replacing the lengths assumed 5001 torque samples, but the paper logged 501 (audit 3.6) |
| Kinematics | UR5, standard DH | Sect. 2.3, Fig. 3 |
| Centre of mass | at each DH frame origin (Robotics Toolbox default) | not published; the alternatives fit the published curves no better (audit 4.2) |
| Gravity | g = 0 | the published responses are flat before the step to < 1e-4 rad; with gravity and the identified gains, joints 2, 3 and 5 would sag 0.14–0.16 rad |
| Friction | zero | Eq. 21 has the model but no coefficients; adding friction does not improve the fit (audit 4.1) |
| Fractional operator | Oustaloup, N = 5, 1e-3…1e3 rad/s | FOMCON defaults |
| Solver | RK4 at 1 ms, controller at 1 kHz, logging at 0.01 s | logging interval read from the paper's figures |
| Metrics | on the 0.01 s grid; settling band 5 % | these reproduce the paper's own figures→table relation (audit 3.2, 3.5) |
| Controller gains | identified from the published curves: per experiment (default) or one set for both experiments (`main('shared')`) | **the paper publishes no gains**, nor says whether the two experiments shared them |

## Requirements

Octave ≥ 6 (tested with 8.4) or MATLAB. No toolboxes. `build_mex` needs a C compiler: Octave on
Windows ships one; on Linux install the Octave development package (e.g. `apt install
octave-dev`); on macOS the Xcode command line tools.
The Python tools in `tools/` (only needed to re-extract curves or re-identify gains) use numpy,
scipy, numba, cma and pymupdf.
