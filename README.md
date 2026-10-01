# Trajectory tracking of a 6-DOF robotic arm: PID, FOPID, FBPA-FOPID and FOPSO-GWO

Octave reproduction of the simulation results in

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025). DOI 10.1007/s40435-025-01620-x

The arm model, both controllers, both experiments and the paper's FBPA optimiser are built from
scratch. The PID and FOPID gains, which the paper does not publish, are **identified from the
paper's own published curves**. These were read exactly from the vector graphics of the PDF.
The FBPA-FOPID is produced the way the paper produced it: FBPA, re-implemented with the
paper's settings and fitness, tunes the FOPID on the same arm.

On top of the reproduction, the FOPID is re-tuned with a **fractional-order PSO / grey-wolf
hybrid (FOPSO-GWO)**, this work's counterpart of FBPA, and compared with FBPA both through the
controllers they produce and as optimisers under identical costs.

This branch (`brand-new-day`) is the final code. It combines
* the audited model and the PID / FOPID reproduction of branch `audit`: a line-by-line audit
  against the paper, why the published graphs could not be matched before, and several
  inconsistencies found in the paper itself, all in [`docs/audit_report.md`](docs/audit_report.md);
* the FOPSO-GWO tuner of branch `claude/pensive-ramanujan-mmu564`, ported onto that model (only
  the optimiser, the tuning driver and its fitness function);
* FBPA (`fbpa.m`), the comparison of the two optimisers, and the compiled simulation that makes
  the paper's tuning budget practical in Octave.

## Run it

```
octave --eval build_mex             # once: compiles the C simulation (about 1000x faster)
octave --eval "tune_fopid_hybrid(struct('Optimizer', 'FBPA', 'RandomSeed', 1))"     # FBPA-FOPID, ~10 min
octave --eval "tune_fopid_hybrid(struct('Target', 'FBPA-all', 'RandomSeed', 2))"    # FOPSO-GWO, ~6 min
octave --eval main                  # all four controllers -> results/
```

The FBPA run must come first: FOPSO-GWO's target includes its result. Optional:

```
octave --eval "addpath tools; compare_optimizers"   # FBPA vs FOPSO-GWO, 3 costs x 4 seeds (~3 h;
                                                     # or one process per seed: compare_optimizers(1), ...)
octave --eval "main('shared')"      # one PID / FOPID gain set for both experiments -> results/shared_gains/
```

Without `build_mex` everything still runs on the plain .m simulation, with identical results,
but about 40 s per simulation instead of 0.03 s; tuning is then only practical with a small
budget. No packages are needed, and the code also runs in MATLAB.

`main` writes [`results/summary.md`](results/summary.md): the paper's Tables 3 and 4 against this
work, FOPSO-GWO against both FBPA-FOPIDs, the optimiser comparison, per-joint metrics, the match
to every published curve, the paper's tables against its own figures, and all gains. It also
writes the figure set and `simulation_results.mat`. `results/` in this branch holds the output of
the commands above, and `results/optimizer_runs/` the 24 runs of `compare_optimizers`.

**Gain sets.** The paper publishes no gains and does not say whether its step and sine
experiments used the same ones. `controller_gains(controller, experiment)` holds three
identified sets per controller: `'step'`, `'sine'` and `'shared'`. The default `main` uses the
per-experiment sets, which match the figures most closely; `main('shared')` uses one set for
both experiments, as a single controller would. The tuned FBPA-FOPID and FOPSO-GWO use one set
for both experiments.

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
| `compare_step.png`, `compare_sine.png` | Figs 6–11, 13–18 | every published curve (thick black) overlaid on this work (thin colour), PID, FOPID and FBPA-FOPID, with the rms difference per joint |
| `convergence.png` | (not in the paper) | tuning the FOPID: best cost against cost evaluations, FBPA and FOPSO-GWO, one panel per cost |

Colours are the paper's own: reference ("dir") red, PID green, FOPID blue, FBPA-FOPID black.
In the sine error panels and in Fig 19 the paper draws FOPID in red, which is kept. FOPSO-GWO
is magenta. All signals are logged every 0.01 s, as in the paper.

## Result: the paper's Tables 3 and 4 against this work

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad²) | Sine Σ\|τ\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Paper | 54.6 | 2.44 | 1.39 | 2.27e-2 | 3.74e4 |
| PID | This work | 44.0 | 2.98 | 1.47 | 1.54e-2 | 1.05e4 |
| FOPID | Paper | 31.2 | 1.89 | 1.33 | 8.8e-3 | 2.57e4 |
| FOPID | This work | 33.2 | 1.95 | 1.42 | 5.5e-3 | 1.03e4 |
| FBPA-FOPID | Paper | 22.1 | 1.43 | 1.09 | 3.7e-3 | 2.32e4 |
| FBPA-FOPID | This work (FBPA re-run) | 26.9 | 1.12 | 1.05 | 2.7e-4 | 1.11e4 |
| **FOPSO-GWO** | **This work (proposed)** | **11.6** | **1.05** | **1.03** | **6.2e-5** | **9438** |

* **PID, FOPID:** reproductions, with gains identified from the paper's curves.
* **FBPA-FOPID:** the paper's method re-run on the same arm: FBPA with its Sect. 4 settings and
  its fitness (step ITAE, Eq. 29). The paper publishes no gains, so these are not the authors'
  gains, and the run lands near, not on, the published row: faster and with a far smaller MSE,
  but with more overshoot.
* **Torque:** the paper's torque column cannot be reproduced. It is a permutation of its own
  Fig. 19, and its torque curves are numerical artefacts (audit 3.3–3.4). This work's torques
  are the physically consistent values.

## FOPSO-GWO against FBPA-FOPID

| Metric | Paper FBPA-FOPID | FBPA-FOPID, re-run | **FOPSO-GWO** | vs paper | vs re-run |
|---|---:|---:|---:|---:|---:|
| Overshoot (%) | 22.1 | 26.9 | **11.6** | −47 % | −57 % |
| Adjustment time (s) | 1.43 | 1.116 | **1.050** | −27 % | −6 % |
| Peak time (s) | 1.09 | 1.053 | **1.035** | −5 % | −2 % |
| Sine MSE (rad²) | 3.7e-3 | 2.73e-4 | **6.21e-5** | −98 % | −77 % |
| Sine Σ\|τ\| (Nm) | 2.32e4 | 1.11e4 | **9438** | −59 % | −15 % |
| ITAE step / sine (Eq. 29) | n/a | 0.274 / 0.443 | **0.231 / 0.278** | n/a | −16 % / −37 % |
| Step peak torque (Nm) | n/a | 1.05e5 | 1.54e5 | n/a | +46 % |

**FOPSO-GWO beats both FBPA-FOPIDs, the paper's and the re-run, on all five of the paper's
metrics**, and on both ITAEs. The one figure that is worse is the peak torque at the step
instant, which the paper does not report. FOPSO-GWO was tuned with `Target = 'FBPA-all'`: each
paper metric is scored against the better of the two FBPA-FOPIDs, so the cost rewards beating
both.

### FBPA against FOPSO-GWO as optimisers

The controller comparison above mixes two things: the optimiser and the cost it was given (the
paper's FBPA optimised step ITAE only). `tools/compare_optimizers.m` separates them by running
both optimisers under the same three costs, with the same four random seeds, search space,
seed controller and initial swarms, and the paper's budget of 30 particles × 100 iterations.
Final best cost (lower is better):

| Cost | Optimiser | seed 1 | seed 2 | seed 3 | seed 4 | mean | evaluations | mean after 3030 evaluations |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| paper (step ITAE) | FBPA | 0.096 | 0.146 | 0.135 | 0.159 | 0.134 | 9030 | 0.279 |
| | FOPSO-GWO | 0.141 | 0.125 | **0.042** | 0.156 | **0.116** | 3030 | **0.116** |
| fbpa (vs paper's FBPA) | FBPA | **0.386** | 0.534 | 0.505 | 0.519 | 0.486 | 9030 | 1.67 |
| | FOPSO-GWO | 0.424 | 0.409 | 0.475 | 0.392 | **0.425** | 3030 | **0.425** |
| fbpa_all (vs both) | FBPA | 1.154 | **0.449** | 2.302 | 0.991 | 1.224 | 9030 | 10.1 |
| | FOPSO-GWO | 2.302 | 0.459 | 1.308 | 0.558 | **1.157** | 3030 | **1.157** |

* **Same iterations (the paper's budget):** FOPSO-GWO has the lower mean under all three costs
  and the lower final cost on 8 of the 12 seed–cost pairs. The best single run is FBPA's under
  two of the three costs, by 1–2 %. Under `fbpa_all`, FBPA's best run (seed 2) also beats both
  FBPA-FOPIDs on all five metrics, practically tied with FOPSO-GWO's (summary.md, Sect. 3).
* **Same evaluations:** FBPA's beetle antennae evaluate the cost twice more per particle and
  iteration, so it spends 9030 evaluations to FOPSO-GWO's 3030. After 3030 evaluations FBPA's
  mean best cost is 2.4×, 3.9× and 8.7× FOPSO-GWO's final one (`convergence.png`). (FBPA's
  schedules run over 100 iterations, so its first 33 iterations are not a tuned 33-iteration
  run.)

So the defensible claim is efficiency: **FOPSO-GWO finds controllers as good as FBPA's, or
better, with a third of the cost evaluations**, and a better mean at the same iterations. It is
not that FBPA cannot find comparable controllers when given the same cost: it can.

### The algorithms

FOPSO-GWO (`hybrid_fopso_gwo.m`) keeps FBPA's velocity equation (paper Eq. 26) and replaces the
beetle term with a grey-wolf term:

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
* Two settings differ from FBPA, both chosen on the paper's four 30-D test functions when the
  optimiser was developed: c1 = c2 = c3 = 1 instead of 2, and the quadratic a_g decay instead of
  GWO's usual linear one. |v| ≤ 0.2 of each range.

FBPA (`fbpa.m`) is the paper's Sect. 3, with its Sect. 4 settings:

```
v(k+1) = (same fractional memory) + c1 r1 (pbest - x) + c2 r2 (gbest - x) + c3 r3 v_l           Eq. 26
v_l    = -step * b * sign(J(x + b d/2) - J(x - b d/2)),   b a random unit vector,  d = step/5     BAS
step  <- 0.95 step   after every iteration, from 0.0001
```

* As in the paper: 30 beetles, 100 iterations, c1 = c2 = c3 = 2, w 0.9 → 0.4, a 0.9 → 0.4
  (Eq. 27), |v| ≤ 1, initial step 0.0001, attenuation 0.95, fitness step ITAE (Eq. 29).
* Not stated in the paper, so chosen: the antenna length (step/5, as in the original BAS code),
  the search space and the initial swarm (both the same as FOPSO-GWO's). Eq. 23 prints the
  inertia weight as (wmax − wmin) k / MaxIter, which grows from 0; the text calls it linearly
  decreasing, so both optimisers use wmax − (wmax − wmin) k / MaxIter.
* The paper calls the dimension 5 but tunes 30 parameters. All 30 are tuned at once, since the
  joints are coupled.

### The tuning (`tune_fopid_hybrid.m`, `fopid_fitness.m`)

* **Optimizer.** `'FOPSO-GWO'` (default) or `'FBPA'`. FBPA defaults to the paper's fitness,
  `Fitness = 'itae'`, and writes `results/fbpa_gains.mat`; FOPSO-GWO writes
  `results/fopso_gwo_gains.mat`.
* **Target.** With `Target = 'FBPA'`, the five paper metrics are scored against the paper's
  FBPA-FOPID values (Tables 3 and 4), and any metric not yet better than FBPA's is penalised
  extra. `Target = 'FBPA-all'` scores each metric against the better of the paper's value and
  the re-run FBPA-FOPID's. The two ITAE terms (Eq. 29) are scored against the FOPID. Adjustment
  and peak time are scored after the step instant: measured from t = 0, as the paper reports
  them, every peak time is ≥ 1 s and the ratio would hardly move. The reported metrics are
  unchanged. The default `Target = 'baseline'` scores everything against the FOPID instead.
* **Search space.** Kp, Ki and Kd are searched on a log10 scale, and λ and μ linearly. The bounds
  are log10 Kp ∈ [−2, 5], log10 Ki ∈ [−4, 5], log10 Kd ∈ [−1, 3] and λ, μ ∈ [0.05, 1.95]. They
  contain every gain of the identified FOPID, which seeds the swarm (one particle plus 30 %
  jittered copies), so the result is never worse than the FOPID under the chosen cost. Both
  optimisers push several gains to the upper bounds (Kd near 1e3, Ki or Kp near 1e5), so the
  bounds are part of the result: wider ones would give faster, stiffer controllers with larger
  torque spikes.
* **Robust.** Each finished candidate is simulated again with all gains × (1 + 1e-10), and a
  closed loop that then moves by more than 1e-6 rad is penalised. Without this check, one of the
  test runs returned gains whose response changed by 2e-2 rad for a 1e-14 gain change: a loop
  chattering in a round-off-sensitive regime, which would not reproduce on another machine. The
  check is on by default when the compiled simulation is built.
* **Candidates must complete both experiments**, also under the paper's step-only fitness, and
  stop as soon as any joint error exceeds 5 rad. The step-only fitness does not see the sine
  run otherwise: some of its results track well but with a sine torque of 1e5–3e5 Nm
  (summary.md, Sect. 3).

### Speed: the compiled simulation (`simulate_mex.c`, `build_mex.m`)

`simulate_mex.c` is the closed-loop simulation in C: the same Newton–Euler dynamics, FOPID
update and RK4 loop as the .m code. `tools/check_mex.m` verifies that both agree, to round-off
(1e-14 rad, and 6e-9 rad on the least well-conditioned gain set). One 5 s simulation takes
0.03 s instead of ~40 s in Octave, so the paper's tuning budget runs in about 6 minutes
(FOPSO-GWO) or 10 minutes (FBPA) instead of days. `simulate_closed_loop` uses it automatically
once `build_mex` has compiled it.

### Why the earlier FOPSO-GWO results (branch `claude/pensive-ramanujan-mmu564`) differ

That branch reported, for example, an overshoot of 13.4 % and an MSE of 3.2e-4. Those numbers
came from its **pre-audit plant**: real UR5 link lengths with each link's mass placed on its
joint axis, about 3× less inertia and far less coupling than the paper's Table-2 arm. Evaluated
on the paper's plant, its tuned gains overshoot by 66–112 % (peak time 1.71–1.73 s). Its torque
sums (3.1e4) also used 5001 samples instead of the paper's 501; on the paper's grid the same run
gives ~3.0e3. Even there, FBPA was not beaten on torque, and in the latest run peak time only
tied. The compiled simulation and the optimiser come from that branch, ported to the audited
plant. Its `benchmark_optimizer.m` (an optimiser test on mathematical test functions) is not
needed here and was not carried over.

## Result: match to the published curves

Every published curve of Figs 6–11 and 13–18 was read from the PDF, and the PID and FOPID gains
were identified so that the simulation matches them (rms of q_this work − q_paper, rad):

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean, separate gains (`main`) | mean, shared gains (`main('shared')`) | previous gains |
|---|---|---|---|---|---|---|---|---|---|---|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | **0.092** | 0.108 | 0.297 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | **0.080** | 0.099 | 0.188 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | **0.058** | 0.070 | 0.171 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | **0.051** | 0.062 | 0.099 |

The per-joint columns are for the separate gains. Separate gains lower every mean by 15–19 %,
mostly on joints 3–5. Joint 1's step does not improve (audit report 4.3). The re-run
FBPA-FOPID is not fitted to the curves; its distance from the paper's FBPA-FOPID curves is in
summary.md, Sect. 5.

Why an exact match of every panel is not possible (details in the audit report):

* **The paper contradicts itself.** Its joint-1 step and sine curves cannot come from the same arm
  and controllers. This is shown exactly from the angular-momentum balance about the base axis,
  for any link lengths, centre of mass or friction (audit 4.1).
* **The torque figures are numerical artefacts.** Figs 12 and 19 are not outputs of the control
  law and differ from each other by 4–5 orders of magnitude, and Table 4's torque column is a
  *permutation* of Fig. 19 (audit 3.3–3.4). The torque metric therefore cannot be reproduced.
  Ours is the physically consistent value.
* **The paper's tables and figures disagree.** The same metrics recomputed from its published
  curves differ from its tables (summary.md, Sect. 6).
* The paper publishes no gains, no solver settings and no fractional-operator settings. The
  identified gains are not unique; they show the curves are attainable, not that they are the
  authors' gains.

## Architecture

```
main.m                      runner: all controllers x both experiments, tables and figures
│
├── robot_params.m          arm parameters: Table 2 of the paper, UR DH, COM / gravity / friction options
├── robot_dynamics.m        M(q) qdd + C(q,qd) qd + G(q) + tau_f = tau  (Eq. 21), batched Newton-Euler
│
├── controller_gains.m      gains identified from the published curves (+ FBPA, FOPSO-GWO from results/)
├── fopid_controller.m      builds the six joint controllers
├── fopid_update.m          one control step: u = Kp e + Ki D^-lambda e + Kd D^mu e  (Eq. 9)
├── fractional_operator.m   Oustaloup approximation of s^alpha, discretised
│
├── simulate_closed_loop.m  controller at 1 kHz + RK4 integration of the plant, logged at 0.01 s
│                           (runs simulate_mex when built)
├── performance_metrics.m   the metrics of the paper's Tables 3 and 4
├── paper_curves.m          the paper's published curves, in the same format as a simulation
├── plot_paper_figures.m    the figure set, in the paper's own layout, plus overlays
└── plot_convergence.m      FBPA vs FOPSO-GWO convergence (from results/optimizer_runs/)

tune_fopid_hybrid.m         tunes the 30 FOPID parameters -> results/<optimizer>_gains.mat
├── hybrid_fopso_gwo.m      FO-PSO / grey-wolf hybrid optimiser (this work)
├── fbpa.m                  fractional-order beetle antennae PSO (the paper's FBPA)
└── fopid_fitness.m         cost of one gain set (ITAE + the paper's metrics)
tools/compare_optimizers.m  both optimisers x 3 costs x 4 seeds -> results/optimizer_runs/

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
