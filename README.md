# Improved FOPID tuning for a 6-DOF robotic arm: reproduction, audit and FOPSO-GWO-CC

**Branch `escape-orbit`: the MATLAB version of the work, for review.**

Base paper:

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm
> based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025).
> DOI 10.1007/s40435-025-01620-x

> **Status of this branch.** Every number in this document was produced by this same code
> running in GNU Octave 8.4 (branches `brand-new-day` and `fopso-gwo-hybrid`), where all
> results were generated and verified. This branch holds the code in MATLAB form. It has
> been checked statically:
> * all 28 `.m` files parse as MATLAB (MISS_HIT linter, no findings);
> * the C source compiles as strict C99 with no variable-length arrays, so every compiler
>   that MATLAB supports accepts it;
> * all 307 stored `.mat` files load outside Octave and hold only numeric, text and
>   logical data.
>
> **The code has not yet been run in MATLAB.** Sect. 11 gives the commands that confirm the
> port and what each should print. The stored controllers will reproduce exactly. New
> optimiser runs will differ seed by seed, because MATLAB and Octave draw different random
> numbers for the same seed.

## Contents

1. [Summary for the decision](#1-summary-for-the-decision)
2. [The base paper and the problem](#2-the-base-paper-and-the-problem)
3. [What was implemented](#3-what-was-implemented)
4. [Design decisions](#4-design-decisions)
5. [Comparison with the base paper](#5-comparison-with-the-base-paper)
6. [The proposed controller: FOPSO-GWO-CC against the baselines](#6-the-proposed-controller-fopso-gwo-cc-against-the-baselines)
7. [The optimisers compared under identical conditions](#7-the-optimisers-compared-under-identical-conditions)
8. [What was tried and did not work](#8-what-was-tried-and-did-not-work)
9. [Problems found in the base paper](#9-problems-found-in-the-base-paper)
10. [Limitations](#10-limitations)
11. [Running it in MATLAB](#11-running-it-in-matlab)
12. [Scope options for the next phase](#12-scope-options-for-the-next-phase)
13. [Where everything is](#13-where-everything-is)

---

## 1. Summary for the decision

**What was done**

1. **Reproduced the paper from scratch.** This covers the arm (its Table 2, UR5 kinematics,
   Eq. 21), the six FOPID controllers (Eq. 9), both experiments, the paper's metrics and its
   FBPA optimiser. The paper publishes no controller gains, so the gains of its PID, FOPID
   and FBPA-FOPID were **identified from the paper's own curves**, which were read exactly
   from the PDF's vector graphics. The reproduction keeps the paper's ranking
   (PID worse than FOPID worse than FBPA-FOPID) on all five metrics. It follows the
   published curves to 0.03–0.09 rad rms, averaged over the joints (Sect. 5).
2. **Audited the paper.** Some of its results cannot be reproduced by anyone, because the
   paper contradicts itself:
   * its joint-1 step and sine curves are physically incompatible;
   * its torque figures are numerical artefacts;
   * its torque table is a reordering of its own figure (Sect. 9, `docs/audit_report.md`).
3. **Proposed a new tuning method, FOPSO-GWO-CC.** It is a fractional-order PSO / grey-wolf
   hybrid (FOPSO-GWO) followed by cooperative coevolution with one sub-swarm per joint. It
   tunes the FOPID as a whole controller: the best tracking it can get while staying, on
   every joint, within the torque that the paper's own controllers use.

**Headline results.** The table shows the paper's five metrics, each averaged over the six
joints; lower is better.

| Controller | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad²) | Sine Σ\|τ\| (Nm) |
|---|---:|---:|---:|---:|---:|
| FBPA-FOPID, the paper's Tables 3–4 | 22.1 | 1.43 | 1.09 | 3.70e-3 | 2.32e4 ¹ |
| FBPA-FOPID, reproduced here | 19.8 | 1.45 | 1.25 | 2.25e-3 | 1.01e4 |
| PSO-FOPID: the paper's improved PSO, same cost, budget and starting swarm | 17.8 | 1.33 | 1.11 | 2.75e-3 | 1.01e4 |
| **FOPSO-GWO-CC (proposed)** | **11.0** | **1.24** | **1.09** | **1.62e-3** | **9609** |

¹ This value cannot be compared: the paper's torque values are not physically consistent
(Sect. 9).

* **Against the paper's own FBPA-FOPID table**, FOPSO-GWO-CC is better on 4 of the 5 metrics
  and equal on peak time (1.090 s against 1.09 s). Overshoot is 50 % lower, adjustment time
  14 % lower and sine MSE 56 % lower.
* **Against PSO-FOPID**, tuned with exactly the same cost, budget and starting swarm, it is
  better on all five metrics and on both ITAEs:

  | Metric | Change against PSO-FOPID |
  |---|---:|
  | Overshoot | −38 % |
  | Adjustment time, measured from the step | −28 % |
  | Peak time, measured from the step | −21 % |
  | Sine MSE | −41 % |
  | Torque | −5 % |
  | ITAE, step / sine | −26 % / −22 % |

* **As an optimiser**, it was compared on 8 held-out random seeds with everything except the
  algorithm held the same:
  * under the whole-controller cost it beats PSO on all 8 seeds, with a 47 % lower mean
    cost (one-sided sign test, p = 0.004);
  * it beats PSO under 3 of the 4 costs tested;
  * under the paper's own fitness (step ITAE) it is level with PSO, winning 4 of 8 seeds;
  * it beats the paper's FBPA on 19 of 20 seed and cost pairs, using a third of FBPA's cost
    evaluations.
* **The torque gain is small (−5 %) because both controllers are close to the physical
  minimum.** Tracking the sine perfectly takes 7395 Nm (computed by inverse dynamics).
  PSO-FOPID uses 1.013e4 Nm and FOPSO-GWO-CC 9609 Nm (Sect. 6.3).

**Main caveats**

* The code has not yet been run in MATLAB (Sect. 11).
* Each optimiser was run on 8 seeds and one plant. That is enough for the main claim but not
  for the small differences.
* The plant follows the paper's assumptions: no gravity, no friction, nominal parameters.
  Robustness has not been tested yet (Sect. 10).

The decisions this review needs are in Sect. 12.

---

## 2. The base paper and the problem

The paper controls a UR5-type 6-DOF arm with six independent FOPID controllers:

```
u = Kp e + Ki D^(-λ) e + Kd D^(μ) e                        (Eq. 9)
```

It tunes their 30 parameters with **FBPA**, a fractional-order PSO combined with
beetle-antennae search (Sect. 3, Eqs. 22–27). FBPA minimises the ITAE of the step response
(Eq. 29). The paper then compares PID, FOPID and FBPA-FOPID in two simulations:

* **Step response:** 1 rad on every joint at t = 1 s. Metrics: overshoot, adjustment
  (settling) time and peak time (Table 3).
* **Sine tracking:** sin(1.5 t) rad on every joint for 5 s. Metrics: mean squared error and
  the summed absolute torque Σ|τ| (Table 4).

Every metric is averaged over the six joints, and the curves are shown in Figs 6–19.

The paper does **not** publish the controller gains, the solver, the settings of the
fractional operator, the centre-of-mass positions, the settling band or the friction
coefficients. The reproduction therefore had to identify or choose each of these (Sect. 4).

---

## 3. What was implemented

```
main.m                      runs every controller through both experiments -> results/ (tables, figures)
│
├── robot_params.m          the arm: Table 2 masses, inertias and lengths, UR5 standard DH;
│                           options for centre of mass, gravity and friction
├── robot_dynamics.m        M(q) q̈ + C(q,q̇) q̇ + G(q) + τ_f = τ (Eq. 21), recursive Newton-Euler
├── controller_gains.m      PID / FOPID / FBPA-FOPID gains identified from the paper's curves;
│                           tuned gains read from results/
├── fopid_controller.m      builds the six joint FOPIDs (Eq. 9)
├── fractional_operator.m   Oustaloup approximation of s^α, discretised exactly (ZOH)
├── fopid_update.m          one control step
├── simulate_closed_loop.m  controller at 1 kHz + RK4 plant, logged every 0.01 s
│                           (calls simulate_mex automatically once it is built)
├── performance_metrics.m   the metrics of the paper's Tables 3 and 4, and ITAE (Eq. 29)
├── control_effort.m        torque at 1 kHz: derivative kick, peaks, Σ|τ|, total variation
├── paper_curves.m          the paper's published curves (data/paper_grid/) as a simulation result
├── plot_paper_figures.m    Figs 6–19 in the paper's layout, plus overlays on the published curves
└── plot_convergence.m      optimiser convergence

tune_fopid_hybrid.m         tunes all 30 FOPID parameters -> results/<optimizer>_gains.mat
├── fopid_fitness.m         the costs: paper (ITAE), fbpa, fbpa_all, whole (Sect. 4, D7)
├── hybrid_fopso_cc.m       FOPSO-GWO-CC: FOPSO-GWO + cooperative coevolution by joint (proposed)
├── hybrid_fopso_gwo.m      FOPSO-GWO: fractional-order PSO with grey-wolf leaders
├── hybrid_fopso_cma.m      FOPSO-GWO + CMA-ES (tried; a negative result, Sect. 8)
├── cmaes.m                 CMA-ES / sep-CMA-ES on the unit box
├── fbpa.m                  the paper's FBPA (its Sect. 3, with its Sect. 4 settings)
└── pso.m                   plain PSO: the paper's "improved PSO" (Eqs. 23-24), the baseline

simulate_mex.c              the closed-loop simulation in C: the same model, hundreds of times faster
build_mex.m                 compiles it with MATLAB's mex

tools/verify_dynamics.m     dynamics against an independent Jacobian M(q), and energy conservation
tools/check_mex.m           the C simulation against the .m simulation
tools/compare_optimizers.m  PSO / FBPA / FOPSO-GWO / -CMA / -CC, 4 costs, seeds 1-8 -> results/optimizer_runs/
tools/develop_fopso_gwo.m   every candidate setting, on development seeds 101-104 -> results/dev_runs/
tools/ablate_optimizers.m   one setting changed at a time (why plain PSO beat FBPA) -> results/ablation_runs/
tools/compare_weighted.m    PSO against FOPSO-GWO-CC with torque weighted 3x -> results/torque_runs/

data/paper_grid/            the paper's published curves (Figs 6-19), read from the PDF's vector
                            graphics, on its 0.01 s grid
results/                    output of the Octave runs: summary.md (every table), figures, gains,
                            every optimiser run
docs/audit_report.md        the audit of the paper and of the reproduction
```

No toolbox is needed. The Parallel Computing Toolbox is used only when `UseParallel` is
switched on, and it changes the run time, not the results.

**Figures** (in `results/`):

| File | Corresponds to | Content |
|---|---|---|
| `fig06…fig11_jointN_step.png` | Figs 6–11 | step response of each joint: (a) position, (b) error |
| `fig12_step_torque.png` | Fig 12 | joint torques, step response |
| `fig13…fig18_jointN_sine.png` | Figs 13–18 | sine tracking of each joint: (a) position, (b) error |
| `fig19_sine_torque.png` | Fig 19 | joint torques, sine tracking |
| `compare_step.png`, `compare_sine.png` | Figs 6–11, 13–18 | every published curve (thick black) overlaid on this work's (thin colour), with the rms difference per joint |
| `convergence.png` | (not in the paper) | best cost against cost evaluations, for each optimiser and cost |

---

## 4. Design decisions

Each decision gives what was chosen, why, and the evidence for it.

### D1. The plant: the paper's arm as printed

* **Masses, inertias and link lengths:** the paper's Table 2, as printed.
  * An earlier version used real UR5 link lengths instead. Its justification relied on
    5001 torque samples, but the paper logged 501, so the change was reverted
    (audit 3.6).
* **Kinematics:** UR5 in standard DH.
* **Centre of mass:** at each DH frame origin, the Robotics Toolbox default. The paper does
  not give it, and the alternatives fit the published curves no better (audit 4.2).
* **Gravity: g = 0.** The published responses are flat before the step to within
  1e-4 rad. With gravity and these gains, joints 2, 3 and 5 would sag 0.14–0.16 rad.
* **Friction: none.** Eq. 21 contains a friction term but gives no coefficients, and adding
  friction does not improve the fit (audit 4.1).

All of these are options of `robot_params` and can be switched for robustness studies.

### D2. Dynamics: recursive Newton–Euler

M, C and G are the same as those of the paper's Lagrange formulation (Eqs. 10–20).

* The seven recursions (the six columns of M and the bias vector) run as one batch.
* They are verified against an independent Jacobian formula for M(q), to 1e-15.
* Energy is conserved under zero torque to 1e-14 (`tools/verify_dynamics.m`).

### D3. The fractional operator

* **Approximation:** Oustaloup, N = 5, over [1e-3, 1e3] rad/s. These are the FOMCON
  defaults; the paper gives no settings.
* **Structure:** parallel form, discretised exactly (zero-order hold) at 1 kHz.
* **Integer parts:** an integrator and a backward difference.

### D4. Simulation: RK4 at 1 ms, logging every 0.01 s

* The controller runs at 1 kHz, and the plant is integrated with RK4 at 1 ms.
* Signals are logged every 0.01 s. This interval was read from the paper's figures
  (audit 3.1).
* Computing the metrics on the same 501-sample grid reproduces the paper's step from figures
  to tables: its MSE column is matched to 0.7 % (audit 3.2).

### D5. Metrics, as the paper defines them

* **Overshoot:** 100 · (max q − 1).
* **Adjustment time:** the last exit from a **5 %** band, measured from t = 0 as in the
  paper. The 5 % band best reproduces Table 3 from the paper's own curves (audit 3.5).
* **Peak time:** measured from t = 0.
* **Sine MSE and Σ|τ|:** over the 501 logged samples.
* **ITAE:** as in Eq. 29.

No toolbox is used: the original code's `stepinfo` (2 % band) was replaced.

### D6. The paper's controllers: gains identified from its published curves

* **Method:** least squares on the six joint trajectories of each experiment, with all
  joints in the fully coupled closed loop, solved by block-coordinate CMA-ES. This was done
  by the Python tools on branch `brand-new-day`; the resulting gains are written into
  `controller_gains.m`.
* **Why:** an earlier version fitted the gains to the five averaged numbers of Tables 3–4.
  That is 48 free gains (PID and FOPID together) constrained by 5 averaged numbers, so the
  curves were never constrained, and they missed by 0.30 / 0.19 rad rms (PID step / sine).
* **Two variants:**
  * one gain set per experiment (the default). The paper does not say whether its two
    experiments shared gains, and separate sets fit 15–19 % better;
  * one set for both experiments, `main('shared')`.
* **Caveat:** the identified gains are not unique. They show the curves are attainable, not
  that they are the authors' gains.

### D7. The whole-controller cost (`Fitness = 'whole'`)

Tracking alone rewards ever stiffer controllers. An earlier tuning without torque limits
beat every metric, but used about 8 times the torque of the paper's own controllers after
the step. The cost therefore scores what a real controller must balance:

* **Tracking:** both ITAEs and four paper metrics (overshoot, adjustment and peak time after
  the step, sine MSE). Each is a ratio to the paper's FBPA-FOPID: to its table, or, for the
  ITAEs, to its reproduction. Each ratio is floored at 0.5, so there is no extra credit for
  being more than twice as good on one metric.
* **Effort:** Σ|τ| and the total variation of τ in both experiments, at 1 kHz and excluding
  the 50 ms derivative kick, relative to the reproduced FBPA-FOPID.
* **Caps (heavily penalised), 24 per controller:**
  * on every joint, the peak torque in the kick, in the rest of the step and in the sine run
    may not exceed the largest that any of the paper's three controllers needs on that joint;
  * no joint may overshoot more than 33.1 %, the worst joint of the paper's own FBPA-FOPID
    curves.
* **Regret 2:** an extra penalty on any paper metric that is not better than the paper's
  FBPA-FOPID table.
* **Robustness:** every finished candidate is simulated again with all gains multiplied by
  (1 + 1e-10). If the response then moves by more than 1e-6 rad, the candidate is penalised.
  Without this check, one run returned a loop chattering in a round-off-sensitive regime,
  which would not reproduce on another machine.
* **Stability:** unstable candidates are stopped once an error exceeds 5 rad, and every
  candidate must complete both experiments.

Inside the cost, adjustment and peak time are measured from the step at t = 1 s. Measured
from t = 0, every peak time is at least 1 s and the ratio would hardly move. The reported
metrics follow the paper and are measured from t = 0.

### D8. The search space

* **All 30 parameters are tuned at once.** The paper says "dimension 5" but tunes 30
  parameters, and the joints are coupled, so they cannot be tuned separately.
* **Bounds:**

  | Parameter | Range |
  |---|---|
  | log10 Kp | [−2, 5] |
  | log10 Ki | [−4, 5] |
  | log10 Kd | [−1, 3] |
  | λ, μ | [0.05, 1.95] |

* **Starting swarm:** seeded with the identified FOPID, as one particle plus 30 % jittered
  copies. A result is therefore never worse than the FOPID. The box contains every gain of
  that FOPID.
* **The bounds are part of the result:** the optimisers push some gains onto them.

### D9. A fair comparison protocol

Every optimiser gets the same conditions, so that only the algorithm differs:

* 30 particles × 100 iterations, the paper's FBPA budget: 3030 cost evaluations in all.
  FBPA uses 9030, because its antennae need two extra evaluations per particle.
* The same search space, seed controller and cost function, with the same abort and
  robustness checks.
* For each random seed, the same initial swarm: the optimisers share the initialisation
  code and its random draws.
* Each algorithm uses its standard or published coefficients. The swarms limit |v| to 0.2
  of each range; FBPA uses the paper's |v| ≤ 1.
* Each reported controller is the best of seeds 1–8 under its cost, for every optimiser
  alike.

### D10. Development and test seeds kept apart

* Every setting was chosen on **development seeds 101–104**.
* The selection rule was fixed before the deciding candidates ran.
* **Seeds 1–8** were used only for the final test.
* Every candidate and its result is kept (`results/dev_runs/`, `results/summary.md` Sect. 4).

### D11. FOPSO-GWO (`hybrid_fopso_gwo.m`)

FOPSO-GWO keeps FBPA's velocity equation (paper Eq. 26) and replaces the beetle term with a
grey-wolf term:

```
v(k+1) = (w-1+a) v(k) + a(1-a)/2 v(k-1) + a(1-a)(2-a)/6 v(k-2) + a(1-a)(2-a)(3-a)/24 v(k-3)   fractional memory, Eq. 25
         + c1 r1 (pbest - x) + c2 r2 (gbest - x)                                               PSO
         + c3 r3 (x_gwo - x)                                                                    grey wolf
x_gwo  = mean over L in {alpha, beta, delta} of  L - A .* |C .* L - x|,   A = 2 a_g r - a_g,  C = 2 r'
```

Here alpha, beta and delta are the three best personal bests. The settings, chosen on the
development seeds:

* **c1 = c2 = 1.5 and c3 = 1.** The three pulls add up to PSO's total of 4, instead of adding
  the wolves on top of it.
* **The fractional order a is held at 0.9.** The paper's Eq. 27 lowers it to 0.4, which drains
  the velocity memory: the total weight on the remembered velocities falls from 0.86 to 0.03.
* **The other settings:**
  * inertia w falls linearly from 0.9 to 0.4;
  * a_g = 2 (1 − k/K)²;
  * |v| ≤ 0.2 of each range.

### D12. FOPSO-GWO-CC, the proposed optimiser (`hybrid_fopso_cc.m`)

**Why.** The cost is close to a sum over the joints, because each joint's controller mostly
shapes its own response. A swarm searching all 30 gains at once settles in one basin for
every joint together. Joint 3, for example, has a basin in which it peaks within 10 ms of
the step. A 30-D swarm reaches that basin only if the other 25 gains happen to line up.

**How.** Cooperative coevolution (Potter & De Jong 1994; for PSO, van den Bergh &
Engelbrecht 2004) searches each joint on its own:

1. FOPSO-GWO (D11) runs for **30 %** of the iterations. Its best controller becomes the
   *context*.
2. The same 30 particles become **six sub-swarms of five, one per joint**. In every
   iteration:
   * each sub-swarm moves by the FOPSO-GWO equations in its joint's five gains, over their
     whole range;
   * each particle is evaluated as the context with that joint's gains replaced;
   * any improvement is written into the context at once.

   Each sub-swarm starts from the context's gains plus random ones, so a joint can leave its
   basin.

The population, budget, cost and initial swarm are the same as for PSO. The 30 % share was
chosen on the development seeds: all three variants tried beat PSO on all 4 development
seeds.

| Variant | Full-swarm share | Mean cost, `whole` |
|---|---:|---:|
| C1 | 50 % | 1.320 |
| **C2 (chosen)** | **30 %** | **1.243** |
| C3 | 0 % | 1.278 |
| PSO | — | 1.767 |

### D13. The compiled simulation (`simulate_mex.c`)

* It is a line-by-line C port of the simulation loop, the FOPID update and the dynamics.
* It equals the `.m` code to round-off (`tools/check_mex.m`): 1e-14 rad typically, and
  6e-9 rad on the least well-conditioned gain set.
* In Octave it turned 40 s per simulation into 0.03 s, which is what makes 3030-evaluation
  runs, and dozens of them, practical: about 6 minutes per run.
* It is plain C99 with fixed-size arrays, so MinGW-w64, MSVC, gcc and clang all compile it.

### D14. The MATLAB port (this branch)

* **Toolboxes:** none required.
* **Results files:** written as MAT v7.
* **Reading the CSVs:** `readmatrix` from R2019a, `dlmread` before that.
* **MEX:** built with the `-R2017b` API flag from R2018a on.
* **Random numbers:** seeded with `rng`.
* **Parallel runs:** `parfor` only when `UseParallel` is on. The three `parfor` loops are
  written so that MATLAB accepts them: sliced input and output, and a broadcast cost handle.
* **What was removed:** the Octave-only cross-check against the Python model and the Python
  tools themselves. They remain on `brand-new-day` and `fopso-gwo-hybrid`.

---

## 5. Comparison with the base paper

### 5.1 The paper's Tables 3 and 4 against this work

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad²) | Sine Σ\|τ\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Paper | 54.6 | 2.44 | 1.39 | 2.27e-2 | 3.74e4 |
| PID | This work | 44.0 | 2.98 | 1.47 | 1.54e-2 | 1.05e4 |
| FOPID | Paper | 31.2 | 1.89 | 1.33 | 8.80e-3 | 2.57e4 |
| FOPID | This work | 33.2 | 1.95 | 1.42 | 5.51e-3 | 1.03e4 |
| FBPA-FOPID | Paper | 22.1 | 1.43 | 1.09 | 3.70e-3 | 2.32e4 |
| FBPA-FOPID | This work | 19.8 | 1.45 | 1.25 | 2.25e-3 | 1.01e4 |
| PSO-FOPID | This work (the paper's improved PSO) | 17.8 | 1.33 | 1.11 | 2.75e-3 | 1.01e4 |
| FOPSO-GWO | This work | 13.6 | 1.35 | 1.11 | 1.88e-3 | 9610 |
| **FOPSO-GWO-CC** | **This work (proposed)** | **11.0** | **1.24** | **1.09** | **1.62e-3** | **9609** |

* **The ranking is reproduced.** PID is worse than FOPID, and FOPID worse than FBPA-FOPID, on
  all five metrics.
* **Most differences from the paper's table are within the paper's own disagreements.** The
  table does not match the paper's figures (Sect. 5.2). For example, the FBPA-FOPID peak time
  is 1.09 s in the table, 1.26 s in the paper's own curves and 1.25 s in the reproduction,
  which follows the curves.
* **The largest gap is PID's adjustment time** (2.98 against 2.44 s). It comes mostly from
  joint 1 (3.84 s, against 2.12 s in the paper's curves). Joint 1 is where the paper's step
  and sine curves contradict each other (Sect. 9).
* **The torque column cannot be reproduced.** This work's values (about 1e4 Nm) are
  physically consistent. The paper's values are a reordering of its own figure (Sect. 9).

### 5.2 The paper's tables against its own figures

The same metrics, recomputed from the paper's published curves:

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad²) | Sine Σ\|τ\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Table | 54.6 | 2.44 | 1.39 | 2.27e-2 | 3.742e4 |
| PID | Figures | 59.1 | 2.32 | 1.44 | 2.26e-2 | 2.315e4 |
| FOPID | Table | 31.2 | 1.89 | 1.33 | 8.80e-3 | 2.569e4 |
| FOPID | Figures | 34.4 | 1.95 | 1.46 | 8.83e-3 | 3.741e4 |
| FBPA-FOPID | Table | 22.1 | 1.43 | 1.09 | 3.70e-3 | 2.315e4 |
| FBPA-FOPID | Figures | 21.2 | 1.52 | 1.26 | 3.68e-3 | 2.568e4 |

The MSE agrees to 1 %. The torque values match, but assigned to different controllers
(Sect. 9). The step times differ by up to 0.17 s, and the overshoots by up to 4.5 points.

### 5.3 Match to the published curves

The table gives the rms of q (this work) − q (paper's figure), in rad. The per-joint columns
use the default per-experiment gains. "Previous gains" means the gains fitted to the five
averaged table values.

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | Mean | Mean, one set for both experiments | Previous gains |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | **0.092** | 0.108 | 0.297 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | **0.080** | 0.099 | 0.188 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | **0.058** | 0.070 | 0.171 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | **0.051** | 0.062 | 0.099 |
| FBPA-FOPID | step | 0.026 | 0.027 | 0.038 | 0.053 | 0.052 | 0.044 | **0.040** | 0.043 | – |
| FBPA-FOPID | sine | 0.038 | 0.018 | 0.050 | 0.015 | 0.061 | 0.015 | **0.033** | 0.040 | – |

* The coupled-arm features in the paper's figures are reproduced:
  * joint 4's negative dip after the step;
  * joint 3's early jump;
  * joint 5's fast spike followed by a slow swing.
* `results/compare_step.png` and `results/compare_sine.png` overlay every published curve on
  the reproduction.

---

## 6. The proposed controller: FOPSO-GWO-CC against the baselines

### 6.1 The paper's metrics

PSO-FOPID, FOPSO-GWO and FOPSO-GWO-CC were each tuned with the same whole-controller cost,
budget and starting swarms. Each controller shown is the best of seeds 1–8: PSO seed 4,
FOPSO-GWO seed 6 and FOPSO-GWO-CC seed 7.

| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work | PSO-FOPID | FOPSO-GWO | **FOPSO-GWO-CC** | vs paper table | vs this work's FBPA-FOPID | vs PSO-FOPID | vs PSO-FOPID, times after the step |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Overshoot (%) | 22.1 | 21.2 | 19.8 | 17.8 | 13.6 | **11.0** | −50.3 % | −44.4 % | −38.2 % | |
| Adjustment time (s) | 1.430 | 1.524 | 1.453 | 1.329 | 1.353 | **1.236** | −13.6 % | −14.9 % | −7.0 % | −28.3 % |
| Peak time (s) | 1.090 | 1.258 | 1.250 | 1.113 | 1.112 | **1.090** | ±0.0 % | −12.8 % | −2.1 % | −20.6 % |
| Sine MSE (rad²) | 3.70e-3 | 3.68e-3 | 2.25e-3 | 2.75e-3 | 1.88e-3 | **1.62e-3** | −56.1 % | −27.9 % | −40.8 % | |
| Sine Σ\|τ\| (Nm) | 2.315e4 | 2.568e4 | 1.014e4 | 1.013e4 | 9610 | **9609** | (n/a) | −5.3 % | −5.2 % | |
| ITAE step (Eq. 29) | n/a | 1.189 | 1.158 | 1.047 | 1.188 | **0.779** | n/a | −32.8 % | −25.6 % | |
| ITAE sine | n/a | 3.56 | 2.44 | 2.274 | 1.719 | **1.769** | n/a | −27.5 % | −22.2 % | |

* The paper measures adjustment and peak times from t = 0, so every value is at least 1 s and
  the percentages look small. The last column measures them from the step at t = 1 s, as
  the cost does.
* FOPSO-GWO-CC beats all three FBPA-FOPID references: the paper's figures and this work's
  reproduction on all five metrics, and the paper's table on four, with peak time equal.
* All eight FOPSO-GWO-CC runs, not only the best, beat PSO-FOPID on four metrics:

  | Metric | Change against PSO-FOPID, over the 8 runs |
  |---|---:|
  | Overshoot | −25 to −42 % |
  | Adjustment time after the step | −3 to −38 % |
  | Sine MSE | −28 to −69 % |
  | Torque | −1 to −8 % |
  | Peak time after the step | −22 to +6 % (the one metric that is not always better) |

### 6.2 The mechanism is visible joint by joint

* FOPSO-GWO-CC's joint 3 peaks at 1.01 s, which is in the fast basin. FOPSO-GWO's joint 3
  peaks at 1.24 s.
* The other joints of the two controllers peak within 0.04 s of each other.

Searching each joint on its own moved joint 3 to a better basin without disturbing the
others. Per-joint tables for every metric are in `results/summary.md` Sect. 5.

### 6.3 Torque: within the paper's limits, close to the physical floor

All torques below are at the 1 kHz control rate. Peaks are taken over the joints (the
largest joint). Sums and total variations, Σ|τ(k+1) − τ(k)|, are averaged over the joints.
The kick is the first 50 ms after the step.

| Controller | Kick peak | Step peak after kick | Sine peak | Step Σ\|τ\| after kick | Sine Σ\|τ\| | Step total variation | Sine total variation |
|---|---:|---:|---:|---:|---:|---:|---:|
| PID | 2.11e5 | 1078 | 350 | 1.25e4 | 1.05e4 | 935 | 5055 |
| FOPID | 7.87e5 | 1524 | 1902 | 8499 | 1.03e4 | 922 | 2.28e4 |
| FBPA-FOPID | 8.17e4 | 1495 | 495 | 6343 | 1.01e4 | 935 | 1445 |
| PSO-FOPID | 1.86e5 | 856 | 291 | 6025 | 1.01e4 | 795 | 643 |
| FOPSO-GWO | 4.64e4 | 1517 | 413 | 8480 | 9610 | 1552 | 597 |
| **FOPSO-GWO-CC** | 1.05e5 | 1324 | 329 | 6715 | 9609 | 874 | 598 |

* **Every cap is met.** FOPSO-GWO-CC stays within all 24 per-joint caps without reaching any.
  The closest is joint 1 after the kick, at 87 %. Its kick is 44 % smaller than PSO-FOPID's.
* **The torque gain is small because both controllers are near the physical minimum.**
  Following the sine perfectly takes 7395 Nm, computed by inverse dynamics: 699 Nm in the
  first 0.5 s and 6696 Nm after.

  | Controller | First 0.5 s (Nm) | After 0.5 s (Nm) |
  |---|---:|---:|
  | Perfect tracking (inverse dynamics) | 699 | 6696 |
  | PSO-FOPID | 2368 | 7767 |
  | FOPSO-GWO-CC | 2362 | 7247 |

  * In steady tracking, FOPSO-GWO-CC halves PSO-FOPID's excess over the floor: 551 Nm
    against 1071 Nm.
  * The catch-up in the first 0.5 s costs every fast-tracking controller about the same. The
    reference starts at 1.5 rad/s while the arm is at rest, and spending less torque there
    means a larger error.
* **Weighting the torque more does not change this.** The sine torque was made three times
  as heavy in the cost (`tools/compare_weighted.m`, seeds 1–8, `results/torque_runs/`).
  * FOPSO-GWO-CC still beats PSO on all 8 seeds, with mean cost 0.954 against 2.391.
  * PSO's best run is the same PSO-FOPID controller as before.
  * FOPSO-GWO-CC's best run beats it as follows:

    | Metric | Change against PSO-FOPID |
    |---|---:|
    | Overshoot | −38 % |
    | Adjustment time after the step | −54 % |
    | Peak time after the step | −25 % |
    | Sine MSE | −33 % |
    | Torque | −7 % |

  * Neither optimiser's mean torque falls (PSO 9934 → 9966 Nm, FOPSO-GWO-CC 9509 → 9554 Nm).
    Within the caps and at this level of tracking, there is little torque left to save.

---

## 7. The optimisers compared under identical conditions

`tools/compare_optimizers.m` runs every optimiser under four costs, with the protocol of D9:

* **paper:** the paper's own fitness, the ITAE of the step (Eq. 29), relative to the FOPID;
* **fbpa:** both ITAEs and the five paper metrics, scored against the paper's FBPA-FOPID;
* **fbpa_all:** the same, scored against the better of the paper's FBPA-FOPID and the best
  FBPA re-run;
* **whole:** the whole-controller cost (D7).

The table gives the final best cost over held-out seeds 1–8; lower is better. FBPA ran on
seeds 1–4 for the first three costs.

| Cost | Optimiser | best | median | mean | worst |
|---|---|---:|---:|---:|---:|
| paper | PSO | **0.035** | **0.089** | **0.088** | 0.144 |
| | FBPA | 0.095 | 0.140 | 0.134 | 0.159 |
| | FOPSO-GWO | 0.087 | 0.122 | 0.116 | 0.135 |
| | FOPSO-GWO-CC | 0.040 | 0.103 | 0.093 | **0.130** |
| fbpa | PSO | 0.341 | 0.379 | 0.388 | 0.444 |
| | FBPA | 0.386 | 0.512 | 0.486 | 0.534 |
| | FOPSO-GWO | 0.331 | 0.348 | 0.352 | 0.384 |
| | FOPSO-GWO-CC | **0.172** | **0.266** | **0.257** | **0.293** |
| fbpa_all | PSO | 0.389 | 0.643 | 0.785 | 1.396 |
| | FBPA | 0.449 | 1.072 | 1.224 | 2.302 |
| | FOPSO-GWO | 0.518 | 1.371 | 1.193 | 1.472 |
| | FOPSO-GWO-CC | **0.326** | **0.477** | **0.615** | **1.040** |
| whole | PSO | 1.378 | 2.223 | 2.181 | 2.983 |
| | FBPA | 1.186 | 2.412 | 2.462 | 4.418 |
| | FOPSO-GWO | 1.416 | 1.718 | 1.764 | 2.160 |
| | FOPSO-GWO-CMA | 1.610 | 1.881 | 2.106 | 3.646 |
| | FOPSO-GWO-CC | **0.734** | **1.286** | **1.162** | **1.554** |

Head to head, the seeds on which the first optimiser ends lower (same cost and seed):

| Cost | FOPSO-GWO-CC vs PSO | FOPSO-GWO-CC vs FBPA | FOPSO-GWO-CC vs FOPSO-GWO | FOPSO-GWO vs PSO |
|---|---:|---:|---:|---:|
| paper | 4 of 8 | 3 of 4 | 5 of 8 | 1 of 8 |
| fbpa | **8 of 8** | 4 of 4 | 8 of 8 | 6 of 8 |
| fbpa_all | **6 of 8** | 4 of 4 | 8 of 8 | 2 of 8 |
| whole | **8 of 8** | 8 of 8 | 8 of 8 | 7 of 8 |

* **FOPSO-GWO-CC beats PSO under three of the four costs.** It has the lowest best, median,
  mean and worst run under each of them:

  | Cost | Seeds won against PSO | Mean cost against PSO |
  |---|---:|---:|
  | whole | 8 of 8 | −47 % |
  | fbpa | 8 of 8 | −34 % |
  | fbpa_all | 6 of 8 | −22 % |

  Its median run under `whole` is better than PSO's best.
* **Under the paper's own fitness it is level with PSO.** It wins 4 of 8 seeds, its mean is
  5 % higher, and its best run is 0.040 against PSO's 0.035. FOPSO-GWO without coevolution
  lost there on 7 of 8 seeds. The best runs under this cost all have joint 3's Kp at the top
  of its range, its fast basin. FOPSO-GWO-CC's three best runs reach it, as do PSO's two best,
  and none of FOPSO-GWO's.
* **Over all four costs together**, FOPSO-GWO-CC ends lower than:
  * PSO on 26 of the 32 seed and cost pairs;
  * FOPSO-GWO on 29 of 32;
  * FBPA on 19 of 20.
* **The paper's FBPA is the weakest optimiser here** (`tools/ablate_optimizers.m`, summary
  Sect. 4). This holds even though it uses three times the evaluations:
  * its beetle step (1e-4 of the range, shrinking to 6e-7) moves a particle by about a
    thousandth of a typical velocity. The two extra evaluations per particle change nothing;
  * its velocity limit of |v| ≤ 1 spans the whole search range, so particles jump from bound
    to bound. With PSO's |v| ≤ 0.2, FBPA's mean under `fbpa` falls from 0.486 to 0.352;
  * its Eq. 27 drains the fractional velocity memory (D11). With everything else as in PSO,
    the memory alone makes the result worse: 0.469 against 0.390 (seeds 1–4).

---

## 8. What was tried and did not work

These negative results are kept in the repository with their numbers.

* **FOPSO-GWO's first settings lost to plain PSO.** The settings were c1 = c2 = c3 = 1, with
  the fractional order falling from 0.9 to 0.4 as in the paper. They lost on all three
  tracking costs, 11 of 12 seed pairs.
  * With c = 1 the swarm collapses onto one point.
  * With c = 2 the grey-wolf term throws it about.

  The final settings (D11) came out of 13 candidate settings tried on the development
  seeds.
* **A CMA-ES refinement stage (memetic; `hybrid_fopso_cma.m`, `cmaes.m`)** did not
  generalise. It runs FOPSO-GWO for 60 %, then sep-CMA-ES hunts from the three leaders.
  * It won all 4 development seeds against PSO (mean 1.489 against 1.767).
  * On the held-out seeds it beat PSO on only 4 of 8 and lost to plain FOPSO-GWO on 6 of 8
    (mean 2.106): shortening the swarm cost more than the refinement won back.
* **Joint-block crossover, a GA operator (`CrossFraction`),** did not help.
  * Mixing PSO-FOPID and FOPSO-GWO joint by joint showed the joint structure: the best of
    the 64 mixes scores 1.065, against 1.378 and 1.416 for the two parents.
  * Inside one run, though, the elite shares one basin, and swapping its joints changes
    little. The development mean was 1.965, worse than PSO's.
* **How much better the cost allows.** A long CMA-ES run from PSO-FOPID's controller brings
  its cost from 1.378 to 0.710 in 6000 evaluations. It needs steps of 0.5 % of the range; at
  5 % every sample is worse than the start. FOPSO-GWO-CC's best run (0.734 in 3030
  evaluations) comes close to this.

---

## 9. Problems found in the base paper

The details and evidence are in [`docs/audit_report.md`](docs/audit_report.md).

1. **The joint-1 step and sine curves cannot both be correct** (audit 4.1). From the
   angular-momentum balance about the base axis, no plant and controller pair produces both.
   This holds exactly, for any link lengths, centre of mass or non-negative friction. No
   reproduction can match every panel.
2. **The torque figures are numerical artefacts** (audit 3.4). Fig. 12 has spikes of up to
   1e9 Nm. Figs 12 and 19 differ from each other by 4–5 orders of magnitude, and neither is
   an output of the control law.
3. **Table 4's torque column is a permutation of Fig. 19** (audit 3.3). The curves labelled
   PID / FOPID / FBPA-FOPID sum to 2.3153e4 / 3.7412e4 / 2.5675e4. These are the table's
   values for FBPA / PID / FOPID.
4. **The tables disagree with the figures** (Sect. 5.2). For example, the FBPA-FOPID peak
   time is 1.09 s in the table and 1.26 s in the curves.
5. **Eq. 23 prints the inertia weight as increasing**, (wmax − wmin)·k/MaxIter, while the
   text says it decreases linearly. The decreasing form is used here.
6. **The paper calls the problem "dimension 5" but tunes 30 parameters.**
7. **Not published:** the gains, the solver, the fractional-operator settings, the centre of
   mass, the settling band and the friction coefficients.

---

## 10. Limitations

* **The code has not been run in MATLAB yet.** See Sect. 11 for the checks.
* **Statistics.** Each optimiser ran on 8 seeds per cost. That is enough for the main claim
  (8 of 8 under `whole`, p = 0.004). It is not enough for the small differences: under the
  paper's fitness the seed-to-seed spread is as large as the gap between the optimisers. A
  publication-grade claim needs 20–30 seeds and a Wilcoxon signed-rank test.
* **One problem.** FOPSO-GWO-CC has been tested only on this arm. A one-off, informal run on
  the 30-D Rastrigin function favoured it (57.8, against FOPSO-GWO's 119 and PSO's 154). That
  script is not in the repository, and no benchmark suite has been run.
* **The plant follows the paper's assumptions:** no gravity, no friction and nominal
  parameters. Robustness to model error, noise or other trajectories has not been tested.
* **The cost is this work's own design.** The proposed controller is tuned with the
  whole-controller cost (D7), not with the paper's fitness. Under the paper's fitness,
  FOPSO-GWO-CC is only level with PSO (Sect. 7).
* **The identified gains of the paper's controllers are not unique.** They show the curves
  are attainable, not that they are the authors' gains.
* **Two conventions for times.** The paper measures adjustment and peak times from t = 0 and
  this work reports both conventions. Measured from the step, the improvements are 3–10
  times larger.

---

## 11. Running it in MATLAB

### Requirements

* **MATLAB R2018a or newer** is recommended. R2016b is the minimum: the code relies on
  implicit expansion. No toolboxes are needed.
* **A C compiler for MEX**, set up once with `mex -setup C`:

  | Platform | Compiler |
  |---|---|
  | Windows | the free MinGW-w64 add-on (Home → Add-Ons), or Microsoft Visual C++ |
  | Linux | gcc |
  | macOS | Xcode command line tools |

  Without the compiler everything still runs on the `.m` simulation, with the same results.
  Tuning is then impractically slow.

### Steps (from the repository root)

```matlab
mex -setup C                 % once per machine: pick the C compiler
build_mex                    % compiles simulate_mex.c -> simulate_mex.<mexext>
addpath tools

verify_dynamics              % dynamics checks
check_mex                    % compiled simulation == .m simulation (slow: runs the .m loop)
main                         % every controller from the stored gains -> results/ (seconds)
```

### What each step should show

| Step | Expected output |
|---|---|
| `verify_dynamics` | `robot_dynamics: all checks passed`. It asserts \|M − M_jacobian\| < 1e-12, M positive definite and energy drift < 1e-8; the Octave run gave about 1e-15 and 1e-14. |
| `check_mex` | `check_mex: simulate_mex and the .m loop agree`. It asserts max \|Δq\| < 1e-8 rad and a relative torque difference < 1e-4. |
| `main` | Rewrites `results/summary.md`, the figures and `simulation_results.mat`. |

**The decisive check comes after `main`:**

```
git diff results/summary.md
```

The simulation is deterministic, and every controller is loaded from the stored gains. If
the port is faithful, `summary.md` changes in exactly one line, the sentence that says
"in Octave", now "in MATLAB", and in no number. The PNG files are re-rendered, so they will
show as changed.

A difference in the last printed digit of a number would come from floating-point
differences between platforms; the controllers are numerically well-conditioned (D7, D13),
so this is unlikely. Anything larger would be a porting error.

### Re-running an optimiser

```matlab
% FOPSO-GWO-CC, whole-controller cost, random seed 7 (about 6 minutes with the MEX)
tune_fopid_hybrid(struct('Optimizer', 'FOPSO-GWO-CC', 'Fitness', 'whole', 'RandomSeed', 7, ...
                         'OutFile', fullfile('results', 'matlab_fopso_gwo_cc_seed7.mat')))
```

* **Use a separate `OutFile`.** By default the tuner writes `results/<optimizer>_gains.mat`,
  replacing the stored controller that `main` reports. The old file is kept as `_prev.mat`.
* **This will not reproduce the stored seed-7 controller.** MATLAB and Octave generate
  different random numbers for the same seed: after `rng(0)`, `rand(1,3)` gives
  0.8147 0.9058 0.1270 in MATLAB and 0.8444 0.7580 0.4206 in Octave. A MATLAB run is
  therefore a new, independent sample.
* **The claim to re-verify is statistical**: FOPSO-GWO-CC should beat PSO on most or all
  seeds under the whole-controller cost. To test it in MATLAB, write to a new folder; the
  stored runs are skipped if the file names match. With the MEX, this takes about 16 runs ×
  6 min:

```matlab
compare_weighted(1:8, {'PSO', 'FOPSO-GWO-CC'}, [1 1 1 1], 'matlab_runs')   % [1 1 1 1] = the 'whole' cost
for s = 1:8
    p = load(fullfile('results', 'matlab_runs', sprintf('pso_whole_seed%d.mat', s)), 'cost');
    c = load(fullfile('results', 'matlab_runs', sprintf('fopso_gwo_cc_whole_seed%d.mat', s)), 'cost');
    fprintf('seed %d   PSO %.3f   FOPSO-GWO-CC %.3f\n', s, p.cost, c.cost);
end
```

To use several cores, start one MATLAB per seed (`-batch` needs R2019a or newer):
`matlab -batch "addpath tools; compare_weighted(3, {'PSO','FOPSO-GWO-CC'}, [1 1 1 1], 'matlab_runs')"`.
Alternatively, pass `'UseParallel', true` to `tune_fopid_hybrid`, which needs the Parallel
Computing Toolbox.

### Other entry points

```matlab
main('shared')                                          % one gain set per paper controller -> results/shared_gains/
tune_fopid_hybrid(struct('Optimizer', 'FBPA', 'RandomSeed', 1, ...
                         'OutFile', fullfile('results', 'matlab_fbpa_seed1.mat')))   % the paper's FBPA, ~15 min
```

The scripts in `tools/` (`compare_optimizers`, `develop_fopso_gwo`, `ablate_optimizers`,
`compare_weighted`) skip any run whose result file already exists. Called with their stored
folders, they therefore do nothing. To repeat runs in MATLAB, use a new folder, as with
`compare_weighted(..., 'matlab_runs')` above, or move the stored files aside.

---

## 12. Scope options for the next phase

The work now has three separable contributions:

* (a) a verified reproduction and audit of the base paper;
* (b) a whole-controller tuning formulation;
* (c) a new optimiser, FOPSO-GWO-CC.

The next phase can strengthen any of them. Each option below states what it adds, what it
needs, and what it risks.

| Option | What it adds | What it needs | Risk |
|---|---|---|---|
| **A. Confirm and consolidate** | Turns the current results into publishable evidence | Run Sect. 11 in MATLAB; 20–30 seeds per optimiser and a Wilcoxon test (about 240 runs × 6 min: 24 CPU-hours, 6 h on 4 cores); sensitivity of CC's two settings (swarm share, sub-swarm size) | Low: the code exists |
| **B. Robust controller** | Answers "does the better controller survive reality?" | ±10–20 % mass and inertia errors, gravity on, friction, sensor noise, other trajectories (`robot_params` already has the switches) | Medium: the margin over PSO-FOPID may shrink under model error |
| **C. General optimiser** | Makes FOPSO-GWO-CC a contribution beyond this arm | A standard benchmark suite (e.g. CEC), other near-separable control problems, comparison with strong baselines at equal budget (L-SHADE, CMA-ES with restarts) | Medium to high: CC is designed for near-separable problems and may lose on non-separable ones |
| **D. Extend the control problem** | A broader controller paper | Multi-objective tuning (a Pareto front of tracking against torque instead of one weighted cost); task-space trajectories; higher-fidelity plant (Simscape Multibody, now possible in MATLAB); real UR5 parameters | High effort; new modelling decisions |
| **E. Report the paper's inconsistencies** | A short comment on the base paper | The audit as it stands (`docs/audit_report.md`) | A judgment for the supervisor: tone, venue, contact with the authors |

**Recommendation:** start with A, then B. Both are cheap with the existing code and directly
de-risk the central claim. Then choose between C (an optimiser paper) and D (a control
paper), which sets the angle of the eventual publication.

**Questions for the decision**

1. Is the whole-controller cost (tracking within the paper's own torque) the right primary
   objective, rather than the paper's step-ITAE fitness? Under the latter, FOPSO-GWO-CC is
   only level with PSO.
2. Should times be reported after the step (as the cost uses them), from t = 0 (as the paper
   does), or both?
3. Is the 5 % torque improvement, together with the physical-floor argument (Sect. 6.3),
   acceptable as it stands?
4. Should the next phase aim at an optimiser contribution (C), a controller contribution
   (B/D), or both?

---

## 13. Where everything is

| What | Where |
|---|---|
| This document, MATLAB code for review | branch `escape-orbit` (this branch) |
| The same code in Octave, with the Python tools (curve extraction, gain identification), the progress report and the earlier README | branch `fopso-gwo-hybrid` |
| Final code before FOPSO-GWO-CC; the Python tools | branch `brand-new-day` |
| The original audit | branch `audit` |
| The audit of the paper | [`docs/audit_report.md`](docs/audit_report.md) |
| Every table, regenerated by `main` | [`results/summary.md`](results/summary.md) |
| Proposed optimiser | [`hybrid_fopso_cc.m`](hybrid_fopso_cc.m) (uses [`hybrid_fopso_gwo.m`](hybrid_fopso_gwo.m)) |
| Proposed controller's gains | `results/fopso_gwo_cc_gains.mat`, `controller_gains('FOPSO_GWO_CC')` |
| Every optimiser run behind Sect. 7 | `results/optimizer_runs/`, `results/dev_runs/`, `results/ablation_runs/`, `results/torque_runs/` |

`results/` holds the Octave output that every number above comes from. In MATLAB, `main`
regenerates `summary.md`, the figures and `simulation_results.mat` in place. It only reads
the optimiser runs.
