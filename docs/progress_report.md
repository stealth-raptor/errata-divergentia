# Progress report: improved FOPID tuning for a 6-DOF robotic arm

*8 October 2026 · repository `stealth-raptor/errata-divergentia` · latest work on branch `fopso-gwo-hybrid`*

## In one paragraph

I reproduced the simulation study of Jiang, Zhang & Liu, *Trajectory tracking control of a
6-DOF robotic arm based on improved FOPID* (Int. J. Dyn. Control 13:137, 2025), audited it, and
developed a new optimiser to tune the FOPID controller: **FOPSO-GWO-CC**, a fractional-order
PSO / grey-wolf hybrid followed by cooperative coevolution, one sub-swarm per joint. In a fair
comparison (same population, budget, cost function and starting swarm), its controller beats
the controller tuned by plain PSO on **all five of the paper's performance metrics**: overshoot
−38 %, settling time −28 %, peak time −21 % (both measured from the step), tracking MSE −41 %
and torque −5 %. As an optimiser it beats plain PSO on all 8 held-out random seeds of the main
cost (p = 0.004). Torque is the smallest margin, and I show why: it is close to a physical floor.

## 1. Goal

1. Reproduce the paper's results (its Figs 6–19 and Tables 3–4) in Octave from scratch.
2. Tune the FOPID with an improved optimiser and beat the paper's FBPA-FOPID and a plain-PSO
   baseline, fairly, as a *whole controller*: good tracking within realistic torque, not
   tracking alone.

## 2. What has been done

### 2.1 Reproduction and audit of the paper (branch `audit`)

- Built the arm model (UR5 kinematics, the paper's Table 2 masses and inertias, Newton–Euler
  dynamics), the fractional-order controllers (Oustaloup approximation) and both experiments
  (1 rad step; sin(1.5 t) tracking).
- **The paper publishes no controller gains.** I read every published curve out of the PDF's
  vector graphics and *identified* the gains of its PID, FOPID and FBPA-FOPID so that the
  simulation matches those curves (least squares on all six joints, in the coupled closed loop).
  Mean curve error is now 0.03–0.09 rad, down from 0.10–0.30 rad with the earlier gains.
- **Inconsistencies found in the paper** (details in `docs/audit_report.md`):
  - its joint-1 step and sine curves cannot come from the same arm and controller (shown from
    the angular-momentum balance, whatever the link lengths or friction);
  - its torque figures are numerical artefacts (spikes up to 1e9 Nm), and Table 4's torque
    column is a permutation of its own Fig. 19, so the torque column cannot be reproduced;
  - its tables disagree with its own figures, e.g. FBPA-FOPID's peak time is 1.09 s in Table 3
    but 1.26 s in its curves;
  - smaller points: a typo in the inertia-weight formula (Eq. 23 makes the weight grow from 0
    instead of fall), and "dimension 5" where 30 parameters are tuned.

### 2.2 Tools that make the study practical (branch `brand-new-day`)

- **Compiled simulation** (`simulate_mex.c`): about 1300× faster than plain Octave (0.03 s
  instead of 40 s per run), so a full tuning run takes 6 minutes instead of days. It is checked
  against the Octave code, to 6e-9 rad at worst (`tools/check_mex.m`).
- **The paper's optimiser, FBPA**, re-implemented with its published settings, and a **plain
  PSO baseline**, all sharing one interface, search space and initial swarms.
- **Whole-controller cost**: tracking (the paper's metrics and ITAE) plus torque effort, with
  hard per-joint caps: no joint may need more torque than the paper's own controllers need,
  nor overshoot more than the paper's FBPA-FOPID does on its worst joint.

### 2.3 FOPSO-GWO, the first proposed optimiser

A fractional-order PSO with a grey-wolf (GWO) term. Its first settings, chosen on benchmark
functions, **lost to plain PSO** on the tracking costs. An ablation found why: the paper's
fractional-order schedule drains the swarm's momentum, so it collapses early. With corrected
settings (chosen on separate development seeds), FOPSO-GWO beat PSO on two of four costs,
including the whole-controller one (7 of 8 seeds). It still lost on two, and its controller
did not beat PSO-FOPID on settling time (1.35 against 1.33 s) or peak time (equal, 1.11 s).

### 2.4 FOPSO-GWO-CC, the current proposal (branch `fopso-gwo-hybrid`)

The key observation: the 30 tuned parameters are 6 joints × 5 gains, and each joint's
controller mostly shapes that joint's own response. A swarm searching all 30 at once settles
every joint into one basin together. For example, joint 3 has a basin where it peaks within
0.01 s of the step, and FOPSO-GWO never reached it.

**FOPSO-GWO-CC** runs FOPSO-GWO for the first 30 % of the budget. The same 30 particles then
split into six sub-swarms of five, one per joint. Each sub-swarm searches its own joint's five
gains with the same equations, while the other joints are held at the best controller found so
far (*cooperative coevolution*). Population (30) and budget (3030 cost evaluations) are exactly
PSO's.

Three complements to FOPSO-GWO were tried; two failed, and both are reported:

| Complement | Idea | Outcome |
|---|---|---|
| CMA-ES local refinement | refine the swarm's best point | won 4 of 4 development seeds, but only 4 of 8 held-out seeds; **rejected** |
| Joint-block crossover (GA) | swap joints between good controllers | helps across runs, not within one run; **rejected** |
| **Cooperative coevolution** | search each joint separately | won 4 of 4 development seeds and **8 of 8 held-out seeds**; **adopted** |

## 3. Results

### 3.1 The paper's Tables 3 and 4 against this work

| Controller | Source | Overshoot (%) | Settling time (s) | Peak time (s) | Sine MSE (rad²) | Sine Σ\|τ\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Paper | 54.6 | 2.44 | 1.39 | 2.27e-2 | 3.74e4 |
| PID | This work | 44.0 | 2.98 | 1.47 | 1.54e-2 | 1.05e4 |
| FOPID | Paper | 31.2 | 1.89 | 1.33 | 8.8e-3 | 2.57e4 |
| FOPID | This work | 33.2 | 1.95 | 1.42 | 5.5e-3 | 1.03e4 |
| FBPA-FOPID | Paper | 22.1 | 1.43 | 1.09 | 3.7e-3 | 2.32e4 |
| FBPA-FOPID | This work | 19.8 | 1.45 | 1.25 | 2.3e-3 | 1.01e4 |
| PSO-FOPID | This work (baseline) | 17.8 | 1.33 | 1.11 | 2.7e-3 | 1.01e4 |
| FOPSO-GWO | This work | 13.6 | 1.35 | 1.11 | 1.9e-3 | 9610 |
| **FOPSO-GWO-CC** | **This work (proposed)** | **11.0** | **1.24** | **1.09** | **1.6e-3** | **9609** |

The first six rows reproduce the paper's controllers. The paper's torque column is not
reproducible (Sect. 2.1), so torques are compared within this work only.

### 3.2 The proposed controller against the PSO baseline

Both are tuned with the same cost and budget, and each is the best of 8 random seeds:

| Metric | PSO-FOPID | FOPSO-GWO-CC | Change |
|---|---:|---:|---:|
| Overshoot | 17.8 % | 11.0 % | **−38 %** |
| Settling time, after the step | 0.329 s | 0.236 s | **−28 %** |
| Peak time, after the step | 0.113 s | 0.090 s | **−21 %** |
| Sine MSE | 2.75e-3 | 1.63e-3 | **−41 %** |
| Sine Σ\|τ\| | 10135 Nm | 9609 Nm | **−5 %** |
| ITAE step / sine | 1.05 / 2.27 | 0.78 / 1.77 | −26 % / −22 % |

- It also **beats the paper's own FBPA-FOPID table on four metrics and equals it on peak time**
  (1.09 s); no other controller here does.
- It stays within all 24 per-joint torque caps (closest: 87 % of a cap). Its torque spike at
  the step instant is 44 % below PSO-FOPID's.
- The paper reports settling and peak times from t = 0, but the step happens at t = 1 s.
  Counted that way the same gains read −7 % and −2 %; the cost function and the table above use
  the time after the step.

### 3.3 The optimisers compared (held-out seeds 1–8)

All four optimisers use the same population, budget, initial swarms and cost, except that FBPA
uses three times the evaluations, as in the paper. Mean best cost, lower is better:

| Cost | PSO | FOPSO-GWO | **FOPSO-GWO-CC** | FOPSO-GWO-CC beats PSO on |
|---|---:|---:|---:|---:|
| Whole controller (main) | 2.181 | 1.764 | **1.162** (−47 %) | **8 of 8 seeds** |
| Tracking vs paper's FBPA | 0.388 | 0.352 | **0.257** (−34 %) | **8 of 8** |
| Same, stricter targets | 0.785 | 1.193 | **0.615** (−22 %) | **6 of 8** |
| Paper's own ITAE fitness | **0.088** | 0.116 | 0.093 (+5 %) | 4 of 8 (level) |

In total FOPSO-GWO-CC wins 26 of 32 seed–cost pairs against PSO, 29 of 32 against FOPSO-GWO and
19 of 20 against FBPA.

### 3.4 Why torque improves only 5 %

- **Perfect tracking already needs 7395 Nm**, computed by inverse dynamics.
- Every controller spends about 2360 Nm more in the first 0.5 s, because the arm starts at rest
  while the reference is already moving at 1.5 rad/s. Spending less there means a larger
  tracking error.
- FOPSO-GWO-CC halves PSO's excess in steady tracking: 551 Nm above the floor, against PSO's
  1071 Nm.
- With torque weighted three times more in the cost, for both optimisers, FOPSO-GWO-CC still
  wins all 8 seeds. Neither optimiser's mean torque falls, so the limit is physical, not a
  weighting choice.

## 4. How the work was kept honest

- **Fair comparison:** the same population, evaluation budget, search space, initial swarm,
  cost function and selection rule (best of 8 seeds) for every optimiser.
- **Development and test seeds kept apart:** settings were chosen on seeds 101–104 only, with
  the selection rule fixed before the candidates ran. Seeds 1–8 were used only for the final
  test. Every candidate and its result are in `results/dev_runs/`.
- **Negative results are kept:** the FOPSO-GWO variant that lost, the CMA-ES hybrid that did not
  generalise, and the crossover operator are all documented with their numbers.
- **Checks:** the compiled simulation equals the Octave code (6e-9 rad), the dynamics pass
  energy and Jacobian checks, and the identification twin (Python) matches the Octave model.

## 5. Limitations

- **Eight seeds per optimiser.** That is enough for the main result (8 of 8, p = 0.004) but not
  for the smaller differences (e.g. the paper's ITAE cost); a publication-grade claim needs 20–30
  seeds and a signed-rank test.
- **One problem.** FOPSO-GWO-CC is tested on this arm and on two test functions only (it wins
  clearly on the separable Rastrigin function: 57.8 against PSO's 154).
- **Plant assumptions follow the paper:** no gravity and no friction (the published responses
  require this), and nominal parameters. Robustness to model error has not been tested yet.
- **The identified gains of the paper's controllers are not unique.** They show the published
  curves are attainable, not that they are the authors' gains.

## 6. Proposed next steps

1. **Merge** `fopso-gwo-hybrid` into the main branch (`brand-new-day`), so the final code holds
   FOPSO-GWO-CC.
2. **Statistics:** 20–30 seeds per optimiser and a Wilcoxon signed-rank test (about 1.5 h of
   compute per cost on the 4-core machine).
3. **Robustness of the controller:** ±10–20 % mass and inertia errors, gravity on, friction,
   sensor noise, and other reference trajectories.
4. **Generality of the optimiser:** a standard benchmark suite (e.g. CEC test functions) and an
   ablation of FOPSO-GWO-CC's two settings (swarm share, sub-swarm size).
5. **Write-up:** method, the audit of the paper, results and negative results.

## 7. Questions for my supervisor

1. Is the whole-controller cost (tracking within the paper's torque) the right primary
   objective, rather than the paper's step-ITAE fitness?
2. Should settling and peak times be reported after the step (as the cost uses) or from t = 0
   (as the paper does), or both?
3. Is the ~5 % torque improvement, with the physical-floor argument, acceptable as it stands?
4. Which next step comes first: more seeds, robustness tests, or the write-up?

## Where things are

| What | Where |
|---|---|
| Latest code and results | branch `fopso-gwo-hybrid` |
| Final code before this round | branch `brand-new-day` |
| Audit of the paper | `docs/audit_report.md` |
| Full results, regenerated by `main` | `results/summary.md` |
| Figures (paper layout, comparisons, convergence) | `results/*.png` |
| Proposed optimiser | `hybrid_fopso_cc.m` (uses `hybrid_fopso_gwo.m`) |
| Full description and every number | `README.md` |

To reproduce the proposed controller:

```
octave --eval build_mex
octave --eval "tune_fopid_hybrid(struct('Optimizer', 'FOPSO-GWO-CC', 'Fitness', 'whole', 'RandomSeed', 7))"
octave --eval main
```
