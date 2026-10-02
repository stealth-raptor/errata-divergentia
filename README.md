# Trajectory tracking of a 6-DOF robotic arm: PID, FOPID, FBPA-FOPID, FOPSO-GWO and FOPSO-GWO-CC

Octave reproduction of the simulation results in

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025). DOI 10.1007/s40435-025-01620-x

The arm model, the controllers, both experiments and the paper's FBPA optimiser are built from
scratch. The gains of the paper's three controllers (PID, FOPID and FBPA-FOPID), which it does
not publish, are **identified from the paper's own published curves**. These were read exactly
from the vector graphics of the PDF.

On top of the reproduction, the FOPID is re-tuned with a **fractional-order PSO / grey-wolf
hybrid (FOPSO-GWO)**, this work's counterpart of FBPA, as a whole controller: within the torque
the paper's own controllers use, it tracks better than the paper's FBPA-FOPID on every metric
of its curves and of its reproduction, and on four of the five in its table (peak time 1.11
against 1.09 s). FBPA and plain PSO are re-implemented and compared with FOPSO-GWO as
optimisers under identical costs: FOPSO-GWO beats FBPA, and beats PSO under two of four costs,
including the whole-controller one.

**FOPSO-GWO-CC** (this branch, `fopso-gwo-hybrid`) adds a complementary method to FOPSO-GWO:
**cooperative coevolution by joint**. With the same 30 particles, 3030 evaluations, cost and
starting swarms as PSO, its controller beats PSO-FOPID on all five of the paper's metrics, by
38 % in overshoot, 28 % in adjustment time and 21 % in peak time (both after the step), 41 %
in sine MSE and 5 % in torque, and ties the paper's own FBPA-FOPID table on peak time while
beating it on the other four. As an optimiser it beats PSO on all 8 held-out seeds of the
whole-controller cost, with half PSO's mean cost.

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
octave --eval "tune_fopid_hybrid(struct('Fitness', 'whole', 'RandomSeed', 6))"   # FOPSO-GWO, ~6 min
octave --eval "tune_fopid_hybrid(struct('Optimizer', 'PSO', 'Fitness', 'whole', 'RandomSeed', 4))"   # PSO-FOPID, ~6 min
octave --eval "tune_fopid_hybrid(struct('Optimizer', 'FOPSO-GWO-CC', 'Fitness', 'whole', 'RandomSeed', 7))"   # FOPSO-GWO-CC, ~6 min
octave --eval main                  # all five controllers -> results/
```

Optional:

```
octave --eval "tune_fopid_hybrid(struct('Optimizer', 'FBPA', 'RandomSeed', 1))"   # FBPA as an optimiser, ~10 min
octave --eval "addpath tools; compare_optimizers"   # PSO vs FBPA vs FOPSO-GWO, 4 costs x 4 seeds (~7 h;
                                                     # or one process per seed: compare_optimizers(1), ...)
octave --eval "addpath tools; compare_optimizers(5:8, {'PSO', 'FOPSO-GWO'})"   # seeds 5-8, PSO and FOPSO-GWO
octave --eval "addpath tools; compare_optimizers(5:8, {'FBPA'}, {'whole'})"      # seeds 5-8 of FBPA, whole cost
octave --eval "addpath tools; develop_fopso_gwo(101:104, {}, 'fbpa')"          # FOPSO-GWO's development runs
octave --eval "main('shared')"      # one gain set per paper controller for both experiments -> results/shared_gains/
```

Without `build_mex` everything still runs on the plain .m simulation, with identical results,
but about 40 s per simulation instead of 0.03 s; tuning is then only practical with a small
budget. No packages are needed, and the code also runs in MATLAB.

`main` writes [`results/summary.md`](results/summary.md): the paper's Tables 3 and 4 against this
work, FOPSO-GWO against both FBPA-FOPIDs, the optimiser comparison, per-joint metrics, the match
to every published curve, the paper's tables against its own figures, and all gains. It also
writes the figure set and `simulation_results.mat`. `results/` in this branch holds the output of
the commands above, and `results/optimizer_runs/` the runs of `compare_optimizers`.

**Gain sets.** The paper publishes no gains and does not say whether its step and sine
experiments used the same ones. `controller_gains(controller, experiment)` holds three
identified sets per controller: `'step'`, `'sine'` and `'shared'`. The default `main` uses the
per-experiment sets, which match the figures most closely; `main('shared')` uses one set for
both experiments, as a single controller would. FOPSO-GWO uses one tuned set for both
experiments.

Checks:

```
octave --eval "addpath tools; verify_dynamics"   # dynamics vs Jacobian formula, energy conservation
octave --eval "addpath tools; check_twin"        # Octave model == Python identification twin
octave --eval "addpath tools; check_twin('shared')"
octave --eval "addpath tools; check_mex"         # compiled simulation == .m loop (~14 min)
```

### Figures

| File | Corresponds to | Content |
|---|---|---|
| `fig06…fig11_jointN_step.png` | Figs 6–11 | step response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig12_step_torque.png` | Fig 12 | joint driving torques, step response (3×2 grid) |
| `fig13…fig18_jointN_sine.png` | Figs 13–18 | sine response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig19_sine_torque.png` | Fig 19 | joint driving torques, sine response (3×2 grid) |
| `compare_step.png`, `compare_sine.png` | Figs 6–11, 13–18 | every published curve (thick black) overlaid on this work (thin colour), PID, FOPID and FBPA-FOPID, with the rms difference per joint |
| `convergence.png` | (not in the paper) | tuning the FOPID: best cost against cost evaluations, PSO, FBPA and FOPSO-GWO, one panel per cost |

Colours are the paper's own: reference ("dir") red, PID green, FOPID blue, FBPA-FOPID black.
In the sine error panels and in Fig 19 the paper draws FOPID in red, which is kept. PSO-FOPID
is cyan and FOPSO-GWO magenta. All signals are logged every 0.01 s, as in the paper.

## Result: the paper's Tables 3 and 4 against this work

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad²) | Sine Σ\|τ\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Paper | 54.6 | 2.44 | 1.39 | 2.27e-2 | 3.74e4 |
| PID | This work | 44.0 | 2.98 | 1.47 | 1.54e-2 | 1.05e4 |
| FOPID | Paper | 31.2 | 1.89 | 1.33 | 8.8e-3 | 2.57e4 |
| FOPID | This work | 33.2 | 1.95 | 1.42 | 5.5e-3 | 1.03e4 |
| FBPA-FOPID | Paper | 22.1 | 1.43 | 1.09 | 3.7e-3 | 2.32e4 |
| FBPA-FOPID | This work | 19.8 | 1.45 | 1.25 | 2.3e-3 | 1.01e4 |
| PSO-FOPID | This work (improved PSO) | 17.8 | 1.33 | 1.11 | 2.7e-3 | 1.01e4 |
| FOPSO-GWO | This work | 13.6 | 1.35 | 1.11 | 1.9e-3 | 9610 |
| **FOPSO-GWO-CC** | **This work (proposed)** | **11.0** | **1.24** | **1.09** | **1.6e-3** | **9609** |

* **PID, FOPID, FBPA-FOPID:** reproductions of the paper's three controllers, with the gains
  that make the simulation match each controller's published curves (below). The paper
  publishes no gains.
* **PSO-FOPID, FOPSO-GWO and FOPSO-GWO-CC:** the FOPID tuned by plain PSO (the paper's
  improved PSO, Eqs. 23–24), by FOPSO-GWO and by FOPSO-GWO-CC in exactly the same way: the same
  whole-controller cost, budget (30 particles, 3030 evaluations), seed controller and starting
  swarms, each the best of random seeds 1–8 under that cost.
* **FBPA-FOPID's peak time:** the paper's table says 1.09 s, but its own FBPA-FOPID curves
  peak at 1.26 s on average (summary.md, Sect. 7). The reproduction follows the curves (1.25 s).
* **Torque:** the paper's torque column cannot be reproduced. It is a permutation of its own
  Fig. 19, and its torque curves are numerical artefacts (audit 3.3–3.4). This work's torques
  are the physically consistent values.

## FOPSO-GWO-CC: FOPSO-GWO with cooperative coevolution

The FOPID has six joints with five gains each, and the whole-controller cost is close to a sum
over the joints: each joint's controller mostly shapes its own response. A swarm searching all
30 gains settles in one basin for every joint at once. Joint 3, for example, has a basin in
which it peaks within 10 ms of the step, which pulls the mean peak time down; PSO-FOPID has
joint 3 there, FOPSO-GWO's controllers do not, and no local refinement moves one joint to
another basin while the other 25 gains stay put.

**Cooperative coevolution** (Potter & De Jong 1994; for PSO, van den Bergh & Engelbrecht 2004)
searches each joint on its own. `hybrid_fopso_cc.m`:

1. FOPSO-GWO (its final settings) runs for 30 % of the iterations and finds a good controller,
   the *context*.
2. The same 30 particles become six sub-swarms of five, one per joint. Every iteration each
   sub-swarm moves by the FOPSO-GWO equations (fractional velocity memory, PSO and grey-wolf
   pulls) in its joint's five gains, over their whole range, and each particle is evaluated
   as the context with that joint's gains replaced. A better one replaces them at once.

Same population (30), budget (3030 evaluations, 30 per iteration), cost, seed controller and
initial swarms as PSO and FOPSO-GWO. On the 30-dimensional Rastrigin function, which is
separable and has many local minima, it reaches 57.8 against FOPSO-GWO's 119 and PSO's 154.

**The controller** (best of seeds 1–8 under the whole-controller cost, seed 7, as for the
others):

| Metric | Paper FBPA-FOPID: table | PSO-FOPID | FOPSO-GWO | **FOPSO-GWO-CC** | vs PSO-FOPID | vs PSO-FOPID, after the step |
|---|---:|---:|---:|---:|---:|---:|
| Overshoot (%) | 22.1 | 17.8 | 13.6 | **11.0** | **−38 %** | |
| Adjustment time (s) | 1.43 | 1.329 | 1.353 | **1.236** | −7 % | **−28 %** |
| Peak time (s) | 1.09 | 1.113 | 1.112 | **1.090** | −2 % | **−21 %** |
| Sine MSE (rad²) | 3.7e-3 | 2.75e-3 | 1.88e-3 | **1.63e-3** | **−41 %** | |
| Sine Σ\|τ\| (Nm) | 2.32e4 | 10135 | 9610 | **9609** | **−5 %** | |
| ITAE step / sine (Eq. 29) | n/a | 1.05 / 2.27 | 1.19 / 1.72 | **0.78 / 1.77** | −26 % / −22 % | |

* **All five metrics and both ITAEs** beat PSO-FOPID, which was tuned with the same cost and
  budget. Adjustment and peak times all start at the step (t = 1 s), so as the paper reports
  them, from t = 0, the same improvements read 7 % and 2 %.
* **Against the paper's own FBPA-FOPID table** it is better on four metrics and equal on peak
  time (1.090 s against 1.09 s); against the paper's FBPA-FOPID figures and this work's
  reproduction it is better on all five.
* **The mechanism shows in the joints:** FOPSO-GWO-CC's joint 3 peaks at 1.01 s, in the fast
  basin, where FOPSO-GWO's peaks at 1.24 s; the other joints peak within 0.04 s of FOPSO-GWO's
  (summary.md, Sect. 5).
* **Within the paper's torque on every joint:** all 24 caps are met, none is reached (the
  closest, joint 1 after the kick, at 87 %), and its derivative kick is 44 % below PSO-FOPID's
  (summary.md, Sect. 3).
* **Torque is the smallest margin** (−5 %, from −1 % to −8 % over the eight runs), and it is
  close to what the physics allows. Following the sine perfectly takes 7395 Nm (inverse
  dynamics): 699 Nm in the first 0.5 s and 6696 Nm after. PSO-FOPID spends 2368 + 7767 Nm,
  FOPSO-GWO-CC 2362 + 7247 Nm: it halves PSO's excess in steady tracking (551 against 1071 Nm
  above the floor), but the catch-up of the first 0.5 s, with a reference that starts at
  1.5 rad/s while the arm is at rest, costs every fast-tracking controller about the same, and
  less of it means a larger error.
* **Weighting the torque more does not change this.** With the sine torque three times as heavy
  in the cost (`tools/compare_weighted.m`, both optimisers, seeds 1–8, `results/torque_runs/`),
  FOPSO-GWO-CC still beats PSO on all 8 seeds (mean cost 0.954 against 2.391), and PSO's best
  run is the same PSO-FOPID controller; FOPSO-GWO-CC's best beats it by 38 % in overshoot, 54 %
  and 25 % in adjustment and peak time after the step, 33 % in MSE and still only 7 % in torque.
  The mean torque of the eight runs falls by 4 % for PSO and FOPSO-GWO-CC alike.

**As an optimiser**, on the whole-controller cost, held-out seeds 1–8:

| | mean | median | best | worst | seeds won against PSO | against FOPSO-GWO |
|---|---:|---:|---:|---:|---:|---:|
| PSO | 2.181 | 2.223 | 1.378 | 2.983 | | |
| FOPSO-GWO | 1.764 | 1.718 | 1.416 | 2.160 | 7 of 8 | |
| **FOPSO-GWO-CC** | **1.162** | **1.286** | **0.734** | **1.554** | **8 of 8** | **8 of 8** |

Its median run is better than PSO's best, and every one of its eight controllers beats
PSO-FOPID on overshoot (−25 to −42 %), adjustment time after the step (−3 to −38 %), MSE (−28 to
−69 %) and torque (−1 to −8 %); peak time after the step ranges from −22 % to +6 %. Winning all
8 seeds has a one-sided sign-test p of 0.004.

**How it was found.** The settings were chosen on development seeds 101–104, never on seeds
1–8 (`tools/develop_fopso_gwo.m`, `results/dev_runs/`, summary.md Sect. 4), with each rule
fixed before the candidates it chose between had run. Three complements were tried:

1. **CMA-ES refinement (memetic, `hybrid_fopso_cma.m`, `cmaes.m`).** Long CMA-ES runs from good
   controllers showed what the cost allows: from PSO-FOPID's controller, 6000 evaluations bring
   its cost from 1.378 to 0.710 (−38 / −42 / −21 / −43 / −4 % against PSO-FOPID), but only with
   steps of 0.5 % of the range; at 5 % every sample is worse than the start. FOPSO-GWO-CMA (the
   swarm for 60 %, then sep-CMA-ES hunts from the pack's three leaders and the best one
   refined) won all four development seeds against PSO but only 4 of 8 held-out seeds, and lost
   to plain FOPSO-GWO on 6 of 8: shortening the swarm cost more than the refinement won back.
   Its held-out runs stay in `results/optimizer_runs/` as the negative result.
2. **Joint-block crossover (a GA operator, `CrossFraction` in `hybrid_fopso_gwo.m`).** Of the 64
   joint-by-joint mixes of PSO-FOPID (cost 1.378) and FOPSO-GWO's controller (1.416), the best
   scores 1.065, which shows the joint structure; inside one run, though, the elite shares one
   basin and swapping its joints changes little (development mean 1.965, worse than PSO).
3. **Cooperative coevolution.** All three variants (full swarm for 50 %, 30 %, none) beat PSO
   on all four development seeds, with means 1.320, 1.243 and 1.278 against PSO's 1.767; the
   rule (lowest mean among those beating PSO on at least 3 of 4) chose 30 %.


## FOPSO-GWO: a whole controller

FOPSO-GWO without the coevolution stage, the previous result of this work; FOPSO-GWO-CC above
uses the same cost and improves on it in four of the five metrics (torque is equal).

FOPSO-GWO re-tunes the FOPID as a whole controller (`Fitness = 'whole'`): better tracking than
the paper's FBPA-FOPID, **within the torque the paper's own controllers use**, joint by joint,
and without one joint hiding behind the averages. PSO-FOPID is the same tuning done by plain
PSO. Each is the best of random seeds 1–8 under that cost (FOPSO-GWO seed 6, PSO seed 4).

| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work | PSO-FOPID | **FOPSO-GWO** | vs table | vs this work's FBPA-FOPID | vs PSO-FOPID |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Overshoot (%) | 22.1 | 21.2 | 19.8 | 17.8 | **13.6** | −38 % | −31 % | −23 % |
| Adjustment time (s) | 1.43 | 1.52 | 1.45 | 1.33 | **1.35** | −5 % | −7 % | +2 % |
| Peak time (s) | 1.09 | 1.258 | 1.250 | 1.113 | **1.112** | +2.0 % | −11 % | −0.1 % |
| Sine MSE (rad²) | 3.7e-3 | 3.7e-3 | 2.3e-3 | 2.7e-3 | **1.9e-3** | −49 % | −17 % | −32 % |
| Sine Σ\|τ\| (Nm) | 2.32e4 | 2.57e4 | 1.01e4 | 1.01e4 | **9610** | −59 % | −5 % | −5 % |
| ITAE step / sine (Eq. 29) | n/a | 1.19 / 3.56 | 1.16 / 2.44 | 1.05 / 2.27 | **1.19 / 1.72** | n/a | +3 % / −30 % | +13 % / −24 % |

**Torque.** Peaks are the largest joint; sums and total variations (Σ\|τ(k+1) − τ(k)\| at
1 kHz, which grows with chattering) are averaged over the joints. The first 50 ms after the step
are the derivative kick, which every controller with a D-term produces on an ideal step; it is
listed on its own.

| Controller | Kick peak (Nm) | Step peak after kick (Nm) | Sine peak (Nm) | Step total variation | Sine total variation |
|---|---:|---:|---:|---:|---:|
| PID | 2.1e5 | 1078 | 350 | 935 | 5055 |
| FOPID | 7.9e5 | 1524 | 1902 | 922 | 22840 |
| FBPA-FOPID | 8.2e4 | 1495 | 495 | 935 | 1445 |
| PSO-FOPID | 1.9e5 | 856 | 291 | 795 | 643 |
| **FOPSO-GWO** | 4.6e4 | 1517 | 413 | 1552 | 597 |

* **Within the paper's torque, on every joint.** FOPSO-GWO stays within all 24 caps: on every
  joint, its peak torque in the kick, in the rest of the step and in the sine run is no larger
  than the largest any of the paper's three controllers needs there, and no joint overshoots
  more than the paper's FBPA-FOPID does on its worst joint (33 %). It has the smallest kick of
  the five controllers (4.6e4 Nm, 1.8× below FBPA-FOPID's), a sine peak below FBPA-FOPID's and
  under a quarter of FOPID's, and the smoothest sine torque (2.4× less variation than
  FBPA-FOPID). The price is in the step after the kick: joint 1 runs at 99.5 % of its cap
  (1517 of 1524 Nm) and the step torque varies the most of the five (1552 against 795–935).
  PSO-FOPID also stays within all 24 caps, with lower peaks after the kick (856 Nm) and in the
  sine run (291 Nm), but a kick 4× larger than FOPSO-GWO's.
* **Tracking.** FOPSO-GWO beats this work's FBPA-FOPID and the paper's own FBPA-FOPID curves on
  all five metrics, and the paper's table on four of the five. Against PSO-FOPID, tuned with
  the same cost, it is better on four of the five: a quarter less overshoot (13.6 against
  17.8 %), a third lower sine MSE, 5 % less torque and the same peak time (1.112 against
  1.113 s). It settles 0.02 s later, and its step ITAE is 13 % higher (sine ITAE 24 % lower).
  Under the whole-controller cost itself, PSO-FOPID's run scores slightly better (1.378
  against 1.416): FOPSO-GWO is the better optimiser on that cost on average, not in its best
  run (next section).
* **Peak time** is the exception against the table, 1.11 s against 1.09 s. The table's 1.09 s
  contradicts the paper's own curves (1.26 s), and within the paper's torque it is out of
  reach: over the eight seeds the mean peak time is 1.112–1.142 s, the three heavy joints of
  this controller peak at 1.15–1.24 s, and joint 1 runs at its torque cap. More torque buys
  it. With FOPSO-GWO's first settings (seed 2; `results/ablation_runs/`), the same cost with
  the caps relaxed gives:

| Torque allowed | Overshoot (%) | Adjustment (s) | Peak (s) | MSE (rad²) | Step peak after kick / cap | Kick / cap | Step total variation |
|---|---:|---:|---:|---:|---:|---:|---:|
| The paper's controllers' | 18.6 | 1.29 | 1.100 | 1.3e-3 | 0.97 | 0.53 | 1175 |
| 2× that | 11.2 | 1.23 | 1.090 | 8.6e-4 | 1.44 | 1.95 | 1123 |
| Unlimited (the previous FOPSO-GWO, `Target 'FBPA-all'`) | 11.6 | 1.05 | 1.035 | 6.2e-5 | 8.33 | 4.82 | 3876 |

So the previous result's lead on every metric came from about 8× the torque of the paper's own
controllers after the step and 3.3× more torque variation. The whole-controller result gives
up some tracking for a controller that the paper's own actuator demands can drive.

The first row, the first settings' seed-2 run (whole cost 1.076), is the best whole
controller any FOPSO-GWO run has found. It is not the one shown above: the controllers above
are each the best of the eight runs of their optimiser's final settings, and picking from both
FOPSO-GWO settings, sixteen runs against PSO's eight, would not be a fair comparison.

### PSO, FBPA, FOPSO-GWO and FOPSO-GWO-CC as optimisers

The comparison above is between controllers, each tuned with its own cost. To compare the
optimisers themselves, `tools/compare_optimizers.m` runs plain PSO (`pso.m`), FBPA (`fbpa.m`),
FOPSO-GWO and FOPSO-GWO-CC under the same four costs. Everything but the algorithm is the same:
30 particles, the search space, the seed controller (the identified FOPID), the initial swarm
for each random seed (they share the initialisation code and its random draws), the cost
function and its checks. PSO uses its standard coefficients, FBPA the paper's, FOPSO-GWO and
FOPSO-GWO-CC their final settings (below and above); the swarms limit the velocity to 0.2 of
each range. PSO and the FOPSO-GWO variants evaluate the cost 3030 times per run, FBPA three
times as often (9030).

The costs are the paper's fitness (`paper`: step ITAE, Eq. 29), tracking against the paper's
FBPA-FOPID (`fbpa`), the same against the better of it and the best FBPA run (`fbpa_all`), and
the whole-controller cost of the final controller (`whole`: tracking, torque and per-joint
caps). Final best cost, lower is better, over random seeds 1–8 (FBPA: 1–4 on the first three
costs):

| Cost | Optimiser | best | median | mean | worst | mean after 3030 evaluations |
|---|---|---:|---:|---:|---:|---:|
| paper | PSO | **0.035** | **0.089** | **0.088** | 0.144 | **0.088** |
| | FBPA | 0.095 | 0.140 | 0.134 | 0.159 | 0.279 |
| | FOPSO-GWO | 0.087 | 0.122 | 0.116 | 0.135 | 0.116 |
| | FOPSO-GWO-CC | 0.040 | 0.103 | 0.093 | **0.130** | 0.093 |
| fbpa | PSO | 0.341 | 0.379 | 0.388 | 0.444 | 0.388 |
| | FBPA | 0.386 | 0.512 | 0.486 | 0.534 | 1.67 |
| | FOPSO-GWO | 0.331 | 0.348 | 0.352 | 0.384 | 0.352 |
| | FOPSO-GWO-CC | **0.172** | **0.266** | **0.257** | **0.293** | **0.257** |
| fbpa_all | PSO | 0.389 | 0.643 | 0.785 | 1.396 | 0.785 |
| | FBPA | 0.449 | 1.072 | 1.224 | 2.302 | 10.1 |
| | FOPSO-GWO | 0.518 | 1.371 | 1.193 | 1.472 | 1.193 |
| | FOPSO-GWO-CC | **0.326** | **0.477** | **0.615** | **1.040** | **0.615** |
| whole | PSO | 1.378 | 2.223 | 2.181 | 2.983 | 2.181 |
| | FBPA | 1.186 | 2.412 | 2.462 | 4.418 | 5.47 |
| | FOPSO-GWO | 1.416 | 1.718 | 1.764 | 2.160 | 1.764 |
| | FOPSO-GWO-CMA | 1.610 | 1.881 | 2.106 | 3.646 | 2.106 |
| | FOPSO-GWO-CC | **0.734** | **1.286** | **1.162** | **1.554** | **1.162** |

Head to head, the random seeds on which an optimiser ends lower (same cost and seed):

| Cost | FOPSO-GWO-CC against PSO | against FBPA | against FOPSO-GWO | FOPSO-GWO against PSO |
|---|---:|---:|---:|---:|
| paper | 4 of 8 | 3 of 4 | 5 of 8 | 1 of 8 |
| fbpa | **8 of 8** | 4 of 4 | 8 of 8 | 6 of 8 |
| fbpa_all | **6 of 8** | 4 of 4 | 8 of 8 | 2 of 8 |
| whole | **8 of 8** | 8 of 8 | 8 of 8 | 7 of 8 |

* **FOPSO-GWO-CC beats PSO under three of the four costs**: `whole` (8 of 8 seeds, mean
  −47 %), `fbpa` (8 of 8, −34 %) and `fbpa_all` (6 of 8, −22 %), in all with the lowest best,
  median, mean and worst run. Under the paper's own fitness it is level with PSO (4 of 8 seeds,
  mean 5 % higher, best run 0.040 against PSO's 0.035); FOPSO-GWO lost there on 7 of 8. In all,
  it ends lower than PSO on 26 of the 32 seed and cost pairs, than FOPSO-GWO on 29 and than FBPA
  on 19 of 20. Winning 8 of 8 seeds has a one-sided sign-test p of 0.004.
* **The two costs FOPSO-GWO lost are the ones with joint basins, and searching each joint on
  its own reaches them.** Under `paper` the best runs have joint 3's Kp at the top of its range,
  its fast basin: FOPSO-GWO-CC's three best runs (0.040, 0.058, 0.062) do, as do PSO's two best,
  and none of FOPSO-GWO's. Under `fbpa_all` a run scores well only with a peak time below
  1.048 s: 5 of FOPSO-GWO-CC's 8 runs get there (PSO 6, FOPSO-GWO 2), and there its costs are
  lower than PSO's.
* **FOPSO-GWO-CMA**, the CMA-ES complement (whole cost only), is worse than plain FOPSO-GWO:
  the refinement does not pay for the shorter swarm (above).
* **Against FBPA**, every FOPSO-GWO variant has the lower mean under every cost, with a third
  of FBPA's evaluations.

#### Why plain PSO beat the first FOPSO-GWO, and what the final settings change

FOPSO-GWO's first settings (c1 = c2 = c3 = 1, the paper's fractional order 0.9 → 0.4), chosen
on the paper's test functions, lost to plain PSO on all three tracking costs (11 of 12 seed
pairs) and won only under `whole`. Both FBPA and FOPSO-GWO add to PSO, but adding to an
optimiser does not make it better on every problem. Their improvements were shown on benchmark
functions, from random starts, over long runs (the paper's Fig. 4: 500 iterations). Tuning this
FOPID is a different problem: the swarm starts next to a good controller (the identified
FOPID), the budget is 100 iterations, and most of the search space is unstable. What decides it
is the balance between staying near the best points and exploring, and the classic PSO settings
strike it well here. The ablation (`tools/ablate_optimizers.m`, cost `fbpa`, seeds 1–4,
summary.md Sect. 4) changes one setting at a time:

| Optimiser | mean cost | swarm, last 25 iterations |
|---|---:|---:|
| PSO (c = 2, inertia 0.9 → 0.4, \|v\| ≤ 0.2) | 0.390 | 397 (still exploring) |
| PSO with c = 1 | 0.631 | 48 (collapsed) |
| FOPSO-GWO, first settings (c = 1, fractional order 0.9 → 0.4) | 0.425 | 8 (collapsed) |
| FOPSO-GWO, first settings with c = 2 | 0.634 | 1141 (thrown about) |
| FO-PSO alone (the first settings without the wolves, c = 2) | 0.469 | 739 |
| FBPA (the paper's \|v\| ≤ 1) | 0.486 | 1312 (thrown about) |
| FBPA with \|v\| ≤ 0.2 | 0.352 | 722 |
| **FOPSO-GWO, final settings** (c = 1.5 / 1.5 / 1, order held at 0.9) | **0.346** | 102 |

The swarm column is the mean cost of the particles' current positions: unstable candidates
score 1e3–2e3, so a large value means the swarm is still spread over the search space, a small
one that it has collapsed onto one point.

1. **FBPA's beetle antennae do nothing here.** With the paper's step of 1e-4, shrinking by
   0.95 per iteration to 6e-7, the beetle term moves a particle by about a thousandth of a
   typical velocity. It costs two extra evaluations per particle and iteration and changes
   nothing, so FBPA is effectively a fractional-order PSO at three times the cost.
2. **FBPA's velocity limit throws its swarm about.** The paper's \|v\| ≤ 1 is in raw gain units;
   in this search space it is the whole range, so particles jump from bound to bound (1312).
   With PSO's \|v\| ≤ 0.2, the same FBPA does much better (0.352 mean, with 9030 evaluations
   against 3030).
3. **The fractional velocity memory drains momentum.** In the paper's Eq. 25 the weight on the
   last velocity is w − 1 + α, which falls from 0.79 to −0.20 over the run, and the total over
   the four remembered velocities from 0.86 to 0.03. PSO keeps an inertia of 0.9 to 0.4. On its
   own, with everything else as in PSO (FO-PSO alone), the memory makes the result worse:
   0.469 against 0.390.
4. **The first FOPSO-GWO either collapses or is thrown about.** With its c = 1 the swarm has
   collapsed onto one point by the last quarter (8); with c = 2 the grey-wolf term adds so much
   movement that it never settles (1141). PSO with c = 1 collapses the same way and loses
   (0.631).
5. **Four seeds are not enough to rank them precisely.** FBPA with \|v\| ≤ 0.2 and FO-PSO alone
   are the same search, differing only in their random numbers (point 1), yet their means differ
   by 0.12, with one seed 0.24 apart.

**The final settings** follow from points 3 and 4. The fractional order is held at 0.9, so the
memory keeps PSO's momentum (total 0.86 → 0.36, like PSO's inertia) while still remembering
four velocities. And PSO's total pull of 4 is split between the three attractors, c1 = c2 = 1.5
and c3 = 1, instead of adding the wolves on top. They were chosen on development seeds 101–104,
which the comparison never uses (`tools/develop_fopso_gwo.m`, all 14 candidates in summary.md
Sect. 4). Some of them, mean cost against PSO's on the same seeds:

| Candidate | `paper` | `fbpa` |
|---|---:|---:|
| First settings | | +8.9 % |
| **Final: c = 1.5 / 1.5 / 1, order 0.9** | **−18.6 %** | **−9.4 %** |
| c = 2 / 1 / 1 (PSO's cognitive pull kept) | −5.0 % | −11.1 % |
| Starting as PSO, the wolves' share growing over the run | −20.6 % | +2.6 % |
| Half the swarm pulled as in PSO, outside the pack | −9.5 % | +0.8 % |

The first round of development used `fbpa` only. On the comparison's seeds 1–4 the chosen
settings then lost the paper's fitness to PSO on all four seeds, so a second round added
`paper` to the development, with four new candidates designed for it. Under a rule fixed before
its last two candidates ran (beat PSO's mean under both costs, by the largest margin on the
weaker of the two), the same settings stayed. Nothing was chosen on the comparison's seeds;
their runs are the ones reported above.

What the final settings changed, on the comparison's seeds (seeds won against PSO):

| Cost | first settings (seeds 1–4; `whole` 1–8) | final settings (seeds 1–8) |
|---|---:|---:|
| paper | 0.116 (0 of 4) | 0.116 (1 of 8) |
| fbpa | 0.425 (0 of 4) | **0.352 (6 of 8)** |
| fbpa_all | 1.157 (1 of 4) | 1.193 (2 of 8) |
| whole | 1.929 (6 of 8) | **1.764 (7 of 8)** |

**Why it is still not better under every cost.** The two costs it loses reward a swarm that
keeps exploring to the end, and the final settings, like the grey-wolf term itself, trade
some exploration for refinement around the best points (swarm column: 102 against PSO's 397).
* `fbpa_all` has a cliff: every paper metric must beat the better of the paper's FBPA-FOPID and
  the FBPA run, whose peak time is 1.053 s. Of the PSO and FOPSO-GWO runs, every one that
  reached a peak time of 1.048 s or less scores 0.76 or better, every one at 1.068 s or more
  1.28 or worse. PSO reached the faster region in 6 of 8 seeds, FOPSO-GWO in 2.
* Under `paper`, PSO's two best runs (0.035, 0.053) found a region with joint 3's Kp near the
  top of its range that no FOPSO-GWO run reached; without them PSO still has the lower mean
  (0.102 against 0.116). On the development seeds the order was the reverse (0.091 against
  0.112): under this cost the spread from seed to seed is as large as the difference.

No candidate on the development seeds beat PSO under both development costs by more than the
final settings did: those designed to explore more either lost the advantage under `fbpa`
(starting as PSO, half the swarm as PSO) or gained less under `paper` (c = 2 / 1 / 1). Going
on, with the comparison's seeds now seen, would be tuning to them.

What the comparison supports, then: FOPSO-GWO with its final settings has a lower mean cost
than the paper's FBPA under every cost (under `fbpa_all` only just, and on one seed of four),
at a third of its evaluations, and beats plain PSO where refinement matters:
under the whole-controller cost, the one its controller is tuned with (7 of 8 seeds), and under
`fbpa`. Where wide exploration matters, the paper's fitness and `fbpa_all`, plain PSO with its
standard settings is better. "FOPSO-GWO is a better PSO" in general would need 20–30 seeds per
optimiser, a significance test, and more than one problem.

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
* The inertia w falls linearly from 0.9 to 0.4. The fractional order a is held at 0.9; the
  paper's Eq. 27 lowers it to 0.4, which drains the velocity memory (below).
* c1 = c2 = 1.5 and c3 = 1: the three pulls add up to PSO's c1 + c2 = 4.
* The GWO coefficient falls as a_g = 2(1 − k/K)², so the GWO term first explores around the three
  leaders and then refines around them.
* The quadratic a_g decay comes from the optimiser's first development, on the paper's four 30-D
  test functions. The coefficients and the fractional order were chosen on this problem, on
  development seeds, against plain PSO (`tools/develop_fopso_gwo.m`, below). |v| ≤ 0.2 of each
  range.

FOPSO-GWO-CC (`hybrid_fopso_cc.m`) runs FOPSO-GWO for 30 % of the iterations and then the
same 30 particles as six sub-swarms of five, one per joint, each moving by the same equations
in its joint's five gains, evaluated in the context of the best controller (see "FOPSO-GWO-CC"
above). FOPSO-GWO-CMA (`hybrid_fopso_cma.m`) runs FOPSO-GWO for 60 % and then CMA-ES
(`cmaes.m`): short sep-CMA-ES hunts from the pack's three leaders and the rest of the budget
on the best of them.

PSO (`pso.m`) is the plain baseline: global-best PSO with a linearly decreasing inertia weight
(Shi and Eberhart), the paper's "improved PSO" (Eqs. 23–24):

```
v(k+1) = w v(k) + c1 r1 (pbest - x) + c2 r2 (gbest - x),   w = 0.9 -> 0.4,   c1 = c2 = 2
```

It has none of the additions of the other two (fractional memory, beetle antennae, grey-wolf
leaders) and shares everything else with FOPSO-GWO: the initialisation, the unit box and
|v| ≤ 0.2 of each range. The paper's Eq. 22, the original PSO without inertia, is
`wmin = wmax = 1`.

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

* **Optimizer.** `'FOPSO-GWO'` (default), `'FBPA'` or `'PSO'`. FBPA defaults to the paper's fitness,
  `Fitness = 'itae'`, and writes `results/fbpa_gains.mat` (an optimiser result, used by the
  `FBPA-all` target and the optimiser comparison; the FBPA-FOPID shown everywhere else is the
  reproduction identified from the paper's curves). FOPSO-GWO writes
  `results/fopso_gwo_gains.mat`.
* **Whole controller** (`Fitness = 'whole'`, the result above). Tracking alone rewards ever
  stiffer controllers, so this cost balances it against the torque it takes:
  * *tracking:* both ITAEs and four paper metrics, scored against the paper's FBPA-FOPID (its
    table; for the ITAEs its reproduction), with no extra credit beyond twice as good;
  * *effort:* Σ\|τ\| and the total variation of τ in both experiments, at 1 kHz and without
    the kick, scored against the reproduced FBPA-FOPID;
  * *caps,* heavily penalised: on every joint, the peak torque in the kick, in the rest of the
    step and in the sine run must stay within the largest any of the paper's three
    controllers needs (all identified gain sets), times `CapScale` (default 1); and no joint
    may overshoot more than the paper's FBPA-FOPID figures do on their worst joint (33.1 %);
  * every paper metric must still beat the paper's FBPA-FOPID table (regret 2).
  The result above is the best of random seeds 1–8 (seed 6).
* **Target.** With `Target = 'FBPA'`, the five paper metrics are scored against the paper's
  FBPA-FOPID values (Tables 3 and 4), and any metric not yet better than FBPA's is penalised
  extra. `Target = 'FBPA-all'` scores each metric against the better of the paper's value and
  that of the best FBPA optimiser run (`results/fbpa_gains.mat`). The two ITAE terms (Eq. 29) are scored against the FOPID. Adjustment
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
  (summary.md, Sect. 4).

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

Every published curve of Figs 6–11 and 13–18 was read from the PDF, and the PID, FOPID and
FBPA-FOPID gains were identified so that the simulation matches them (rms of q_this work −
q_paper, rad):

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean, separate gains (`main`) | mean, shared gains (`main('shared')`) | previous gains |
|---|---|---|---|---|---|---|---|---|---|---|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | **0.092** | 0.108 | 0.297 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | **0.080** | 0.099 | 0.188 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | **0.058** | 0.070 | 0.171 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | **0.051** | 0.062 | 0.099 |
| FBPA-FOPID | step | 0.026 | 0.027 | 0.038 | 0.053 | 0.052 | 0.044 | **0.040** | 0.043 | – |
| FBPA-FOPID | sine | 0.038 | 0.018 | 0.050 | 0.015 | 0.061 | 0.015 | **0.033** | 0.040 | – |

The per-joint columns are for the separate gains. For PID and FOPID, separate gains lower every
mean by 15–19 %, mostly on joints 3–5; joint 1's step does not improve (audit report 4.3).
FBPA-FOPID matches its curves best of the three, and one gain set fits both of its experiments
almost as well as two: the paper's FBPA-FOPID step and sine curves are more consistent with each
other than its PID and FOPID ones. All identified sets are numerically well-conditioned (a
relative gain change of 1e-10 moves the response by less than 1e-8 rad), so they reproduce
across platforms (`tools/check_twin.m`).

Why an exact match of every panel is not possible (details in the audit report):

* **The paper contradicts itself.** Its joint-1 step and sine curves cannot come from the same arm
  and controllers. This is shown exactly from the angular-momentum balance about the base axis,
  for any link lengths, centre of mass or friction (audit 4.1).
* **The torque figures are numerical artefacts.** Figs 12 and 19 are not outputs of the control
  law and differ from each other by 4–5 orders of magnitude, and Table 4's torque column is a
  *permutation* of Fig. 19 (audit 3.3–3.4). The torque metric therefore cannot be reproduced.
  Ours is the physically consistent value.
* **The paper's tables and figures disagree.** The same metrics recomputed from its published
  curves differ from its tables (summary.md, Sect. 7).
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
├── controller_gains.m      gains identified from the published curves (+ PSO-FOPID, FOPSO-GWO from results/)
├── fopid_controller.m      builds the six joint controllers
├── fopid_update.m          one control step: u = Kp e + Ki D^-lambda e + Kd D^mu e  (Eq. 9)
├── fractional_operator.m   Oustaloup approximation of s^alpha, discretised
│
├── simulate_closed_loop.m  controller at 1 kHz + RK4 integration of the plant, logged at 0.01 s
│                           (runs simulate_mex when built)
├── performance_metrics.m   the metrics of the paper's Tables 3 and 4
├── control_effort.m        the torque a controller needs: peaks, sums, total variation, the step's kick
├── paper_curves.m          the paper's published curves, in the same format as a simulation
├── plot_paper_figures.m    the figure set, in the paper's own layout, plus overlays
└── plot_convergence.m      FBPA vs FOPSO-GWO convergence (from results/optimizer_runs/)

tune_fopid_hybrid.m         tunes the 30 FOPID parameters -> results/<optimizer>_gains.mat
├── hybrid_fopso_cc.m       FOPSO-GWO + cooperative coevolution by joint (this work, proposed)
├── hybrid_fopso_gwo.m      FO-PSO / grey-wolf hybrid optimiser (this work)
├── hybrid_fopso_cma.m      FOPSO-GWO + CMA-ES refinement (tried; not better on held-out seeds)
├── cmaes.m                 CMA-ES / sep-CMA-ES on the unit box
├── fbpa.m                  fractional-order beetle antennae PSO (the paper's FBPA)
├── pso.m                   plain PSO, the baseline
└── fopid_fitness.m         cost of one gain set (ITAE + the paper's metrics)
tools/compare_optimizers.m  PSO, FBPA, FOPSO-GWO x 4 costs x 8 seeds -> results/optimizer_runs/
tools/ablate_optimizers.m   the same with one setting changed (why PSO won) -> results/ablation_runs/
tools/develop_fopso_gwo.m   the candidate FOPSO-GWO and hybrid settings on development seeds -> results/dev_runs/
tools/compare_weighted.m    PSO and FOPSO-GWO-CC with the torque weighted more -> results/torque_runs/

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
| Controller gains | PID, FOPID and FBPA-FOPID identified from the published curves: per experiment (default) or one set for both experiments (`main('shared')`) | **the paper publishes no gains**, nor says whether the two experiments shared them |

## Requirements

Octave ≥ 6 (tested with 8.4) or MATLAB. No toolboxes. `build_mex` needs a C compiler: Octave on
Windows ships one; on Linux install the Octave development package (e.g. `apt install
octave-dev`); on macOS the Xcode command line tools.
The Python tools in `tools/` (only needed to re-extract curves or re-identify gains) use numpy,
scipy, numba, cma and pymupdf.
