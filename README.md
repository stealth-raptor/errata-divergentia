# Trajectory tracking of a 6-DOF robotic arm: PID, FOPID, FBPA-FOPID and FOPSO-GWO

Octave reproduction of the simulation results in

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025). DOI 10.1007/s40435-025-01620-x

The arm model, the controllers, both experiments and the paper's FBPA optimiser are built from
scratch. The gains of the paper's three controllers (PID, FOPID and FBPA-FOPID), which it does
not publish, are **identified from the paper's own published curves**. These were read exactly
from the vector graphics of the PDF.

On top of the reproduction, the FOPID is re-tuned with a **fractional-order PSO / grey-wolf
hybrid (FOPSO-GWO)**, this work's counterpart of FBPA, as a whole controller: within the torque
the paper's own controllers use, it tracks better than the paper's FBPA-FOPID on every metric
of its curves and of its reproduction, and on four of the five in its table (peak time 1.10
against 1.09 s). FBPA itself is re-implemented and compared with FOPSO-GWO as an optimiser under
identical costs.

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
octave --eval "tune_fopid_hybrid(struct('Fitness', 'whole', 'RandomSeed', 2))"   # FOPSO-GWO, ~6 min
octave --eval main                  # all four controllers -> results/
```

Optional:

```
octave --eval "tune_fopid_hybrid(struct('Optimizer', 'FBPA', 'RandomSeed', 1))"   # FBPA as an optimiser, ~10 min
octave --eval "addpath tools; compare_optimizers"   # PSO vs FBPA vs FOPSO-GWO, 4 costs x 4 seeds (~7 h;
                                                     # or one process per seed: compare_optimizers(1), ...)
octave --eval "addpath tools; compare_optimizers(5:8, {}, {'whole'})"   # seeds 5-8 of the whole cost
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
| FBPA-FOPID | This work | 19.8 | 1.45 | 1.25 | 2.3e-3 | 1.01e4 |
| **FOPSO-GWO** | **This work (proposed)** | **18.6** | **1.29** | **1.10** | **1.3e-3** | **9951** |

* **PID, FOPID, FBPA-FOPID:** reproductions of the paper's three controllers, with the gains
  that make the simulation match each controller's published curves (below). The paper
  publishes no gains.
* **FBPA-FOPID's peak time:** the paper's table says 1.09 s, but its own FBPA-FOPID curves
  peak at 1.26 s on average (summary.md, Sect. 7). The reproduction follows the curves (1.25 s).
* **Torque:** the paper's torque column cannot be reproduced. It is a permutation of its own
  Fig. 19, and its torque curves are numerical artefacts (audit 3.3–3.4). This work's torques
  are the physically consistent values.

## FOPSO-GWO: a whole controller

FOPSO-GWO re-tunes the FOPID as a whole controller (`Fitness = 'whole'`): better tracking than
the paper's FBPA-FOPID, **within the torque the paper's own controllers use**, joint by joint,
and without one joint hiding behind the averages.

| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work | **FOPSO-GWO** | vs table | vs this work's FBPA-FOPID |
|---|---:|---:|---:|---:|---:|---:|
| Overshoot (%) | 22.1 | 21.2 | 19.8 | **18.6** | −16 % | −6 % |
| Adjustment time (s) | 1.43 | 1.52 | 1.45 | **1.29** | −10 % | −11 % |
| Peak time (s) | 1.09 | 1.26 | 1.25 | **1.10** | +0.9 % | −12 % |
| Sine MSE (rad²) | 3.7e-3 | 3.7e-3 | 2.3e-3 | **1.3e-3** | −65 % | −42 % |
| Sine Σ\|τ\| (Nm) | 2.32e4 | 2.57e4 | 1.01e4 | **9951** | −57 % | −2 % |
| ITAE step / sine (Eq. 29) | n/a | 1.19 / 3.56 | 1.16 / 2.44 | **0.84 / 1.69** | n/a | −28 % / −31 % |

**Torque.** Peaks are the largest joint; sums and total variations (Σ\|τ(k+1) − τ(k)\| at
1 kHz, which grows with chattering) are averaged over the joints. The first 50 ms after the step
are the derivative kick, which every controller with a D-term produces on an ideal step; it is
listed on its own.

| Controller | Kick peak (Nm) | Step peak after kick (Nm) | Sine peak (Nm) | Step total variation | Sine total variation |
|---|---:|---:|---:|---:|---:|
| PID | 2.1e5 | 1078 | 350 | 935 | 5055 |
| FOPID | 7.9e5 | 1524 | 1902 | 922 | 22840 |
| FBPA-FOPID | 8.2e4 | 1495 | 495 | 935 | 1445 |
| **FOPSO-GWO** | **8.1e4** | **1476** | **365** | 1175 | **598** |

* **Within the paper's torque, on every joint.** FOPSO-GWO stays within all 24 caps: on every
  joint, its peak torque in the kick, in the rest of the step and in the sine run is no larger
  than the largest any of the paper's three controllers needs there, and no joint overshoots
  more than the paper's FBPA-FOPID does on its worst joint (33 %). Its sine peak is a fifth
  of FOPID's and below FBPA-FOPID's, and its sine torque varies 2.4× less than FBPA-FOPID's.
* **Tracking.** It beats this work's FBPA-FOPID and the paper's own FBPA-FOPID curves on all
  five metrics and both ITAEs, and the paper's table on four of the five.
* **Peak time** is the exception, 1.10 s against the table's 1.09 s. The table's 1.09 s
  contradicts the paper's own curves (1.26 s), and within the paper's torque it is out of
  reach. In all eight seeds tried (best mean 1.100 s, the others 1.125–1.155 s), joint 2, the
  shoulder that swings the whole arm, peaks at 1.23–1.31 s, and one of the three heavy joints
  runs at 96–100 % of its torque cap. More torque buys it:

| Torque allowed | Overshoot (%) | Adjustment (s) | Peak (s) | MSE (rad²) | Step peak after kick / cap | Kick / cap | Step total variation |
|---|---:|---:|---:|---:|---:|---:|---:|
| **The paper's controllers' (this result)** | 18.6 | 1.29 | 1.100 | 1.3e-3 | 0.97 | 0.53 | 1175 |
| 2× that | 11.2 | 1.23 | 1.090 | 8.6e-4 | 1.44 | 1.95 | 1123 |
| Unlimited (the previous FOPSO-GWO, `Target 'FBPA-all'`) | 11.6 | 1.05 | 1.035 | 6.2e-5 | 8.33 | 4.82 | 3876 |

So the previous result's lead on every metric came from about 8× the torque of the paper's own
controllers after the step and 3.3× more torque variation. The whole-controller result gives
up some tracking for a controller that the paper's own actuator demands can drive.

### PSO, FBPA and FOPSO-GWO as optimisers

The comparison above is between controllers, each tuned with its own cost. To compare the
optimisers themselves, `tools/compare_optimizers.m` runs plain PSO (`pso.m`), FBPA (`fbpa.m`)
and FOPSO-GWO under the same four costs. Everything but the algorithm is the same: 30 particles
× 100 iterations, the search space, the seed controller (the identified FOPID), the initial
swarm for each random seed (the three share the initialisation code and its random draws), the
cost function and its checks. Each algorithm uses its standard or published coefficients, and
PSO the same velocity limit as FOPSO-GWO. PSO and FOPSO-GWO evaluate the cost once per particle
and iteration (3030 evaluations per run), FBPA three times (9030).

The costs are the paper's fitness (`paper`: step ITAE, Eq. 29), tracking against the paper's
FBPA-FOPID (`fbpa`), the same against the better of it and the best FBPA run (`fbpa_all`), and
the whole-controller cost of the final controller (`whole`: tracking, torque and per-joint
caps). Final best cost, lower is better, over random seeds 1–4 (1–8 for `whole`):

| Cost | Optimiser | best | median | mean | worst | mean after 3030 evaluations |
|---|---|---:|---:|---:|---:|---:|
| paper | PSO | **0.035** | **0.069** | **0.073** | **0.119** | **0.073** |
| | FBPA | 0.095 | 0.140 | 0.134 | 0.159 | 0.279 |
| | FOPSO-GWO | 0.042 | 0.133 | 0.116 | 0.156 | 0.116 |
| fbpa | PSO | **0.350** | **0.384** | **0.390** | **0.444** | **0.390** |
| | FBPA | 0.386 | 0.512 | 0.486 | 0.534 | 1.67 |
| | FOPSO-GWO | 0.392 | 0.417 | 0.425 | 0.475 | 0.425 |
| fbpa_all | PSO | **0.389** | **0.535** | **0.540** | **0.704** | **0.540** |
| | FBPA | 0.449 | 1.072 | 1.224 | 2.302 | 10.1 |
| | FOPSO-GWO | 0.459 | 0.933 | 1.157 | 2.302 | 1.157 |
| whole | PSO | 1.378 | 2.223 | 2.181 | 2.983 | 2.181 |
| | FBPA | 1.186 | 2.412 | 2.462 | 4.418 | 5.47 |
| | FOPSO-GWO | **1.076** | **1.951** | **1.929** | **2.694** | **1.929** |

FOPSO-GWO head to head, same cost and random seed:

| Against | seed–cost pairs won | of which `whole` | costs with the lower mean | costs with the lower best run |
|---|---:|---:|---:|---:|
| PSO | 7 of 20 | 6 of 8 | 1 of 4 (`whole`) | 1 of 4 (`whole`) |
| FBPA | 13 of 20 | 5 of 8 | 4 of 4 | 2 of 4 |

* **Plain PSO is the strongest optimiser on the three tracking costs.** With the same budget
  and starting swarm it beats FOPSO-GWO on 11 of their 12 seed pairs, and has the lowest best,
  median, mean and worst cost on each.
* **FOPSO-GWO is the strongest on the whole-controller cost**, the one its final controller is
  tuned with. It has the lowest best, median, mean and worst cost there, and beats PSO on 6 of 8
  seeds and FBPA on 5 of 8. The final controller (seed 2, cost 1.076) is the best of all 24
  `whole` runs. With 8 seeds this is suggestive, not statistically established (6 of 8 has a
  one-sided sign-test p of 0.14).
* **Against FBPA**, FOPSO-GWO has the lower mean under all four costs, with a third of FBPA's
  evaluations. At equal evaluations FBPA is 2–9× worse. FBPA's best run beats FOPSO-GWO's
  under two of the four costs, by 1–2 %.

A plausible reading, not tested here: PSO's stronger pull to the best points (c1 = c2 = 2
against FOPSO-GWO's 1) pays off when the swarm starts next to a good controller and the cost is
smooth tracking. The grey-wolf leaders' extra exploration pays off on the whole-controller
cost, whose caps and penalties make the landscape much more rugged. The defensible claims are
therefore: FOPSO-GWO beats FBPA, the paper's optimiser, at a third of its cost; and for the
constrained whole-controller problem it found the best controller of the three optimisers.
For unconstrained tracking costs, plain PSO with the same budget did better.

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
  The result above is the best of random seeds 1–8 (seed 2).
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
├── controller_gains.m      gains identified from the published curves (+ FBPA, FOPSO-GWO from results/)
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
├── hybrid_fopso_gwo.m      FO-PSO / grey-wolf hybrid optimiser (this work)
├── fbpa.m                  fractional-order beetle antennae PSO (the paper's FBPA)
├── pso.m                   plain PSO, the baseline
└── fopid_fitness.m         cost of one gain set (ITAE + the paper's metrics)
tools/compare_optimizers.m  PSO, FBPA, FOPSO-GWO x 4 costs x 4-8 seeds -> results/optimizer_runs/

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
