# Results

Jiang, Zhang & Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved
FOPID*, Int. J. Dyn. Control 13:137 (2025), reproduced on its own arm (Table 2, UR DH
kinematics, Eq. 21) in Octave, and extended with this work's optimiser, FOPSO-GWO.

* **Experiments:** step of 1 rad on every joint at t = 1 s; sine tracking, sin(1.5 t) rad.
* **Metrics (paper, Tables 3 and 4):** step overshoot, adjustment time (5 % band) and peak
  time, all from t = 0; sine MSE and sum of |tau|; each averaged over the six joints and
  computed on the paper's 0.01 s logging grid.
* **Controllers:**
  * PID, FOPID and FBPA-FOPID, the paper's three controllers: gains identified from the
    paper's published curves of each controller, one set per experiment
    (`controller_gains(name, experiment)`).
  * PSO-FOPID: the FOPID tuned by plain PSO (`pso.m`, the paper's improved PSO,
    Eqs. 23-24) exactly as FOPSO-GWO below: same budget, seed controller, cost and
    initial swarms; the best of random seeds 1-8 (seed 4), as for FOPSO-GWO.
  * FOPSO-GWO: the FOPID tuned by this work's FO-PSO / grey-wolf hybrid
    (`hybrid_fopso_gwo.m`), 30 particles x 100 iterations (the paper's FBPA budget),
    seeded with the identified FOPID, one gain set for both experiments; whole-controller cost: tracking scored against the paper's FBPA-FOPID with no credit beyond twice as good, the torque it takes, and per-joint caps on the peak torque and the overshoot (Sect. 3)
    (random seed 2).

## 1. The paper's Tables 3 and 4 against this work

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad^2) | Sine sum \|tau\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Paper (Tables 3-4) | 54.6 | 2.44 | 1.39 | 2.27e-02 | 3.742e+04 |
| PID | This work | 44.0 | 2.98 | 1.47 | 1.54e-02 | 1.046e+04 |
| FOPID | Paper (Tables 3-4) | 31.2 | 1.89 | 1.33 | 8.80e-03 | 2.569e+04 |
| FOPID | This work | 33.2 | 1.95 | 1.42 | 5.51e-03 | 1.034e+04 |
| FBPA-FOPID | Paper (Tables 3-4) | 22.1 | 1.43 | 1.09 | 3.70e-03 | 2.315e+04 |
| FBPA-FOPID | This work | 19.8 | 1.45 | 1.25 | 2.25e-03 | 1.014e+04 |
| PSO-FOPID | This work (improved PSO) | 17.8 | 1.33 | 1.11 | 2.75e-03 | 1.013e+04 |
| **FOPSO-GWO** | **This work (proposed)** | **18.6** | **1.29** | **1.10** | **1.31e-03** | **9951** |

PID, FOPID, FBPA-FOPID: reproductions, with the gains that make the simulation match each
controller's published curves (Sect. 6); the paper publishes no gains. PSO-FOPID and
FOPSO-GWO: tuned by the two optimisers with the same cost, budget and starting swarm.
The paper's torque column is not a reproducible target: it is a permutation of its own
Fig. 19 and its torque curves are numerical artefacts (Sect. 7; docs/audit_report.md,
3.3-3.4).

## 2. FOPSO-GWO against FBPA-FOPID and PSO-FOPID

| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work | PSO-FOPID | **FOPSO-GWO** | vs paper table | vs this work's FBPA-FOPID | vs PSO-FOPID |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Overshoot (%) | 22.1 | 21.2 | 19.8 | 17.8 | **18.6** | -15.9 % | -6.0 % | +4.4 % |
| Adjustment time (s) | 1.430 | 1.524 | 1.453 | 1.329 | **1.287** | -10.0 % | -11.4 % | -3.2 % |
| Peak time (s) | 1.090 | 1.258 | 1.250 | 1.113 | **1.100** | +0.9 % | -12.0 % | -1.2 % |
| Sine MSE (rad^2) | 3.700e-03 | 3.676e-03 | 2.253e-03 | 2.747e-03 | **1.308e-03** | -64.6 % | -41.9 % | -52.4 % |
| Sine sum \|tau\| (Nm) | 2.315e+04 | 2.568e+04 | 1.014e+04 | 1.013e+04 | **9951** | -57.0 % | -1.9 % | -1.8 % |
| ITAE step (Eq. 29) | n/a | 1.189 | 1.158 | 1.047 | **0.8385** | n/a | -27.6 % | -19.9 % |
| ITAE sine | n/a | 3.56 | 2.44 | 2.274 | **1.692** | n/a | -30.7 % | -25.6 % |

On the paper's five metrics FOPSO-GWO is better than the paper's FBPA-FOPID table on
4 of 5, than its figures on 5 of 5, and than this work's FBPA-FOPID on 5 of 5;
than PSO-FOPID, tuned with the same cost, on 4 of 5 and both ITAEs 2 of 2.
Negative changes are improvements. What the torque costs is in Sect. 3.

## 3. Control effort

Torque at the control rate (1 kHz), from `control_effort.m`. The first 50 ms after the step
are the derivative kick: the fractional derivative of the ideal 1 rad step drives every
controller with a D-term to 1e4-1e6 Nm there, so the kick is listed on its own and the
other step columns exclude it. Peaks are the largest joint; sums and total variations
(sum of |tau(k+1) - tau(k)|, which grows with chattering) are averaged over the joints.

| Controller | Kick peak | Step peak after kick | Sine peak | Step sum \|tau\| after kick | Sine sum \|tau\| | Step total variation | Sine total variation |
|---|---:|---:|---:|---:|---:|---:|---:|
| PID | 2.11e+05 | 1078 | 350.1 | 1.246e+04 | 1.046e+04 | 934.5 | 5055 |
| FOPID | 7.87e+05 | 1524 | 1902 | 8499 | 1.034e+04 | 922.3 | 2.284e+04 |
| FBPA-FOPID | 8.17e+04 | 1495 | 494.8 | 6343 | 1.014e+04 | 934.7 | 1445 |
| PSO-FOPID | 1.86e+05 | 855.8 | 290.8 | 6025 | 1.013e+04 | 794.6 | 643 |
| FOPSO-GWO | 8.05e+04 | 1476 | 364.7 | 7662 | 9951 | 1175 | 597.7 |

PSO-FOPID and FOPSO-GWO were tuned with caps per joint: for the torque, the largest peak any of the
paper's three controllers needs on that joint over all their identified gain sets (Nm);
for the overshoot, the worst joint of the paper's own FBPA-FOPID figures (%):

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kick: cap | 1.113e+06 | 2.105e+05 | 1.502e+05 | 1.09e+04 | 4.552e+04 | 5.085e+04 |
| Kick: PSO-FOPID | 1.857e+05 | 2.492e+04 | 6.009e+04 | 2469 | 1.017e+04 | 3.892e+04 |
| Kick: FOPSO-GWO | 8.052e+04 | 3.022e+04 | 7.873e+04 | 5757 | 1.297e+04 | 1.594e+04 |
| Step after kick: cap | 1524 | 1495 | 754.3 | 159.7 | 60.9 | 639 |
| Step after kick: PSO-FOPID | 855.8 | 751.2 | 328.6 | 78.05 | 30.52 | 32.65 |
| Step after kick: FOPSO-GWO | 1476 | 1163 | 690.1 | 98.07 | 46.59 | 8.98 |
| Sine: cap | 1669 | 1902 | 627.2 | 495.1 | 480.8 | 64.29 |
| Sine: PSO-FOPID | 278.5 | 290.8 | 141.5 | 14.13 | 23.13 | 56 |
| Sine: FOPSO-GWO | 195.1 | 364.7 | 173.3 | 20.91 | 26.2 | 23.92 |
| Overshoot: cap | 33.07 | 33.07 | 33.07 | 33.07 | 33.07 | 33.07 |
| Overshoot: PSO-FOPID | 30.63 | 15.66 | 4.716 | 13.1 | 29.17 | 13.43 |
| Overshoot: FOPSO-GWO | 19.4 | 16.48 | 6.273 | 10.06 | 29.97 | 29.27 |

PSO-FOPID stays within 24 of the 24 caps. FOPSO-GWO stays within 24 of the 24 caps. 

## 4. The optimisers: PSO, FBPA and FOPSO-GWO under the same costs, seeds and budget

`tools/compare_optimizers.m` runs PSO, FBPA, FOPSO-GWO under 4 costs. For a fair comparison
everything but the algorithm is the same:

* 30 particles x 100 iterations (the paper's FBPA budget), the same search space and the
  same seed controller (the identified FOPID);
* for each random seed the same initial swarm: the optimisers share the initialisation code
  and its random draws;
* the same cost function, conditioning check and early abort of unstable candidates;
* each algorithm with its standard or published coefficients: PSO c1 = c2 = 2 and inertia
  0.9 -> 0.4 (the paper's improved PSO, Eqs. 23-24), FBPA the paper's Sect. 4 settings,
  FOPSO-GWO c1 = c2 = c3 = 1; PSO and FOPSO-GWO limit |v| to 0.2 of each range, FBPA to 1;
* PSO and FOPSO-GWO evaluate the cost once per particle and iteration (3030 evaluations), FBPA
  three times (its beetle antennae).

The costs:

* **paper:** the paper's fitness, ITAE of the step response (Eq. 29), relative to the FOPID
  (the FOPID scores 1).
* **fbpa:** both ITAEs relative to the FOPID and the five paper metrics relative to the
  paper's FBPA-FOPID, with a penalty on every metric not better than it.
* **fbpa_all:** the same, with each paper metric scored against the better of the paper's
  FBPA-FOPID and the best FBPA run under the paper's fitness (`results/fbpa_gains.mat`).
* **whole:** the whole-controller cost of this work's final controller (Sect. 3).

Final best cost (lower is better), over the random seeds run; the last column compares at
equal evaluations, the best cost each run had reached after 3030 evaluations (FBPA's first
33 iterations).

| Cost | Optimiser | runs | best | median | mean | worst | mean after 3030 evaluations |
|---|---|---:|---:|---:|---:|---:|---:|
| paper | PSO | 4 | 0.0352 | 0.0685 | 0.0727 | 0.1186 | 0.0727 |
| paper | FBPA | 4 | 0.0955 | 0.1402 | 0.1338 | 0.1591 | 0.2786 |
| paper | FOPSO-GWO | 4 | 0.0419 | 0.1331 | 0.1161 | 0.1562 | 0.1161 |
| fbpa | PSO | 4 | 0.3495 | 0.3842 | 0.3904 | 0.4437 | 0.3904 |
| fbpa | FBPA | 4 | 0.3856 | 0.5118 | 0.4858 | 0.5340 | 1.6694 |
| fbpa | FOPSO-GWO | 4 | 0.3916 | 0.4166 | 0.4249 | 0.4748 | 0.4249 |
| fbpa_all | PSO | 4 | 0.3886 | 0.5346 | 0.5405 | 0.7043 | 0.5405 |
| fbpa_all | FBPA | 4 | 0.4491 | 1.0723 | 1.2239 | 2.3021 | 10.0992 |
| fbpa_all | FOPSO-GWO | 4 | 0.4593 | 0.9329 | 1.1568 | 2.3023 | 1.1568 |
| whole | PSO | 8 | 1.3779 | 2.2230 | 2.1810 | 2.9826 | 2.1810 |
| whole | FBPA | 8 | 1.1864 | 2.4115 | 2.4620 | 4.4183 | 5.4653 |
| whole | FOPSO-GWO | 8 | 1.0764 | 1.9514 | 1.9294 | 2.6937 | 1.9294 |

Head to head, FOPSO-GWO against each of the others (same cost and random seed):

| FOPSO-GWO against | seed and cost pairs with the lower final cost | costs with the lower mean | costs with the lower best run | costs with the lower mean at 3030 evaluations |
|---|---:|---:|---:|---:|
| PSO | 7 of 20 | 1 of 4 | 1 of 4 | 1 of 4 |
| FBPA | 13 of 20 | 4 of 4 | 2 of 4 | 4 of 4 |

The best run of each, on the paper's metrics:

| Cost | Optimiser (seed) | ITAE step | ITAE sine | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad^2) | Sine sum \|tau\| (Nm) | better than the paper's FBPA-FOPID | better than this work's FBPA-FOPID |
|---|---|---:|---:|---:|---:|---:|---:|---:|:---:|:---:|
| paper | PSO (3) | 0.101 | 0.1409 | 5.1 | 1.026 | 1.028 | 2.150e-03 | 4.394e+05 | 4 of 5 | 4 of 5 |
| paper | FBPA (1) | 0.2742 | 0.4428 | 26.9 | 1.116 | 1.053 | 2.734e-04 | 1.113e+04 | 4 of 5 | 3 of 5 |
| paper | FOPSO-GWO (3) | 0.1204 | 0.09293 | 14.8 | 1.052 | 1.038 | 6.890e-04 | 1.996e+05 | 4 of 5 | 4 of 5 |
| fbpa | PSO (1) | 0.8914 | 0.7566 | 7.1 | 1.139 | 1.065 | 2.878e-04 | 9691 | 5 of 5 | 5 of 5 |
| fbpa | FBPA (1) | 0.9997 | 0.8657 | 7.5 | 1.142 | 1.078 | 3.459e-04 | 9629 | 5 of 5 | 5 of 5 |
| fbpa | FOPSO-GWO (4) | 0.6952 | 0.5673 | 10.3 | 1.162 | 1.077 | 2.472e-04 | 9427 | 5 of 5 | 5 of 5 |
| fbpa_all | PSO (4) | 0.1752 | 0.2028 | 8.1 | 1.034 | 1.033 | 4.214e-05 | 9342 | 5 of 5 | 5 of 5 |
| fbpa_all | FBPA (2) | 0.1569 | 0.1596 | 11.5 | 1.053 | 1.037 | 3.290e-05 | 9623 | 5 of 5 | 5 of 5 |
| fbpa_all | FOPSO-GWO (2) | 0.2311 | 0.2778 | 11.6 | 1.050 | 1.035 | 6.211e-05 | 9438 | 5 of 5 | 5 of 5 |
| whole | PSO (4) | 1.047 | 2.274 | 17.8 | 1.329 | 1.113 | 2.747e-03 | 1.013e+04 | 4 of 5 | 4 of 5 |
| whole | FBPA (7) | 1.03 | 1.772 | 19.3 | 1.422 | 1.102 | 1.357e-03 | 1.016e+04 | 4 of 5 | 4 of 5 |
| whole | FOPSO-GWO (2) | 0.8385 | 1.692 | 18.6 | 1.287 | 1.100 | 1.308e-03 | 9951 | 4 of 5 | 5 of 5 |

Convergence: `convergence.png` (best cost against cost evaluations).

### Why plain PSO wins: one change at a time

`tools/ablate_optimizers.m` reruns the optimisers with one setting changed, under the
same budget, seed controller, initial swarms and cost. The last column is the mean cost of
the swarm's current positions over the last 25 iterations: unstable candidates score
1e3-2e3, so a large value means the swarm is still scattered (or thrown about), a value
near the best cost that it has collapsed onto one point.

Cost `fbpa`:

| Optimiser | runs | best | median | mean | worst | swarm, last 25 iterations |
|---|---:|---:|---:|---:|---:|---:|
| PSO, unmodified (c1 = c2 = 2, inertia 0.9 -> 0.4, \|v\| <= 0.2) | 4 | 0.3495 | 0.3842 | 0.3904 | 0.4437 | 397.1 |
| PSO with c1 = c2 = 1 | 4 | 0.3596 | 0.5914 | 0.6314 | 0.9834 | 48.19 |
| FOPSO-GWO, unmodified (c1 = c2 = c3 = 1, fractional memory, \|v\| <= 0.2) | 4 | 0.3916 | 0.4166 | 0.4249 | 0.4748 | 8.011 |
| FOPSO-GWO with c1 = c2 = c3 = 2 | 4 | 0.4743 | 0.5112 | 0.6344 | 1.0411 | 1141 |
| FO-PSO alone: FOPSO-GWO without the grey-wolf term, c1 = c2 = 2 | 4 | 0.3975 | 0.4374 | 0.4686 | 0.6023 | 738.6 |
| FBPA, unmodified (c = 2, fractional memory, beetle, \|v\| <= 1) | 4 | 0.3856 | 0.5118 | 0.4858 | 0.5340 | 1312 |
| FBPA with \|v\| <= 0.2 | 4 | 0.2775 | 0.3538 | 0.3524 | 0.4245 | 721.5 |

FBPA with \|v\| <= 0.2 and FO-PSO alone are the same search: FBPA's beetle step (1e-4 of
the range, shrinking to 6e-7) does not move the particles, so the two differ only in their
random numbers. The gap between them is the run-to-run noise at this number of seeds.

## 5. Per joint

### Step overshoot (%)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 46.2 | 53.8 | 18.1 | 8.3 | 150.2 | 77.8 |
| PID | This work | 15.5 | 54.7 | 12.9 | 10.9 | 142.8 | 27.0 |
| FOPID | Paper figures | 37.6 | 38.1 | 5.9 | 9.3 | 85.0 | 30.7 |
| FOPID | This work | 23.5 | 38.8 | 10.6 | 2.9 | 94.2 | 29.5 |
| FBPA-FOPID | Paper figures | 29.0 | 25.0 | 10.5 | 2.5 | 33.1 | 26.9 |
| FBPA-FOPID | This work | 30.6 | 16.0 | 4.0 | 7.4 | 50.2 | 10.3 |
| PSO-FOPID | This work | 30.6 | 15.7 | 4.7 | 13.1 | 29.2 | 13.4 |
| FOPSO-GWO | This work | 19.4 | 16.5 | 6.3 | 10.1 | 30.0 | 29.3 |

### Step adjustment time (s)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 2.12 | 2.17 | 2.31 | 1.42 | 4.60 | 1.31 |
| PID | This work | 3.84 | 2.67 | 3.28 | 2.08 | 4.80 | 1.24 |
| FOPID | Paper figures | 2.38 | 1.90 | 2.00 | 1.35 | 2.71 | 1.34 |
| FOPID | This work | 2.64 | 2.00 | 1.84 | 1.44 | 2.53 | 1.24 |
| FBPA-FOPID | Paper figures | 1.78 | 1.44 | 1.58 | 1.26 | 1.80 | 1.28 |
| FBPA-FOPID | This work | 2.09 | 1.42 | 1.26 | 1.56 | 1.34 | 1.05 |
| PSO-FOPID | This work | 1.58 | 1.48 | 1.18 | 1.28 | 1.36 | 1.09 |
| FOPSO-GWO | This work | 1.52 | 1.36 | 1.17 | 1.26 | 1.09 | 1.33 |

### Step peak time (s)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 1.65 | 1.35 | 1.52 | 1.29 | 1.79 | 1.04 |
| PID | This work | 1.53 | 1.30 | 1.46 | 1.72 | 1.70 | 1.09 |
| FOPID | Paper figures | 1.52 | 1.34 | 1.95 | 1.29 | 1.58 | 1.10 |
| FOPID | This work | 1.50 | 1.29 | 1.32 | 1.63 | 1.62 | 1.15 |
| FBPA-FOPID | Paper figures | 1.37 | 1.25 | 1.43 | 1.36 | 1.06 | 1.08 |
| FBPA-FOPID | This work | 1.36 | 1.24 | 1.41 | 1.41 | 1.06 | 1.02 |
| PSO-FOPID | This work | 1.24 | 1.31 | 1.01 | 1.09 | 1.01 | 1.02 |
| FOPSO-GWO | This work | 1.16 | 1.24 | 1.01 | 1.06 | 1.01 | 1.12 |

### Sine MSE (rad^2)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 7.82e-02 | 3.12e-03 | 1.58e-02 | 1.34e-03 | 3.69e-02 | 4.43e-04 |
| PID | This work | 6.38e-02 | 2.44e-03 | 5.49e-03 | 4.82e-04 | 2.01e-02 | 9.24e-05 |
| FOPID | Paper figures | 1.84e-02 | 1.91e-03 | 6.67e-03 | 1.17e-03 | 2.44e-02 | 4.06e-04 |
| FOPID | This work | 1.42e-02 | 2.51e-03 | 1.98e-03 | 6.46e-04 | 1.36e-02 | 9.58e-05 |
| FBPA-FOPID | Paper figures | 1.13e-02 | 1.40e-03 | 3.85e-03 | 1.42e-03 | 3.71e-03 | 3.37e-04 |
| FBPA-FOPID | This work | 9.69e-03 | 1.15e-03 | 1.36e-03 | 1.20e-03 | 7.38e-10 | 1.22e-04 |
| PSO-FOPID | This work | 9.20e-03 | 6.50e-03 | 3.02e-04 | 3.62e-04 | 1.16e-04 | 1.63e-07 |
| FOPSO-GWO | This work | 4.49e-03 | 2.64e-03 | 3.52e-04 | 2.08e-04 | 6.82e-05 | 9.95e-05 |

### Sine sum |tau| (Nm)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 8.141e+04 | 2.819e+04 | 1.95e+04 | 8347 | 1467 | 5.803 |
| PID | This work | 2.096e+04 | 2.597e+04 | 1.239e+04 | 2700 | 544.1 | 166.9 |
| FOPID | Paper figures | 1.442e+05 | 3.271e+04 | 3.338e+04 | 1.225e+04 | 1880 | 10.04 |
| FOPID | This work | 2.12e+04 | 2.468e+04 | 1.033e+04 | 3131 | 2487 | 220.6 |
| FBPA-FOPID | Paper figures | 9.094e+04 | 3.12e+04 | 2.224e+04 | 8262 | 1409 | 5.614 |
| FBPA-FOPID | This work | 2.241e+04 | 2.527e+04 | 1.079e+04 | 1658 | 602 | 137.8 |
| PSO-FOPID | This work | 2.317e+04 | 2.463e+04 | 1.07e+04 | 1696 | 490.2 | 129.4 |
| FOPSO-GWO | This work | 2.19e+04 | 2.517e+04 | 1.042e+04 | 1643 | 421.4 | 140.2 |

## 6. Match to the published curves

rms of q (this work) - q (paper's figure), rad.

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | 0.092 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | 0.080 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | 0.058 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | 0.051 |
| FBPA-FOPID | step | 0.026 | 0.027 | 0.038 | 0.053 | 0.052 | 0.044 | 0.040 |
| FBPA-FOPID | sine | 0.038 | 0.018 | 0.050 | 0.015 | 0.061 | 0.015 | 0.033 |

## 7. The paper's tables against its own figures

The same metrics recomputed from the paper's published curves (Figs 6-19, read from the
PDF's vector graphics) do not fully agree with its tables.

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad^2) | Sine sum \|tau\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Table | 54.6 | 2.44 | 1.39 | 2.27e-02 | 3.742e+04 |
| PID | Figures | 59.1 | 2.32 | 1.44 | 2.26e-02 | 2.315e+04 |
| FOPID | Table | 31.2 | 1.89 | 1.33 | 8.80e-03 | 2.569e+04 |
| FOPID | Figures | 34.4 | 1.95 | 1.46 | 8.83e-03 | 3.741e+04 |
| FBPA-FOPID | Table | 22.1 | 1.43 | 1.09 | 3.70e-03 | 2.315e+04 |
| FBPA-FOPID | Figures | 21.2 | 1.52 | 1.26 | 3.68e-03 | 2.568e+04 |

The torque column of Table 4 is a permutation of what Fig. 19 shows: the curves labelled
PID / FOPID / FBPA-FOPID sum to 2.3153e4 / 3.7412e4 / 2.5675e4, which are the table's
FBPA / PID / FOPID values. The torque curves themselves are not outputs of the control
law (spikes of up to 1e9 Nm in Fig. 12), so this work's torques, which are physically
consistent, are much smaller. See docs/audit_report.md, Sect. 3.

## 8. Gains

### PID, step experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 97.38 | 1565 | 359.3 | 102.5 | 4.088 | 56.88 |
| Ki | 557.5 | 1.049e+04 | 163.1 | 6.115 | 0.9552 | 310.6 |
| Kd | 160 | 209 | 149.9 | 2.754 | 0.177 | 2.982 |
| lambda | 1 | 1 | 1 | 1 | 1 | 1 |
| mu | 1 | 1 | 1 | 1 | 1 | 1 |

### PID, sine experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 85.75 | 1083 | 386.1 | 8.583 | 4.747 | 8.473 |
| Ki | 308.9 | 5038 | 0.0001003 | 50.57 | 2.164 | 178.3 |
| Kd | 70.71 | 195 | 56.35 | 233.4 | 1.856 | 10.41 |
| lambda | 1 | 1 | 1 | 1 | 1 | 1 |
| mu | 1 | 1 | 1 | 1 | 1 | 1 |

### FOPID, step experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 0.2246 | 0.001009 | 233.1 | 129.1 | 0.1001 | 37.97 |
| Ki | 1012 | 4092 | 66.46 | 0.0001 | 2.323 | 655.2 |
| Kd | 111.6 | 488.4 | 310.7 | 2.102 | 0.8776 | 0.147 |
| lambda | 0.3763 | 0.5867 | 0.732 | 1.94 | 0.09078 | 1.706 |
| mu | 1.283 | 0.7291 | 0.6236 | 1.223 | 0.7143 | 1.808 |

### FOPID, sine experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 496 | 0.002705 | 0.1237 | 153.4 | 15.18 | 28.58 |
| Ki | 532.7 | 2433 | 323.6 | 0.02101 | 5.179 | 71.35 |
| Kd | 82.04 | 114.7 | 450.4 | 11.69 | 0.9272 | 1.965 |
| lambda | 1.774 | 0.2692 | 1.95 | 1.07 | 1.948 | 0.8092 |
| mu | 1.227 | 1.348 | 0.7744 | 1.387 | 1.75 | 1.275 |

### FBPA-FOPID, step experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 0.0302 | 0.003118 | 36.34 | 99.23 | 0.01844 | 1633 |
| Ki | 48.97 | 21.9 | 14.34 | 0.0002432 | 760.6 | 2.973e+04 |
| Kd | 14.08 | 917.4 | 81.67 | 8.897 | 9.23 | 0.2295 |
| lambda | 0.05003 | 1.949 | 0.4206 | 0.1691 | 0.9006 | 1.582 |
| mu | 0.9999 | 0.5148 | 0.9999 | 0.4464 | 0.7844 | 1.747 |

### FBPA-FOPID, sine experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 364.8 | 3.334 | 0.002545 | 116.4 | 1.076e+05 | 0.3871 |
| Ki | 582 | 1901 | 372.1 | 0.0003336 | 9221 | 1158 |
| Kd | 169.7 | 1543 | 520.2 | 5.553 | 0.1541 | 186.6 |
| lambda | 1.592 | 1.091 | 1.948 | 1.307 | 1.392 | 1.94 |
| mu | 0.8398 | 0.4539 | 0.7758 | 1.406 | 1.91 | 0.5412 |

### PSO-FOPID (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 604.6 | 0.2201 | 0.05336 | 146.3 | 0.7653 | 1267 |
| Ki | 256.2 | 199.5 | 13.19 | 45.93 | 142.4 | 0.0004158 |
| Kd | 67 | 481.2 | 1000 | 8.223 | 29.96 | 0.164 |
| lambda | 1.35 | 0.7031 | 1.894 | 0.3154 | 0.7827 | 0.05 |
| mu | 1.147 | 0.5714 | 0.5929 | 0.8167 | 0.8436 | 1.781 |

### FOPSO-GWO (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 561.5 | 0.02776 | 0.03934 | 255.8 | 97.4 | 48.33 |
| Ki | 19.09 | 100.5 | 1.081 | 3.311 | 0.2999 | 0.0003511 |
| Kd | 91.45 | 747 | 880.1 | 5.666 | 20.67 | 0.2909 |
| lambda | 1.578 | 0.5538 | 1.118 | 0.5768 | 0.6475 | 0.1035 |
| mu | 0.9806 | 0.5356 | 0.6505 | 0.9957 | 0.9315 | 1.579 |
