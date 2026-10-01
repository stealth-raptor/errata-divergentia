# Results

Jiang, Zhang & Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved
FOPID*, Int. J. Dyn. Control 13:137 (2025), reproduced on its own arm (Table 2, UR DH
kinematics, Eq. 21) in Octave, and extended with this work's optimiser, FOPSO-GWO.

* **Experiments:** step of 1 rad on every joint at t = 1 s; sine tracking, sin(1.5 t) rad.
* **Metrics (paper, Tables 3 and 4):** step overshoot, adjustment time (5 % band) and peak
  time, all from t = 0; sine MSE and sum of |tau|; each averaged over the six joints and
  computed on the paper's 0.01 s logging grid.
* **Controllers:**
  * PID and FOPID: gains identified from the paper's published curves, one set per
    experiment (`controller_gains(name, experiment)`).
  * FBPA-FOPID: the FOPID tuned by the paper's FBPA (`fbpa.m`), re-implemented with the
    paper's settings; the paper's fitness, step ITAE (Eq. 29) (random seed 1).
  * FOPSO-GWO: the FOPID tuned by this work's FO-PSO / grey-wolf hybrid
    (`hybrid_fopso_gwo.m`); cost: both ITAEs, and the five paper metrics scored against the better of the paper's FBPA-FOPID and the FBPA-FOPID re-run here, metric by metric (random seed 2).
  * Both optimisers: 30 particles x 100 iterations (the paper's budget), the same search
    space, seeded with the identified FOPID; one gain set for both experiments.

## 1. The paper's Tables 3 and 4 against this work

| Controller | Source | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad^2) | Sine sum \|tau\| (Nm) |
|---|---|---:|---:|---:|---:|---:|
| PID | Paper (Tables 3-4) | 54.6 | 2.44 | 1.39 | 2.27e-02 | 3.742e+04 |
| PID | This work | 44.0 | 2.98 | 1.47 | 1.54e-02 | 1.046e+04 |
| FOPID | Paper (Tables 3-4) | 31.2 | 1.89 | 1.33 | 8.80e-03 | 2.569e+04 |
| FOPID | This work | 33.2 | 1.95 | 1.42 | 5.51e-03 | 1.034e+04 |
| FBPA-FOPID | Paper (Tables 3-4) | 22.1 | 1.43 | 1.09 | 3.70e-03 | 2.315e+04 |
| FBPA-FOPID | This work | 26.9 | 1.12 | 1.05 | 2.73e-04 | 1.113e+04 |
| **FOPSO-GWO** | **This work (proposed)** | **11.6** | **1.05** | **1.03** | **6.21e-05** | **9438** |

PID, FOPID: this work reproduces the paper's controllers with gains identified from its
curves. FBPA-FOPID: the paper's optimiser re-run on the same arm; its gains are not the
authors' (the paper publishes none). The paper's torque column is not a reproducible
target: it is a permutation of its own Fig. 19 and its torque curves are numerical
artefacts (Sect. 6 below; docs/audit_report.md, 3.3-3.4).

## 2. FOPSO-GWO against FBPA-FOPID

| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work | **FOPSO-GWO** | vs paper FBPA-FOPID | vs FBPA-FOPID (this work) |
|---|---:|---:|---:|---:|---:|---:|
| ITAE step (Eq. 29) | n/a | 1.189 | 0.2742 | **0.2311** | n/a | -15.7 % |
| ITAE sine | n/a | 3.56 | 0.4428 | **0.2778** | n/a | -37.3 % |
| Overshoot (%) | 22.1 | 21.2 | 26.9 | **11.6** | -47.4 % | -56.7 % |
| Adjustment time (s) | 1.430 | 1.524 | 1.116 | **1.050** | -26.6 % | -5.9 % |
| Peak time (s) | 1.090 | 1.258 | 1.053 | **1.035** | -5.0 % | -1.7 % |
| Sine MSE (rad^2) | 3.700e-03 | 3.676e-03 | 2.734e-04 | **6.211e-05** | -98.3 % | -77.3 % |
| Sine sum \|tau\| (Nm) | 2.315e+04 | 2.568e+04 | 1.113e+04 | **9438** | -59.2 % | -15.2 % |
| Step peak torque (Nm) | n/a | n/a | 1.053e+05 | **1.541e+05** | n/a | +46.4 % |

On the paper's five metrics FOPSO-GWO is better than the paper's FBPA-FOPID on 5 of 5,
and better than FBPA-FOPID re-run on this arm on 5 of 5. Negative changes are improvements.
The step peak torque (max |tau| at the step instant, averaged over the joints) is not one
of the paper's metrics.

## 3. FBPA against FOPSO-GWO: same costs, seeds and budget

Each optimiser was run with random seeds 1, 2, 3, 4 under 3 costs (`tools/compare_optimizers.m`),
with the paper's budget of 30 particles x 100 iterations, the same search space, the same
seed (the identified FOPID) and, for each random seed, the same initial swarm:

* **paper:** the paper's fitness, ITAE of the step response (Eq. 29), relative to the FOPID
  (the FOPID scores 1).
* **fbpa:** this work's cost: both ITAEs relative to the FOPID and the five paper metrics
  relative to the paper's FBPA-FOPID, with a penalty on every metric not better than it.
* **fbpa_all:** the same, with each paper metric scored against the better of the paper's
  FBPA-FOPID and the FBPA-FOPID re-run here (Sect. 1), so a ratio below 1 on a metric
  means beating both.

Lower is better. FBPA's beetle antennae cost two extra evaluations per particle and
iteration, so for the same 100 iterations it uses three times the evaluations of FOPSO-GWO.
The last column compares at equal evaluations: the best cost each run had reached after
3030 evaluations (FOPSO-GWO's whole run, FBPA's first 33 iterations).

| Cost | Optimiser | seed 1 | seed 2 | seed 3 | seed 4 | mean | best | evaluations per run | mean after 3030 evaluations |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| paper | FBPA | 0.0955 | 0.1458 | 0.1347 | 0.1591 | 0.1338 | 0.0955 | 9030 | 0.2786 |
| paper | FOPSO-GWO | 0.1412 | 0.1250 | 0.0419 | 0.1562 | 0.1161 | 0.0419 | 3030 | 0.1161 |
| fbpa | FBPA | 0.3856 | 0.5340 | 0.5046 | 0.5191 | 0.4858 | 0.3856 | 9030 | 1.6694 |
| fbpa | FOPSO-GWO | 0.4242 | 0.4090 | 0.4748 | 0.3916 | 0.4249 | 0.3916 | 3030 | 0.4249 |
| fbpa_all | FBPA | 1.1536 | 0.4491 | 2.3021 | 0.9910 | 1.2239 | 0.4491 | 9030 | 10.0992 |
| fbpa_all | FOPSO-GWO | 2.3023 | 0.4593 | 1.3078 | 0.5580 | 1.1568 | 0.4593 | 3030 | 1.1568 |

Head to head, FOPSO-GWO reached the lower final cost on 8 of 12 seed and cost pairs, the
lower mean under 3 of 3 costs and the lower best run under 1 of 3. At equal evaluations
it had the lower mean under 3 of 3 costs.

The best run of each, on the paper's metrics:

| Cost | Optimiser (seed) | ITAE step | ITAE sine | Overshoot (%) | Adjustment time (s) | Peak time (s) | Sine MSE (rad^2) | Sine sum \|tau\| (Nm) | better than the paper's FBPA-FOPID | better than FBPA-FOPID (this work) |
|---|---|---:|---:|---:|---:|---:|---:|---:|:---:|:---:|
| paper | FBPA (1) | 0.2742 | 0.4428 | 26.9 | 1.116 | 1.053 | 2.734e-04 | 1.113e+04 | 4 of 5 | 0 of 5 |
| paper | FOPSO-GWO (3) | 0.1204 | 0.09293 | 14.8 | 1.052 | 1.038 | 6.890e-04 | 1.996e+05 | 4 of 5 | 3 of 5 |
| fbpa | FBPA (1) | 0.9997 | 0.8657 | 7.5 | 1.142 | 1.078 | 3.459e-04 | 9629 | 5 of 5 | 2 of 5 |
| fbpa | FOPSO-GWO (4) | 0.6952 | 0.5673 | 10.3 | 1.162 | 1.077 | 2.472e-04 | 9427 | 5 of 5 | 3 of 5 |
| fbpa_all | FBPA (2) | 0.1569 | 0.1596 | 11.5 | 1.053 | 1.037 | 3.290e-05 | 9623 | 5 of 5 | 5 of 5 |
| fbpa_all | FOPSO-GWO (2) | 0.2311 | 0.2778 | 11.6 | 1.050 | 1.035 | 6.211e-05 | 9438 | 5 of 5 | 5 of 5 |

Convergence: `convergence.png` (best cost against cost evaluations).

## 4. Per joint

### Step overshoot (%)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 46.2 | 53.8 | 18.1 | 8.3 | 150.2 | 77.8 |
| PID | This work | 15.5 | 54.7 | 12.9 | 10.9 | 142.8 | 27.0 |
| FOPID | Paper figures | 37.6 | 38.1 | 5.9 | 9.3 | 85.0 | 30.7 |
| FOPID | This work | 23.5 | 38.8 | 10.6 | 2.9 | 94.2 | 29.5 |
| FBPA-FOPID | Paper figures | 29.0 | 25.0 | 10.5 | 2.5 | 33.1 | 26.9 |
| FBPA-FOPID | This work | 39.9 | 28.3 | 71.1 | 11.4 | 6.7 | 3.9 |
| FOPSO-GWO | This work | 4.3 | 6.4 | 32.1 | 10.8 | 2.7 | 13.4 |

### Step adjustment time (s)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 2.12 | 2.17 | 2.31 | 1.42 | 4.60 | 1.31 |
| PID | This work | 3.84 | 2.67 | 3.28 | 2.08 | 4.80 | 1.24 |
| FOPID | Paper figures | 2.38 | 1.90 | 2.00 | 1.35 | 2.71 | 1.34 |
| FOPID | This work | 2.64 | 2.00 | 1.84 | 1.44 | 2.53 | 1.24 |
| FBPA-FOPID | Paper figures | 1.78 | 1.44 | 1.58 | 1.26 | 1.80 | 1.28 |
| FBPA-FOPID | This work | 1.20 | 1.10 | 1.14 | 1.15 | 1.09 | 1.01 |
| FOPSO-GWO | This work | 1.03 | 1.10 | 1.07 | 1.06 | 1.01 | 1.03 |

### Step peak time (s)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 1.65 | 1.35 | 1.52 | 1.29 | 1.79 | 1.04 |
| PID | This work | 1.53 | 1.30 | 1.46 | 1.72 | 1.70 | 1.09 |
| FOPID | Paper figures | 1.52 | 1.34 | 1.95 | 1.29 | 1.58 | 1.10 |
| FOPID | This work | 1.50 | 1.29 | 1.32 | 1.63 | 1.62 | 1.15 |
| FBPA-FOPID | Paper figures | 1.37 | 1.25 | 1.43 | 1.36 | 1.06 | 1.08 |
| FBPA-FOPID | This work | 1.08 | 1.05 | 1.08 | 1.01 | 1.08 | 1.02 |
| FOPSO-GWO | This work | 1.04 | 1.08 | 1.04 | 1.02 | 1.02 | 1.01 |

### Sine MSE (rad^2)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 7.82e-02 | 3.12e-03 | 1.58e-02 | 1.34e-03 | 3.69e-02 | 4.43e-04 |
| PID | This work | 6.38e-02 | 2.44e-03 | 5.49e-03 | 4.82e-04 | 2.01e-02 | 9.24e-05 |
| FOPID | Paper figures | 1.84e-02 | 1.91e-03 | 6.67e-03 | 1.17e-03 | 2.44e-02 | 4.06e-04 |
| FOPID | This work | 1.42e-02 | 2.51e-03 | 1.98e-03 | 6.46e-04 | 1.36e-02 | 9.58e-05 |
| FBPA-FOPID | Paper figures | 1.13e-02 | 1.40e-03 | 3.85e-03 | 1.42e-03 | 3.71e-03 | 3.37e-04 |
| FBPA-FOPID | This work | 1.34e-03 | 1.70e-06 | 2.97e-04 | 1.27e-07 | 2.98e-07 | 6.64e-09 |
| FOPSO-GWO | This work | 1.32e-04 | 2.33e-04 | 1.74e-06 | 4.07e-07 | 5.11e-06 | 9.03e-08 |

### Sine sum |tau| (Nm)

| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---:|---:|---:|---:|---:|---:|
| PID | Paper figures | 8.141e+04 | 2.819e+04 | 1.95e+04 | 8347 | 1467 | 5.803 |
| PID | This work | 2.096e+04 | 2.597e+04 | 1.239e+04 | 2700 | 544.1 | 166.9 |
| FOPID | Paper figures | 1.442e+05 | 3.271e+04 | 3.338e+04 | 1.225e+04 | 1880 | 10.04 |
| FOPID | This work | 2.12e+04 | 2.468e+04 | 1.033e+04 | 3131 | 2487 | 220.6 |
| FBPA-FOPID | Paper figures | 9.094e+04 | 3.12e+04 | 2.224e+04 | 8262 | 1409 | 5.614 |
| FBPA-FOPID | This work | 2.358e+04 | 2.751e+04 | 1.184e+04 | 2634 | 414.3 | 819.9 |
| FOPSO-GWO | This work | 2.004e+04 | 2.498e+04 | 9520 | 1567 | 382.5 | 134.8 |

## 5. Match to the published curves

rms of q (this work) - q (paper's figure), rad. FBPA-FOPID is not fitted to the paper's
curves: it is the result of re-running the optimiser, so this row measures how close an
independent FBPA run comes to the authors'.

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | 0.092 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | 0.080 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | 0.058 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | 0.051 |
| FBPA-FOPID | step | 0.183 | 0.117 | 0.162 | 0.197 | 0.087 | 0.045 | 0.132 |
| FBPA-FOPID | sine | 0.092 | 0.037 | 0.061 | 0.037 | 0.061 | 0.018 | 0.051 |

## 6. The paper's tables against its own figures

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

## 7. Gains

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

### FBPA-FOPID (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 1903 | 9.997e+04 | 0.04385 | 1.2e+04 | 2155 | 1.483e+04 |
| Ki | 269.1 | 1.843 | 16.98 | 210.5 | 0.3012 | 0.0002227 |
| Kd | 69.43 | 722.1 | 997.8 | 0.6148 | 52.52 | 0.208 |
| lambda | 1.798 | 0.6081 | 1.949 | 0.6449 | 0.1178 | 0.0585 |
| mu | 1.155 | 0.3949 | 0.5965 | 1.8 | 0.9209 | 1.762 |

### FOPSO-GWO (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Kp | 1356 | 0.02271 | 0.1358 | 7104 | 0.1008 | 2883 |
| Ki | 5.632 | 6372 | 9.941e+04 | 1.509 | 102.9 | 0.0001723 |
| Kd | 982.6 | 964.8 | 944.2 | 5.21 | 369.4 | 0.6522 |
| lambda | 1.483 | 0.5123 | 0.3503 | 0.6175 | 0.315 | 0.08313 |
| mu | 0.892 | 0.818 | 0.5771 | 1.313 | 0.6203 | 1.598 |
