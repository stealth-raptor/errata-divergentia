# Reproduction of the published PID and FOPID results

Columns: the paper's table; the same metric recomputed from the paper's own
published curves (Figs 6-19, read from the PDF's vector graphics); this work.
Settling band 5 %, all metrics on the paper's 0.01 s logging grid.
Gains: identified separately for each experiment (controller_gains(name, experiment)).

## PID

| Metric | Paper table | Paper figures | This work |
|---|---:|---:|---:|
| Step overshoot (%) | 54.6 | 59.1 | 44.0 |
| Step adjustment time (s) | 2.44 | 2.32 | 2.98 |
| Step peak time (s) | 1.39 | 1.44 | 1.47 |
| Sine MSE (rad^2) | 2.270e-02 | 2.263e-02 | 1.542e-02 |
| Sine sum |tau| (Nm) | 3.742e+04 | 2.315e+04 | 1.046e+04 |

## FOPID

| Metric | Paper table | Paper figures | This work |
|---|---:|---:|---:|
| Step overshoot (%) | 31.2 | 34.4 | 33.2 |
| Step adjustment time (s) | 1.89 | 1.95 | 1.95 |
| Step peak time (s) | 1.33 | 1.46 | 1.42 |
| Sine MSE (rad^2) | 8.800e-03 | 8.830e-03 | 5.506e-03 |
| Sine sum |tau| (Nm) | 2.569e+04 | 3.741e+04 | 1.034e+04 |

The sine-torque column of the paper's Table 4 is a permutation of what its
Fig. 19 shows: the curves labelled PID / FOPID / FBPA-FOPID sum to
2.3153e4 / 3.7412e4 / 2.5675e4, i.e. the table's FBPA / PID / FOPID values.
See docs/audit_report.md.

# FOPID re-tuned by the FO-PSO / GWO hybrid (FOPSO-GWO)

Tuned with TUNE_FOPID_HYBRID: 30 particles x 100 iterations (3030 cost evaluations), fitness weights [0.5 0.5 1 1 1 1 1 0] on [ITAE step, ITAE sine, the five paper metrics, step peak torque];
target FBPA: the five paper metrics scored against the paper's FBPA-FOPID (times after
the step instant), final cost 0.3916 where meeting that reference scores 1.
Baseline: the identified FOPID, one gain set for both experiments (controller_gains('FOPID')),
which seeded the swarm.

| Metric | FOPID baseline | FOPSO-GWO | change | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | beats FBPA table |
|---|---:|---:|---:|---:|---:|:---:|
| ITAE step | 2.871 | 0.6952 | -75.8% | n/a | 1.189 | n/a |
| ITAE sine | 4.742 | 0.5673 | -88.0% | n/a | 3.56 | n/a |
| Step overshoot (%) | 35.9 | 10.3 | -71.3% | 22.1 | 21.2 | yes |
| Step adjustment time (s) | 2.10 | 1.16 | -44.7% | 1.43 | 1.52 | yes |
| Step peak time (s) | 1.32 | 1.08 | -18.1% | 1.09 | 1.26 | yes |
| Sine MSE (rad^2) | 1.050e-02 | 2.472e-04 | -97.6% | 3.700e-03 | 3.676e-03 | yes |
| Sine sum |tau| (Nm) | 9961 | 9427 | -5.4% | 2.315e+04 | 2.568e+04 | yes |
| Step peak torque (Nm) | 1.725e+05 | 2.139e+05 | +24.0% | n/a | n/a | n/a |

The paper's FBPA-FOPID torque values are numerical artefacts (docs/audit_report.md, 3.3-3.4),
and its step peak torque is not comparable; FOPSO-GWO is a different optimiser, not a
reproduction of FBPA.

## Match to the published curves (rms of q_this_work - q_paper, rad)

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |
|---|---|---|---|---|---|---|---|---|
| PID | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | 0.092 |
| PID | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | 0.080 |
| FOPID | step | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | 0.058 |
| FOPID | sine | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | 0.051 |

## Per joint: Step overshoot (%)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 46.2 | 53.8 | 18.1 | 8.3 | 150.2 | 77.8 |
| PID, this work | 15.5 | 54.7 | 12.9 | 10.9 | 142.8 | 27.0 |
| FOPID, paper figures | 37.6 | 38.1 | 5.9 | 9.3 | 85.0 | 30.7 |
| FOPID, this work | 23.5 | 38.8 | 10.6 | 2.9 | 94.2 | 29.5 |
| FBPA-FOPID, paper figures | 29.0 | 25.0 | 10.5 | 2.5 | 33.1 | 26.9 |
| FOPSO-GWO, this work | 3.7 | 4.6 | 5.1 | 13.4 | 14.6 | 20.3 |

## Per joint: Step adjustment time (s)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 2.12 | 2.17 | 2.31 | 1.42 | 4.60 | 1.31 |
| PID, this work | 3.84 | 2.67 | 3.28 | 2.08 | 4.80 | 1.24 |
| FOPID, paper figures | 2.38 | 1.90 | 2.00 | 1.35 | 2.71 | 1.34 |
| FOPID, this work | 2.64 | 2.00 | 1.84 | 1.44 | 2.53 | 1.24 |
| FBPA-FOPID, paper figures | 1.78 | 1.44 | 1.58 | 1.26 | 1.80 | 1.28 |
| FOPSO-GWO, this work | 1.03 | 1.12 | 1.47 | 1.05 | 1.07 | 1.23 |

## Per joint: Step peak time (s)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 1.65 | 1.35 | 1.52 | 1.29 | 1.79 | 1.04 |
| PID, this work | 1.53 | 1.30 | 1.46 | 1.72 | 1.70 | 1.09 |
| FOPID, paper figures | 1.52 | 1.34 | 1.95 | 1.29 | 1.58 | 1.10 |
| FOPID, this work | 1.50 | 1.29 | 1.32 | 1.63 | 1.62 | 1.15 |
| FBPA-FOPID, paper figures | 1.37 | 1.25 | 1.43 | 1.36 | 1.06 | 1.08 |
| FOPSO-GWO, this work | 1.06 | 1.18 | 1.15 | 1.01 | 1.01 | 1.05 |

## Per joint: Sine MSE (rad^2)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 7.82e-02 | 3.12e-03 | 1.58e-02 | 1.34e-03 | 3.69e-02 | 4.43e-04 |
| PID, this work | 6.38e-02 | 2.44e-03 | 5.49e-03 | 4.82e-04 | 2.01e-02 | 9.24e-05 |
| FOPID, paper figures | 1.84e-02 | 1.91e-03 | 6.67e-03 | 1.17e-03 | 2.44e-02 | 4.06e-04 |
| FOPID, this work | 1.42e-02 | 2.51e-03 | 1.98e-03 | 6.46e-04 | 1.36e-02 | 9.58e-05 |
| FBPA-FOPID, paper figures | 1.13e-02 | 1.40e-03 | 3.85e-03 | 1.42e-03 | 3.71e-03 | 3.37e-04 |
| FOPSO-GWO, this work | 3.14e-05 | 1.13e-03 | 3.10e-04 | 5.74e-08 | 6.43e-07 | 9.32e-06 |

## Per joint: Sine sum |tau| (Nm)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 8.141e+04 | 2.819e+04 | 1.95e+04 | 8347 | 1467 | 5.803 |
| PID, this work | 2.096e+04 | 2.597e+04 | 1.239e+04 | 2700 | 544.1 | 166.9 |
| FOPID, paper figures | 1.442e+05 | 3.271e+04 | 3.338e+04 | 1.225e+04 | 1880 | 10.04 |
| FOPID, this work | 2.12e+04 | 2.468e+04 | 1.033e+04 | 3131 | 2487 | 220.6 |
| FBPA-FOPID, paper figures | 9.094e+04 | 3.12e+04 | 2.224e+04 | 8262 | 1409 | 5.614 |
| FOPSO-GWO, this work | 1.964e+04 | 2.511e+04 | 9753 | 1580 | 349.2 | 137.2 |

## PID gains, step experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 97.38 | 1565 | 359.3 | 102.5 | 4.088 | 56.88 |
| Ki | 557.5 | 1.049e+04 | 163.1 | 6.115 | 0.9552 | 310.6 |
| Kd | 160 | 209 | 149.9 | 2.754 | 0.177 | 2.982 |
| lambda | 1 | 1 | 1 | 1 | 1 | 1 |
| mu | 1 | 1 | 1 | 1 | 1 | 1 |

## PID gains, sine experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 85.75 | 1083 | 386.1 | 8.583 | 4.747 | 8.473 |
| Ki | 308.9 | 5038 | 0.0001003 | 50.57 | 2.164 | 178.3 |
| Kd | 70.71 | 195 | 56.35 | 233.4 | 1.856 | 10.41 |
| lambda | 1 | 1 | 1 | 1 | 1 | 1 |
| mu | 1 | 1 | 1 | 1 | 1 | 1 |

## FOPID gains, step experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 0.2246 | 0.001009 | 233.1 | 129.1 | 0.1001 | 37.97 |
| Ki | 1012 | 4092 | 66.46 | 0.0001 | 2.323 | 655.2 |
| Kd | 111.6 | 488.4 | 310.7 | 2.102 | 0.8776 | 0.147 |
| lambda | 0.3763 | 0.5867 | 0.732 | 1.94 | 0.09078 | 1.706 |
| mu | 1.283 | 0.7291 | 0.6236 | 1.223 | 0.7143 | 1.808 |

## FOPID gains, sine experiment

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 496 | 0.002705 | 0.1237 | 153.4 | 15.18 | 28.58 |
| Ki | 532.7 | 2433 | 323.6 | 0.02101 | 5.179 | 71.35 |
| Kd | 82.04 | 114.7 | 450.4 | 11.69 | 0.9272 | 1.965 |
| lambda | 1.774 | 0.2692 | 1.95 | 1.07 | 1.948 | 0.8092 |
| mu | 1.227 | 1.348 | 0.7744 | 1.387 | 1.75 | 1.275 |

## FOPSO-GWO gains (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 6204 | 0.4034 | 0.07568 | 1.741e+04 | 1333 | 202.6 |
| Ki | 5.449 | 153.3 | 0.7901 | 3.394 | 0.003141 | 0.0001953 |
| Kd | 997.4 | 973.4 | 998.9 | 0.2318 | 5.627 | 0.18 |
| lambda | 0.9811 | 0.8113 | 1.939 | 0.2166 | 0.451 | 0.09724 |
| mu | 0.9882 | 0.7136 | 0.5066 | 1.781 | 1.408 | 1.719 |
