# Reproduction of the published PID and FOPID results

Columns: the paper's table; the same metric recomputed from the paper's own
published curves (Figs 6-19, read from the PDF's vector graphics); this work.
Settling band 5 %, all metrics on the paper's 0.01 s logging grid.
Gains: one set per controller for both experiments (controller_gains(name)).

## PID

| Metric | Paper table | Paper figures | This work |
|---|---:|---:|---:|
| Step overshoot (%) | 54.6 | 59.1 | 49.8 |
| Step adjustment time (s) | 2.44 | 2.32 | 2.84 |
| Step peak time (s) | 1.39 | 1.44 | 1.43 |
| Sine MSE (rad^2) | 2.270e-02 | 2.263e-02 | 2.043e-02 |
| Sine sum |tau| (Nm) | 3.742e+04 | 2.315e+04 | 1.053e+04 |

## FOPID

| Metric | Paper table | Paper figures | This work |
|---|---:|---:|---:|
| Step overshoot (%) | 31.2 | 34.4 | 35.9 |
| Step adjustment time (s) | 1.89 | 1.95 | 2.10 |
| Step peak time (s) | 1.33 | 1.46 | 1.32 |
| Sine MSE (rad^2) | 8.800e-03 | 8.830e-03 | 1.050e-02 |
| Sine sum |tau| (Nm) | 2.569e+04 | 3.741e+04 | 9961 |

The sine-torque column of the paper's Table 4 is a permutation of what its
Fig. 19 shows: the curves labelled PID / FOPID / FBPA-FOPID sum to
2.3153e4 / 3.7412e4 / 2.5675e4, i.e. the table's FBPA / PID / FOPID values.
See docs/audit_report.md.

## Match to the published curves (rms of q_this_work - q_paper, rad)

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |
|---|---|---|---|---|---|---|---|---|
| PID | step | 0.157 | 0.050 | 0.076 | 0.104 | 0.187 | 0.076 | 0.108 |
| PID | sine | 0.143 | 0.050 | 0.118 | 0.046 | 0.218 | 0.021 | 0.099 |
| FOPID | step | 0.095 | 0.034 | 0.041 | 0.051 | 0.141 | 0.055 | 0.070 |
| FOPID | sine | 0.062 | 0.032 | 0.076 | 0.017 | 0.157 | 0.030 | 0.062 |

## Per joint: Step overshoot (%)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 46.2 | 53.8 | 18.1 | 8.3 | 150.2 | 77.8 |
| PID, this work | 21.0 | 64.1 | 25.8 | 42.7 | 141.5 | 3.4 |
| FOPID, paper figures | 37.6 | 38.1 | 5.9 | 9.3 | 85.0 | 30.7 |
| FOPID, this work | 30.5 | 46.8 | 18.8 | 9.1 | 86.9 | 23.2 |

## Per joint: Step adjustment time (s)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 2.12 | 2.17 | 2.31 | 1.42 | 4.60 | 1.31 |
| PID, this work | NaN | 2.57 | 2.32 | 2.32 | 3.84 | 1.02 |
| FOPID, paper figures | 2.38 | 1.90 | 2.00 | 1.35 | 2.71 | 1.34 |
| FOPID, this work | 1.89 | 1.81 | 1.88 | 1.60 | 4.20 | 1.22 |

## Per joint: Step peak time (s)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 1.65 | 1.35 | 1.52 | 1.29 | 1.79 | 1.04 |
| PID, this work | 1.55 | 1.33 | 1.42 | 1.61 | 1.67 | 1.03 |
| FOPID, paper figures | 1.52 | 1.34 | 1.95 | 1.29 | 1.58 | 1.10 |
| FOPID, this work | 1.49 | 1.30 | 1.32 | 1.53 | 1.10 | 1.15 |

## Per joint: Sine MSE (rad^2)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 7.82e-02 | 3.12e-03 | 1.58e-02 | 1.34e-03 | 3.69e-02 | 4.43e-04 |
| PID, this work | 3.55e-02 | 2.27e-03 | 1.71e-03 | 1.51e-03 | 8.16e-02 | 6.53e-09 |
| FOPID, paper figures | 1.84e-02 | 1.91e-03 | 6.67e-03 | 1.17e-03 | 2.44e-02 | 4.06e-04 |
| FOPID, this work | 1.75e-02 | 1.87e-03 | 1.24e-03 | 8.44e-04 | 4.08e-02 | 7.10e-04 |

## Per joint: Sine sum |tau| (Nm)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 8.141e+04 | 2.819e+04 | 1.95e+04 | 8347 | 1467 | 5.803 |
| PID, this work | 2.136e+04 | 2.694e+04 | 1.223e+04 | 1884 | 588.2 | 162 |
| FOPID, paper figures | 1.442e+05 | 3.271e+04 | 3.338e+04 | 1.225e+04 | 1880 | 10.04 |
| FOPID, this work | 1.992e+04 | 2.601e+04 | 1.148e+04 | 1691 | 499.8 | 167 |

## PID gains (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 75.45 | 1504 | 696.3 | 69.67 | 4.702 | 6389 |
| Ki | 565.5 | 3041 | 71.11 | 188.2 | 0.6566 | 1305 |
| Kd | 137.2 | 102.8 | 113.9 | 1.024 | 0.4189 | 10.2 |
| lambda | 1 | 1 | 1 | 1 | 1 | 1 |
| mu | 1 | 1 | 1 | 1 | 1 | 1 |

## FOPID gains (both experiments)

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| Kp | 466.5 | 0.205 | 0.07599 | 124.6 | 2.135 | 28.46 |
| Ki | 348.2 | 2206 | 54.68 | 11.92 | 2.285 | 0.00922 |
| Kd | 49.95 | 645 | 591.5 | 0.4691 | 1.474 | 0.1511 |
| lambda | 1.663 | 0.6526 | 1.95 | 0.3801 | 0.7206 | 0.05007 |
| mu | 1.425 | 0.5509 | 0.3845 | 1.454 | 0.7628 | 1.818 |
