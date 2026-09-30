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

## Per joint: Step adjustment time (s)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 2.12 | 2.17 | 2.31 | 1.42 | 4.60 | 1.31 |
| PID, this work | 3.84 | 2.67 | 3.28 | 2.08 | 4.80 | 1.24 |
| FOPID, paper figures | 2.38 | 1.90 | 2.00 | 1.35 | 2.71 | 1.34 |
| FOPID, this work | 2.64 | 2.00 | 1.84 | 1.44 | 2.53 | 1.24 |

## Per joint: Step peak time (s)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 1.65 | 1.35 | 1.52 | 1.29 | 1.79 | 1.04 |
| PID, this work | 1.53 | 1.30 | 1.46 | 1.72 | 1.70 | 1.09 |
| FOPID, paper figures | 1.52 | 1.34 | 1.95 | 1.29 | 1.58 | 1.10 |
| FOPID, this work | 1.50 | 1.29 | 1.32 | 1.63 | 1.62 | 1.15 |

## Per joint: Sine MSE (rad^2)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 7.82e-02 | 3.12e-03 | 1.58e-02 | 1.34e-03 | 3.69e-02 | 4.43e-04 |
| PID, this work | 6.38e-02 | 2.44e-03 | 5.49e-03 | 4.82e-04 | 2.01e-02 | 9.24e-05 |
| FOPID, paper figures | 1.84e-02 | 1.91e-03 | 6.67e-03 | 1.17e-03 | 2.44e-02 | 4.06e-04 |
| FOPID, this work | 1.42e-02 | 2.51e-03 | 1.98e-03 | 6.46e-04 | 1.36e-02 | 9.58e-05 |

## Per joint: Sine sum |tau| (Nm)

| Source | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID, paper figures | 8.141e+04 | 2.819e+04 | 1.95e+04 | 8347 | 1467 | 5.803 |
| PID, this work | 2.096e+04 | 2.597e+04 | 1.239e+04 | 2700 | 544.1 | 166.9 |
| FOPID, paper figures | 1.442e+05 | 3.271e+04 | 3.338e+04 | 1.225e+04 | 1880 | 10.04 |
| FOPID, this work | 2.12e+04 | 2.468e+04 | 1.033e+04 | 3131 | 2487 | 220.6 |

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
