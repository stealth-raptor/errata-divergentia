# Trajectory tracking of a 6-DOF robotic arm: PID and FOPID

Octave reproduction of the simulation results in

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025). DOI 10.1007/s40435-025-01620-x

The arm model, both controllers and both experiments of the paper are built from scratch. The
controller gains, which the paper does not publish, are **identified from the paper's own
published curves**. These were read exactly from the vector graphics of the PDF.

**Branch `audit`:** a line-by-line audit against the paper, the reasons the published graphs
could not be matched before, and several inconsistencies found in the paper itself are in
[`docs/audit_report.md`](docs/audit_report.md).

## Run it

```
octave --eval main
```

It runs in a few minutes in Octave; no packages are needed, and it also runs in MATLAB. It
simulates both controllers through both experiments and compares them with the paper in three
ways:
1. the published Tables 3 and 4;
2. the same metrics recomputed from the paper's own figures, which do not fully agree with its
   tables;
3. every published curve, joint by joint.

Outputs: `results/summary.md`, the figure set, and `results/simulation_results.mat`.

Checks:

```
octave --eval "addpath tools; verify_dynamics"   # dynamics vs Jacobian formula, energy conservation
octave --eval "addpath tools; check_twin"        # Octave model == Python identification twin
```

### Figures

| File | Corresponds to | Content |
|---|---|---|
| `fig06…fig11_jointN_step.png` | Figs 6–11 | step response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig12_step_torque.png` | Fig 12 | joint driving torques, step response (3×2 grid) |
| `fig13…fig18_jointN_sine.png` | Figs 13–18 | sine response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig19_sine_torque.png` | Fig 19 | joint driving torques, sine response (3×2 grid) |
| `compare_step.png`, `compare_sine.png` | Figs 6–11, 13–18 | every published curve (black) overlaid on this work (dashed), with the rms difference per joint |

Colours are the paper's own: reference ("dir") red, PID green, FOPID blue. In the sine error
panels and in Fig 19 the paper draws FOPID in red, which is kept. All signals are logged every
0.01 s, as in the paper.

## Result

Every published curve of Figs 6–11 and 13–18 was read from the PDF, and the gains were
identified so that the simulation matches them: one gain set per controller, for both
experiments, as in the paper.

**Match to the published curves** (rms of q_this work − q_paper, rad):

| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean | previous gains |
|---|---|---|---|---|---|---|---|---|---|
| PID | step | 0.157 | 0.050 | 0.076 | 0.104 | 0.187 | 0.076 | **0.108** | 0.297 |
| PID | sine | 0.143 | 0.050 | 0.118 | 0.046 | 0.218 | 0.021 | **0.099** | 0.188 |
| FOPID | step | 0.095 | 0.034 | 0.041 | 0.051 | 0.141 | 0.055 | **0.070** | 0.171 |
| FOPID | sine | 0.062 | 0.032 | 0.076 | 0.017 | 0.157 | 0.030 | **0.062** | 0.099 |

**Tables 3 and 4.** "Paper figures" is the same metric recomputed from the paper's own published
curves. The paper's tables and figures do not fully agree with each other.

| Metric | PID: paper table | PID: paper figures | PID: this work | FOPID: paper table | FOPID: paper figures | FOPID: this work |
|---|---:|---:|---:|---:|---:|---:|
| Step overshoot | 54.6 % | 59.1 % | 49.8 % | 31.2 % | 34.4 % | 35.9 % |
| Step adjustment time (5 %) | 2.44 s | 2.32 s | 2.84 s | 1.89 s | 1.95 s | 2.10 s |
| Step peak time | 1.39 s | 1.44 s | 1.43 s | 1.33 s | 1.46 s | 1.32 s |
| Sine MSE | 2.27e-2 | 2.26e-2 | 2.04e-2 | 8.8e-3 | 8.8e-3 | 1.05e-2 |
| Sine Σ\|τ\| | 3.74e4 | 2.32e4 | 1.05e4 | 2.57e4 | 3.74e4 | 9.96e3 |

Why an exact match of every panel is not possible (details in the audit report):

* **The paper contradicts itself.** Its joint-1 step and sine curves cannot come from the same arm
  and controllers. This is shown exactly from the angular-momentum balance about the base axis,
  for any link lengths, centre of mass or friction (audit 4.1).
* **The torque figures are numerical artefacts.** Figs 12 and 19 are not outputs of the control
  law and differ from each other by 4–5 orders of magnitude, and Table 4's torque column is a
  *permutation* of Fig. 19 (audit 3.3–3.4). The torque metric therefore cannot be reproduced.
  Ours is the physically consistent value.
* The paper publishes no gains, no solver settings and no fractional-operator settings. The
  identified gains are not unique; they show the curves are attainable, not that they are the
  authors' gains.

## Architecture

```
main.m                      runner: both controllers x both experiments, tables and figures
│
├── robot_params.m          arm parameters: Table 2 of the paper, UR DH, COM / gravity / friction options
├── robot_dynamics.m        M(q) qdd + C(q,qd) qd + G(q) + tau_f = tau  (Eq. 21), batched Newton-Euler
│
├── controller_gains.m      gains identified from the published curves
├── fopid_controller.m      builds the six joint controllers
├── fopid_update.m          one control step: u = Kp e + Ki D^-lambda e + Kd D^mu e  (Eq. 9)
├── fractional_operator.m   Oustaloup approximation of s^alpha, discretised
│
├── simulate_closed_loop.m  controller at 1 kHz + RK4 integration of the plant, logged at 0.01 s
├── performance_metrics.m   the metrics of the paper's Tables 3 and 4
├── paper_curves.m          the paper's published curves, in the same format as a simulation
└── plot_paper_figures.m    the figure set, in the paper's own layout, plus overlays

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
| Controller gains | identified from the 24 published curves of each controller (step and sine together) | **the paper publishes no gains** |

## Requirements

Octave ≥ 6 (tested with 8.4) or MATLAB. No toolboxes.
The Python tools in `tools/` (only needed to re-extract curves or re-identify gains) use numpy,
scipy, numba, cma and pymupdf.
