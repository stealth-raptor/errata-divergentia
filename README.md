# Trajectory tracking of a 6-DOF robotic arm: PID and FOPID

MATLAB reproduction of the simulation results in

> Zhou Jiang, Xiaohua Zhang, Guoquan Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, International Journal of Dynamics and Control **13**:137 (2025). DOI 10.1007/s40435-025-01620-x

The arm model, both controllers and both experiments of the paper were built from scratch and the published performance figures were reproduced.

## Run it

```matlab
main
```

About three minutes. It simulates both controllers through both experiments, prints the comparison with the paper, and writes `results/summary.md`, the figure set and `results/simulation_results.mat`.

### Figures

The figures follow the layout of the source paper and keep its numbering, so each one can be put side by side with the published version:

| File | Corresponds to | Content |
|---|---|---|
| `fig06…fig11_jointN_step.png` | Figs 6–11 | step response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig12_step_torque.png` | Fig 12 | joint driving torques, step response (3×2 grid) |
| `fig13…fig18_jointN_sine.png` | Figs 13–18 | sine response per joint: (a) position tracking trajectory, (b) tracking error |
| `fig19_sine_torque.png` | Fig 19 | joint driving torques, sine response (3×2 grid) |

Every panel labels both controllers. Colours are the paper's own: reference ("dir") red, PID green, FOPID blue — except in the error panels of the sine figures, where the paper reuses red for FOPID since no reference is drawn there. The published figures also contain an FBPA-FOPID curve, which is outside the scope of this submission.

## Result

| Metric | Paper PID | This work | Paper FOPID | This work |
|---|---:|---:|---:|---:|
| Step overshoot | 54.6 % | 54.5 % (−0.2 %) | 31.2 % | 31.5 % (+0.9 %) |
| Step adjustment time | 2.44 s | 2.53 s (+3.8 %) | 1.89 s | 1.94 s (+2.5 %) |
| Step peak time | 1.39 s | 1.33 s (−4.1 %) | 1.33 s | 1.39 s (+4.5 %) |
| Sine MSE | 2.27e-2 | 2.22e-2 (−2.3 %) | 8.8e-3 | 8.96e-3 (+1.8 %) |
| Sine torque | 3.742e4 | 3.767e4 (+0.7 %) | 2.569e4 | 2.983e4 (+16.1 %) |

Nine of the ten published values are reproduced within 5 %; the FOPID torque is 16 % high. FOPID improves on PID exactly as the paper reports: lower overshoot, shorter settling time and less than half the tracking error.

## Architecture

The code follows the physical structure of the problem: a plant, a controller, a simulation loop, and metrics.

```
main.m                      runner: both controllers x both experiments, tables and figures
│
├── robot_params.m          arm parameters: masses, inertias, UR5 DH, spatial inertias
├── robot_dynamics.m        M(q) qdd + h(q,qd) = tau, by recursive Newton-Euler
│
├── controller_gains.m      the tuned per-joint gains for PID and FOPID
├── fopid_controller.m      builds the six joint controllers
├── fopid_update.m          one control step: u = Kp e + Ki D^-lambda e + Kd D^mu e
├── fractional_operator.m   Oustaloup approximation of s^alpha, discretised
│
├── simulate_closed_loop.m  controller at 1 kHz + RK4 integration of the plant
├── performance_metrics.m   the metrics defined in the paper's Tables 3 and 4
└── plot_paper_figures.m    the figure set, in the paper's own layout
```

Call graph: `main` → `simulate_closed_loop` → (`fopid_update` → `fractional_operator` filters) and (`robot_dynamics`) → `performance_metrics`.

## The two experiments

* **Step response.** All six joints are commanded 1 rad at t = 1 s, from rest. Measured: overshoot, adjustment (settling) time and peak time, averaged over the joints (paper Table 3).
* **Sine tracking.** All six joints follow sin(1.5 t) rad for 5 s. Measured: mean squared error and summed absolute torque, averaged over the joints (paper Table 4).

## Modelling choices

The paper leaves several quantities unstated; each choice below is visible in the code and documented where it is made.

| Quantity | Choice | Reason |
|---|---|---|
| Masses, inertias | Table 2 of the paper | published |
| Link lengths | real UR5 values | Table 2 lists UR10 lengths, which contradict the UR5 the paper studies. With them the arm cannot physically produce the published torques: tracking the sine perfectly would already need 5.8e4 against the 3.7e4 reported. |
| Centre of mass | at each joint axis | not published; this is the usual default when a link is attached at its joint frame |
| Gravity | compensated | the published step responses stay exactly at zero before the step, which uncompensated gravity would not allow |
| Friction | zero | the paper gives the friction model but no coefficients |
| Fractional operator | Oustaloup, N = 5, 1e-3…1e3 rad/s | standard choice for FOPID in MATLAB/Simulink |
| Solver | RK4 at 1 ms, controller at 1 kHz | not published |
| Controller gains | fitted to the five published values | **the paper publishes no gains at all** |

## On the gains

Because no gains are published, they cannot be recomputed; they were obtained by fitting the simulation to the paper's own five performance numbers with a pattern-search optimiser. The table above therefore shows that the published results are **attainable** with this model, not that these were the authors' gains. The fit is also not unique: different gain sets reach similar averages.

One honest caveat worth stating alongside the results: the fitted controllers reach the paper's *averaged* metrics, but a few individual joints behave less tidily than the paper's per-joint figures (for example PID joint 4 overshoots strongly, and two joints do not settle within the 2 % band by 5 s). Constraining every joint to settle is possible, but then the averaged metrics drift further from the published values.

## Requirements

MATLAB R2024b. Control System Toolbox is used for `stepinfo` in the metrics; nothing else is required.
