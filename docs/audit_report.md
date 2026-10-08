# Audit: why the published curves were not reproduced

Paper: Jiang, Zhang & Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, IJDC 13:137 (2025).
Branch: `audit`. Everything below can be re-run from the repository (commands in Sect. 7).

> **On branch `escape-orbit` (MATLAB):** this report is kept as the reference for the
> reproduction. The Python tools it cites (`tools/*.py`) and the raw data they read or write
> (`data/paper_curves/`, `data/identification/`) are not part of this MATLAB branch; they are on
> branches `brand-new-day` and `fopso-gwo-hybrid`. Their results are already built into the MATLAB
> code: the identified gains in `controller_gains.m`, the extracted curves in `data/paper_grid/`.

---

## 0. Summary

1. **Root cause of the "way-off" graphs.** The previous gains were fitted to five averaged
   numbers (Table 3 and Table 4). That is 48 free controller parameters constrained by 5 scalars,
   so the per-joint curves were never constrained at all. Against the published curves, the old
   simulation was off by up to 0.52 rad rms per joint, 0.10–0.30 rad on average (Sect. 2).
2. **The published curves are now available as data.** Figs 6–19 of the PDF are vector
   graphics, so every curve was read back exactly (0.001 rad rms accuracy) instead of digitised
   from a raster (Sect. 1). The gains were then **identified from the curves themselves**, for all
   six joints together in the fully coupled arm.
3. **Result of the new identification.** Mean rms difference to the published curves
   (PID step / sine; FOPID step / sine):
   * previous gains: 0.30 / 0.19; 0.17 / 0.10 rad;
   * one gain set per controller for both experiments (`main('shared')`), as the paper implies:
     **0.108 / 0.099; 0.070 / 0.062 rad**;
   * a separate gain set for each experiment (the default `main`; the paper does not say whether
     its experiments shared gains): **0.092 / 0.080; 0.058 / 0.051 rad** (4.3).

   Joints 2, 4 and 6 follow the published curves to 0.02–0.05 rad (FOPID). The distinctive
   features of the coupled arm are reproduced: joint 4's negative dip after the step, joint 3's
   early jump, and joint 5's fast spike followed by a slow swing. Full tables are in
   `results/summary.md` (separate gains; `main('shared')` writes `results/shared_gains/summary.md`)
   and the README.
4. **Why an exact match is not possible for every panel: the paper contradicts itself.**
   The paper's joint-1 sine curves cannot come from the same plant and controllers as its
   joint-1 step curves. This is shown exactly, without any optimiser, from the angular-momentum
   balance about the base axis (Sect. 4). It holds for any link lengths, any centre-of-mass
   placement and any non-negative friction.
5. **New inconsistencies found in the paper** (Sect. 3):
   * Table 4's torque column is a **permutation** of what Fig. 19 shows (to 0.05 %).
   * Its torque figures (Figs 12 and 19) cannot be the output of the control law acting on
     its own error curves, and they differ from each other by 4–5 orders of magnitude.
   * Table 3 deviates from its own step figures by 5–15 %, while Table 4's MSE matches its
     figures to 0.7 %.
   * All signals were logged every **0.01 s**. This also invalidates the "torque floor"
     argument that had been used to replace Table 2's link lengths.
6. **Code changes** (Sect. 5): Table-2 link lengths restored, Octave port (about 40× faster
   dynamics), metrics on the paper's 0.01 s grid, and no Control Toolbox dependency.

---

## 1. The published curves, read exactly

`tools/extract_paper_curves.py` reads every polyline of Figs 6–19 from the PDF's vector
content (PyMuPDF). It calibrates each panel on its own grid lines (the 11 vertical lines are
t = 0, 0.5, …, 5 s; the horizontal lines sit at the labelled y ticks) and writes

* `data/paper_curves/<fig>_<panel>_<series>.csv`: the raw curves;
* `data/paper_grid/<step|sine>_<PID|FOPID|FBPA>_<q|e|tau>.csv`: the same curves on the paper's
  0.01 s grid, loaded in Octave by `paper_curves.m`.

Accuracy checks:

| Check | Result |
|---|---|
| Sine reference vs sin(1.5 t) (Figs 13–18) | 0.0010 rad rms, 0.0045 max |
| Step reference level (Figs 6–11) | 1.0000 ± 0.0001 |
| Panel (a) vs panel (b), r − q_a − e_b | 0.0001–0.0018 rad rms |

The publisher's PDF pipeline stored the curves as cubic Béziers fitted to MATLAB's polylines.
They are sampled densely; the error is well below a line width.

---

## 2. Why the previous reproduction's graphs were off

The previous `controller_gains.m` was obtained by fitting 48 free numbers (six joints × Kp, Ki, Kd
for PID, plus λ and μ for FOPID) to the paper's five averaged metrics. The averages were reproduced, but
nothing tied the individual joints to the published curves.

RMS difference between the previous simulation and the published curves (rad):

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| PID step | 0.246 | 0.168 | 0.149 | 0.517 | 0.450 | 0.250 |
| PID sine | 0.281 | 0.056 | 0.125 | 0.131 | 0.192 | 0.343 |
| FOPID step | 0.205 | 0.132 | 0.122 | 0.181 | 0.276 | 0.112 |
| FOPID sine | 0.257 | 0.043 | 0.085 | 0.034 | 0.154 | 0.023 |

For scale: the curves themselves are known to 0.001 rad, and a 1 rad step with ~50 % overshoot
has an rms of ~0.9 rad.

---

## 3. What the paper's own figures reveal

All numbers in this section are computed from the extracted curves (`paper_curves.m`,
`performance_metrics.m`) and can be re-derived.

### 3.1 The signals were logged every 0.01 s

* The step reference in Figs 6–11 rises from 0 at **t = 0.990 s** to 1 at **t = 1.000 s**. A step
  logged by a variable-step solver would be vertical.
* The torque curves in Figs 12 and 19 have one vertex every 0.01 s (~500 per curve).

The paper's metrics are sums and means over these samples (3.2), so everything here is logged
and scored on the same grid.

### 3.2 Table 4 MSE: matched by the figures to 0.7 %

| | Table 4 | From Figs 13–18 (0.01 s, t = 0–5 s) |
|---|---:|---:|
| PID | 2.27e-2 | 2.263e-2 |
| FOPID | 0.88e-2 | 0.883e-2 |
| FBPA-FOPID | 0.37e-2 | 0.368e-2 |

This fixes the MSE definition: mean of e² over the 501 samples, averaged over joints.

### 3.3 Table 4 torque column is permuted relative to Fig. 19

Σ|τ| over the 501 samples, averaged over joints, from the curves as labelled in Fig. 19:

| Curve in Fig. 19 | Σ\|τ\| from the figure | Table 4 value that matches |
|---|---:|---|
| green, "PID" | 2.3153e4 | **FBPA-FOPID** 2.3154e4 |
| red, "FOPID" | 3.7412e4 | **PID** 3.7422e4 |
| black, "FBPA-FOPID" | 2.5675e4 | **FOPID** 2.5686e4 |

Every value matches to 0.05 %, but under a different label. Either Fig. 19's legend or Table 4's
torque column is wrong. By the figure, PID uses the *least* torque, not the most.

### 3.4 The torque figures are numerical artefacts, not controller outputs

* **Fig. 19 (sine):** the torques are isolated spikes of up to 1.3e4 N·m on a baseline of a few
  tens of N·m. The spikes occur **at the same instants in all six joints**. That is a
  signature of solver chattering, not of six independent joint controllers. Σ|τ| is
  dominated by the spikes.
* **Fig. 12 (step):** the torques reach −4.4e9 N·m. After the step (t > 1.2 s) they are still
  1e5–1e7 N·m, while the same controllers in the sine test produce ~1e2 N·m for comparable
  errors: a mismatch of 4–5 orders of magnitude.
* A least-squares fit of τ = Kp e + Ki ∫e + Kd ė using the paper's own error curves explains
  neither figure: R² is mostly below 0.6, and the gains come out negative
  (`tools/torque_consistency.py`).

So neither the torque curves nor the torque column of Table 4 can be a reproduction target.
Our model produces physically consistent torques, and they are much smaller.

### 3.5 Table 3 vs the step figures

| | Table 3 | From Figs 6–11 |
|---|---:|---:|
| PID overshoot | 54.6 % | 59.1 % |
| FOPID overshoot | 31.2 % | 34.5 % |
| FBPA overshoot | 22.1 % | 21.2 % |
| PID / FOPID / FBPA adjustment time, 5 % band | 2.44 / 1.89 / 1.43 s | 2.32 / 1.95 / 1.52 s |
| same, 2 % band | | 3.12* / 2.13 / 1.68 s |
| PID / FOPID / FBPA peak time | 1.39 / 1.33 / 1.09 s | 1.44 / 1.46 / 1.26 s |

\* Joint 5 under PID ends at 0.95 rad and never enters the 2 % band; it is counted at 5 s.

* Table 3 agrees with the figures only to 5–15 %, unlike Table 4 (0.7 %).
* The 5 % band fits the table better than the 2 % band for all three controllers, so the code
  uses 5 %.
* The per-joint FBPA overshoots quoted in the text (28.0, 20.1, 10.1, 2.5, 31.1, 27.5 %) are close
  to the figure (29.0, 25.0, 10.5, 2.5, 33.1, 26.9 %) but average 19.9 %, not 22.1 %.

### 3.6 The "torque floor" argument for UR5 lengths does not hold

`docs/paper_issues.md` argued that Table 2's (UR10-sized) lengths make Table 4's torques
physically unreachable: 5.79e4 is needed for perfect tracking, against the published 3.74e4.
That floor was computed as a sum over **5001** samples (1 ms). The paper sums over **501**
(0.01 s), which puts the floor at ~5.8e3, far below every published value. On top of that, the
published values are dominated by numerical spikes (3.4). So nothing in the paper contradicts
Table 2's lengths, and the audited code uses them as printed.

---

## 4. The limit: the paper's step and sine curves are mutually inconsistent

### 4.1 An exact test for joint 1

Joint 1 rotates about the vertical base axis. Nothing in the inertia matrix depends on q1, so
its equation of motion is exactly the balance of angular momentum about that axis:

    d/dt p1 = tau1,    p1 = sum_k M_1k(q) qd_k.

Integrating the PID law once gives, straight through the step,

    p1(t) = Kd e1(t) + Kp ∫e1 + Ki ∫∫e1.

p1(t) is computed from the paper's published trajectories of all six joints and needs only
first derivatives. This makes it a *linear* test of whether any gains can explain the published
joint-1 curves: no simulation, no optimiser (`tools/joint1_momentum_test.py`).

| Plant hypothesis | step: R² | sine: R² | step + sine: R² |
|---|---:|---:|---:|
| Table-2 lengths, COM distal | 0.908 | 0.575 (needs Kd = −13.6) | 0.491 |
| Table-2 lengths, COM middle | 0.901 | 0.581 (Kd < 0) | 0.490 |
| Table-2 lengths, COM proximal | 0.903 | 0.594 (Kd < 0) | 0.501 |
| UR5 lengths, COM distal / middle / proximal | 0.914 / 0.916 / 0.916 | 0.586 / 0.592 / 0.607 (Kd < 0) | 0.51 / 0.51 / 0.52 |
| + viscous and Coulomb friction (≥ 0) | unchanged | unchanged, friction → 0 | unchanged |
| decoupled, constant-inertia joint | 0.713 | 0.662 | 0.606 |

* **Step:** the coupled arm explains the published step well. Joint 1 creeps at ~1.2 rad/s,
  then spins up to 6.5 rad/s as joints 2/3 fold the arm and M11 drops ~20×, exactly as angular
  momentum conservation requires. The gains are sensible (e.g. Kp = 35, Ki = 2.2, Kd = 17).
  A decoupled joint explains it clearly worse (R² 0.71), so the paper really did simulate the
  coupled arm.
* **Sine:** in the paper's sine test, the joint-1 angular momentum reaches **32 N·m·s within
  0.3 s** while the joint-1 error is still ≤ 0.16 rad. In the step test, a full 1 rad error held
  for ~0.15 s produces only ~20–25 N·m·s. No positive gain set gives both. The best sine-only fit
  needs a negative derivative gain, and still explains only ~58 % of the variance.
* This holds for every length/COM hypothesis, with or without friction.

Consequence: whatever gains are used, the paper's joint-1 step and sine panels cannot both be
matched with the arm and controllers the paper describes.

### 4.2 All joints, in the fully coupled arm

Identification in the full closed loop (`tools/identify_gains.py`: block-coordinate CMA-ES, then
CMA-ES over all gains, then least squares), with the Table-2 plant and COM distal. RMS
difference to the published curves, per joint (rad):

| Fit | Scored on | J1 | J2 | J3 | J4 | J5 | J6 | mean |
|---|---|---|---|---|---|---|---|---|
| PID, one gain set for both experiments | step | 0.157 | 0.050 | 0.076 | 0.104 | 0.187 | 0.076 | 0.108 |
| | sine | 0.143 | 0.050 | 0.118 | 0.046 | 0.218 | 0.021 | 0.099 |
| PID, fitted to the step curves only (4.3) | step | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | 0.092 |
| PID, fitted to the sine curves only (4.3) | sine | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | 0.080 |
| FOPID, one gain set for both experiments | step | 0.095 | 0.034 | 0.041 | 0.051 | 0.141 | 0.055 | 0.070 |
| | sine | 0.062 | 0.032 | 0.076 | 0.017 | 0.157 | 0.030 | 0.062 |

* Joints 2, 4 and 6 are reproduced closely in both experiments. The residual concentrates on
  joint 1 (proven inconsistent, 4.1), joint 5 and the joint-3 sine.
* Fitting each experiment separately helps only moderately (PID: step 0.108 → 0.092, sine 0.099 → 0.080; 4.3).
  So for PID even a single experiment cannot be matched exactly with the paper's plant and
  controller. What remains depends on unpublished simulation details (solver, fractional-operator
  block, derivative realisation) that the curves cannot pin down.
* In the coupled arm, joint 1 cannot even reach the accuracy the isolated test allows (0.047 rad
  on the step). Its spin-up depends on how accurately joints 2 and 3 fold the arm, i.e. on
  M11(q2, q3). A 1,690-point grid search over joint 1's PID gains found none better than the identified
  ones (best grid point: 0.124 rad total vs 0.1195).
* FOPID fits better than PID on every joint and experiment except the joint-6 sine: the two
  orders per joint add freedom.
* **The gains are not unique.** FOPID gain sets that differ by orders of magnitude on some joints
  give nearly the same curves (0.0805 vs 0.0785 rad total; `data/identification/FOPID_shared*.json`). Two orders
  sit at the search bounds (λ3 = 1.95, λ6 = 0.05). As before, the gains show that the curves are
  attainable with this model; they are not claimed to be the authors' gains.

**Model selection** (PID, both experiments, same search budget): COM distal 0.121 rad, COM middle
0.124, UR5 lengths with COM distal 0.124, COM proximal 0.158 (a poorer local optimum on joint 5).
Lengths and COM placement are therefore not what limits the reproduction, and the paper's
Table 2 is kept as printed.

**Gravity:** with g = 9.81 and the identified gains, joints 2, 3 and 5 would sag 0.14–0.16 rad
during the first second. The paper's curves are flat to < 1e-4 rad before the step, so gravity
was absent or exactly compensated in the paper's simulation, and g = 0 here.


### 4.3 Separate gains for the step and sine experiments

The paper describes one PID and one FOPID controller, but it never says that both experiments
used the same gains. So each experiment was also fitted with its own gain set: same
identification, residuals of the other experiment switched off (`--weights 1,0` / `0,1`), two
starts per fit, best kept. `main` now uses these per-experiment sets by default, and
`main('shared')` keeps one set per controller (results in `results/shared_gains/`).

RMS difference to the published curves, per joint (rad):

| Controller | Experiment | Gains | J1 | J2 | J3 | J4 | J5 | J6 | mean |
|---|---|---|---|---|---|---|---|---|---|
| PID | step | shared | 0.157 | 0.050 | 0.076 | 0.104 | 0.187 | 0.076 | 0.108 |
| | | **separate** | 0.168 | 0.083 | 0.049 | 0.046 | 0.142 | 0.065 | **0.092** |
| PID | sine | shared | 0.143 | 0.050 | 0.118 | 0.046 | 0.218 | 0.021 | 0.099 |
| | | **separate** | 0.124 | 0.064 | 0.105 | 0.025 | 0.143 | 0.021 | **0.080** |
| FOPID | step | shared | 0.095 | 0.034 | 0.041 | 0.051 | 0.141 | 0.055 | 0.070 |
| | | **separate** | 0.118 | 0.038 | 0.034 | 0.024 | 0.077 | 0.054 | **0.058** |
| FOPID | sine | shared | 0.062 | 0.032 | 0.076 | 0.017 | 0.157 | 0.030 | 0.062 |
| | | **separate** | 0.050 | 0.046 | 0.069 | 0.016 | 0.108 | 0.019 | **0.051** |

* Separate gains lower all four means by 15–19 %. Most of the gain is on joints 3–5, e.g. the
  FOPID step for joint 5 (0.141 → 0.077) and joint 4 (0.051 → 0.024).
* **Joint 1's step does not improve** (PID 0.157 → 0.168, FOPID 0.095 → 0.118). The objective is
  the total rms, and it gives up a little on joint 1 to gain more on the others. The two PID
  step-only fits reached the same optimum from different starts, one of them seeded with the
  joint-1 gains of the momentum test, so this is the limit, not a search failure. In the coupled
  arm, joint 1's spin-up depends on joints 2/3 folding the arm exactly as in the paper.
* The joint-1 sine improves for FOPID (0.062 → 0.050). The PID value, 0.124, is the floor that 4.1
  predicts: no PID explains more than ~58 % of that curve.
* **Numerical conditioning.** Left unconstrained, the sine-only FOPID fit (from both starts)
  settled on joint-5 orders λ ≈ 1.9, μ ≈ 1.75. That drives the coupled loop into a chattering
  regime where a 1e-12 relative change of any gain moves the response by ~2e-3 rad, so the result
  would differ between Octave and MATLAB, or between versions. `tools/check_twin.m` caught it
  (Octave vs Python: 4e-3 rad instead of ~1e-14). The identification now has a `--robust` mode:
  every candidate is simulated again with gains × (1 + 1e-10), and loops whose response moves by
  more than a tolerance are penalised. The FOPID sine set in use was identified with a tolerance
  of 1e-8 rad. It fits as well as the ill-conditioned optimum (0.0602 vs 0.0603 rad total) and,
  like the other five sets, moves by only ~1e-9 rad for gain changes of 1e-10 to 1e-14. Each
  set's measured sensitivity is stored in its JSON; the rejected set is kept as
  `data/identification/FOPID_sine_unconstrained.json`.
* So separate gains help, by about a sixth, but they are not the missing ingredient. The
  joint-1 inconsistency (4.1) and the unpublished simulation details remain. The paper does not
  say whether its authors used separate gains, and an improvement this modest leaves both
  readings plausible.

---

## 5. Audit of the code against the paper

| Item | Paper | Previous code | Audited code |
|---|---|---|---|
| Plant equation | Eq. 21: M q̈ + C q̇ + G + τ_f = τ | same (RNEA) | same, with the friction term τ_f = f_c sgn(q̇) + b q̇ now in the code (coefficients 0, none published) |
| Masses, inertias | Table 2 | Table 2 | Table 2 |
| Link lengths | Table 2 (0.128, 0.612, 0.571, 0.164, 0.115, 0.092 m) | **replaced by real UR5 values** | **Table 2, as printed** (3.6) |
| Kinematic structure | UR5 (Fig. 3) | UR5 DH | UR5 DH (standard) |
| Centre of mass | not given | at the proximal joint axis | at each DH frame origin (Robotics Toolbox default); the alternatives fit no better (4.2); switchable in `robot_params` |
| Gravity | in the model (G(q)) | g = 0 | g = 0 (the published responses are exactly flat before the step); switchable |
| Controller | Eq. 9, six independent FOPIDs, λ, μ > 0 | same | same |
| Fractional operator | not specified | Oustaloup N = 5, [1e-3, 1e3] | same (FOMCON defaults) |
| Gains | **not published** | fitted to 5 averaged metrics | **identified from the published curves**: one set per experiment (default, 4.3), or one set for both experiments (`main('shared')`) |
| Step input | 1 rad at t = 1 s (Figs 6–11) | same | same |
| Sine input | sin(1.5 t) (Figs 13–18) | same | same |
| Logging / metric grid | 0.01 s (3.1) | 1 ms | **0.01 s** |
| MSE, Σ\|τ\| | Table 4 text | over 5001 samples | over the 501 samples (3.2) |
| Settling time | "adjustment time", band not given | `stepinfo`, 2 % | own implementation, 5 % band (3.5) |
| Toolboxes | Simulink | Control System Toolbox | none (Octave core) |
| FBPA optimiser | Sect. 3 | not implemented | not implemented in the audit; added on branch `brand-new-day` (`fbpa.m`, with the paper's Sect. 4 settings), where the FBPA-FOPID gains are also identified from its curves, like PID and FOPID here |

Octave port:
* `robot_dynamics.m` runs all seven Newton–Euler recursions (six columns of M plus the bias
  vector h) as one batched 3×7 recursion with inlined cross products. That is about 40× faster
  in Octave than the per-column version, and verified against an independent Jacobian inertia
  matrix to 1e-15 and against energy conservation to 1e-14 (`tools/verify_dynamics.m`).
* `exportgraphics` → `print`; `stepinfo` → own implementation; `xticks` → `set(gca,'XTick',…)`.
* The Python twin used for identification (`tools/ur_twin.py`) and the Octave code agree to
  10 decimal places on a full closed-loop run.

---

## 6. What would be needed for an exact reproduction

From the authors (the paper's data statement: "available from the corresponding author upon
reasonable request"):

1. the controller gains Kp, Ki, Kd, λ, μ of all six joints for PID and FOPID, and whether
   the same gains were used in the step and sine experiments;
2. the Simulink model: solver and step size, fractional-operator block and its settings,
   centre-of-mass positions, gravity and friction settings;
3. clarification of Table 4's torque column versus Fig. 19 (3.3) and of Table 3 versus Figs 6–11
   (3.5).

---

## 7. How to re-run

```
octave --eval main                         # the reproduction (per-experiment gains): results/
octave --eval "main('shared')"             # one gain set per controller: results/shared_gains/
octave --eval "addpath tools; verify_dynamics"

python3 tools/extract_paper_curves.py paper.pdf   # re-extract the published curves
python3 tools/joint1_momentum_test.py             # Sect. 4.1
python3 tools/torque_consistency.py               # Sect. 3.4
python3 tools/identify_gains.py PID   --out fit_PID.json                  # gain identification, both experiments
python3 tools/identify_gains.py FOPID --weights 1,0 --out fit_step.json  # step curves only (0,1: sine only)
```

The Python tools need numpy, scipy, numba, cma and pymupdf. Octave needs no packages.
