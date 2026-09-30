# Issues found in the source paper

> **Audit note (branch `audit`, see `docs/audit_report.md`).** The published curves have since
> been extracted exactly from the PDF's vector graphics, which corrects parts of this document:
>
> * **Section 1 is withdrawn.** The paper logged its signals every 0.01 s, so its torque sums are over
>   501 samples, not 5001. The "physical floor" for Table-2 lengths is therefore ~5.8e3, not 5.79e4,
>   and nothing contradicts Table 2. The published torques are in any case dominated by numerical
>   spikes (audit report 3.4). The audited code uses Table 2's lengths as printed.
> * **Section 2, figure digitisation:** superseded by the exact extraction. The overshoots from the
>   figures are 59.1 % (PID) and 34.5 % (FOPID), and the MSEs 2.263e-2 and 8.83e-3.
> * **Section 3, torque metric and time origin:** now resolved from the figures (0.01 s grid, times
>   from t = 0). The settling band that best fits Table 3 is 5 %.
> * **New findings**, in the audit report: Table 4's torque column is a permutation of Fig. 19
>   (3.3); the torque figures are not outputs of the control law (3.4); the joint-1 step and sine
>   curves are mutually inconsistent under the paper's own model (4).


Jiang, Zhang & Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved FOPID*, IJDC 13:137 (2025).

Found while reproducing the paper from scratch. Two framing points first, because both matter if the findings are challenged:

* The paper **says UR5** (Sect. 2.3) but **tabulates UR10 link lengths** (Table 2) — not the other way round.
* The tables are **not uniformly wrong**. Table 4's MSE values agree with the paper's own figures almost exactly. The problems sit in the geometry and in Table 3.

---

## 1. Table 2 mixes two robots, which makes Table 4 physically unreachable

Table 2 lists lengths of 0.128, 0.612, 0.571, 0.164, 0.115, 0.092 m. These are the **UR10** DH values (0.1273, 0.612, 0.5723, 0.1639, 0.1157, 0.0922) to within about 1%. The real UR5 values (0.089, 0.425, 0.392, 0.109, 0.095, 0.082) are roughly 30% smaller and do not match. The masses, however, sum to 19.1 kg, which is UR5-scale (a real UR5 is about 17 kg, a UR10 about 28 kg); per link they match neither robot.

Because torque scales with link length, this is not a cosmetic slip. The torque needed to track the paper's own sine reference **perfectly** — an absolute lower bound that no controller can beat — is:

| Geometry | Σ\|τ\| for perfect tracking |
|---|---:|
| Table-2 (UR10) lengths | **5.79e4** |
| Real UR5 lengths | 2.42e4 – 2.88e4 |
| *Paper's reported values* | *PID 3.74e4, FOPID 2.57e4, FBPA 2.32e4* |

With the tabulated arm, all three published torques lie **below the physical floor**. No gain set can produce them. With real UR5 lengths they become attainable, and a PID fitted only to the Table 3 step numbers then predicted the Table 4 torque to within −5% to +12% without ever being shown it.

Conclusion: the simulated arm behaved like the UR5 named in the text, while Table 2 prints UR10 lengths. Either the table is wrong or the torque metric is not what it claims to be. This is why our reproduction uses UR5 geometry with the Table-2 masses and inertias.

## 2. The figures disagree with Table 3

Digitised from the paper's own curves; the digitiser is calibrated on the red reference line, which reads y = 1.000 ± 0.003 with the step edge at t = 1.00 ± 0.01 s.

| Quantity | From their figures | Their table | Gap |
|---|---:|---:|---|
| PID step overshoot | 57.8% | 54.6% | +6% |
| FOPID step overshoot | 33.6% | 31.2% | +8% |
| PID sine MSE | 2.19e-2 | 2.27e-2 | −3.5% |
| FOPID sine MSE | 8.72e-3 | 8.8e-3 | −0.9% |

Table 4 is consistent with the figures; Table 3's overshoots read 6–8% low against them.

**The sharpest contradiction needs no digitising at all.** Sect. 4 lists the per-joint FBPA-FOPID overshoots as 28.0, 20.1, 10.1, 2.5, 31.1 and 27.5%. Those average **19.9%**, while Table 3 reports **22.1%** for the same controller. The paper contradicts itself, and anyone can check it with a calculator.

**Third**, in Fig. 10 the PID curve for joint 5 settles at roughly 0.95 rad — a permanent 5% error. It therefore never enters a ±2% band, so Table 3's "adjustment time" cannot be a 2% settling time referenced to the 1 rad command for that joint. The paper never states which definition was used.

## 3. Metrics defined too loosely to reproduce

* **Torque metric**: "absolute values of the input signals were summed". A sum over samples scales with the logging rate, which is never given — halving the time step doubles the number. It is not a physical quantity.
* **Time origin**: Table 3's peak times (1.09–1.39 s) only make sense measured from t = 0 with the step applied at t = 1 s. Never stated.
* **Settling band**: never stated.
* **Fig. 12** shows step torques of ±1e9 Nm on a ~19 kg arm. That is the numerical artefact of an unfiltered derivative acting on a step, and it reveals there was no torque limit, no derivative filter and a variable-step solver — none of it documented.

## 4. The method description does not close

* "The dimension is 5" (Sect. 4) versus "a total of 30 parameters need to be optimized" (Sect. 3), for the same problem.
* Eq. 23 as printed, w = (w_max − w_min)·k/MaxIter, *increases* from 0 to 0.5, while the text calls it a linearly decreasing inertia weight.
* Eq. 21 omits q̈ from the dynamics (a typesetting slip).
* Fig. 4 shows 500 benchmark iterations against the stated MaxIter = 100.
* No controller gains, parameter bounds, friction coefficients, COM positions, solver or sample rate appear anywhere in the paper.

## 5. Severity

| Severity | Issue |
|---|---|
| **Fatal** | No gains published — results cannot be recomputed, only tested for attainability |
| **Fatal** | Geometry contradiction — with the tabulated arm, Table 4 is below the physical floor |
| **Serious** | Torque metric depends on an unpublished logging rate |
| **Serious** | Text vs Table 3 contradiction on FBPA overshoot (19.9 vs 22.1) |
| **Moderate** | Figure overshoots run 6–8% above Table 3; joint-5 settling definition |
| **Cosmetic** | Eq. 21 typo, Eq. 23 sign, "FPBA" legend typo, FOPID colour swapping between panels |

## Summary line

The paper's **method** reproduces, and its own figures are self-consistent with Table 4. But Table 2's geometry contradicts the robot named in the text, and with that geometry the reported torques are physically impossible — which is precisely why this reproduction uses UR5 link lengths.
