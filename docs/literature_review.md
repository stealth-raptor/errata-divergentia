# Metaheuristic tuning of FOPID controllers — literature for the next step

Context: our reproduction fixed the plant and reproduced the paper's PID and FOPID baseline. The next step is to **tune the FOPID gains with the Grey Wolf Optimizer (GWO)** and compare against that baseline (and against the paper's FBPA-FOPID figures). These ten papers cover the algorithm itself, its use on FOPID and on manipulators, and the competing metaheuristics we would be expected to compare with.

Ordering: GWO first (our proposed method), then the other metaheuristics that any reviewer will ask about.

---

## A. Grey Wolf Optimizer and its use on FOPID / robots

### 1. Mirjalili, Mirjalili & Lewis (2014) — the original Grey Wolf Optimizer
*Advances in Engineering Software* 69:46–61. DOI 10.1016/j.advengsoft.2013.12.007

This is the source of the algorithm we intend to use. GWO imitates the social hierarchy of grey wolves: the best three solutions found so far are labelled alpha, beta and delta, and every other wolf (omega) updates its position by averaging three candidate positions computed relative to those leaders. Hunting is modelled in three phases — searching for prey, encircling it and attacking — and the balance between exploration and exploitation is governed by a single coefficient **a** that decreases linearly from 2 to 0 over the run.

For us the practical appeal is that GWO has only one tuning constant (the schedule of **a**) and no velocity term, unlike PSO which needs inertia weight and two learning factors. That makes results easier to defend: there is little room to "tune the tuner". The linear decrease of **a** is also directly comparable to the linearly decreasing inertia weight that the paper's FBPA uses, which makes a like-for-like comparison against their algorithm straightforward.

Relevance: this is the method reference we will cite for the implementation, and the baseline against which any "improved GWO" variant would have to be justified.

### 2. Oglah, Saleh & Shallal (2025) — GWO-tuned fractional PID for a robotic manipulator
*Tikrit Journal of Engineering Sciences* 32(3):1–8. DOI 10.25130/tjes.32.3.9

This is the closest published analogue to what we are proposing: a fractional-order PID controller whose gains are tuned by GWO, applied to joint tracking of a **PUMA 560** manipulator — a 6-DOF industrial arm of the same class as the UR5 in our study. The authors model the arm with only partial knowledge of its dynamics and let GWO absorb the modelling uncertainty by fine-tuning the controller gains, benchmarking the outcome against a classical PID.

Methodologically it follows the same recipe we would use: encode the FOPID parameters (gains plus the two fractional orders) as the search vector, simulate the closed loop for each candidate, and score it with a time-domain error criterion. The reported conclusion is that the GWO-tuned fractional controller gives a superior response with short execution time compared with conventional strategies.

Relevance: it establishes precedent — GWO on FOPID for a 6-DOF industrial arm is an accepted, publishable combination — and it is the paper to cite when justifying our choice of algorithm. Note that the article's landing page does not expose the full numerical tables, so we should not quote specific overshoot or settling-time figures from it without the full text.

### 3. Choubey & Ohri (2021) — GWO for a 6-DOF parallel manipulator
*Robotica* 39(3):411–427

Cited by the paper we reproduced (its reference [12]), this work applies GWO to optimal trajectory generation for a 6-DOF parallel manipulator. The objective combines tracking error with motion-quality terms — velocity, joint acceleration ripple and joint jerk — so that the resulting path is not merely accurate but also smooth and continuous, with fast convergence reported as a key property of the GWO search.

The important methodological point for us is the **composite objective**. Our reproduction showed that optimising a single error criterion (ITAE) drives the search towards gains that are numerically excellent but physically unattractive: joints that oscillate violently or never settle. This paper is the precedent for adding smoothness or effort terms to the fitness so the optimiser cannot buy accuracy with unusable behaviour.

Relevance: direct support for using GWO in a 6-DOF robotic setting, and a template for designing a fitness function richer than plain ITAE. Being already in the paper's own bibliography, it is also a natural thing for our professor to recognise.

### 4. Yamparala, Lakshminarasimman & Rao (2022) — GWO-tuned FOPID for a DFIG wind system
*International Journal of Renewable Energy Research* 12(4):2111–2120

This is reference [7] of the paper we reproduced — the work its authors cite as evidence that FOPID plus a swarm optimiser is a proven combination. The controller is a fractional-order PID for a doubly-fed induction generator in a wind energy conversion system, with GWO selecting the five FOPID parameters, and the design is validated against conventional tuning under changing operating conditions.

Its value to us is argumentative rather than technical: the original authors justified their own FBPA by pointing at GWO-tuned FOPID results in the power-systems literature. If we now apply GWO to their robot model, we are using the very method their motivation rests on, which makes the comparison difficult to dismiss.

Relevance: cite it when explaining *why GWO* — it is the algorithm the source paper itself invokes as the state of the art, applied to the same controller structure but a different plant.

### 5. Karahan & Karci (2025) — hybrid GWO–PSO for a 6-DOF manipulator
*Robotica* 43(3):1110–1139

The authors design a fractional-order fuzzy PID sliding-mode controller for a nonlinear, multi-input multi-output **6-DOF** manipulator, and tune its parameters with a **hybrid GWO–PSO** algorithm that combines the leader-following of GWO with the velocity memory of PSO. Several controller variants (PID, FOPID, fuzzy and sliding-mode combinations) are tuned by the same optimiser and compared on the same plant.

The evaluation protocol is the part worth copying: the tuned controllers are judged not only on nominal tracking but also on **disturbance rejection and payload-mass uncertainty**, with the fractional-order fuzzy variant reported as the most consistent performer. That is exactly the kind of robustness evidence a reviewer asks for once the nominal numbers look good.

Relevance: shows the natural next move after plain GWO — hybridising it — and gives us a ready-made robustness test protocol for the UR5 model. It is also a caution: on a 6-DOF arm the literature has already moved to hybrids, so plain GWO should be presented as a controlled comparison, not as a novelty.

### 6. Ali, Saleh & Abbas (2025) — modified FOPID for a rehabilitation exoskeleton, tuned by a GWO hybrid
*Frontiers in Robotics and AI*. DOI 10.3389/frobt.2025.1667688

A 2-DOF lower-limb exoskeleton (hip and knee, Euler–Lagrange model) is controlled by a *modified* FOPID that adds a nonlinear error term to the standard structure. The gains are tuned by a hybrid of Improved Elk Herd Optimization with GWO and Multi-Verse Optimization, and the comparison set includes FOPID tuned by PSO–GWO and by GWO–MVO.

Two details are directly transferable. First, the fitness is **ITAE with explicit penalties on overshoot, settling time and torque bounds** — the paper we reproduced uses bare ITAE, which is precisely why its optimiser had no reason to keep torque sensible. Second, the reported gains are large: hip settling time falls from 6.998 s to 0.430 s and knee from 7.150 s to 0.829 s, with overshoot driven to zero and ITAE reduced by roughly 80–90%, while torques stay bounded.

Relevance: the strongest argument in this list for adding constraint terms to our fitness function, and a recent demonstration that GWO-family hybrids on FOPID are an active 2025 research line rather than a settled topic.

---

## B. Competing metaheuristics on the same controller structure

### 7. Ahmed, Eltayeb, Alyazidi, Imran, Sheltami & El-Ferik (2024) — improved PSO for FOPID on a manipulator
*Results in Engineering* 24:103089. DOI 10.1016/j.rineng.2024.103089

The authors tune FOPID gains for a robotic manipulator with an **improved PSO** that adds chaos-based initialisation and adaptive mutation to avoid premature convergence and local minima, and benchmark it against conventional PSO under several objective functions.

This is the most direct competitor to our GWO plan, on the same controller and the same class of plant and published very recently. Its use of *several* objective functions rather than one is notable: it shows that the choice of cost function is treated as part of the experimental design, not a detail.

Relevance: the natural second algorithm to implement after GWO, so the comparison is GWO vs an improved-PSO baseline rather than GWO vs nothing. Note that our reproduction already found that random initialisation in a wide gain space is mostly unstable — chaos-based initialisation is one published remedy we could adopt.

### 8. Eltayeb, Ahmed, Imran, Alyazidi & Abubaker (2024) — GA-tuned FOPID vs PID for a robotic arm
*Automation* 5(3):230–245. DOI 10.3390/automation5030014

A genetic algorithm tunes both a PID and an FOPID controller for a nonlinear robotic arm, and the two are compared under four cost functions: ISE, IAE, ITAE and ITSE. The structure of the study is almost exactly the comparison we are running, with GA in place of GWO.

The cost-function sweep is the lesson here. In our own work the ITAE-only fitness produced controllers with good averages and poor per-joint behaviour, and this paper shows the standard way to handle that: report results under each criterion and let the comparison be explicit rather than hidden in one arbitrary choice.

Relevance: a template for presenting a PID-vs-FOPID comparison honestly, and a reminder that our claims should be stated per cost function. It also gives us a third algorithm (GA) if the professor asks for more than one competitor.

### 9. Kumar & Suhag (2019) — whale optimisation for FOPID
*International Journal of Bio-Inspired Computation* 13(4). DOI 10.1504/IJBIC.2019.10021709

This is reference [8] of the reproduced paper. A FOPID controller for load frequency control of a multi-source power system is tuned by the Whale Optimization Algorithm, and the WOA-tuned FOPID is reported to give the best dynamic performance in settling time and maximum overshoot among the compared designs, while remaining robust to ±25% parameter variation.

The robustness test is the transferable idea: performance is re-checked after perturbing the plant parameters. Our reproduction is unusually well placed to do this, because we already know which plant assumptions are uncertain — link lengths, centre-of-mass placement, friction — so a ±25%-style sweep is a natural robustness section.

Relevance: together with entry 4, this is the evidence base the original authors used to motivate swarm-tuned FOPID. Re-using its robustness protocol strengthens our own comparison at little cost.

### 10. Bingul & Karahan (2018) — PSO vs ABC for PID and FOPID
*Optimal Control Applications and Methods* 39(4):1431–1450. DOI 10.1002/oca.2419

An older but frequently cited study that tunes both integer-order and fractional-order PID controllers with PSO and with Artificial Bee Colony for unstable and integrating plants with time delay, judging the outcome on settling time, rise time, overshoot and steady-state error.

Its usefulness for us is that it separates two questions that are easy to conflate: *does the fractional structure help*, and *does the optimiser help*. Answering both requires the 2×2 design — {PID, FOPID} × {algorithm A, algorithm B} — which is the experiment matrix we should adopt so that any improvement we report can be attributed to the right cause.

Relevance: the methodological backbone for our comparison table, and a defence against the obvious criticism that a better result might come from the optimiser rather than from the controller structure.

---

## What this means for our next step

**Proposed experiment.** Tune the six joint FOPID controllers of our UR5 model with GWO: 30 decision variables (Kp, Ki, Kd, λ, μ per joint), bounds stated explicitly, population and iteration count matched to the paper's FBPA settings (30 wolves, 100 iterations) so the comparison is fair on evaluation count.

**Fitness.** Start with plain ITAE to match the paper, then repeat with ITAE plus penalties on overshoot and torque, following entry 6 — our reproduction showed bare ITAE is what allows physically unattractive solutions. Reporting both is more convincing than picking one.

**Comparison set.** GWO against (i) our fitted FOPID baseline, (ii) the paper's published FBPA-FOPID numbers, and if time allows (iii) a PSO baseline, following entries 7, 8 and 10. Several random seeds each, with convergence curves and the spread across seeds, since our earlier runs showed seed-to-seed variation matters.

**Robustness.** Re-evaluate the tuned controllers under perturbed plant parameters, as in entries 5 and 9 — this is where our documented uncertainty about link geometry, COM and friction becomes a strength rather than a caveat.

---

### A note on sources

Entries 1, 3, 4, 7, 8, 9 and 10 were verified from publisher or indexing records (title, authors, venue, year, volume/pages, DOI). Entries 2, 5 and 6 were read from the article landing pages; only entry 6 (open access) provided full numerical results, which is why specific figures are quoted there and not for entries 2 and 5. Several publishers block automated access, so where a full text was not reachable, no quantitative claims are made.

Sources:
- [Grey Wolf Optimizer — Advances in Engineering Software](https://dl.acm.org/doi/abs/10.1016/j.advengsoft.2013.12.007)
- [Design of Fractional–PID Controller Based on Grey Wolf Optimization for Robotic Manipulator](https://doi.org/10.25130/tjes.32.3.9)
- [Optimal Trajectory Generation for a 6-DOF Parallel Manipulator Using Grey Wolf Optimization Algorithm](https://www.cambridge.org/core/journals/robotica/article/abs/optimal-trajectory-generation-for-a-6dof-parallel-manipulator-using-grey-wolf-optimization-algorithm/0E8B7694B790122914EF1E95455E41AB)
- [Optimal design of FoPID controller for DFIG based wind energy conversion system using Grey-Wolf optimization algorithm](https://www.semanticscholar.org/paper/Optimal-design-of-FoPID-controller-for-DFIG-based/75fba034ff89908469abd7668e460aed72567330)
- [Design of robust fractional order fuzzy PID sliding mode controller ... for a 6-DOF robotic manipulator](https://www.cambridge.org/core/product/7588BEAB3A0ECE3570E66319487B27B5)
- [Design of modified fractional-order PID controller for lower limb rehabilitation exoskeleton robot](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1667688/full)
- [Improved particle swarm optimization for fractional order PID control design in robotic manipulator system](https://www.sciencedirect.com/science/article/pii/S2590123024013446)
- [Comparative Analysis: Fractional PID vs. PID Controllers for Robotic Arm Using Genetic Algorithm Optimization](https://doi.org/10.3390/automation5030014)
- [Whale optimisation algorithm tuned fractional order PID controller for load frequency control](https://doi.org/10.1504/IJBIC.2019.10021709)
- [Comparison of PID and FOPID controllers tuned by PSO and ABC algorithms](https://onlinelibrary.wiley.com/doi/abs/10.1002/oca.2419)
