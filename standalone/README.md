# One file per method

Each method of the work is isolated in its own MATLAB file. **Every file is self-contained:**
it carries its own copy of the arm model, the controller, the simulation and the paper's
metrics, plus, where relevant, the optimiser and its cost function. A file can be copied
anywhere and run on its own; nothing else from the repository is needed, and no toolbox.

| File | Method | Of the three asked for |
|---|---|---|
| [`PID_conventional.m`](PID_conventional.m) | The conventional controller: six PID joint controllers (the paper's Eq. 9 with λ = μ = 1) | **conventional controller** |
| [`FOPID_conventional.m`](FOPID_conventional.m) | The paper's FOPID (Eq. 9, Oustaloup fractional operators) with fixed gains, before any optimiser | conventional controller (fractional) |
| [`FBPA_FOPID.m`](FBPA_FOPID.m) | The paper's FBPA-FOPID: the FOPID tuned by FBPA, its fractional-order beetle antennae PSO, on its ITAE fitness (Eq. 29) | **FBPA-FOPID** |
| [`FOPSO_GWO.m`](FOPSO_GWO.m) | The FOPID tuned by FOPSO-GWO, this work's fractional-order PSO / grey-wolf hybrid: the first stage of the proposed method | (part of the proposed method) |
| [`FOPSO_GWO_CC.m`](FOPSO_GWO_CC.m) | **The proposed method:** the FOPID tuned by FOPSO-GWO-CC, FOPSO-GWO followed by cooperative coevolution with one sub-swarm per joint | **proposed algorithm** |

**If only three files are sent:** `PID_conventional.m`, `FBPA_FOPID.m` and `FOPSO_GWO_CC.m`.
`FOPSO_GWO_CC.m` contains FOPSO-GWO in full, as its first stage.

## Running them

Start MATLAB in this folder (or put it on the path) and run any of these:

```matlab
PID_conventional              % PID: both experiments, Tables 3-4 against the paper, plots
FOPID_conventional            % FOPID with fixed gains: the same
FBPA_FOPID                    % the paper's FBPA-FOPID (reproduced): the same
FBPA_FOPID('tuned')           % the FOPID tuned by this file's FBPA (stored run, seed 1)
FOPSO_GWO                     % the FOPID tuned by FOPSO-GWO (stored run, seed 6)
FOPSO_GWO_CC                  % the proposed controller (stored run, seed 7)
```

Each prints the paper's five metrics (overshoot, adjustment time, peak time, sine MSE,
sine Σ|τ|) next to the paper's values, joint by joint, and plots the joint angles and torques
of both experiments. The two tuned controllers also print:
* their whole-controller cost;
* their ITAE;
* how much of each of the 24 per-joint torque and overshoot caps they use.

Every call returns a struct with the gains, the runs and the metrics, e.g.
`r = FOPSO_GWO_CC; r.summary`.

The PID and FOPID files have a second gain set: `PID_conventional('shared')`,
`FOPID_conventional('shared')` and `FBPA_FOPID('shared')` use one gain set for both
experiments instead of one per experiment.

**Gains fitted to the paper's tables.** `PID_conventional('table')` and
`FOPID_conventional('table')` use PID and FOPID gains fitted to the paper's Tables 3 and 4
(repository: `tools/fit_table_gains.m`; README, Sect. 6, 5.4). Every metric of the tables,
the sine torque included, comes within 4 % of the paper's value:

| | Overshoot | Adjustment time | Peak time | Sine MSE | Sine Σ\|τ\| |
|---|---:|---:|---:|---:|---:|
| PID | −3.9 % | −4.0 % | +2.0 % | −4.0 % | −3.8 % |
| FOPID | −0.2 % | −1.0 % | −0.4 % | −1.1 % | −2.7 % |

These gains follow the paper's curves less closely than the default ones, because the paper's
tables disagree with its own figures.

### Re-running the optimisers

```matlab
FBPA_FOPID('tune', 1)         % FBPA from the paper's FOPID, random seed 1 (9030 evaluations)
FOPSO_GWO('tune', 6)          % FOPSO-GWO, random seed 6 (3030 evaluations)
FOPSO_GWO_CC('tune', 7)       % FOPSO-GWO-CC, random seed 7 (3030 evaluations)
FOPSO_GWO_CC('tune', 7, 4)    % a quick trial: 4 iterations instead of 100
```

* **Run time.** A full run evaluates the cost 3030 times (FBPA 9030), with two to four
  5-second simulations each. With the plain `.m` simulation that takes many hours. With
  the compiled simulation it takes about 6 minutes (FBPA about 15).
  * **To use the compiled simulation,** build it once in the repository root (`mex -setup C`,
    then `build_mex`) and keep that folder on the path. The files pick up `simulate_mex`
    automatically.
  * Try a few iterations first; the third argument sets the number of iterations.
* **Random numbers.** The stored runs were made in Octave. MATLAB's generator gives
  different numbers for the same seed, so a MATLAB re-run is a new, independent sample: it
  will not reproduce the stored controller digit for digit. What should hold is the
  statistical result (repository README, Sect. 8).
* **Output.** The tuned gains are returned (`r.gains`) and printed, ready to paste. No file
  is written.

## How the files relate to the repository

The shared parts are copies of the repository's code, with the same arithmetic in the same
order:

| In the files | Repository |
|---|---|
| `arm_parameters`, `arm_dynamics` | `robot_params.m`, `robot_dynamics.m` |
| `fopid_create`, `fopid_update`, `oustaloup` | `fopid_controller.m`, `fopid_update.m`, `fractional_operator.m` |
| `pid_create`, `pid_update` | the same FOPID code with λ = μ = 1, written as a plain PID |
| `simulate_arm` | `simulate_closed_loop.m` |
| `step_metrics`, `sine_metrics`, `itae`, `control_effort` | `performance_metrics.m`, `fopid_fitness.m`, `control_effort.m` |
| `paper_cost`, `whole_cost` | `fopid_fitness.m` with fitness `'itae'` / `'whole'`, as set by `tune_fopid_hybrid.m` |
| `search_space`, `initial_swarm` | `tune_fopid_hybrid.m`, and the initialisation in `fbpa.m`, `hybrid_fopso_gwo.m` |
| `run_fbpa` | `fbpa.m` |
| `run_fopso_gwo`, `run_fopso_gwo_cc` | `hybrid_fopso_gwo.m` (its final settings), `hybrid_fopso_cc.m` |

Two kinds of number are written into the files instead of being recomputed:
* **the gains:** identified from the paper's curves, or stored by the tuning runs;
* **the whole-controller cost's references:** the 18 per-joint torque caps, the overshoot
  cap and the effort and ITAE references, measured by the repository from its
  reproductions of the paper's three controllers. They are copied exactly from the stored
  runs, so the cost here is the cost those runs used.

Left out, because they are not part of these methods:
* checkpointing;
* parallel evaluation;
* the options that were tried and rejected during development;
* PSO and the CMA-ES hybrid.

## Verification

Each file was checked against the repository, using the repository's compiled simulation:

* **Same trajectories.** `FOPID_conventional` and `FBPA_FOPID` (both gain sets) give the
  repository's joint angles and torques with a difference of exactly 0. `PID_conventional`
  and `FOPID_conventional` reproduce every number of `results/summary.md`, per joint.
* **Same costs.** The stored controllers score exactly their stored costs:
  * `FBPA_FOPID('tuned')`: 0.0954847282015242;
  * `FOPSO_GWO`: 1.41564169274606;
  * `FOPSO_GWO_CC`: 0.733581254781147.

  The repository's own `fopid_fitness.m` gives the same values.
* **Same optimiser runs.** Re-running the optimisers from scratch with the stored random
  seeds, on the platform the stored runs were made on (GNU Octave), reproduces every stored
  run iteration for iteration. They end on the identical controller: the same gains to the
  last digit and the same cost.
  * `FOPSO_GWO_CC('tune', 7)`: 0.733581254781147;
  * `FOPSO_GWO('tune', 6)`: 1.41564169274606;
  * `FBPA_FOPID('tune', 1)`: 0.0954847282015242.

  So the files contain exactly the algorithms that produced the reported results.
* **The plain PID equals the repository's FOPID code with λ = μ = 1.** The joint angles and
  torques differ by exactly 0.
* **The files parse as MATLAB** (MISS_HIT linter, no findings).
