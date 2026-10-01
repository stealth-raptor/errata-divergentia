function results = main(mode)
%MAIN  Reproduce the paper's results and compare FOPSO-GWO with its FBPA-FOPID.
%
%   results = MAIN()           PID / FOPID / FBPA-FOPID gains fitted to each experiment
%   results = MAIN('shared')   one gain set per controller for both experiments
%
%   Runs every controller through both experiments of the paper,
%     step response : 1 rad on every joint at t = 1 s
%     sine tracking : sin(1.5 t) rad on every joint
%   The controllers are
%     PID, FOPID,   the paper's three controllers, with gains identified from
%     FBPA          its published curves (CONTROLLER_GAINS); FBPA is the
%                   FBPA-FOPID
%     FOPSO_GWO     the FOPID tuned by this work's FO-PSO / grey-wolf hybrid
%                   (results/fopso_gwo_gains.mat, from TUNE_FOPID_HYBRID), when
%                   that file exists; one gain set for both experiments
%
%   results/summary.md compares them with the paper's Tables 3 and 4, with
%   the same metrics recomputed from the paper's own published curves (read
%   from the vector graphics of the PDF, see PAPER_CURVES), and with the
%   published curves themselves, and reports the torque each controller
%   needs (CONTROL_EFFORT).  If TOOLS/COMPARE_OPTIMIZERS has been run
%   (results/optimizer_runs/), it also compares the FBPA and FOPSO-GWO
%   optimisers under the same costs, seeds and budget, and plots their
%   convergence.
%
%   Saves results/summary.md, the paper's figure set (Figs 6-19), two
%   comparison figures, convergence.png and results/simulation_results.mat;
%   with 'shared', the same under results/shared_gains/.
%
%   Runtime: a few seconds with the compiled simulation (BUILD_MEX),
%   a few minutes without.
%
%   Pipeline:
%     ROBOT_PARAMS         -> arm parameters (Table 2 of the paper)
%     CONTROLLER_GAINS     -> identified and tuned gains
%     SIMULATE_CLOSED_LOOP -> closed-loop simulation (ROBOT_DYNAMICS + FOPID_*)
%     PERFORMANCE_METRICS  -> the metrics defined in the paper
%     PAPER_CURVES         -> the paper's own published curves
%     PLOT_PAPER_FIGURES   -> the figures, in the paper's own layout
%     CONTROL_EFFORT       -> the torque each controller needs
%     TUNE_FOPID_HYBRID    -> (separately) the FOPSO-GWO gains

if nargin < 1, mode = 'separate'; end
if ~any(strcmp(mode, {'separate', 'shared'}))
    error('main:mode', 'mode must be ''separate'' or ''shared''');
end
here = fileparts(mfilename('fullpath'));
cd(here);
outdir = 'results';
if strcmp(mode, 'shared'), outdir = fullfile('results', 'shared_gains'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end

P = robot_params();
controllers = {'PID', 'FOPID', 'FBPA'};
if exist(tuned_file('FOPSO_GWO'), 'file'), controllers{end+1} = 'FOPSO_GWO'; end

% published values: [overshoot %, adjustment time s, peak time s, sine MSE, sine torque]
paper.PID   = [54.6 2.44 1.39 2.27e-2 3.7422e4];
paper.FOPID = [31.2 1.89 1.33 0.88e-2 2.5686e4];
paper.FBPA  = [22.1 1.43 1.09 0.37e-2 2.3154e4];

% the paper's own curves, through the same metrics
for c = {'PID', 'FOPID', 'FBPA'}
    pstep = paper_curves(c{1}, 'step');
    psine = paper_curves(c{1}, 'sine');
    figs.(c{1}).step_metrics = performance_metrics(pstep, 'step');
    figs.(c{1}).sine_metrics = performance_metrics(psine, 'sine');
    figs.(c{1}).summary = [figs.(c{1}).step_metrics.mean, figs.(c{1}).sine_metrics.mean];
    figs.(c{1}).itae = [itae(pstep), itae(psine)];
    curves.(c{1}) = struct('step', pstep, 'sine', psine);
end

fprintf('Simulating %d controllers x 2 experiments (%s gains) ...\n', numel(controllers), mode);
for i = 1:numel(controllers)
    name = controllers{i};
    tuned = strcmp(name, 'FOPSO_GWO');
    if tuned || strcmp(mode, 'shared')
        gains.step = controller_gains(name);      % one set for both experiments
        gains.sine = gains.step;
    else
        gains.step = controller_gains(name, 'step');
        gains.sine = controller_gains(name, 'sine');
    end
    tic;
    step_run = simulate_closed_loop(P, gains.step, 'step');
    sine_run = simulate_closed_loop(P, gains.sine, 'sine');

    R = struct();
    R.step  = step_run;
    R.sine  = sine_run;
    R.gains = gains;
    R.step_metrics = performance_metrics(step_run, 'step');
    R.sine_metrics = performance_metrics(sine_run, 'sine');
    R.summary = [R.step_metrics.mean, R.sine_metrics.mean];
    R.itae = [itae(step_run), itae(sine_run)];
    R.peak_torque = mean(max(abs(step_run.u), [], 2));   % as in FOPID_FITNESS
    fast = struct('log_dt', 1e-3);                       % torque at the control rate
    R.effort.step = control_effort(simulate_closed_loop(P, gains.step, 'step', fast), 'step');
    R.effort.sine = control_effort(simulate_closed_loop(P, gains.sine, 'sine', fast), 'sine');
    if isfield(curves, name)                             % PID, FOPID and FBPA-FOPID
        R.curve_rms.step = sqrt(mean((step_run.q - curves.(name).step.q).^2, 2));
        R.curve_rms.sine = sqrt(mean((sine_run.q - curves.(name).sine.q).^2, 2));
    end
    if tuned
        R.tuning = load(tuned_file(name));               % the tuner's record
    end
    results.(name) = R;
    fprintf('  %-9s done (%.1f s)\n', name, toc);
end

runs = load_optimizer_runs(fullfile('results', 'optimizer_runs'));

write_summary(results, paper, figs, runs, controllers, outdir, mode);
plot_paper_figures(results, outdir);
if ~isempty(runs)
    plot_convergence(runs, fullfile(outdir, 'convergence.png'));
end
matfile = fullfile(outdir, 'simulation_results.mat');
if exist('OCTAVE_VERSION', 'builtin')
    save('-mat7-binary', matfile, 'results', 'paper', 'figs', 'runs');
else
    save(matfile, 'results', 'paper', 'figs', 'runs', '-v7');
end
fprintf('\nSaved %s/summary.md, %s/*.png and %s\n', outdir, outdir, matfile);
end

% ------------------------------------------------------------------------
function file = tuned_file(name)
%TUNED_FILE  Gain file written by TUNE_FOPID_HYBRID (as in CONTROLLER_GAINS).
file = fullfile('results', [lower(name) '_gains.mat']);
end

function v = itae(out)
%ITAE  Integral of time-weighted absolute error summed over the joints (Eq. 29).
v = trapz(out.t, out.t .* sum(abs(out.r - out.q), 1));
end

function s = display_name(name)
switch name
    case 'FBPA',      s = 'FBPA-FOPID';
    case 'FOPSO_GWO', s = 'FOPSO-GWO';
    otherwise,        s = name;
end
end

% ------------------------------------------------------------------------
function runs = load_optimizer_runs(folder)
%LOAD_OPTIMIZER_RUNS  The runs of TOOLS/COMPARE_OPTIMIZERS, if any.
runs = struct('optimizer', {}, 'cost', {}, 'seed', {}, 'J', {}, 'evaluations', {}, ...
              'metrics', {}, 'history', {}, 'popsize', {});
if ~exist(folder, 'dir'), return; end
d = dir(fullfile(folder, '*_seed*.mat'));
for f = d(:)'
    tok = regexp(f.name, '_(paper|fbpa_all|fbpa)_seed(\d+)\.mat$', 'tokens', 'once');
    if isempty(tok), continue; end
    S = load(fullfile(folder, f.name));
    r.optimizer   = S.optimizer;
    r.cost        = tok{1};
    r.seed        = str2double(tok{2});
    r.J           = S.cost;
    r.evaluations = S.info.evaluations;
    r.metrics     = S.metrics;
    r.history     = S.info.history;
    r.popsize     = S.info.options.PopSize;
    runs(end+1) = r; %#ok<AGROW>
end
end

% ------------------------------------------------------------------------
function write_summary(results, paper, figs, runs, controllers, outdir, mode)
%WRITE_SUMMARY  The comparison tables, on the screen and in <outdir>/summary.md.

fid = fopen(fullfile(outdir, 'summary.md'), 'w');
out = @(varargin) both(fid, varargin{:});
has = @(c) any(strcmp(controllers, c));
metric_names = {'Overshoot (%)', 'Adjustment time (s)', 'Peak time (s)', ...
                'Sine MSE (rad^2)', 'Sine sum \|tau\| (Nm)'};
f5 = {'%.1f', '%.2f', '%.2f', '%.2e', '%.4g'};

out('# Results\n\n');
out('Jiang, Zhang & Liu, *Trajectory tracking control of a 6-DOF robotic arm based on improved\n');
out('FOPID*, Int. J. Dyn. Control 13:137 (2025), reproduced on its own arm (Table 2, UR DH\n');
out('kinematics, Eq. 21) in Octave, and extended with this work''s optimiser, FOPSO-GWO.\n\n');
out('* **Experiments:** step of 1 rad on every joint at t = 1 s; sine tracking, sin(1.5 t) rad.\n');
out('* **Metrics (paper, Tables 3 and 4):** step overshoot, adjustment time (5 %% band) and peak\n');
out('  time, all from t = 0; sine MSE and sum of |tau|; each averaged over the six joints and\n');
out('  computed on the paper''s 0.01 s logging grid.\n');
out('* **Controllers:**\n');
if strcmp(mode, 'separate')
    out('  * PID, FOPID and FBPA-FOPID, the paper''s three controllers: gains identified from the\n');
    out('    paper''s published curves of each controller, one set per experiment\n');
    out('    (`controller_gains(name, experiment)`).\n');
else
    out('  * PID, FOPID and FBPA-FOPID, the paper''s three controllers: gains identified from the\n');
    out('    paper''s published curves of each controller, one set for both experiments\n');
    out('    (`controller_gains(name)`).\n');
end
if has('FOPSO_GWO')
    T = results.FOPSO_GWO.tuning;
    out('  * FOPSO-GWO: the FOPID tuned by this work''s FO-PSO / grey-wolf hybrid\n');
    out('    (`hybrid_fopso_gwo.m`), 30 particles x 100 iterations (the paper''s FBPA budget),\n');
    out('    seeded with the identified FOPID, one gain set for both experiments; %s\n', fitness_text(T));
    out('    (random seed %d).\n', T.info.options.RandomSeed);
end
out('\n');

% ---- 1. the paper's Tables 3 and 4 against this work ---------------------
out('## 1. The paper''s Tables 3 and 4 against this work\n\n');
out('| Controller | Source | %s |\n', strjoin(metric_names, ' | '));
out('|---|---|---:|---:|---:|---:|---:|\n');
for c = {'PID', 'FOPID', 'FBPA'}
    out('| %s | Paper (Tables 3-4) | %s |\n', display_name(c{1}), row(f5, paper.(c{1})));
    if has(c{1})
        out('| %s | This work | %s |\n', display_name(c{1}), row(f5, results.(c{1}).summary));
    end
end
if has('FOPSO_GWO')
    out('| **FOPSO-GWO** | **This work (proposed)** | %s |\n', row(f5, results.FOPSO_GWO.summary, true));
end
out('\n');
out('PID, FOPID, FBPA-FOPID: reproductions, with the gains that make the simulation match each\n');
out('controller''s published curves (Sect. 6); the paper publishes no gains. The paper''s torque\n');
out('column is not a reproducible target: it is a permutation of its own Fig. 19 and its torque\n');
out('curves are numerical artefacts (Sect. 7; docs/audit_report.md, 3.3-3.4).\n');

% ---- 2. FOPSO-GWO against FBPA-FOPID -------------------------------------
if has('FOPSO_GWO')
    out('\n## 2. FOPSO-GWO against FBPA-FOPID\n\n');
    labels = [metric_names, {'ITAE step (Eq. 29)', 'ITAE sine'}];
    f7 = {'%.1f', '%.3f', '%.3f', '%.3e', '%.4g', '%.4g', '%.4g'};
    H = [results.FOPSO_GWO.summary, results.FOPSO_GWO.itae];
    F = [results.FBPA.summary, results.FBPA.itae];
    tab = [paper.FBPA NaN NaN];
    fig = [figs.FBPA.summary, figs.FBPA.itae];
    out(['| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work | ' ...
         '**FOPSO-GWO** | vs paper table | vs this work''s FBPA-FOPID |\n']);
    out('|---|---:|---:|---:|---:|---:|---:|\n');
    for k = 1:7
        cells = {num_or_na(f7{k}, tab(k)), num_or_na(f7{k}, fig(k)), sprintf(f7{k}, F(k)), ...
                 ['**' sprintf(f7{k}, H(k)) '**'], change(H(k), tab(k)), change(H(k), F(k))};
        out('| %s | %s |\n', labels{k}, strjoin(cells, ' | '));
    end
    out('\nOn the paper''s five metrics FOPSO-GWO is better than the paper''s FBPA-FOPID table on\n');
    out('%d of 5, than its figures on %d of 5, and than this work''s FBPA-FOPID on %d of 5.\n', ...
        sum(H(1:5) < tab(1:5)), sum(H(1:5) < fig(1:5)), sum(H(1:5) < F(1:5)));
    out('Negative changes are improvements. What the torque costs is in Sect. 3.\n');
end

% ---- 3. control effort ---------------------------------------------------
report_effort(out, results, controllers);

% ---- 4. optimiser comparison ---------------------------------------------
if ~isempty(runs)
    rerun = [];
    if has('FBPA'), rerun = results.FBPA.summary; end
    report_optimizers(out, runs, paper, rerun);
end

% ---- 5. per joint --------------------------------------------------------
rows = {'Step overshoot (%)',        'step_metrics', 'overshoot',       '%.1f'
        'Step adjustment time (s)',  'step_metrics', 'adjustment_time', '%.2f'
        'Step peak time (s)',        'step_metrics', 'peak_time',       '%.2f'
        'Sine MSE (rad^2)',          'sine_metrics', 'mse',             '%.2e'
        'Sine sum |tau| (Nm)',       'sine_metrics', 'torque',          '%.4g'};
out('\n## 5. Per joint\n');
for r = 1:size(rows, 1)
    out('\n### %s\n\n| Controller | Source | J1 | J2 | J3 | J4 | J5 | J6 |\n', rows{r,1});
    out('|---|---|---:|---:|---:|---:|---:|---:|\n');
    for c = {'PID', 'FOPID', 'FBPA', 'FOPSO_GWO'}
        if isfield(figs, c{1})
            out('| %s | Paper figures | %s |\n', display_name(c{1}), ...
                join_fmt(rows{r,4}, figs.(c{1}).(rows{r,2}).(rows{r,3})));
        end
        if has(c{1})
            out('| %s | This work | %s |\n', display_name(c{1}), ...
                join_fmt(rows{r,4}, results.(c{1}).(rows{r,2}).(rows{r,3})));
        end
    end
end

% ---- 6. match to the published curves ------------------------------------
out('\n## 6. Match to the published curves\n\n');
out('rms of q (this work) - q (paper''s figure), rad.\n\n');
out('| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |\n');
out('|---|---|---:|---:|---:|---:|---:|---:|---:|\n');
for c = {'PID', 'FOPID', 'FBPA'}
    if ~has(c{1}), continue; end
    for ex = {'step', 'sine'}
        v = results.(c{1}).curve_rms.(ex{1});
        out('| %s | %s | %s | %.3f |\n', display_name(c{1}), ex{1}, join_fmt('%.3f', v), mean(v));
    end
end

% ---- 7. the paper's own consistency -------------------------------------
out('\n## 7. The paper''s tables against its own figures\n\n');
out('The same metrics recomputed from the paper''s published curves (Figs 6-19, read from the\n');
out('PDF''s vector graphics) do not fully agree with its tables.\n\n');
out('| Controller | Source | %s |\n|---|---|---:|---:|---:|---:|---:|\n', strjoin(metric_names, ' | '));
for c = {'PID', 'FOPID', 'FBPA'}
    out('| %s | Table | %s |\n', display_name(c{1}), row(f5, paper.(c{1})));
    out('| %s | Figures | %s |\n', display_name(c{1}), row(f5, figs.(c{1}).summary));
end
out('\nThe torque column of Table 4 is a permutation of what Fig. 19 shows: the curves labelled\n');
out('PID / FOPID / FBPA-FOPID sum to 2.3153e4 / 3.7412e4 / 2.5675e4, which are the table''s\n');
out('FBPA / PID / FOPID values. The torque curves themselves are not outputs of the control\n');
out('law (spikes of up to 1e9 Nm in Fig. 12), so this work''s torques, which are physically\n');
out('consistent, are much smaller. See docs/audit_report.md, Sect. 3.\n');

% ---- 8. gains ------------------------------------------------------------
out('\n## 8. Gains\n');
for i = 1:numel(controllers)
    c = controllers{i};
    sets = {'step', 'sine'};
    if strcmp(mode, 'shared') || strcmp(c, 'FOPSO_GWO'), sets = {'step'}; end
    for e = sets
        g = results.(c).gains.(e{1});
        if numel(sets) == 1
            out('\n### %s (both experiments)\n\n', display_name(c));
        else
            out('\n### %s, %s experiment\n\n', display_name(c), e{1});
        end
        out('| | J1 | J2 | J3 | J4 | J5 | J6 |\n|---|---:|---:|---:|---:|---:|---:|\n');
        for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
            out('| %s | %s |\n', f{1}, join_fmt('%.4g', g.(f{1})));
        end
    end
end
fclose(fid);
end

% ------------------------------------------------------------------------
function report_effort(out, results, controllers)
%REPORT_EFFORT  The torque each controller needs (CONTROL_EFFORT).
out('\n## 3. Control effort\n\n');
out('Torque at the control rate (1 kHz), from `control_effort.m`. The first 50 ms after the step\n');
out('are the derivative kick: the fractional derivative of the ideal 1 rad step drives every\n');
out('controller with a D-term to 1e4-1e6 Nm there, so the kick is listed on its own and the\n');
out('other step columns exclude it. Peaks are the largest joint; sums and total variations\n');
out('(sum of |tau(k+1) - tau(k)|, which grows with chattering) are averaged over the joints.\n\n');
out(['| Controller | Kick peak | Step peak after kick | Sine peak | Step sum \\|tau\\| after kick | ' ...
     'Sine sum \\|tau\\| | Step total variation | Sine total variation |\n']);
out('|---|---:|---:|---:|---:|---:|---:|---:|\n');
for i = 1:numel(controllers)
    e = results.(controllers{i}).effort;
    out('| %s | %.3g | %.4g | %.4g | %.4g | %.4g | %.4g | %.4g |\n', display_name(controllers{i}), ...
        max(e.step.kick), max(e.step.peak), max(e.sine.peak), mean(e.step.sum), mean(e.sine.sum), ...
        mean(e.step.tv), mean(e.sine.tv));
end
if ~any(strcmp(controllers, 'FOPSO_GWO')), return; end
T = results.FOPSO_GWO.tuning;
if ~isfield(T, 'fitness') || ~isfield(T.fitness, 'whole') || isempty(T.fitness.whole), return; end
W = T.fitness.whole;
e = results.FOPSO_GWO.effort;
out('\nFOPSO-GWO was tuned with caps per joint: for the torque, the largest peak any of the\n');
out('paper''s three controllers needs on that joint over all their identified gain sets (Nm,\n');
out('x %g); for the overshoot, the worst joint of the paper''s own FBPA-FOPID figures (%%):\n\n', ...
    W.cap_scale);
out('| | J1 | J2 | J3 | J4 | J5 | J6 |\n|---|---:|---:|---:|---:|---:|---:|\n');
rows = {'Kick', W.cap.kick, e.step.kick; 'Step after kick', W.cap.step, e.step.peak; ...
        'Sine', W.cap.sine, e.sine.peak; ...
        'Overshoot', W.os_cap * ones(1, 6), results.FOPSO_GWO.step_metrics.overshoot(:)'};
within = 0;
for r = 1:size(rows, 1)
    out('| %s: cap | %s |\n', rows{r, 1}, join_fmt('%.4g', rows{r, 2}));
    out('| %s: FOPSO-GWO | %s |\n', rows{r, 1}, join_fmt('%.4g', rows{r, 3}));
    within = within + sum(rows{r, 3} <= rows{r, 2} * (1 + 1e-9));
end
out('\nFOPSO-GWO stays within %d of the %d caps.\n', within, 6 * size(rows, 1));
end

% ------------------------------------------------------------------------
function report_optimizers(out, runs, paper, rerun)
%REPORT_OPTIMIZERS  FBPA against FOPSO-GWO under the same costs, seeds and budget.
%   rerun: the five paper metrics of the FBPA-FOPID shown in Sects 1-2 ([] if none).
costs = {'paper', 'fbpa', 'fbpa_all'};
costs = costs(ismember(costs, {runs.cost}));
seeds = unique([runs.seed]);
budget = min([runs.evaluations]);                  % FOPSO-GWO's evaluations per run
out('\n## 4. The FBPA and FOPSO-GWO optimisers: same costs, seeds and budget\n\n');
out('Each optimiser was run with random seeds %s under %d costs (`tools/compare_optimizers.m`),\n', ...
    strjoin(arrayfun(@num2str, seeds, 'UniformOutput', false), ', '), numel(costs));
out('with the paper''s budget of 30 particles x 100 iterations, the same search space, the same\n');
out('seed (the identified FOPID) and, for each random seed, the same initial swarm:\n\n');
out('* **paper:** the paper''s fitness, ITAE of the step response (Eq. 29), relative to the FOPID\n');
out('  (the FOPID scores 1).\n');
out('* **fbpa:** this work''s cost: both ITAEs relative to the FOPID and the five paper metrics\n');
out('  relative to the paper''s FBPA-FOPID, with a penalty on every metric not better than it.\n');
if any(strcmp(costs, 'fbpa_all'))
    out('* **fbpa_all:** the same, with each paper metric scored against the better of the paper''s\n');
    out('  FBPA-FOPID and the best FBPA run under the paper''s fitness (`results/fbpa_gains.mat`),\n');
    out('  so a ratio below 1 on a metric means beating both.\n');
end
out('\nLower is better. FBPA''s beetle antennae cost two extra evaluations per particle and\n');
out('iteration, so for the same 100 iterations it uses three times the evaluations of FOPSO-GWO.\n');
out('The last column compares at equal evaluations: the best cost each run had reached after\n');
out('%d evaluations (FOPSO-GWO''s whole run, FBPA''s first %d iterations).\n\n', budget, ...
    floor((budget - 30) / 90));
out('| Cost | Optimiser | %s | mean | best | evaluations per run | mean after %d evaluations |\n', ...
    strjoin(arrayfun(@(s) sprintf('seed %d', s), seeds, 'UniformOutput', false), ' | '), budget);
out('|---|---|%s---:|---:|---:|---:|\n', repmat('---:|', 1, numel(seeds)));
best = struct('cost', {}, 'optimizer', {}, 'run', {});
for c = costs
    for o = {'FBPA', 'FOPSO-GWO'}
        sel = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, o{1}));
        if isempty(sel), continue; end
        cells = cell(1, numel(seeds));
        wins = 0;
        for s = 1:numel(seeds)
            k = find([sel.seed] == seeds(s), 1);
            if isempty(k), cells{s} = '-'; else, cells{s} = sprintf('%.4f', sel(k).J); end
        end
        at_budget = arrayfun(@(r) cost_after(r, budget), sel);
        [~, b] = min([sel.J]);
        out('| %s | %s | %s | %.4f | %.4f | %d | %.4f |\n', c{1}, o{1}, strjoin(cells, ' | '), ...
            mean([sel.J]), sel(b).J, sel(b).evaluations, mean(at_budget));
        best(end+1) = struct('cost', c{1}, 'optimizer', o{1}, 'run', sel(b)); %#ok<AGROW>
    end
end
% head to head: same cost and random seed
pairs = 0;  seed_wins = 0;  mean_wins = 0;  best_wins = 0;  equal_wins = 0;  nc = 0;
for c = costs
    A = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, 'FOPSO-GWO'));
    B = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, 'FBPA'));
    if isempty(A) || isempty(B), continue; end
    nc = nc + 1;
    for a = A(:)'
        k = find([B.seed] == a.seed, 1);
        if isempty(k), continue; end
        pairs = pairs + 1;
        seed_wins = seed_wins + (a.J < B(k).J);
    end
    mean_wins  = mean_wins  + (mean([A.J]) < mean([B.J]));
    best_wins  = best_wins  + (min([A.J]) < min([B.J]));
    equal_wins = equal_wins + (mean(arrayfun(@(r) cost_after(r, budget), A)) < ...
                               mean(arrayfun(@(r) cost_after(r, budget), B)));
end
out('\nHead to head, FOPSO-GWO reached the lower final cost on %d of %d seed and cost pairs, the\n', ...
    seed_wins, pairs);
out('lower mean under %d of %d costs and the lower best run under %d of %d. At equal evaluations\n', ...
    mean_wins, nc, best_wins, nc);
out('it had the lower mean under %d of %d costs.\n', equal_wins, nc);
out('\nThe best run of each, on the paper''s metrics:\n\n');
hdr = ['| Cost | Optimiser (seed) | ITAE step | ITAE sine | Overshoot (%) | Adjustment time (s) | ' ...
       'Peak time (s) | Sine MSE (rad^2) | Sine sum \|tau\| (Nm) | better than the paper''s FBPA-FOPID |'];
sep = '|---|---|---:|---:|---:|---:|---:|---:|---:|:---:|';
if ~isempty(rerun)
    hdr = [hdr ' better than this work''s FBPA-FOPID |'];
    sep = [sep ':---:|'];
end
out('%s\n%s\n', hdr, sep);
f7 = {'%.4g', '%.4g', '%.1f', '%.3f', '%.3f', '%.3e', '%.4g'};
for i = 1:numel(best)
    m = best(i).run.metrics;
    cells = {sprintf('%d of 5', sum(m(3:7) < paper.FBPA))};
    if ~isempty(rerun), cells{end+1} = sprintf('%d of 5', sum(m(3:7) < rerun)); end %#ok<AGROW>
    out('| %s | %s (%d) | %s | %s |\n', best(i).cost, best(i).optimizer, best(i).run.seed, ...
        row(f7, m(1:7)), strjoin(cells, ' | '));
end
out('\nConvergence: `convergence.png` (best cost against cost evaluations).\n');
end

function J = cost_after(r, budget)
%COST_AFTER  Best cost of run r after BUDGET cost evaluations (NaN before the first iteration).
K = numel(r.history);
per_iter = (r.evaluations - r.popsize) / K;
k = min(K, floor((budget - r.popsize) / per_iter + 1e-9));
if k < 1, J = NaN; else, J = r.history(k); end
end

% ------------------------------------------------------------------------
function s = fitness_text(T)
name = 'composite';
if isfield(T, 'fitness_name'), name = T.fitness_name; end
switch lower(name)
    case 'itae'
        s = 'the paper''s fitness, step ITAE (Eq. 29)';
    otherwise
        target = '';
        if isfield(T, 'target'), target = upper(T.target); end
        if strcmpi(name, 'whole')
            s = ['whole-controller cost: tracking scored against the paper''s FBPA-FOPID with no ' ...
                 'credit beyond twice as good, the torque it takes, and per-joint peak-torque caps (Sect. 3)'];
        elseif strcmp(target, 'FBPA')
            s = 'cost: both ITAEs, and the five paper metrics scored against the paper''s FBPA-FOPID';
        elseif strcmp(target, 'FBPA-ALL')
            s = ['cost: both ITAEs, and the five paper metrics scored against the better of the ' ...
                 'paper''s FBPA-FOPID and the FBPA-FOPID re-run here, metric by metric'];
        else
            s = sprintf('cost ''%s''', name);
        end
end
end

function s = row(fmt, v, bold)
if nargin < 3, bold = false; end
c = cell(1, numel(v));
for k = 1:numel(v)
    c{k} = sprintf(fmt{k}, v(k));
    if bold, c{k} = ['**' c{k} '**']; end
end
s = strjoin(c, ' | ');
end

function s = join_fmt(fmt, v)
s = strjoin(arrayfun(@(x) sprintf(fmt, x), v(:)', 'UniformOutput', false), ' | ');
end

function s = change(v, ref)
if isnan(ref), s = 'n/a'; else, s = sprintf('%+.1f %%', 100 * (v / ref - 1)); end
end

function s = num_or_na(f, v)
if isnan(v), s = 'n/a'; else, s = sprintf(f, v); end
end

function both(fid, varargin)
fprintf(varargin{:});
fprintf(fid, varargin{:});
end
