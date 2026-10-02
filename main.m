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
%     PSO           the FOPID tuned by plain PSO (the paper's improved PSO) and
%     FOPSO_GWO     by this work's FO-PSO / grey-wolf hybrid and
%     FOPSO_GWO_CMA by that hybrid with its CMA-ES refinement stage, all with
%                   the whole-controller cost (results/<name>_gains.mat, from
%                   TUNE_FOPID_HYBRID), when those files exist; one gain set
%                   for both experiments
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
for c = {'PSO', 'FOPSO_GWO', 'FOPSO_GWO_CMA'}
    if exist(tuned_file(c{1}), 'file'), controllers{end+1} = c{1}; end %#ok<AGROW>
end

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
    tuned = any(strcmp(name, {'PSO', 'FOPSO_GWO', 'FOPSO_GWO_CMA'}));
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
ablation = load_optimizer_runs(fullfile('results', 'ablation_runs'));
ablation = [ablation, load_optimizer_runs(fullfile('results', 'dev_runs'))];

write_summary(results, paper, figs, runs, controllers, outdir, mode, ablation);
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
    case 'FOPSO_GWO_CMA', s = 'FOPSO-GWO-CMA';
    case 'PSO',       s = 'PSO-FOPID';
    otherwise,        s = name;
end
end

% ------------------------------------------------------------------------
function runs = load_optimizer_runs(folder)
%LOAD_OPTIMIZER_RUNS  The runs of TOOLS/COMPARE_OPTIMIZERS, if any.
%   (or of TOOLS/ABLATE_OPTIMIZERS, from results/ablation_runs/, and of
%   TOOLS/DEVELOP_FOPSO_GWO, from results/dev_runs/).  variant is the file
%   name's prefix: the optimiser, the ablation variant or the candidate.
runs = struct('optimizer', {}, 'variant', {}, 'cost', {}, 'seed', {}, 'J', {}, ...
              'evaluations', {}, 'metrics', {}, 'history', {}, 'popsize', {}, 'late', {}, ...
              'dev', {});
if ~exist(folder, 'dir'), return; end
d = dir(fullfile(folder, '*_seed*.mat'));
for f = d(:)'
    tok = regexp(f.name, '^(.*)_(paper|fbpa_all|fbpa|whole)_seed(\d+)\.mat$', 'tokens', 'once');
    if isempty(tok), continue; end
    S = load(fullfile(folder, f.name));
    r.optimizer   = S.optimizer;
    r.variant     = tok{1};
    r.cost        = tok{2};
    r.seed        = str2double(tok{3});
    r.J           = S.cost;
    r.evaluations = S.info.evaluations;
    r.metrics     = S.metrics;
    r.history     = S.info.history;
    r.popsize     = S.info.options.PopSize;
    r.late        = mean(S.info.mean_history(max(1, end-24):end));   % swarm state at the end
    r.dev         = r.seed > 100;                                     % a development seed
    runs(end+1) = r; %#ok<AGROW>
end
end

% ------------------------------------------------------------------------
function write_summary(results, paper, figs, runs, controllers, outdir, mode, ablation)
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
if has('PSO')
    T = results.PSO.tuning;
    out('  * PSO-FOPID: the FOPID tuned by plain PSO (`pso.m`, the paper''s improved PSO,\n');
    out('    Eqs. 23-24) exactly as FOPSO-GWO below: same budget, seed controller, cost and\n');
    out('    initial swarms; the best of random seeds 1-8 (seed %d), as for FOPSO-GWO.\n', ...
        T.info.options.RandomSeed);
end
if has('FOPSO_GWO')
    T = results.FOPSO_GWO.tuning;
    out('  * FOPSO-GWO: the FOPID tuned by this work''s FO-PSO / grey-wolf hybrid\n');
    out('    (`hybrid_fopso_gwo.m`), 30 particles x 100 iterations (the paper''s FBPA budget),\n');
    out('    seeded with the identified FOPID, one gain set for both experiments; %s\n', fitness_text(T));
    out('    (random seed %d).\n', T.info.options.RandomSeed);
end
if has('FOPSO_GWO_CMA')
    T = results.FOPSO_GWO_CMA.tuning;
    out('  * FOPSO-GWO-CMA: the same FOPSO-GWO for 60 %% of the budget, then sep-CMA-ES hunts
');
    out('    from the pack''s three leaders and a CMA-ES refinement of the best one
');
    out('    (`hybrid_fopso_cma.m`), within the same 3030 evaluations, cost, seed controller and
');
    out('    initial swarms; the best of random seeds 1-8 (seed %d), as for the others.
', ...
        T.info.options.RandomSeed);
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
if has('PSO')
    out('| PSO-FOPID | This work (improved PSO) | %s |\n', row(f5, results.PSO.summary));
end
if has('FOPSO_GWO') && has('FOPSO_GWO_CMA')
    out('| FOPSO-GWO | This work | %s |\n', row(f5, results.FOPSO_GWO.summary));
elseif has('FOPSO_GWO')
    out('| **FOPSO-GWO** | **This work (proposed)** | %s |\n', row(f5, results.FOPSO_GWO.summary, true));
end
if has('FOPSO_GWO_CMA')
    out('| **FOPSO-GWO-CMA** | **This work (proposed)** | %s |\n', ...
        row(f5, results.FOPSO_GWO_CMA.summary, true));
end
out('\n');
out('PID, FOPID, FBPA-FOPID: reproductions, with the gains that make the simulation match each\n');
out('controller''s published curves (Sect. 6); the paper publishes no gains. PSO-FOPID,\n');
out('FOPSO-GWO and FOPSO-GWO-CMA: tuned by their optimisers with the same cost, budget and\n');
out('starting swarm.\n');
out('The paper''s torque column is not a reproducible target: it is a permutation of its own\n');
out('Fig. 19 and its torque curves are numerical artefacts (Sect. 7; docs/audit_report.md,\n');
out('3.3-3.4).\n');

% ---- 2. the proposed controller against FBPA-FOPID and PSO-FOPID ---------
prop = '';
if has('FOPSO_GWO'), prop = 'FOPSO_GWO'; end
if has('FOPSO_GWO_CMA'), prop = 'FOPSO_GWO_CMA'; end
if ~isempty(prop)
    pname = display_name(prop);
    pso_ = has('PSO');
    gwo_ = strcmp(prop, 'FOPSO_GWO_CMA') && has('FOPSO_GWO');
    if pso_
        out('\n## 2. %s against FBPA-FOPID and PSO-FOPID\n\n', pname);
    else
        out('\n## 2. %s against FBPA-FOPID\n\n', pname);
    end
    labels = [metric_names, {'ITAE step (Eq. 29)', 'ITAE sine'}];
    f7 = {'%.1f', '%.3f', '%.3f', '%.3e', '%.4g', '%.4g', '%.4g'};
    H = [results.(prop).summary, results.(prop).itae];
    F = [results.FBPA.summary, results.FBPA.itae];
    tab = [paper.FBPA NaN NaN];
    fig = [figs.FBPA.summary, figs.FBPA.itae];
    hdr = '| Metric | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures | FBPA-FOPID: this work |';
    sep = '|---|---:|---:|---:|';
    if pso_
        Q = [results.PSO.summary, results.PSO.itae];
        hdr = [hdr ' PSO-FOPID |'];  sep = [sep '---:|'];
    end
    if gwo_
        G = [results.FOPSO_GWO.summary, results.FOPSO_GWO.itae];
        hdr = [hdr ' FOPSO-GWO |'];  sep = [sep '---:|'];
    end
    hdr = [hdr sprintf(' **%s** | vs paper table | vs this work''s FBPA-FOPID |', pname)];
    sep = [sep '---:|---:|---:|'];
    if pso_
        hdr = [hdr ' vs PSO-FOPID | vs PSO-FOPID, times after the step |'];  sep = [sep '---:|---:|'];
    end
    out('%s\n%s\n', hdr, sep);
    for k = 1:7
        cells = {num_or_na(f7{k}, tab(k)), num_or_na(f7{k}, fig(k)), sprintf(f7{k}, F(k))};
        if pso_, cells{end+1} = sprintf(f7{k}, Q(k)); end %#ok<AGROW>
        if gwo_, cells{end+1} = sprintf(f7{k}, G(k)); end %#ok<AGROW>
        cells = [cells, {['**' sprintf(f7{k}, H(k)) '**'], change(H(k), tab(k)), change(H(k), F(k))}]; %#ok<AGROW>
        if pso_
            cells{end+1} = change(H(k), Q(k)); %#ok<AGROW>
            if any(k == [2 3]), cells{end+1} = change(H(k) - 1, Q(k) - 1); else, cells{end+1} = ''; end %#ok<AGROW>
        end
        out('| %s | %s |\n', labels{k}, strjoin(cells, ' | '));
    end
    out('\nOn the paper''s five metrics %s is better than the paper''s FBPA-FOPID table on\n', pname);
    out('%d of 5, than its figures on %d of 5, and than this work''s FBPA-FOPID on %d of 5', ...
        sum(H(1:5) < tab(1:5)), sum(H(1:5) < fig(1:5)), sum(H(1:5) < F(1:5)));
    if pso_
        out(';\nthan PSO-FOPID, tuned with the same cost, on %d of 5 and both ITAEs %d of 2', ...
            sum(H(1:5) < Q(1:5)), sum(H(6:7) < Q(6:7)));
    end
    out('.\nNegative changes are improvements. The adjustment and peak times are measured from t = 0,\n');
    out('as in the paper, so every one is at least 1 s; the last column compares them after the\n');
    out('step at t = 1 s, as the cost does. What the torque costs is in Sect. 3.\n');
end

% ---- 3. control effort ---------------------------------------------------
report_effort(out, results, controllers);

% ---- 4. optimiser comparison ---------------------------------------------
if ~isempty(runs)
    rerun = [];
    if has('FBPA'), rerun = results.FBPA.summary; end
    report_optimizers(out, runs, paper, rerun);
    report_ablation(out, ablation(~[ablation.dev]), runs);
    report_development(out, ablation([ablation.dev]), runs);
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
    for c = {'PID', 'FOPID', 'FBPA', 'PSO', 'FOPSO_GWO', 'FOPSO_GWO_CMA'}
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
    if strcmp(mode, 'shared') || any(strcmp(c, {'PSO', 'FOPSO_GWO', 'FOPSO_GWO_CMA'})), sets = {'step'}; end
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
tuned = {'PSO', 'FOPSO_GWO', 'FOPSO_GWO_CMA'};
tuned = tuned(ismember(tuned, controllers));
if isempty(tuned), return; end
T = results.(tuned{end}).tuning;
if ~isfield(T, 'fitness') || ~isfield(T.fitness, 'whole') || isempty(T.fitness.whole), return; end
W = T.fitness.whole;
scale = '';
if W.cap_scale ~= 1, scale = sprintf(', times %g', W.cap_scale); end
out('\n%s %s tuned with caps per joint: for the torque, the largest peak any of the\n', ...
    strjoin(cellfun(@display_name, tuned, 'UniformOutput', false), ', '), ...
    ifelse(numel(tuned) > 1, 'were', 'was'));
out('paper''s three controllers needs on that joint over all their identified gain sets (Nm%s);\n', scale);
out('for the overshoot, the worst joint of the paper''s own FBPA-FOPID figures (%%):\n\n');
out('| | J1 | J2 | J3 | J4 | J5 | J6 |\n|---|---:|---:|---:|---:|---:|---:|\n');
cats = {'Kick', W.cap.kick, @(R) R.effort.step.kick
        'Step after kick', W.cap.step, @(R) R.effort.step.peak
        'Sine', W.cap.sine, @(R) R.effort.sine.peak
        'Overshoot', W.os_cap * ones(1, 6), @(R) R.step_metrics.overshoot(:)'};
within = zeros(1, numel(tuned));
for r = 1:size(cats, 1)
    out('| %s: cap | %s |\n', cats{r, 1}, join_fmt('%.4g', cats{r, 2}));
    for i = 1:numel(tuned)
        v = cats{r, 3}(results.(tuned{i}));
        out('| %s: %s | %s |\n', cats{r, 1}, display_name(tuned{i}), join_fmt('%.4g', v));
        within(i) = within(i) + sum(v <= cats{r, 2} * (1 + 1e-9));
    end
end
out('\n');
for i = 1:numel(tuned)
    out('%s stays within %d of the %d caps. ', display_name(tuned{i}), within(i), 6 * size(cats, 1));
end
out('\n');
end

function s = ifelse(c, a, b)
if c, s = a; else, s = b; end
end

% ------------------------------------------------------------------------
function report_optimizers(out, runs, paper, rerun)
%REPORT_OPTIMIZERS  PSO, FBPA and FOPSO-GWO under the same costs, seeds and budget.
%   rerun: the five paper metrics of the FBPA-FOPID shown in Sects 1-2 ([] if none).
costs = {'paper', 'fbpa', 'fbpa_all', 'whole'};
costs = costs(ismember(costs, {runs.cost}));
names = {'PSO', 'FBPA', 'FOPSO-GWO', 'FOPSO-GWO-CMA'};
names = names(ismember(names, {runs.optimizer}));
lead = 'FOPSO-GWO';                                % the optimiser compared with the others
if any(strcmp(names, 'FOPSO-GWO-CMA')), lead = 'FOPSO-GWO-CMA'; end
budget = min([runs.evaluations]);                  % one evaluation per particle and iteration
out('\n## 4. The optimisers: %s under the same costs, seeds and budget\n\n', strjoin(names, ', '));
out('`tools/compare_optimizers.m` runs %s under %d costs. For a fair comparison\n', ...
    strjoin(names, ', '), numel(costs));
out('everything but the algorithm is the same:\n\n');
out('* 30 particles x 100 iterations (the paper''s FBPA budget), the same search space and the\n');
out('  same seed controller (the identified FOPID);\n');
out('* for each random seed the same initial swarm: the optimisers share the initialisation code\n');
out('  and its random draws;\n');
out('* the same cost function, conditioning check and early abort of unstable candidates;\n');
out('* each algorithm with its standard or published coefficients: PSO c1 = c2 = 2 and inertia\n');
out('  0.9 -> 0.4 (the paper''s improved PSO, Eqs. 23-24), FBPA the paper''s Sect. 4 settings,\n');
out('  FOPSO-GWO its final settings (c1 = c2 = 1.5, c3 = 1, fractional order 0.9, chosen on\n');
out('  separate development seeds, below), FOPSO-GWO-CMA the same swarm for 60 %% of the budget\n');
out('  and CMA-ES after it (also chosen on development seeds); the swarms limit |v| to 0.2 of\n');
out('  each range, FBPA to 1;\n');
out('* PSO, FOPSO-GWO and FOPSO-GWO-CMA evaluate the cost %d times in all, FBPA three times\n', budget);
out('  as often (its beetle antennae).\n\n');
out('The costs:\n\n');
out('* **paper:** the paper''s fitness, ITAE of the step response (Eq. 29), relative to the FOPID\n');
out('  (the FOPID scores 1).\n');
out('* **fbpa:** both ITAEs relative to the FOPID and the five paper metrics relative to the\n');
out('  paper''s FBPA-FOPID, with a penalty on every metric not better than it.\n');
if any(strcmp(costs, 'fbpa_all'))
    out('* **fbpa_all:** the same, with each paper metric scored against the better of the paper''s\n');
    out('  FBPA-FOPID and the best FBPA run under the paper''s fitness (`results/fbpa_gains.mat`).\n');
end
if any(strcmp(costs, 'whole'))
    out('* **whole:** the whole-controller cost of this work''s final controller (Sect. 3).\n');
end
out('\nFinal best cost (lower is better), over the random seeds run; the last column compares at\n');
out('equal evaluations, the best cost each run had reached after %d evaluations (FBPA''s first\n', budget);
out('%d iterations).\n\n', floor((budget - 30) / 90));
out('| Cost | Optimiser | runs | best | median | mean | worst | mean after %d evaluations |\n', budget);
out('|---|---|---:|---:|---:|---:|---:|---:|\n');
best = struct('cost', {}, 'optimizer', {}, 'run', {});
for c = costs
    for o = names
        sel = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, o{1}));
        if isempty(sel), continue; end
        J = [sel.J];
        at_budget = arrayfun(@(r) cost_after(r, budget), sel);
        [~, b] = min(J);
        out('| %s | %s | %d | %.4f | %.4f | %.4f | %.4f | %.4f |\n', c{1}, o{1}, numel(sel), ...
            min(J), median(J), mean(J), max(J), mean(at_budget));
        best(end+1) = struct('cost', c{1}, 'optimizer', o{1}, 'run', sel(b)); %#ok<AGROW>
    end
end

% head to head: the lead optimiser against each other one, same cost and random seed
if any(strcmp(names, lead)) && numel(names) > 1
    out('\nHead to head, %s against each of the others (same cost and random seed):\n\n', lead);
    out('| %s against | seed and cost pairs with the lower final cost | costs with the lower mean | ', lead);
    out('costs with the lower best run | costs with the lower mean at %d evaluations |\n', budget);
    out('|---|---:|---:|---:|---:|\n');
    for o = names(~strcmp(names, lead))
        pairs = 0;  seed_wins = 0;  mean_wins = 0;  best_wins = 0;  equal_wins = 0;  nc = 0;
        for c = costs
            A = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, lead));
            B = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, o{1}));
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
        out('| %s | %d of %d | %d of %d | %d of %d | %d of %d |\n', o{1}, seed_wins, pairs, ...
            mean_wins, nc, best_wins, nc, equal_wins, nc);
    end
    others = names(~strcmp(names, lead));
    out('\nPer cost, the random seeds on which %s ends lower than ...\n\n', lead);
    out('| Cost |%s\n|---|%s\n', sprintf(' %s |', others{:}), repmat('---:|', 1, numel(others)));
    for c = costs
        out('| %s |', c{1});
        A = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, lead));
        for o = others
            B = runs(strcmp({runs.cost}, c{1}) & strcmp({runs.optimizer}, o{1}));
            [~, a, b] = intersect([A.seed], [B.seed]);
            if isempty(a), out(' |'); continue; end
            out(' %d of %d |', sum([A(a).J] < [B(b).J]), numel(a));
        end
        out('\n');
    end
end

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

function report_ablation(out, abl, runs)
%REPORT_ABLATION  Why plain PSO wins: the optimisers with one setting changed.
if isempty(abl), return; end
order = {'pso', 'PSO, unmodified (c1 = c2 = 2, inertia 0.9 -> 0.4, \|v\| <= 0.2)'
         'pso_c1', 'PSO with c1 = c2 = 1'
         'fopso_gwo', 'FOPSO-GWO, final settings (c1 = c2 = 1.5, c3 = 1, fractional order 0.9)'
         'fopso_gwo_v1', 'FOPSO-GWO, first settings (c1 = c2 = c3 = 1, fractional order 0.9 -> 0.4)'
         'fopso_gwo_c2', 'FOPSO-GWO, first settings with c1 = c2 = c3 = 2'
         'fopso_c2', 'FO-PSO alone: first settings without the grey-wolf term, c1 = c2 = 2'
         'fbpa', 'FBPA, unmodified (c = 2, fractional memory, beetle, \|v\| <= 1)'
         'fbpa_v02', 'FBPA with \|v\| <= 0.2'};
out('\n### From the first FOPSO-GWO settings to the final ones: one change at a time\n\n');
out('`tools/ablate_optimizers.m` reruns the optimisers with one setting changed, under the\n');
out('same budget, seed controller, initial swarms and cost; `fopso_gwo_v1` are the runs of\n');
out('FOPSO-GWO''s first settings, which plain PSO beat on the tracking costs. The final settings\n');
out('were chosen on separate development seeds (next subsection). The last column is the\n');
out('mean cost of the swarm''s current positions over the last 25 iterations: unstable\n');
out('candidates score 1e3-2e3, so a large value means the swarm is still scattered (or thrown\n');
out('about), a value near the best cost that it has collapsed onto one point.\n');
for c = unique({abl.cost})
    out('\nCost `%s`:\n\n', c{1});
    out('| Optimiser | runs | best | median | mean | worst | swarm, last 25 iterations |\n');
    out('|---|---:|---:|---:|---:|---:|---:|\n');
    all_ = [runs(strcmp({runs.cost}, c{1})), abl(strcmp({abl.cost}, c{1}))];
    for i = 1:size(order, 1)
        sel = all_(strcmp({all_.variant}, order{i, 1}));
        if isempty(sel), continue; end
        J = [sel.J];
        out('| %s | %d | %.4f | %.4f | %.4f | %.4f | %.4g |\n', order{i, 2}, numel(sel), ...
            min(J), median(J), mean(J), max(J), mean([sel.late]));
    end
end
if all(ismember({'fbpa_v02', 'fopso_c2'}, {abl.variant}))
    out('\nFBPA with \\|v\\| <= 0.2 and FO-PSO alone are the same search: FBPA''s beetle step (1e-4 of\n');
    out('the range, shrinking to 6e-7) does not move the particles, so the two differ only in their\n');
    out('random numbers. The gap between them is the run-to-run noise at this number of seeds.\n');
end
end

function report_development(out, dev, runs)
%REPORT_DEVELOPMENT  How FOPSO-GWO's final settings were chosen (TOOLS/DEVELOP_FOPSO_GWO).
if isempty(dev), return; end
order = {'V1',  'first settings: c = 1 / 1 / 1, fractional order 0.9 -> 0.4 (Eq. 27)'
         'A',   'c = 2 / 2 / 0.5, order held at 0.9'
         'B',   'c = 2 / 2 / 1, order 0.9'
         'C',   'c = 2 / 2 / 0.5, order 0.9 -> 0.4'
         'E',   'A with a linear a_g'
         'G',   'C with the order held at 0.9 for 60 % of the run'
         'H',   'c = 2 / 2 / 1.5, grey-wolf pull growing from 0'
         'M',   'c = 1.75 / 1.75 / 0.5, order 0.9'
         'N',   'c = 2 / 1 / 1, order 0.9'
         'R',   'L, starting as PSO, grey-wolf share growing over the run'
         'R2',  'L, starting as PSO, grey-wolf share complete at mid-run'
         'S',   'L with half the swarm pulled as in PSO, outside the pack'
         'L',   '**final settings: c = 1.5 / 1.5 / 1, order 0.9**'
         'H1',  'FOPSO-GWO-CMA: L for 60 %, then CMA-ES from the best, shaped by the elite'
         'H1I', 'H1 with a round start covariance'
         'H3',  'H1 with short CMA-ES hunts from three leaders first'
         'H4',  'H1 with the swarm for 40 %'
         'H5',  'H3, the swarm cut short before it converges, round start'
         'H6',  'H3 with the swarm for 40 %'
         'H7',  'H1 refined joint by joint (5-D CMA-ES blocks)'
         'H8',  '**FOPSO-GWO-CMA final: H3 with sep-CMA-ES**'
         'H9',  'H8, refined joint by joint after the hunts'
         'PSO', 'plain PSO'};
costs = {'paper', 'fbpa', 'whole'};
costs = costs(ismember(costs, unique({dev.cost})));
out('\n### How the final FOPSO-GWO and FOPSO-GWO-CMA settings were chosen\n\n');
out('`tools/develop_fopso_gwo.m`: candidate settings on development seeds 101-104, which the\n');
out('comparison above never uses, run exactly as in it. Mean best cost, the change of that mean\n');
out('against PSO''s over the same seeds, and the number of seeds on which the candidate beats PSO:\n\n');
out('| Candidate | Settings |');
for c = costs, out(' %s: mean | vs PSO | seeds won |', c{1}); end
out('\n|---|---|');
for c = costs, out('---:|---:|---:|'); end
out('\n');
for i = 1:size(order, 1)
    if ~any(strcmp({dev.variant}, order{i, 1})), continue; end
    out('| %s | %s |', order{i, 1}, order{i, 2});
    for c = costs
        sel = dev(strcmp({dev.variant}, order{i, 1}) & strcmp({dev.cost}, c{1}));
        ref = dev(strcmp({dev.variant}, 'PSO') & strcmp({dev.cost}, c{1}));
        if isempty(sel), out(' | | |'); continue; end
        [~, a, b] = intersect([sel.seed], [ref.seed]);
        if strcmp(order{i, 1}, 'PSO')
            out(' %.4f | | |', mean([sel.J]));
        else
            out(' %.4f | %+.1f %% | %d of %d |', mean([sel.J]), ...
                100 * (mean([sel(a).J]) / mean([ref(b).J]) - 1), sum([sel(a).J] < [ref(b).J]), numel(a));
        end
    end
    out('\n');
end
out('\nThe first round ran under `fbpa` only and chose L. Under the paper''s fitness L then lost\n');
out('to PSO on the comparison''s seeds 1-4, so a second round added `paper` to the development,\n');
out('with four candidates designed for it (N, R, R2, S). The rule, fixed before its last two\n');
out('candidates had run: take the candidate whose mean beats PSO''s under both costs by the\n');
out('largest margin on the weaker of the two. Only L and N beat PSO under both, L by more\n');
out('(9 %% against 5 %%), so L stayed.');
p_dev  = dev(strcmp({dev.variant}, 'PSO') & strcmp({dev.cost}, 'paper'));
p_test = runs(strcmp({runs.optimizer}, 'PSO') & strcmp({runs.cost}, 'paper') & [runs.seed] <= 4);
if ~isempty(p_dev) && ~isempty(p_test)
    out(' Under `paper` the seed-to-seed spread is as large as the\n');
    out('differences: PSO''s mean is %.3f on these seeds and %.3f on the comparison''s seeds 1-4.', ...
        mean([p_dev.J]), mean([p_test.J]));
end
out('\n');
if any(strcmp({dev.cost}, 'whole'))
    out('\nFOPSO-GWO-CMA (H1-H9) was developed under `whole`, the cost of the final controller, with\n');
    out('the rule fixed before its last candidate ran: the lowest mean among the candidates that beat\n');
    out('PSO on at least 3 of the 4 seeds. That is H8, which beats PSO on all 4.\n');
end
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
                 'credit beyond twice as good, the torque it takes, and per-joint caps on the peak ' ...
                 'torque and the overshoot (Sect. 3)'];
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
