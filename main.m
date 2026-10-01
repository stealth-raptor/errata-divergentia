function results = main(mode)
%MAIN  Reproduce the PID and FOPID results of the FOPID trajectory-tracking paper.
%
%   results = MAIN()           gains fitted to each experiment separately
%   results = MAIN('shared')   one gain set per controller for both experiments
%
%   Runs both controllers through both experiments of the paper,
%     step response : 1 rad on every joint at t = 1 s
%     sine tracking : sin(1.5 t) rad on every joint
%   and compares them with the paper in three ways:
%     1. the published Tables 3 and 4;
%     2. the same metrics recomputed from the paper's own published curves
%        (read from the vector graphics of the PDF, see PAPER_CURVES) -- the
%        figures and tables of the paper do not fully agree with each other;
%     3. the published curves themselves, joint by joint (rms difference).
%   Saves results/summary.md, the paper's figure set (Figs 6-19), two
%   comparison figures and results/simulation_results.mat; with 'shared',
%   the same under results/shared_gains/.
%
%   Gain sets (see CONTROLLER_GAINS): the paper does not say whether its
%   step and sine experiments used the same gains.  By default each
%   experiment uses the gains identified from its own published curves,
%   which matches the figures most closely; 'shared' uses one gain set per
%   controller identified from both experiments together.
%
%   FOPSO-GWO: if TUNE_FOPID_HYBRID has been run (results/fopso_gwo_gains.mat
%   exists), the FOPID re-tuned by the FO-PSO / grey-wolf hybrid is simulated
%   as a third controller (one gain set for both experiments), compared in
%   results/summary.md with its tuning baseline and with the paper's
%   FBPA-FOPID, and drawn in magenta in Figs 6-19.
%
%   Runtime in Octave: a few minutes (two 5 s simulations at 1 ms per
%   controller).
%
%   Pipeline:
%     ROBOT_PARAMS         -> arm parameters (Table 2 of the paper)
%     CONTROLLER_GAINS     -> gains identified from the published curves
%     SIMULATE_CLOSED_LOOP -> closed-loop simulation (ROBOT_DYNAMICS + FOPID_*)
%     PERFORMANCE_METRICS  -> the metrics defined in the paper
%     PAPER_CURVES         -> the paper's own published curves
%     PLOT_PAPER_FIGURES   -> the figures, in the paper's own layout
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
controllers = {'PID', 'FOPID'};
hybrid_file = fullfile('results', 'fopso_gwo_gains.mat');
if exist(hybrid_file, 'file')
    controllers{end+1} = 'FOPSO_GWO';
end

% published values: [overshoot %, adjustment time s, peak time s, sine MSE, sine torque]
paper.PID   = [54.6 2.44 1.39 2.27e-2 3.7422e4];
paper.FOPID = [31.2 1.89 1.33 0.88e-2 2.5686e4];
paper.FBPA  = [22.1 1.43 1.09 0.37e-2 2.3154e4];

fprintf('Simulating %d controllers x 2 experiments (%s gains) ...\n', numel(controllers), mode);
for i = 1:numel(controllers)
    name = controllers{i};
    if strcmp(name, 'FOPSO_GWO')
        gains.step = controller_gains(name);      % one tuned set for both experiments
        gains.sine = gains.step;
    elseif strcmp(mode, 'separate')
        gains.step = controller_gains(name, 'step');
        gains.sine = controller_gains(name, 'sine');
    else
        gains.step = controller_gains(name);
        gains.sine = gains.step;
    end
    tic;
    step_run = simulate_closed_loop(P, gains.step, 'step');
    sine_run = simulate_closed_loop(P, gains.sine, 'sine');

    results.(name).step  = step_run;
    results.(name).sine  = sine_run;
    results.(name).gains = gains;
    results.(name).step_metrics = performance_metrics(step_run, 'step');
    results.(name).sine_metrics = performance_metrics(sine_run, 'sine');
    results.(name).summary = [results.(name).step_metrics.mean, results.(name).sine_metrics.mean];
    results.(name).itae = [itae(step_run), itae(sine_run)];

    % the paper's own curves, through the same metrics (FBPA-FOPID is the
    % paper's counterpart of the re-tuned FOPSO-GWO)
    pname = name;
    if strcmp(name, 'FOPSO_GWO'), pname = 'FBPA'; end
    pstep = paper_curves(pname, 'step');
    psine = paper_curves(pname, 'sine');
    figs.(pname).step_metrics = performance_metrics(pstep, 'step');
    figs.(pname).sine_metrics = performance_metrics(psine, 'sine');
    figs.(pname).summary = [figs.(pname).step_metrics.mean, figs.(pname).sine_metrics.mean];
    figs.(pname).itae = [itae(pstep), itae(psine)];
    if ~strcmp(name, 'FOPSO_GWO')
        results.(name).curve_rms.step = sqrt(mean((step_run.q - pstep.q).^2, 2));
        results.(name).curve_rms.sine = sqrt(mean((sine_run.q - psine.q).^2, 2));
    else
        % the tuner's record: its baseline metrics, settings and history
        results.(name).tuning = load(hybrid_file, 'baseline', 'cost', 'fitness', 'info');
    end
    fprintf('  %-9s done (%.0f s)\n', name, toc);
end

print_report(results, paper, figs, controllers, outdir, mode);
plot_paper_figures(results, outdir);
matfile = fullfile(outdir, 'simulation_results.mat');
if exist('OCTAVE_VERSION', 'builtin')
    save('-mat7-binary', matfile, 'results', 'paper', 'figs');
else
    save(matfile, 'results', 'paper', 'figs', '-v7');
end
fprintf('\nSaved %s/summary.md, %s/*.png and %s\n', outdir, outdir, matfile);
end

% ------------------------------------------------------------------------
function v = itae(out)
%ITAE  Integral of time-weighted absolute error summed over the joints (Eq. 29).
v = trapz(out.t, out.t .* sum(abs(out.r - out.q), 1));
end

% ------------------------------------------------------------------------
function print_report(results, paper, figs, controllers, outdir, mode)
%PRINT_REPORT  Write the comparison tables to the screen and to <outdir>/summary.md.

labels = {'Step overshoot (%)', 'Step adjustment time (s)', 'Step peak time (s)', ...
          'Sine MSE (rad^2)', 'Sine sum |tau| (Nm)'};
fmt = {'%.1f', '%.2f', '%.2f', '%.3e', '%.4g'};

hybrid = any(strcmp(controllers, 'FOPSO_GWO'));
repro  = controllers(~strcmp(controllers, 'FOPSO_GWO'));   % reproductions of the paper

fid = fopen(fullfile(outdir, 'summary.md'), 'w');
out = @(varargin) both(fid, varargin{:});

out('# Reproduction of the published PID and FOPID results\n\n');
out('Columns: the paper''s table; the same metric recomputed from the paper''s own\n');
out('published curves (Figs 6-19, read from the PDF''s vector graphics); this work.\n');
out('Settling band 5 %%, all metrics on the paper''s 0.01 s logging grid.\n');
if strcmp(mode, 'separate')
    out('Gains: identified separately for each experiment (controller_gains(name, experiment)).\n\n');
else
    out('Gains: one set per controller for both experiments (controller_gains(name)).\n\n');
end
for i = 1:numel(repro)
    c = repro{i};
    out('## %s\n\n| Metric | Paper table | Paper figures | This work |\n|---|---:|---:|---:|\n', c);
    for k = 1:5
        out(['| %s | ' fmt{k} ' | ' fmt{k} ' | ' fmt{k} ' |\n'], labels{k}, ...
            paper.(c)(k), figs.(c).summary(k), results.(c).summary(k));
    end
    out('\n');
end
out('The sine-torque column of the paper''s Table 4 is a permutation of what its\n');
out('Fig. 19 shows: the curves labelled PID / FOPID / FBPA-FOPID sum to\n');
out('2.3153e4 / 3.7412e4 / 2.5675e4, i.e. the table''s FBPA / PID / FOPID values.\n');
out('See docs/audit_report.md.\n');

if hybrid
    report_hybrid(out, results, paper, figs, labels, fmt);
end

out('\n## Match to the published curves (rms of q_this_work - q_paper, rad)\n\n');
out('| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |\n|---|---|---|---|---|---|---|---|---|\n');
for i = 1:numel(repro)
    for ex = {'step', 'sine'}
        v = results.(repro{i}).curve_rms.(ex{1});
        out('| %s | %s | %s | %.3f |\n', repro{i}, ex{1}, ...
            strjoin(arrayfun(@(x) sprintf('%.3f', x), v(:)', 'UniformOutput', false), ' | '), mean(v));
    end
end

rows = {'Step overshoot (%)',        'step_metrics', 'overshoot',       '%.1f'
        'Step adjustment time (s)',  'step_metrics', 'adjustment_time', '%.2f'
        'Step peak time (s)',        'step_metrics', 'peak_time',       '%.2f'
        'Sine MSE (rad^2)',          'sine_metrics', 'mse',             '%.2e'
        'Sine sum |tau| (Nm)',       'sine_metrics', 'torque',          '%.4g'};
for r = 1:size(rows, 1)
    out('\n## Per joint: %s\n\n| Source | J1 | J2 | J3 | J4 | J5 | J6 |\n', rows{r,1});
    out('|---|---|---|---|---|---|---|\n');
    srcs = {};                                   % {label, values}
    for i = 1:numel(repro)
        srcs(end+1, :) = {[repro{i} ', paper figures'], figs.(repro{i}).(rows{r,2}).(rows{r,3})}; %#ok<AGROW>
        srcs(end+1, :) = {[repro{i} ', this work'], results.(repro{i}).(rows{r,2}).(rows{r,3})}; %#ok<AGROW>
    end
    if hybrid
        srcs(end+1, :) = {'FBPA-FOPID, paper figures', figs.FBPA.(rows{r,2}).(rows{r,3})};
        srcs(end+1, :) = {'FOPSO-GWO, this work', results.FOPSO_GWO.(rows{r,2}).(rows{r,3})};
    end
    for i = 1:size(srcs, 1)
        out('| %s | %s |\n', srcs{i, 1}, ...
            strjoin(arrayfun(@(x) sprintf(rows{r,4}, x), srcs{i, 2}(:)', 'UniformOutput', false), ' | '));
    end
end

for i = 1:numel(controllers)
    sets = {'step', 'sine'};
    if strcmp(mode, 'shared') || strcmp(controllers{i}, 'FOPSO_GWO'), sets = {'step'}; end
    for e = sets
        g = results.(controllers{i}).gains.(e{1});
        if numel(sets) == 1
            out('\n## %s gains (both experiments)\n\n', strrep(controllers{i}, '_', '-'));
        else
            out('\n## %s gains, %s experiment\n\n', controllers{i}, e{1});
        end
        out('| | J1 | J2 | J3 | J4 | J5 | J6 |\n|---|---|---|---|---|---|---|\n');
        for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
            out('| %s | %s |\n', f{1}, ...
                strjoin(arrayfun(@(x) sprintf('%.4g', x), g.(f{1})(:)', 'UniformOutput', false), ' | '));
        end
    end
end
fclose(fid);
end

% ------------------------------------------------------------------------
function report_hybrid(out, results, paper, figs, labels, fmt)
%REPORT_HYBRID  FOPSO-GWO against its tuning baseline and the paper's FBPA-FOPID.
%   The baseline is the FOPID that TUNE_FOPID_HYBRID started from and
%   normalised its cost by (the identified FOPID, one gain set for both
%   experiments); its metrics are the ones the tuner stored.
H = results.FOPSO_GWO;
T = H.tuning;
pk = mean(max(abs(H.step.u), [], 2));              % step peak torque, as in FOPID_FITNESS
raw = [H.itae, H.summary, pk];                     % same layout as FOPID_FITNESS
o = T.info.options;
out('\n# FOPID re-tuned by the FO-PSO / GWO hybrid (FOPSO-GWO)\n\n');
out('Tuned with TUNE_FOPID_HYBRID: %d particles x %d iterations (%d cost evaluations), ', ...
    o.PopSize, o.MaxIter, T.info.evaluations);
out('fitness weights [%s] on [ITAE step, ITAE sine, the five paper metrics, step peak torque];\n', ...
    strjoin(arrayfun(@(x) sprintf('%g', x), T.fitness.weights, 'UniformOutput', false), ' '));
out('final cost %.4f, where the baseline scores 1. Baseline: the identified FOPID, one gain\n', T.cost);
out('set for both experiments (controller_gains(''FOPID'')), which seeded the swarm.\n\n');
out('| Metric | FOPID baseline | FOPSO-GWO | change | Paper FBPA-FOPID: table | Paper FBPA-FOPID: figures |\n');
out('|---|---:|---:|---:|---:|---:|\n');
all_labels = [{'ITAE step', 'ITAE sine'}, labels, {'Step peak torque (Nm)'}];
all_fmt    = [{'%.4g', '%.4g'}, fmt, {'%.4g'}];
tab  = [NaN NaN paper.FBPA NaN];
fig_ = [figs.FBPA.itae, figs.FBPA.summary, NaN];
for k = 1:8
    cells = {sprintf(all_fmt{k}, T.baseline(k)), sprintf(all_fmt{k}, raw(k)), ...
             sprintf('%+.1f%%', 100 * (raw(k) / T.baseline(k) - 1)), ...
             num_or_na(all_fmt{k}, tab(k)), num_or_na(all_fmt{k}, fig_(k))};
    out('| %s | %s |\n', all_labels{k}, strjoin(cells, ' | '));
end
out('\nThe paper''s FBPA-FOPID torque values are numerical artefacts (docs/audit_report.md, 3.3-3.4),\n');
out('and its step peak torque is not comparable; FOPSO-GWO is a different optimiser, not a\n');
out('reproduction of FBPA.\n');
end

function s = num_or_na(f, v)
if isnan(v), s = 'n/a'; else, s = sprintf(f, v); end
end

function both(fid, varargin)
fprintf(varargin{:});
fprintf(fid, varargin{:});
end
