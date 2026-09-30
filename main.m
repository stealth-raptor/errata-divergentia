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
%   Runtime in Octave: a few minutes (four 5 s simulations at 1 ms).
%
%   Pipeline:
%     ROBOT_PARAMS         -> arm parameters (Table 2 of the paper)
%     CONTROLLER_GAINS     -> gains identified from the published curves
%     SIMULATE_CLOSED_LOOP -> closed-loop simulation (ROBOT_DYNAMICS + FOPID_*)
%     PERFORMANCE_METRICS  -> the metrics defined in the paper
%     PAPER_CURVES         -> the paper's own published curves
%     PLOT_PAPER_FIGURES   -> the figures, in the paper's own layout

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

% published values: [overshoot %, adjustment time s, peak time s, sine MSE, sine torque]
paper.PID   = [54.6 2.44 1.39 2.27e-2 3.7422e4];
paper.FOPID = [31.2 1.89 1.33 0.88e-2 2.5686e4];

fprintf('Simulating %d controllers x 2 experiments (%s gains) ...\n', numel(controllers), mode);
for i = 1:numel(controllers)
    name = controllers{i};
    if strcmp(mode, 'separate')
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

    % the paper's own curves, through the same metrics
    pstep = paper_curves(name, 'step');
    psine = paper_curves(name, 'sine');
    figs.(name).step_metrics = performance_metrics(pstep, 'step');
    figs.(name).sine_metrics = performance_metrics(psine, 'sine');
    figs.(name).summary = [figs.(name).step_metrics.mean, figs.(name).sine_metrics.mean];
    results.(name).curve_rms.step = sqrt(mean((step_run.q - pstep.q).^2, 2));
    results.(name).curve_rms.sine = sqrt(mean((sine_run.q - psine.q).^2, 2));
    fprintf('  %-6s done (%.0f s)\n', name, toc);
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
function print_report(results, paper, figs, controllers, outdir, mode)
%PRINT_REPORT  Write the comparison tables to the screen and to <outdir>/summary.md.

labels = {'Step overshoot (%)', 'Step adjustment time (s)', 'Step peak time (s)', ...
          'Sine MSE (rad^2)', 'Sine sum |tau| (Nm)'};
fmt = {'%.1f', '%.2f', '%.2f', '%.3e', '%.4g'};

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
for i = 1:numel(controllers)
    c = controllers{i};
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

out('\n## Match to the published curves (rms of q_this_work - q_paper, rad)\n\n');
out('| Controller | Experiment | J1 | J2 | J3 | J4 | J5 | J6 | mean |\n|---|---|---|---|---|---|---|---|---|\n');
for i = 1:numel(controllers)
    for ex = {'step', 'sine'}
        v = results.(controllers{i}).curve_rms.(ex{1});
        out('| %s | %s | %s | %.3f |\n', controllers{i}, ex{1}, ...
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
    for i = 1:numel(controllers)
        for src = {'paper figures', 'this work'}
            if strcmp(src{1}, 'this work')
                v = results.(controllers{i}).(rows{r,2}).(rows{r,3});
            else
                v = figs.(controllers{i}).(rows{r,2}).(rows{r,3});
            end
            out('| %s, %s | %s |\n', controllers{i}, src{1}, ...
                strjoin(arrayfun(@(x) sprintf(rows{r,4}, x), v(:)', 'UniformOutput', false), ' | '));
        end
    end
end

sets = {'step', 'sine'};
if strcmp(mode, 'shared'), sets = {'step'}; end
for i = 1:numel(controllers)
    for e = sets
        g = results.(controllers{i}).gains.(e{1});
        if strcmp(mode, 'shared')
            out('\n## %s gains (both experiments)\n\n', controllers{i});
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

function both(fid, varargin)
fprintf(varargin{:});
fprintf(fid, varargin{:});
end
