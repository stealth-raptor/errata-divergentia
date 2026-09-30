function results = main()
%MAIN  Reproduce the PID and FOPID results of the FOPID trajectory-tracking paper.
%
%   results = MAIN()
%
%   Runs both controllers through both experiments of the paper,
%     step response : 1 rad on every joint at t = 1 s
%     sine tracking : sin(1.5 t) rad on every joint
%   prints the comparison against the published Tables 3 and 4, and saves a
%   summary table plus the paper's figure set (Figs 6-19) in results/.
%
%   If TUNE_FOPID_HYBRID has been run, the FOPID tuned by the FO-PSO/GWO
%   hybrid (results/fopso_gwo_gains.mat) is simulated as a third controller,
%   compared against FOPID and the paper's FBPA-FOPID, and drawn in every
%   figure.
%
%   Runtime: about one minute.
%
%   Pipeline:
%     ROBOT_PARAMS         -> arm parameters (UR5, Table-2 masses/inertias)
%     CONTROLLER_GAINS     -> tuned per-joint gains
%     SIMULATE_CLOSED_LOOP -> closed-loop simulation (ROBOT_DYNAMICS + FOPID_*)
%     PERFORMANCE_METRICS  -> the metrics defined in the paper
%     PLOT_PAPER_FIGURES   -> the figures, in the paper's own layout

here = fileparts(mfilename('fullpath'));
cd(here);
if ~exist('results', 'dir'), mkdir('results'); end

P = robot_params();
controllers = {'PID', 'FOPID'};
if exist(fullfile('results', 'fopso_gwo_gains.mat'), 'file')
    controllers{end+1} = 'FOPSO_GWO';
end

% published values: [overshoot %, adjustment time s, peak time s, sine MSE, sine torque]
paper.PID   = [54.6 2.44 1.39 2.27e-2 3.7422e4];
paper.FOPID = [31.2 1.89 1.33 0.88e-2 2.5686e4];
paper.FBPA  = [22.1 1.43 1.09 0.37e-2 2.3154e4];

fprintf('Simulating %d controllers x 2 experiments ...\n', numel(controllers));
for i = 1:numel(controllers)
    name  = controllers{i};
    gains = controller_gains(name);

    step_run = simulate_closed_loop(P, gains, 'step');
    sine_run = simulate_closed_loop(P, gains, 'sine');

    results.(name).step   = step_run;
    results.(name).sine   = sine_run;
    results.(name).gains  = gains;
    results.(name).step_metrics = performance_metrics(step_run, 'step');
    results.(name).sine_metrics = performance_metrics(sine_run, 'sine');
    results.(name).summary = [results.(name).step_metrics.mean, results.(name).sine_metrics.mean];
    fprintf('  %-6s done\n', name);
end

print_report(results, paper, controllers);
plot_paper_figures(results, 'results');
save(fullfile('results', 'simulation_results.mat'), 'results', 'paper');
fprintf('\nSaved results/summary.md, results/fig*.png and results/simulation_results.mat\n');
end

% ------------------------------------------------------------------------
function print_report(results, paper, controllers)
%PRINT_REPORT  Write the comparison tables to the screen and to results/summary.md.

labels = {'Step overshoot (%)', 'Step adjustment time (s)', 'Step peak time (s)', ...
          'Sine MSE (rad^2)', 'Sine torque (Nm)'};
fmt = {'%.1f', '%.2f', '%.2f', '%.3e', '%.4g'};

fid = fopen(fullfile('results', 'summary.md'), 'w');
out = @(varargin) both(fid, varargin{:});

out('# Reproduction of the published PID and FOPID results\n\n');
out('| Metric | Paper PID | This work | error | Paper FOPID | This work | error |\n');
out('|---|---:|---:|---:|---:|---:|---:|\n');
for k = 1:5
    p1 = paper.PID(k);    m1 = results.PID.summary(k);
    p2 = paper.FOPID(k);  m2 = results.FOPID.summary(k);
    out(['| %s | ' fmt{k} ' | ' fmt{k} ' | %+.1f%% | ' fmt{k} ' | ' fmt{k} ' | %+.1f%% |\n'], ...
        labels{k}, p1, m1, 100*(m1/p1 - 1), p2, m2, 100*(m2/p2 - 1));
end

if isfield(results, 'FOPSO_GWO')
    out('\n# FOPID tuned by the FO-PSO/GWO hybrid\n\n');
    out('| Metric | FOPID (this work) | FOPSO-GWO | change vs FOPID | Paper FBPA-FOPID |\n');
    out('|---|---:|---:|---:|---:|\n');
    for k = 1:5
        m0 = results.FOPID.summary(k);
        m3 = results.FOPSO_GWO.summary(k);
        out(['| %s | ' fmt{k} ' | ' fmt{k} ' | %+.1f%% | ' fmt{k} ' |\n'], ...
            labels{k}, m0, m3, 100*(m3/m0 - 1), paper.FBPA(k));
    end
    % not a paper metric: the torque spike at the step instant (max |tau|,
    % averaged over the joints), which the tuner's 'torque' fitness targets
    pk = @(run) mean(max(abs(run.u), [], 2));
    m0 = pk(results.FOPID.step);
    m3 = pk(results.FOPSO_GWO.step);
    out('| Step peak torque (Nm) | %.4g | %.4g | %+.1f%% | n/a |\n', m0, m3, 100*(m3/m0 - 1));
end

rows = {'Step overshoot (%)',        'step_metrics', 'overshoot',       '%.1f'
        'Step adjustment time (s)',  'step_metrics', 'adjustment_time', '%.2f'
        'Step peak time (s)',        'step_metrics', 'peak_time',       '%.2f'
        'Sine MSE (rad^2)',          'sine_metrics', 'mse',             '%.2e'
        'Sine sum |tau| (Nm)',       'sine_metrics', 'torque',          '%.4g'};
for r = 1:size(rows, 1)
    out('\n## Per joint: %s\n\n| Controller | J1 | J2 | J3 | J4 | J5 | J6 |\n', rows{r,1});
    out('|---|---|---|---|---|---|---|\n');
    for i = 1:numel(controllers)
        v = results.(controllers{i}).(rows{r,2}).(rows{r,3});
        out('| %s | %s |\n', controllers{i}, ...
            strjoin(arrayfun(@(x) sprintf(rows{r,4}, x), v(:)', 'UniformOutput', false), ' | '));
    end
end

for i = 1:numel(controllers)
    g = results.(controllers{i}).gains;
    out('\n## %s gains\n\n| | J1 | J2 | J3 | J4 | J5 | J6 |\n|---|---|---|---|---|---|---|\n', controllers{i});
    for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
        out('| %s | %s |\n', f{1}, ...
            strjoin(arrayfun(@(x) sprintf('%.4g', x), g.(f{1})(:)', 'UniformOutput', false), ' | '));
    end
end
fclose(fid);
end

function both(fid, varargin)
fprintf(varargin{:});
fprintf(fid, varargin{:});
end
