function plot_convergence(runs, file)
%PLOT_CONVERGENCE  Best cost against cost evaluations: PSO, FBPA and FOPSO-GWO.
%
%   PLOT_CONVERGENCE(runs, file)  with runs as loaded by MAIN from
%   results/optimizer_runs/ (TOOLS/COMPARE_OPTIMIZERS).  One panel per cost,
%   one thin line per run, coloured by optimiser.  The x axis counts cost
%   evaluations, not iterations, since FBPA's beetle antennae spend three
%   evaluations per particle and iteration, PSO and FOPSO-GWO one.
%
%   See also MAIN, PLOT_PAPER_FIGURES.

costs  = {'paper',    'paper: step ITAE (Eq. 29), FOPID = 1'
          'fbpa',     'fbpa: paper metrics against the paper''s FBPA-FOPID'
          'fbpa_all', 'fbpa\_all: against the paper''s and the best FBPA run'
          'whole',    'whole: the whole-controller cost'};
costs  = costs(ismember(costs(:, 1), {runs.cost}), :);
nc = size(costs, 1);
ncol = min(nc, 2);
nrow = ceil(nc / ncol);
colour = struct('PSO', [0 0.6 0], 'FBPA', [0 0 0], 'FOPSO_GWO', [0.85 0 0.85]);
fig = figure('Visible', 'off', 'Position', [100 100 550*ncol 420*nrow], 'Color', 'w');
for c = 1:nc
    subplot(nrow, ncol, c); hold on; grid on; box on;
    sel = runs(strcmp({runs.cost}, costs{c, 1}));
    handles = [];  labels = {};
    for r = sel(:)'
        K = numel(r.history);
        per_iter = (r.evaluations - r.popsize) / K;      % evaluations per iteration
        evals = r.popsize + per_iter * (1:K);
        key = strrep(r.optimizer, '-', '_');
        h = semilogy(evals, r.history, 'Color', colour.(key), 'LineWidth', 1);
        if ~any(strcmp(labels, r.optimizer))
            handles(end+1) = h; %#ok<AGROW>
            labels{end+1} = r.optimizer; %#ok<AGROW>
        end
    end
    set(gca, 'YScale', 'log', 'FontSize', 9);
    try, set(gca, 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6); catch, end
    xlabel('cost evaluations');
    ylabel('best cost');
    title(costs{c, 2});
    if ~isempty(handles), legend(handles, labels, 'Location', 'northeast'); end
end
print(fig, file, '-dpng', '-r150');
close(fig);
end
