function bench = benchmark_optimizer(runs)
%BENCHMARK_OPTIMIZER  The paper's test functions for PSO, FO-PSO and FO-PSO/GWO.
%
%   bench = BENCHMARK_OPTIMIZER()      5 independent runs per algorithm
%   bench = BENCHMARK_OPTIMIZER(runs)
%
%   Repeats the comparison of the paper's Sect. 3 (Table 1, Fig. 4) on the
%   same four 30-dimensional functions, with the paper's swarm settings
%   (30 particles, 100 iterations, w 0.9 -> 0.4). All three
%   algorithms are the same code, HYBRID_FOPSO_GWO, with terms switched off:
%
%     IPSO        c3 = 0, a = 1 fixed    -> v = w v + PSO terms   (Eq. 24)
%     FO-PSO      c3 = 0, a = 0.9 -> 0.4 -> fractional memory     (Eq. 25)
%     FO-PSO/GWO  all terms                                      (hybrid)
%
%   Takes seconds, so it is a quick check that the optimiser behaves before
%   launching the long TUNE_FOPID_HYBRID run.  Prints mean and best final
%   values and saves the convergence plot to results/benchmark_optimizer.png.
%
%   See also HYBRID_FOPSO_GWO.

if nargin < 1, runs = 5; end
here = fileparts(mfilename('fullpath'));
cd(here);
if ~exist('results', 'dir'), mkdir('results'); end

n = 30;
funcs = {'f1 sphere',          100, @(x) sum(x.^2)
         'f2 Rosenbrock',      10,  @(x) sum(100 * (x(2:end) - x(1:end-1).^2).^2 + (x(1:end-1) - 1).^2)
         'f3 Schwefel 1.2',    100, @(x) sum(cumsum(x).^2)
         'f4 Schwefel 2.21',   100, @(x) max(abs(x))};

base = struct('PopSize', 30, 'MaxIter', 100, 'Verbose', false);
% IPSO and FO-PSO use the paper's learning factors c1 = c2 = 2; the hybrid
% uses its own defaults (c1 = c2 = c3 = 1, see HYBRID_FOPSO_GWO)
ipso = base;   ipso.c1 = 2;  ipso.c2 = 2;  ipso.c3 = 0;  ipso.alpha0 = 1;  ipso.alpha_drop = 0;
fopso = base;  fopso.c1 = 2; fopso.c2 = 2; fopso.c3 = 0;
algos = {'IPSO', ipso;  'FO-PSO', fopso;  'FO-PSO/GWO', base};

fig = figure('Visible', 'off', 'Position', [100 100 1100 800], 'Color', 'w');
colours = lines(size(algos, 1));
fprintf('%-18s %-12s %14s %14s\n', 'function', 'algorithm', 'mean final', 'best final');
for f = 1:size(funcs, 1)
    R = funcs{f, 2};
    fun = funcs{f, 3};
    cost = @(z) fun(R * (2 * z - 1));
    subplot(2, 2, f); hold on; grid on; box on;
    for a = 1:size(algos, 1)
        H = zeros(runs, base.MaxIter);
        final = zeros(runs, 1);
        for r = 1:runs
            o = algos{a, 2};
            o.RandomSeed = r;
            [~, info] = hybrid_fopso_gwo(cost, n, o);
            H(r, :) = info.history;
            final(r) = info.cost;
        end
        bench.(matlab.lang.makeValidName(funcs{f, 1})).(matlab.lang.makeValidName(algos{a, 1})) = final;
        fprintf('%-18s %-12s %14.4e %14.4e\n', funcs{f, 1}, algos{a, 1}, mean(final), min(final));
        semilogy(1:base.MaxIter, mean(H, 1), 'Color', colours(a, :), 'LineWidth', 1.2);
    end
    set(gca, 'YScale', 'log');
    title(funcs{f, 1});
    xlabel('iteration');
    ylabel('best cost (mean of runs)');
    legend(algos(:, 1), 'Location', 'northeast');
end
exportgraphics(fig, fullfile('results', 'benchmark_optimizer.png'), 'Resolution', 150);
close(fig);
fprintf('Saved results/benchmark_optimizer.png\n');
end
