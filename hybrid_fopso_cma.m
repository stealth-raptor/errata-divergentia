function [z_best, info] = hybrid_fopso_cma(cost, nvar, opts)
%HYBRID_FOPSO_CMA  FOPSO-GWO with a swarm-informed CMA-ES refinement stage.
%
%   [z_best, info] = HYBRID_FOPSO_CMA(cost, nvar, opts)   same interface and
%   budget as HYBRID_FOPSO_GWO and PSO: PopSize * (MaxIter + 1) evaluations.
%
%   A memetic hybrid: a population method that explores, then a local method
%   that refines what it found.
%     1. FOPSO-GWO (HYBRID_FOPSO_GWO, its settings and schedules) runs for
%        SwarmFraction of the iterations: the swarm and the grey-wolf pack
%        locate the good basins of the 30-dimensional search.
%     2. CMA-ES (CMAES) spends the rest of the budget around the best point.
%        It does not start blind: its covariance is that of the Elite best
%        personal bests around the best one, so it begins with the shape of
%        the basin the swarm has mapped, and its step size is their spread
%        (clipped to [SigmaMin, SigmaMax]).
%   Swarm methods are good at finding basins and slow at refining inside
%   them, where the steps are a small fraction of the range and the cost
%   valleys are narrow and tilted; CMA-ES learns exactly that tilt.
%
%   Options (besides those of HYBRID_FOPSO_GWO, passed on to it):
%     SwarmFraction  0.6     share of the iterations given to FOPSO-GWO
%     Elite          10      personal bests that shape the CMA-ES start
%     SigmaMin       0.003   bounds on the CMA-ES start step (unit box): good
%     SigmaMax       0.01    controllers sit in narrow valleys, steps of 0.05
%                            are all worse than the point itself
%     Lambda         []      CMA-ES offspring per generation ([]: 4 + 3 ln n)
%
%   info: cost, history (best cost per block of PopSize evaluations, as the
%   swarm optimisers), mean_history, evaluations, options, swarm (the
%   FOPSO-GWO info), cma (the CMA-ES info), switch_eval.
%
%   See also HYBRID_FOPSO_GWO, CMAES, PSO.

if nargin < 3, opts = struct(); end
d = struct('PopSize', 30, 'MaxIter', 100, 'SwarmFraction', 0.6, 'Elite', 10, ...
           'SigmaMin', 0.003, 'SigmaMax', 0.01, 'Lambda', [], 'Verbose', true, ...
           'RandomSeed', []);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
N = opts.PopSize;
budget = N * (opts.MaxIter + 1);
K1 = max(1, round(opts.SwarmFraction * opts.MaxIter));

% ---- 1. FOPSO-GWO -------------------------------------------------------
o1 = rmfield(opts, intersect(fieldnames(opts), {'SwarmFraction', 'Elite', 'SigmaMin', ...
                                                'SigmaMax', 'Lambda', 'MaxEvals'}));
o1.MaxIter = K1;
if opts.Verbose
    fprintf('FOPSO-GWO-CMA: %d swarm iterations, then CMA-ES for %d evaluations\n', ...
            K1, budget - N * (K1 + 1));
end
[g, s1] = hybrid_fopso_gwo(cost, nvar, o1);

% ---- 2. CMA-ES from the swarm's best, shaped by its elite ---------------
[pf, order] = sort(s1.pbest_cost);
m = min(opts.Elite, numel(order));
E = s1.pbest(order(1:m), :);
E = E(pf(1:m) < 1e3, :);                       % only stable controllers
o2 = struct('MaxEvals', budget - s1.evaluations, 'PopSize', N, 'Mean0', g, ...
            'Verbose', opts.Verbose, 'PrintEvery', 10);
if ~isempty(opts.Lambda), o2.Lambda = opts.Lambda; end
if size(E, 1) >= 3
    D = E - g;                                 % spread around the best, not the mean
    C = (D' * D) / size(D, 1);
    s = sqrt(max(eig(C)));
    C = C / max(s^2, eps) + 1e-3 * eye(nvar);  % keep every direction open
    o2.C0 = C;
    o2.Sigma0 = min(max(s, opts.SigmaMin), opts.SigmaMax);
else
    o2.Sigma0 = opts.SigmaMax;
end
if opts.Verbose
    fprintf('CMA-ES from the swarm''s best (cost %.6g), sigma0 %.3g, %d elite\n', ...
            s1.cost, o2.Sigma0, size(E, 1));
end
[z2, s2] = cmaes(cost, nvar, o2);

% ---- the better of the two, and one history -----------------------------
if s2.cost < s1.cost
    z_best = z2;  J = s2.cost;
else
    z_best = g;   J = s1.cost;
end
info.cost = J;
info.history = [s1.history, min(s1.cost, s2.block_history)];
info.mean_history = [s1.mean_history, s2.block_mean];
info.evaluations = s1.evaluations + s2.evaluations;
info.options = opts;
info.swarm = s1;
info.cma = s2;
info.switch_eval = s1.evaluations;
end

