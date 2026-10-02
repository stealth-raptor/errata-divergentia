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
%     ShapeFromElite true    start covariance from the elite (false: identity)
%     Leaders        1       > 1: first a short CMA-ES hunt from each of the
%                            best personal bests that lie LeaderGap apart
%                            (the pack's alpha, beta, delta), sharing
%                            LeaderShare of the CMA-ES budget; the rest goes
%                            to a run from the best point any hunt found
%     LeaderGap      0.05
%     FullSchedule   false   true: the swarm runs the schedules of the full
%                            MaxIter and stops at SwarmFraction, unconverged
%     LeaderShare    0.3
%
%   info: cost, history (best cost per block of PopSize evaluations, as the
%   swarm optimisers), mean_history, evaluations, options, swarm (the
%   FOPSO-GWO info), cma (the CMA-ES info), switch_eval.
%
%   See also HYBRID_FOPSO_GWO, CMAES, PSO.

if nargin < 3, opts = struct(); end
d = struct('PopSize', 30, 'MaxIter', 100, 'SwarmFraction', 0.6, 'Elite', 10, ...
           'SigmaMin', 0.003, 'SigmaMax', 0.01, 'Lambda', [], 'Verbose', true, ...
           'RandomSeed', [], 'Leaders', 1, 'LeaderGap', 0.05, 'LeaderShare', 0.3, ...
           'ShapeFromElite', true, 'FullSchedule', false);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
N = opts.PopSize;
budget = N * (opts.MaxIter + 1);
K1 = max(1, round(opts.SwarmFraction * opts.MaxIter));

% ---- 1. FOPSO-GWO -------------------------------------------------------
o1 = rmfield(opts, intersect(fieldnames(opts), {'SwarmFraction', 'Elite', 'SigmaMin', ...
                                                'SigmaMax', 'Lambda', 'MaxEvals', 'Leaders', ...
                                                'LeaderGap', 'LeaderShare', 'ShapeFromElite', ...
                                                'FullSchedule'}));
if opts.FullSchedule
    o1.StopIter = K1;                          % schedules of a full run, cut short:
else                                           % the swarm has not yet converged
    o1.MaxIter = K1;
end
if opts.Verbose
    fprintf('FOPSO-GWO-CMA: %d swarm iterations, then CMA-ES for %d evaluations\n', ...
            K1, budget - N * (K1 + 1));
end
[g, s1] = hybrid_fopso_gwo(cost, nvar, o1);
s1.history = s1.history(1:K1);
s1.mean_history = s1.mean_history(1:K1);

% ---- 2. CMA-ES around the pack's leaders ---------------------------------
rem = budget - s1.evaluations;
[pf, order] = sort(s1.pbest_cost);
P = s1.pbest(order, :);  pf = pf(:);
% the leaders: the best personal bests that lie apart from each other
lead = 1;
for k = 2:numel(order)
    if numel(lead) >= opts.Leaders || pf(k) >= 1e3, break; end
    if min(sqrt(sum((P(lead, :) - P(k, :)).^2, 2))) > opts.LeaderGap
        lead(end+1) = k; %#ok<AGROW>
    end
end
runs = {};  F_all = [];
if numel(lead) > 1
    % a short hunt from each leader, then the rest of the budget on the best
    b = floor(opts.LeaderShare * rem / numel(lead));
    best_J = Inf;
    for k = lead
        o2 = cma_start(P, pf, k, opts, nvar, N, b);
        if opts.Verbose
            fprintf('CMA-ES hunt from leader %d (cost %.6g), sigma0 %.3g, %d evaluations\n', ...
                    k, pf(k), o2.Sigma0, b);
        end
        [z, s] = cmaes(cost, nvar, o2);
        runs{end+1} = s; F_all = [F_all; s.F_all]; %#ok<AGROW>
        if s.cost < best_J, best_J = s.cost; z_lead = z; sig = s.sigma; end
    end
    o2 = struct('MaxEvals', rem - numel(F_all), 'PopSize', N, 'Mean0', z_lead, ...
                'Sigma0', min(max(sig, opts.SigmaMin), opts.SigmaMax), ...
                'Verbose', opts.Verbose, 'PrintEvery', 10);
    if ~isempty(opts.Lambda), o2.Lambda = opts.Lambda; end
    if opts.Verbose
        fprintf('CMA-ES continues from the best hunt (cost %.6g)\n', best_J);
    end
else
    o2 = cma_start(P, pf, 1, opts, nvar, N, rem);
    if opts.Verbose
        fprintf('CMA-ES from the swarm''s best (cost %.6g), sigma0 %.3g\n', s1.cost, o2.Sigma0);
    end
end
[~, s] = cmaes(cost, nvar, o2);
runs{end+1} = s;  F_all = [F_all; s.F_all];

% ---- the best point of all, and one history ------------------------------
z_best = g;  J = s1.cost;
for k = 1:numel(runs)
    if runs{k}.cost < J
        J = runs{k}.cost;
        z_best = runs{k}.best_z;
    end
end
best = min(s1.cost, cummin(F_all));
nb = floor(numel(F_all) / N);
info.cost = J;
info.history = [s1.history, best(N * (1:nb))'];
info.mean_history = [s1.mean_history, arrayfun(@(k) mean_stable(F_all(N*(k-1)+1:N*k)), 1:nb)];
info.evaluations = s1.evaluations + numel(F_all);
info.options = opts;
info.swarm = s1;
info.cma = runs;
info.leaders = lead;
info.switch_eval = s1.evaluations;
end

% ------------------------------------------------------------------------
function o2 = cma_start(P, pf, k, opts, nvar, N, evals)
%CMA_START  CMA-ES options for a run from personal best k: covariance from
%   the elite around it (if ShapeFromElite), step from their spread, clipped.
o2 = struct('MaxEvals', evals, 'PopSize', N, 'Mean0', P(k, :), ...
            'Sigma0', opts.SigmaMax, 'Verbose', opts.Verbose, 'PrintEvery', 10);
if ~isempty(opts.Lambda), o2.Lambda = opts.Lambda; end
E = P(1:min(opts.Elite, end), :);
E = E(pf(1:size(E, 1)) < 1e3, :);              % only stable controllers
if opts.ShapeFromElite && size(E, 1) >= 3
    D = E - P(k, :);                           % spread around this point
    C = (D' * D) / size(D, 1);
    sp = sqrt(max(eig(C)));
    o2.C0 = C / max(sp^2, eps) + 1e-3 * eye(nvar);   % keep every direction open
    o2.Sigma0 = min(max(sp, opts.SigmaMin), opts.SigmaMax);
end
end

function m = mean_stable(F)
F = F(F < 1e11);
if isempty(F), m = NaN; else, m = mean(F); end
end
