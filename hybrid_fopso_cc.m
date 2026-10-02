function [z_best, info] = hybrid_fopso_cc(cost, nvar, opts)
%HYBRID_FOPSO_CC  FOPSO-GWO, then cooperative coevolution joint by joint.
%
%   [z_best, info] = HYBRID_FOPSO_CC(cost, nvar, opts)   same interface,
%   population and budget as HYBRID_FOPSO_GWO and PSO: PopSize particles,
%   PopSize * (MaxIter + 1) evaluations.
%
%   The FOPID has six joints of five gains each, and the cost is close to a
%   sum over the joints: each joint's controller mostly shapes its own
%   response.  A swarm in all 30 dimensions settles in one basin for every
%   joint at once, and refining it (CMA-ES) cannot move one joint to a
%   better basin of its own, e.g. the fast-peaking basin of joint 3, without
%   the other 25 gains lining up.  Cooperative coevolution (Potter & De Jong
%   1994; CCPSO, van den Bergh & Engelbrecht 2004) searches each joint on
%   its own:
%     1. FOPSO-GWO (HYBRID_FOPSO_GWO) runs for SwarmFraction of the
%        iterations and finds a good controller, the context.
%     2. The same PopSize particles become one sub-swarm per joint
%        (PopSize / 6 each).  Every iteration each sub-swarm moves by the
%        FOPSO-GWO equations in its joint's five gains, over their whole
%        range, and each particle is evaluated as the context with that
%        joint's gains replaced; a better one replaces them in the context
%        at once, so the joints improve together.
%   Each sub-swarm starts with the context's gains and random ones, so a
%   joint can leave its basin.
%
%   Options (besides those of HYBRID_FOPSO_GWO, used for the swarm and the
%   sub-swarms alike):
%     SwarmFraction  0.5   share of the iterations given to the full swarm
%
%   info: cost, history, mean_history, evaluations, options, swarm (the
%   FOPSO-GWO info), switch_eval.
%
%   See also HYBRID_FOPSO_GWO, HYBRID_FOPSO_CMA, PSO.

if nargin < 3, opts = struct(); end
d = struct('PopSize', 30, 'MaxIter', 100, 'SwarmFraction', 0.5, 'Verbose', true, ...
           'RandomSeed', [], 'c1', 1.5, 'c2', 1.5, 'c3', 1, 'gwo_power', 2, ...
           'wmin', 0.4, 'wmax', 0.9, 'alpha0', 0.9, 'vmax', 0.2);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
N = opts.PopSize;
nj = nvar / 5;
M = floor(N / nj);                                  % particles per joint
K1 = max(0, round(opts.SwarmFraction * opts.MaxIter));
R = opts.MaxIter - K1;                              % coevolution iterations

% ---- 1. FOPSO-GWO in all dimensions ---------------------------------------
o1 = rmfield(opts, intersect(fieldnames(opts), {'SwarmFraction'}));
o1.MaxIter = K1;
if opts.Verbose
    fprintf('FOPSO-GWO-CC: %d swarm iterations, then %d of coevolution (%d joints x %d)\n', ...
            K1, R, nj, M);
end
[x, s1] = hybrid_fopso_gwo(cost, nvar, o1);
J = s1.cost;
history = s1.history(1:K1);
mean_history = s1.mean_history(1:K1);
evals = s1.evaluations;

% ---- 2. one sub-swarm per joint ---------------------------------------------
blk = arrayfun(@(j) j + nj * (0:4), 1:nj, 'UniformOutput', false);
for j = 1:nj
    S = struct();
    S.X = rand(M, 5);
    S.X(1, :) = x(blk{j});                          % the context's gains
    S.V = zeros(M, 5);  S.V1 = S.V;  S.V2 = S.V;  S.V3 = S.V;
    S.P = S.X;  S.pf = inf(M, 1);
    S.pf(1) = J;
    sub(j) = S; %#ok<AGROW>
end
put = @(x, idx, y) subsasgn(x, struct('type', '()', 'subs', {{idx}}), y);
a = opts.alpha0;
g1 = a * (1 - a) / 2;  g2 = g1 * (2 - a) / 3;  g3 = g2 * (3 - a) / 4;
for r = 1:R
    tau = r / R;
    w = opts.wmax - (opts.wmax - opts.wmin) * tau;
    a_g = 2 * (1 - tau)^opts.gwo_power;
    F_round = [];
    for j = 1:nj
        S = sub(j);
        gb = x(blk{j});                             % the context holds the best gains
        xg = wolf_target(S.X, S.P, S.pf, a_g);
        V = (w - 1 + a) * S.V + g1 * S.V1 + g2 * S.V2 + g3 * S.V3 ...
            + opts.c1 * rand(M, 5) .* (S.P - S.X) ...
            + opts.c2 * rand(M, 5) .* (gb - S.X) ...
            + opts.c3 * rand(M, 5) .* (xg - S.X);
        V = min(max(V, -opts.vmax), opts.vmax);
        X = S.X + V;
        out = X < 0 | X > 1;
        X = min(max(X, 0), 1);
        V(out) = 0;
        S.V3 = S.V2;  S.V2 = S.V1;  S.V1 = S.V;  S.V = V;  S.X = X;
        F = zeros(M, 1);
        for i = 1:M
            F(i) = cost(put(x, blk{j}, X(i, :)));
        end
        F(~isfinite(F)) = 1e12;
        evals = evals + M;
        better = F < S.pf;
        S.P(better, :) = X(better, :);
        S.pf(better) = F(better);
        [fm, im] = min(F);
        if fm < J                                   % the joint improves the context
            J = fm;
            x(blk{j}) = X(im, :);
        end
        sub(j) = S;
        F_round = [F_round; F]; %#ok<AGROW>
    end
    history(end+1) = J; %#ok<AGROW>
    mean_history(end+1) = mean(F_round(F_round < 1e11)); %#ok<AGROW>
    if opts.Verbose && (mod(r, 5) == 0 || r == R)
        fprintf('  coevolution %3d/%d  best %.6g  (%d evaluations)\n', r, R, J, evals);
    end
end

z_best = x;
info.cost = J;
info.history = history;
info.mean_history = mean_history;
info.evaluations = evals;
info.options = opts;
info.swarm = s1;
info.switch_eval = s1.evaluations;
end

% ------------------------------------------------------------------------
function x_gwo = wolf_target(X, P, pf, a_g)
%WOLF_TARGET  The grey-wolf estimate of the prey from the three best (as in
%   HYBRID_FOPSO_GWO).
[M, n] = size(X);
[~, order] = sort(pf);
x_gwo = zeros(M, n);
for L = order(1:min(3, M))'
    leader = repmat(P(L, :), M, 1);
    A = 2 * a_g * rand(M, n) - a_g;
    C = 2 * rand(M, n);
    x_gwo = x_gwo + (leader - A .* abs(C .* leader - X));
end
x_gwo = x_gwo / min(3, M);
end
