function [z_best, info] = cmaes(cost, nvar, opts)
%CMAES  Covariance matrix adaptation evolution strategy on the unit box.
%
%   [z_best, info] = CMAES(cost, nvar, opts)
%     cost   function handle, J = cost(z) with z a 1 x nvar row in [0, 1]^nvar
%     nvar   number of decision variables
%     opts   optional struct:
%       MaxEvals    budget of cost evaluations (default PopSize*(MaxIter+1),
%                   the swarm optimisers' budget)
%       Lambda      offspring per generation (default 4 + floor(3 ln nvar))
%       Sigma0      initial step size in the unit box (default 0.2)
%       Mean0       1 x nvar start (default: the first row of Seeds, else 0.5)
%       C0          nvar x nvar initial covariance, scaled so its largest
%                   eigenvalue is 1 (default eye)
%       Seeds       as for the swarm optimisers; the first row is the start
%       RandomSeed, Verbose
%   (PopSize, MaxIter, Checkpoint, Resume, CheckpointTag, UseParallel and the
%   swarm optimisers' other options are accepted and ignored, except for the
%   budget.)
%
%   The standard (mu/mu_w, lambda)-CMA-ES (Hansen, "The CMA Evolution
%   Strategy: A Tutorial", 2016) with cumulative step-size adaptation, rank-
%   one and rank-mu covariance updates.  Points outside the unit box are
%   evaluated at their projection onto it and ranked with a quadratic
%   penalty on the distance, so the mean is pulled back inside.
%
%   info: cost, history (best cost after each block of PopSize evaluations,
%   for comparison with the swarm optimisers), mean_history, evaluations,
%   options, and the final mean, sigma and C.
%
%   See also HYBRID_FOPSO_CMA, PSO, HYBRID_FOPSO_GWO.

if nargin < 3, opts = struct(); end
opts = set_defaults(opts, nvar);
if ~isempty(opts.RandomSeed), rng(opts.RandomSeed); end

st = cma_init(nvar, opts);
z_best = st.mean;  f_best = Inf;
evals = 0;
block = opts.PopSize;                     % history is kept per block of evaluations
history = [];  mean_history = [];
blk_F = [];
t0 = tic;
while evals < opts.MaxEvals
    lam = min(st.lambda, opts.MaxEvals - evals);
    [st, Z, X] = cma_ask(st, lam);
    F = zeros(lam, 1);
    for i = 1:lam
        F(i) = cost(X(i, :));
    end
    F(~isfinite(F)) = 1e12;
    evals = evals + lam;
    [fm, im] = min(F);
    if fm < f_best, f_best = fm; z_best = X(im, :); end
    pen = opts.BoxPenalty * sum((Z - X).^2, 2);
    if lam == st.lambda
        st = cma_tell(st, Z, F + pen);
    end
    % bookkeeping per block of PopSize evaluations
    blk_F = [blk_F; F]; %#ok<AGROW>
    while numel(blk_F) >= block
        history(end+1) = f_best; %#ok<AGROW>
        b = blk_F(1:block);
        mean_history(end+1) = mean(b(b < 1e11)); %#ok<AGROW>
        blk_F = blk_F(block+1:end);
    end
    if opts.Verbose && (mod(st.gen, opts.PrintEvery) == 0 || evals >= opts.MaxEvals)
        fprintf('  gen %4d  evals %6d/%d  best %.6g  sigma %.3g  (%.0f s)\n', ...
                st.gen, evals, opts.MaxEvals, f_best, st.sigma, toc(t0));
    end
    if st.sigma < 1e-8, break; end
end
if ~isempty(blk_F), history(end+1) = f_best; mean_history(end+1) = mean(blk_F); end
info.cost = f_best;
info.history = history(2:end);            % the first block is the "initial swarm"
info.mean_history = mean_history(2:end);
info.evaluations = evals;
info.options = opts;
info.mean = st.mean;  info.sigma = st.sigma;  info.C = st.C;
end

% ------------------------------------------------------------------------
function st = cma_init(n, opts)
%CMA_INIT  Strategy parameters and state (Hansen 2016, Table 1).
st.n = n;
st.lambda = opts.Lambda;
mu = floor(st.lambda / 2);
w = log((st.lambda + 1) / 2) - log(1:mu)';
st.w = w / sum(w);
st.mu = mu;
st.mueff = 1 / sum(st.w.^2);
st.cc = (4 + st.mueff / n) / (n + 4 + 2 * st.mueff / n);
st.cs = (st.mueff + 2) / (n + st.mueff + 5);
st.c1 = 2 / ((n + 1.3)^2 + st.mueff);
st.cmu = min(1 - st.c1, 2 * (st.mueff - 2 + 1 / st.mueff) / ((n + 2)^2 + st.mueff));
st.damps = 1 + 2 * max(0, sqrt((st.mueff - 1) / (n + 1)) - 1) + st.cs;
st.chiN = sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n^2));
st.mean = opts.Mean0(:)';
st.sigma = opts.Sigma0;
C = opts.C0;
C = (C + C') / 2;
C = C / max(eig(C));
st.C = C;
[B, D] = eig(C);
st.B = B;  st.D = sqrt(max(diag(D), 1e-20));
st.pc = zeros(1, n);  st.ps = zeros(1, n);
st.gen = 0;
st.eigen_gen = 0;
end

function [st, Z, X] = cma_ask(st, lam)
%CMA_ASK  Sample lam points; Z unconstrained, X projected onto the box.
Y = randn(lam, st.n) .* st.D' * st.B';          % ~ N(0, C)
Z = st.mean + st.sigma * Y;
X = min(max(Z, 0), 1);
end

function st = cma_tell(st, Z, F)
%CMA_TELL  Update mean, evolution paths, covariance and step size.
n = st.n;
[~, idx] = sort(F);
sel = Z(idx(1:st.mu), :);
old = st.mean;
st.mean = st.w' * sel;
y = (st.mean - old) / st.sigma;
invsqrtC = st.B * diag(1 ./ st.D) * st.B';
st.ps = (1 - st.cs) * st.ps + sqrt(st.cs * (2 - st.cs) * st.mueff) * (y * invsqrtC);
st.gen = st.gen + 1;
hsig = norm(st.ps) / sqrt(1 - (1 - st.cs)^(2 * st.gen)) / st.chiN < 1.4 + 2 / (n + 1);
st.pc = (1 - st.cc) * st.pc + hsig * sqrt(st.cc * (2 - st.cc) * st.mueff) * y;
Yk = (sel - old) / st.sigma;
st.C = (1 - st.c1 - st.cmu) * st.C ...
       + st.c1 * (st.pc' * st.pc + (1 - hsig) * st.cc * (2 - st.cc) * st.C) ...
       + st.cmu * (Yk' * diag(st.w) * Yk);
st.sigma = st.sigma * exp((st.cs / st.damps) * (norm(st.ps) / st.chiN - 1));
st.sigma = min(st.sigma, 0.5);                  % never wider than the box
if st.gen - st.eigen_gen > st.lambda / (st.c1 + st.cmu) / n / 10
    st.eigen_gen = st.gen;
    st.C = triu(st.C) + triu(st.C, 1)';
    [B, D] = eig(st.C);
    st.B = B;  st.D = sqrt(max(diag(D), 1e-20));
end
end

% ------------------------------------------------------------------------
function opts = set_defaults(opts, nvar)
d = struct('PopSize', 30, 'MaxIter', 100, 'MaxEvals', [], 'Lambda', 4 + floor(3 * log(nvar)), ...
           'Sigma0', 0.2, 'Mean0', [], 'C0', eye(nvar), 'Seeds', [], 'BoxPenalty', 1, ...
           'RandomSeed', [], 'Verbose', true, 'PrintEvery', 10);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
if isempty(opts.MaxEvals), opts.MaxEvals = opts.PopSize * (opts.MaxIter + 1); end
if isempty(opts.Mean0)
    if ~isempty(opts.Seeds), opts.Mean0 = min(max(opts.Seeds(1, :), 0), 1);
    else, opts.Mean0 = 0.5 * ones(1, nvar); end
end
end
