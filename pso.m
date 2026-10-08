function [z_best, info] = pso(cost, nvar, opts)
%PSO  Plain particle swarm optimisation, the baseline for FBPA and FOPSO-GWO.
%
%   [z_best, info] = PSO(cost, nvar, opts)
%     cost   function handle, J = cost(z) with z a 1 x nvar row in [0, 1]^nvar
%     nvar   number of decision variables
%     opts   optional struct, see the defaults below
%
%   Same interface as HYBRID_FOPSO_GWO and FBPA, so TUNE_FOPID_HYBRID can run
%   any of the three.  Global-best PSO with a linearly decreasing inertia
%   weight (Shi and Eberhart, 1998), the paper's "improved PSO" (Eqs. 22-24):
%
%     v(k+1) = w v(k) + c1 r1 (pbest - x) + c2 r2 (gbest - x)
%     x(k+1) = x(k) + v(k+1)
%     w      = wmax - (wmax - wmin) k / MaxIter
%
%   r1, r2 uniform in [0, 1], drawn per particle and dimension.  The paper's
%   Eq. 22, the original PSO without inertia, is wmin = wmax = 1.
%
%   What it lacks against the other two is exactly what they add: no
%   fractional velocity memory (FO-PSO, Eq. 25), no beetle antennae (FBPA,
%   Eq. 26) and no grey-wolf leaders (FOPSO-GWO).  Everything else is
%   shared, for a fair comparison: the swarm is initialised by the same
%   code, so for the same RandomSeed and Seeds all three start from the same
%   particles; positions are kept in the unit box the same way; and the
%   velocity limit is FOPSO-GWO's.  Like FOPSO-GWO, and unlike FBPA, it
%   costs one evaluation per particle and iteration.
%
%   Options:
%     PopSize      30          swarm size
%     MaxIter      100         iterations
%     c1, c2       2, 2        learning factors (the standard values, and the paper's)
%     wmin, wmax   0.4, 0.9    inertia weight range (the paper's)
%     vmax         0.2         velocity clamp, as a fraction of each range
%                              (as HYBRID_FOPSO_GWO)
%     Seeds, SeedFraction, SeedJitter, UseParallel, Checkpoint, Resume,
%     CheckpointTag, RandomSeed, Verbose   as in HYBRID_FOPSO_GWO
%
%   See also HYBRID_FOPSO_GWO, FBPA, TUNE_FOPID_HYBRID, FOPID_FITNESS.

if nargin < 3, opts = struct(); end
opts = set_defaults(opts, nvar);
N = opts.PopSize;

st = [];
if ~isempty(opts.Checkpoint) && opts.Resume && exist(opts.Checkpoint, 'file')
    S = load(opts.Checkpoint, 'state');
    st = S.state;
    if ~isfield(st, 'tag') || ~isequal(st.tag, opts.CheckpointTag) ...
            || st.nvar ~= nvar || st.N ~= N || st.MaxIter ~= opts.MaxIter
        warning('pso:resume', ...
                ['checkpoint %s is from a run with different settings; ' ...
                 'ignoring it and starting afresh'], opts.Checkpoint);
        st = [];
    end
end

if ~isempty(st)
    rng(st.rng);
    if opts.Verbose
        fprintf('Resuming from %s at iteration %d/%d, best cost %.6g\n', ...
                opts.Checkpoint, st.k, opts.MaxIter, st.gf);
    end
else
    if ~isempty(opts.RandomSeed), rng(opts.RandomSeed); end
    st = initialise(cost, nvar, opts);
    st.tag = opts.CheckpointTag;
    if ~isempty(opts.Checkpoint)
        st.rng = rng;
        save_state(opts.Checkpoint, st);
    end
end

t_start = tic;
t_iter  = [];
for k = st.k + 1 : opts.MaxIter
    t_k = tic;
    w = opts.wmax - (opts.wmax - opts.wmin) * k / opts.MaxIter;

    % ---- velocity: inertia + cognitive + social ---------------------------
    gbest = repmat(st.g, N, 1);
    V = w * st.V ...
        + opts.c1 * rand(N, nvar) .* (st.pbest - st.X) ...
        + opts.c2 * rand(N, nvar) .* (gbest    - st.X);
    V = min(max(V, -opts.vmax), opts.vmax);

    % ---- position, kept inside the unit box ------------------------------
    X = st.X + V;
    out = X < 0 | X > 1;
    X = min(max(X, 0), 1);
    V(out) = 0;
    st.V = V;
    st.X = X;

    % ---- evaluate and update the memories --------------------------------
    F = evaluate(cost, X, opts.UseParallel);
    st.evals = st.evals + N;
    better = F < st.pf;
    st.pbest(better, :) = X(better, :);
    st.pf(better) = F(better);
    [gf, gi] = min(st.pf);
    if gf < st.gf
        st.gf = gf;
        st.g  = st.pbest(gi, :);
    end

    st.k = k;
    st.history(k)      = st.gf;
    st.mean_history(k) = mean(F(isfinite(F)));

    if opts.Verbose
        el = toc(t_start);
        t_iter(end+1) = toc(t_k); %#ok<AGROW>
        eta_s = mean(t_iter(max(1, end-2):end)) * (opts.MaxIter - k);
        fprintf('  iter %3d/%d  best %.6g  swarm mean %.6g  (%.0f s, ETA %s)\n', ...
                k, opts.MaxIter, st.gf, st.mean_history(k), el, format_time(eta_s));
    end
    if ~isempty(opts.Checkpoint)
        st.rng = rng;
        save_state(opts.Checkpoint, st);
    end
end

z_best = st.g;
info.cost         = st.gf;
info.history      = st.history;
info.mean_history = st.mean_history;
info.evaluations  = st.evals;
info.pbest        = st.pbest;
info.pbest_cost   = st.pf;
info.options      = opts;
end

% ------------------------------------------------------------------------
function st = initialise(cost, nvar, opts)
%INITIALISE  Random swarm plus the caller's seeds, evaluated once
%   (the same construction, and random draws, as HYBRID_FOPSO_GWO and FBPA).
N = opts.PopSize;
X = rand(N, nvar);

seeds = opts.Seeds;
if ~isempty(seeds)
    seeds = min(max(seeds, 0), 1);
    m = min(size(seeds, 1), N);
    X(1:m, :) = seeds(1:m, :);
    n_jit = min(N - m, round(opts.SeedFraction * N));
    for i = 1:n_jit
        s = seeds(mod(i - 1, m) + 1, :);
        X(m + i, :) = min(max(s + opts.SeedJitter * randn(1, nvar), 0), 1);
    end
end

if opts.Verbose
    fprintf('Evaluating the initial swarm (%d particles) ...\n', N);
end
F = evaluate(cost, X, opts.UseParallel);

st.nvar  = nvar;
st.N     = N;
st.MaxIter = opts.MaxIter;
st.X     = X;
st.V     = zeros(N, nvar);
st.V1    = zeros(N, nvar);
st.V2    = zeros(N, nvar);
st.V3    = zeros(N, nvar);
st.pbest = X;
st.pf    = F;
[st.gf, gi] = min(F);
st.g     = X(gi, :);
st.k     = 0;
st.evals = N;
st.history      = nan(1, opts.MaxIter);
st.mean_history = nan(1, opts.MaxIter);
if opts.Verbose
    fprintf('  initial best %.6g\n', st.gf);
end
end

% ------------------------------------------------------------------------
function F = evaluate(cost, X, use_parallel)
%EVALUATE  Cost of every row of X; non-finite costs are treated as very bad.
N = size(X, 1);
F = zeros(N, 1);
if use_parallel
    parfor i = 1:N
        F(i) = cost(X(i, :));
    end
else
    for i = 1:N
        F(i) = cost(X(i, :));
    end
end
F(~isfinite(F)) = 1e12;
end

% ------------------------------------------------------------------------
function opts = set_defaults(opts, nvar)
d = struct('PopSize', 30, 'MaxIter', 100, 'c1', 2, 'c2', 2, ...
           'wmin', 0.4, 'wmax', 0.9, 'vmax', 0.2, ...
           'Seeds', zeros(0, nvar), 'SeedFraction', 0.3, 'SeedJitter', 0.05, ...
           'UseParallel', false, 'Checkpoint', '', 'Resume', false, ...
           'CheckpointTag', [], 'RandomSeed', [], 'Verbose', true);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
if ~isempty(opts.Seeds) && size(opts.Seeds, 2) ~= nvar
    error('pso:seeds', 'Seeds must have %d columns', nvar);
end
end

% ------------------------------------------------------------------------
function s = format_time(sec)
if ~isfinite(sec), s = '?'; return; end
h = floor(sec / 3600);
m = floor(mod(sec, 3600) / 60);
if h > 0
    s = sprintf('%dh%02dm', h, m);
else
    s = sprintf('%dm%02ds', m, floor(mod(sec, 60)));
end
end

% ------------------------------------------------------------------------
function save_state(file, state) %#ok<INUSD>
%SAVE_STATE  Checkpoint the optimiser state (MAT-file version 7).
save(file, 'state', '-v7');
end
