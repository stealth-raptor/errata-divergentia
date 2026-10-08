function [z_best, info] = fbpa(cost, nvar, opts)
%FBPA  The paper's fractional-order beetle antennae particle swarm algorithm.
%
%   [z_best, info] = FBPA(cost, nvar, opts)
%     cost   function handle, J = cost(z) with z a 1 x nvar row in [0, 1]^nvar
%            (the caller maps the unit box onto its own parameters)
%     nvar   number of decision variables
%     opts   optional struct, see the defaults below
%
%   Returns the best point found and a struct with its cost, the convergence
%   history and the number of cost evaluations.  Same interface as
%   HYBRID_FOPSO_GWO, so TUNE_FOPID_HYBRID can run either.
%
%   Sect. 3 of the paper.  Every beetle is a particle whose velocity has the
%   fractional-order memory of FO-PSO (Grunwald-Letnikov truncated after four
%   terms) plus the beetle antennae search (BAS) step:
%
%     v(k+1) = (w - 1 + a) v(k) + a(1-a)/2 v(k-1) + a(1-a)(2-a)/6 v(k-2)
%              + a(1-a)(2-a)(3-a)/24 v(k-3)
%              + c1 r1 (pbest - x)            Eq. 26
%              + c2 r2 (gbest - x)
%              + c3 r3 v_l
%     x(k+1) = x(k) + v(k+1)                  Eq. 22
%
%   v_l is the beetle's own search velocity (BAS, refs 26-30 of the paper):
%   the beetle senses the cost with two antennae on either side of x along a
%   random unit direction b and steps towards the better one,
%
%     x_left = x + b d/2,   x_right = x - b d/2,   d = step / c_ant
%     v_l    = -step * b * sign(J(x_left) - J(x_right))
%     step  <- eta * step                     after every iteration
%
%   which costs two extra evaluations per beetle and iteration.
%
%   Schedules, as in the paper:
%     w = wmax - (wmax - wmin) k / MaxIter      linearly decreasing inertia
%         (Eq. 23 prints (wmax - wmin) k / MaxIter, which increases from 0;
%          the text calls it linearly decreasing, so the standard form is used)
%     a = 0.9 - 0.5 k / MaxIter                 fractional order, Eq. 27
%
%   Parameters (paper, Sect. 4): 30 beetles, MaxIter = 100, initial step
%   0.0001, step attenuation eta = 0.95, c1 = c2 = c3 = 2, wmin = 0.4,
%   wmax = 0.9, vmin = -1, vmax = 1.  The paper does not state:
%     * the antenna length; c_ant = 5 (d = step/5) is the value of the
%       original BAS code;
%     * the search space or its units.  Here it is the caller's unit box, so
%       |v| <= 1 is the full range of each parameter;
%     * the initial swarm.  As in HYBRID_FOPSO_GWO, the caller's seeds and
%       jittered copies of them join a random swarm, so both optimisers start
%       from the same points.
%
%   Options:
%     PopSize      30          number of beetles
%     MaxIter      100         iterations
%     c1, c2, c3   2, 2, 2     learning factors (c3 = c1 = c2, as in the paper)
%     wmin, wmax   0.4, 0.9    inertia weight range
%     alpha0       0.9         fractional order at k = 0
%     alpha_drop   0.5         fractional order falls by this much by MaxIter
%     vmax         1           velocity clamp, |v| <= vmax
%     step         1e-4        initial BAS step
%     eta          0.95        BAS step attenuation per iteration
%     c_ant        5           antenna length d = step / c_ant
%     Seeds, SeedFraction, SeedJitter, UseParallel, Checkpoint, Resume,
%     CheckpointTag, RandomSeed, Verbose   as in HYBRID_FOPSO_GWO
%
%   See also HYBRID_FOPSO_GWO, TUNE_FOPID_HYBRID, FOPID_FITNESS.

if nargin < 3, opts = struct(); end
opts = set_defaults(opts, nvar);
N = opts.PopSize;

st = [];
if ~isempty(opts.Checkpoint) && opts.Resume && exist(opts.Checkpoint, 'file')
    S = load(opts.Checkpoint, 'state');
    st = S.state;
    if ~isfield(st, 'tag') || ~isequal(st.tag, opts.CheckpointTag) || ~isfield(st, 'step') ...
            || st.nvar ~= nvar || st.N ~= N || st.MaxIter ~= opts.MaxIter
        warning('fbpa:resume', ...
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
    st.step = opts.step;
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
    tau = k / opts.MaxIter;
    w   = opts.wmax - (opts.wmax - opts.wmin) * tau;
    a   = opts.alpha0 - opts.alpha_drop * tau;

    % ---- beetle antennae: sense left and right, step towards the better --
    b = 2 * rand(N, nvar) - 1;
    b = b ./ (sqrt(sum(b.^2, 2)) + eps);
    d = st.step / opts.c_ant;
    x_left  = min(max(st.X + b * d / 2, 0), 1);
    x_right = min(max(st.X - b * d / 2, 0), 1);
    f_left  = evaluate(cost, x_left,  opts.UseParallel);
    f_right = evaluate(cost, x_right, opts.UseParallel);
    v_beetle = -st.step * b .* sign(f_left - f_right);
    st.evals = st.evals + 2 * N;

    % ---- velocity, Eq. 26 ------------------------------------------------
    g1 = a * (1 - a) / 2;
    g2 = g1 * (2 - a) / 3;
    g3 = g2 * (3 - a) / 4;
    memory = (w - 1 + a) * st.V + g1 * st.V1 + g2 * st.V2 + g3 * st.V3;
    gbest = repmat(st.g, N, 1);
    V = memory ...
        + opts.c1 * rand(N, nvar) .* (st.pbest - st.X) ...
        + opts.c2 * rand(N, nvar) .* (gbest    - st.X) ...
        + opts.c3 * rand(N, nvar) .* v_beetle;
    V = min(max(V, -opts.vmax), opts.vmax);

    % ---- position, Eq. 22, kept inside the unit box ----------------------
    X = st.X + V;
    out = X < 0 | X > 1;
    X = min(max(X, 0), 1);
    V(out) = 0;

    st.V3 = st.V2;  st.V2 = st.V1;  st.V1 = st.V;  st.V = V;
    st.X  = X;
    st.step = opts.eta * st.step;

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
%   (the same construction as HYBRID_FOPSO_GWO).
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
    fprintf('Evaluating the initial swarm (%d beetles) ...\n', N);
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
d = struct('PopSize', 30, 'MaxIter', 100, 'c1', 2, 'c2', 2, 'c3', 2, ...
           'wmin', 0.4, 'wmax', 0.9, 'alpha0', 0.9, 'alpha_drop', 0.5, 'vmax', 1, ...
           'step', 1e-4, 'eta', 0.95, 'c_ant', 5, ...
           'Seeds', zeros(0, nvar), 'SeedFraction', 0.3, 'SeedJitter', 0.05, ...
           'UseParallel', false, 'Checkpoint', '', 'Resume', false, ...
           'CheckpointTag', [], 'RandomSeed', [], 'Verbose', true);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
if ~isempty(opts.Seeds) && size(opts.Seeds, 2) ~= nvar
    error('fbpa:seeds', 'Seeds must have %d columns', nvar);
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
