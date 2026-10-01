function [z_best, info] = hybrid_fopso_gwo(cost, nvar, opts)
%HYBRID_FOPSO_GWO  Fractional-order PSO hybridised with the grey wolf optimiser.
%
%   [z_best, info] = HYBRID_FOPSO_GWO(cost, nvar, opts)
%     cost   function handle, J = cost(z) with z a 1 x nvar row in [0, 1]^nvar
%            (the caller maps the unit box onto its own parameters)
%     nvar   number of decision variables
%     opts   optional struct, see the defaults below
%
%   Returns the best point found and a struct with its cost, the convergence
%   history and the number of cost evaluations.
%
%   The algorithm keeps the structure of the paper's FBPA (Eq. 26) and swaps
%   the beetle-antennae term for a grey-wolf term.  Every particle carries a
%   velocity with fractional-order memory of its last four velocities
%   (Grunwald-Letnikov truncation, Eq. 25) and is pulled by three attractors:
%
%     v(k+1) = (w - 1 + a) v(k) + a(1-a)/2 v(k-1) + a(1-a)(2-a)/6 v(k-2)
%              + a(1-a)(2-a)(3-a)/24 v(k-3)
%              + c1 r1 (pbest - x)            cognitive  (PSO)
%              + c2 r2 (gbest - x)            social     (PSO)
%              + c3 r3 (x_gwo - x)            pack hunt  (GWO)
%     x(k+1) = x(k) + v(k+1)
%
%   x_gwo is the grey-wolf estimate of the prey position built from the three
%   best personal bests, the alpha, beta and delta wolves:
%
%     D_L = |C .* L - x|,  X_L = L - A .* D_L,   L in {alpha, beta, delta}
%     A = 2 a_g r - a_g,   C = 2 r',   a_g = 2 (1 - k/MaxIter)^p,  p = 2
%     x_gwo = (X_alpha + X_beta + X_delta) / 3
%
%   Early on |A| can exceed 1, so the GWO term throws particles around and
%   past the leaders (exploration, which is what the beetle step contributed
%   in FBPA); as a_g falls to zero it collapses onto the leaders and refines
%   the three best basins instead of only the single gbest (exploitation).
%   Standard GWO lets a_g fall linearly; here it falls quadratically
%   (gwo_power = 2), which shortens the noisy exploration phase.
%
%   Settings.  The defaults were chosen on the FOPID tuning problem itself,
%   on development seeds (101-104) kept apart from the seeds of the
%   comparison (TOOLS/COMPARE_OPTIMIZERS), against plain PSO (PSO):
%     * Fractional order held at 0.9 (alpha_drop = 0), not falling to 0.4
%       as in the paper's Eq. 27.  In Eq. 25 the weight on the last velocity
%       is w - 1 + a; with a falling it drops from 0.79 to -0.20 and the
%       total memory from 0.86 to 0.03, so the swarm loses its momentum and
%       collapses onto one point long before the end.  With a = 0.9 the
%       total memory follows PSO's inertia (about w - 0.04, 0.86 -> 0.36)
%       while keeping the fractional memory of the last four velocities.
%     * c1 = c2 = 1.5, c3 = 1: the three pulls add up to 4, PSO's
%       c1 + c2, which is the edge of the swarm's stable region.  Adding a
%       grey-wolf pull on top of PSO's c1 = c2 = 2 pushed the swarm past it
%       (the more c3, the noisier); splitting the same total between the
%       PSO terms and the wolves did best.
%   On the development seeds, cost 'fbpa' (TOOLS/ABLATE_OPTIMIZERS has the
%   first settings' runs): mean best cost 0.324 against PSO's 0.358; the
%   first settings (c1 = c2 = c3 = 1, a 0.9 -> 0.4), developed on the
%   paper's test functions (benchmark_optimizer.m on the branch
%   claude/pensive-ramanujan-mmu564), collapsed the swarm too early on this
%   problem.
%
%   Schedules:
%     w = wmax - (wmax - wmin) k / MaxIter      linearly decreasing inertia
%         (Eq. 23 of the paper prints (wmax - wmin) k / MaxIter, which
%          increases from 0 and is evidently a typo for the standard form)
%     a = alpha0 - alpha_drop k / MaxIter       fractional order (Eq. 27 has
%                                               0.9 - 0.5 k / MaxIter)
%
%   Seeded particles (opts.Seeds) let the caller start from a known good
%   point, e.g. the existing FOPID gains; since gbest never gets worse the
%   result is then guaranteed to be at least as good as the seed.
%
%   Options (the paper's Sect. 4 settings, except c1..c3, alpha_drop, vmax):
%     PopSize      30          swarm size
%     MaxIter      100         iterations
%     c1, c2, c3   1.5, 1.5, 1 learning factors (see above)
%     gwo_power    2           a_g = 2 (1 - k/MaxIter)^gwo_power
%     wmin, wmax   0.4, 0.9    inertia weight range
%     alpha0       0.9         fractional order at k = 0
%     alpha_drop   0           fractional order falls by this much by MaxIter
%                              (the paper: 0.5; see above)
%     alpha_hold   0           ... after being held at alpha0 for this fraction
%                              of the run
%     c3_ramp      false       true: the GWO coefficient grows from 0 to c3
%     pack_ramp    0           > 0: over this fraction of the run the GWO
%                              coefficient grows from 0 to c3 while c1 and c2
%                              shrink from c1 + c3/2 and c2 + c3/2, keeping
%                              the sum of the three (0: off)
%     vmax         0.2         velocity clamp, as a fraction of each range
%                              (the paper's |v| <= 1 is in raw units)
%     Seeds        []          m x nvar rows in [0,1] to put into the swarm
%     SeedFraction 0.3         fraction of the swarm initialised as jittered
%                              copies of the seeds (the seeds themselves are
%                              always included unperturbed)
%     SeedJitter   0.05        standard deviation of that jitter
%     UseParallel  false       evaluate the swarm with PARFOR
%     Checkpoint   ''          .mat file saved after every iteration
%     Resume       false       continue from Checkpoint if it exists and was
%                              written with the same PopSize, MaxIter and
%                              CheckpointTag; otherwise it is ignored with a
%                              warning (and overwritten)
%     CheckpointTag []         any value identifying the cost function
%                              settings, stored in and compared with the
%                              checkpoint
%     RandomSeed   []          seed for RNG, [] leaves the generator alone
%     Verbose      true        print progress
%
%   See also TUNE_FOPID_HYBRID, FOPID_FITNESS.

if nargin < 3, opts = struct(); end
opts = set_defaults(opts, nvar);
N = opts.PopSize;
if N < 3
    error('hybrid_fopso_gwo:pop', 'PopSize must be at least 3 (alpha, beta, delta)');
end

st = [];
if ~isempty(opts.Checkpoint) && opts.Resume && exist(opts.Checkpoint, 'file')
    S = load(opts.Checkpoint, 'state');
    st = S.state;
    if ~isfield(st, 'tag') || ~isequal(st.tag, opts.CheckpointTag) ...
            || st.nvar ~= nvar || st.N ~= N || st.MaxIter ~= opts.MaxIter
        warning('hybrid_fopso_gwo:resume', ...
                ['checkpoint %s is from a run with different settings ' ...
                 '(%d particles x %d iterations); ignoring it and starting afresh'], ...
                opts.Checkpoint, st.N, st.MaxIter);
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
t_iter  = [];                        % durations of the iterations of this call
for k = st.k + 1 : opts.MaxIter
    t_k = tic;
    tau = k / opts.MaxIter;
    w   = opts.wmax - (opts.wmax - opts.wmin) * tau;
    a   = opts.alpha0 - opts.alpha_drop * max(0, (tau - opts.alpha_hold) / (1 - opts.alpha_hold));
    a_g = 2 * (1 - tau)^opts.gwo_power;
    c1  = opts.c1;  c2 = opts.c2;  c3 = opts.c3;
    if opts.c3_ramp, c3 = c3 * tau; end
    if opts.pack_ramp > 0                % PSO at the start, FOPSO-GWO from pack_ramp on
        r  = min(1, tau / opts.pack_ramp);
        c1 = c1 + opts.c3 * (1 - r) / 2;
        c2 = c2 + opts.c3 * (1 - r) / 2;
        c3 = opts.c3 * r;
    end

    % ---- velocity: fractional memory + PSO + GWO attractors -------------
    g1 = a * (1 - a) / 2;
    g2 = g1 * (2 - a) / 3;
    g3 = g2 * (3 - a) / 4;
    memory = (w - 1 + a) * st.V + g1 * st.V1 + g2 * st.V2 + g3 * st.V3;

    x_gwo = grey_wolf_target(st.X, st.pbest, st.pf, a_g);
    gbest = repmat(st.g, N, 1);

    V = memory ...
        + c1      * rand(N, nvar) .* (st.pbest - st.X) ...
        + c2      * rand(N, nvar) .* (gbest    - st.X) ...
        + c3      * rand(N, nvar) .* (x_gwo    - st.X);
    V = min(max(V, -opts.vmax), opts.vmax);

    % ---- position, kept inside the unit box ------------------------------
    X = st.X + V;
    out = X < 0 | X > 1;
    X = min(max(X, 0), 1);
    V(out) = 0;                                  % stop at the wall

    st.V3 = st.V2;  st.V2 = st.V1;  st.V1 = st.V;  st.V = V;
    st.X  = X;

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
        % the ETA follows the last few iterations: early ones are cheap
        % because unstable candidates are aborted within a fraction of a
        % second, and iterations slow down as more of the swarm is stable
        el = toc(t_start);
        t_iter(end+1) = toc(t_k); %#ok<AGROW>
        eta = mean(t_iter(max(1, end-2):end)) * (opts.MaxIter - k);
        fprintf('  iter %3d/%d  best %.6g  swarm mean %.6g  (%.0f s, ETA %s)\n', ...
                k, opts.MaxIter, st.gf, st.mean_history(k), el, format_time(eta));
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
%INITIALISE  Random swarm plus the caller's seeds, evaluated once.
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
t0 = tic;
F = evaluate(cost, X, opts.UseParallel);
if opts.Verbose
    fprintf('  initial swarm done in %.0f s; one iteration will take about as long\n', toc(t0));
end

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
function x_gwo = grey_wolf_target(X, pbest, pf, a_g)
%GREY_WOLF_TARGET  Prey estimate from the alpha, beta and delta wolves.
[N, n] = size(X);
[~, order] = sort(pf);
x_gwo = zeros(N, n);
for L = order(1:3)'
    leader = repmat(pbest(L, :), N, 1);
    A = 2 * a_g * rand(N, n) - a_g;
    C = 2 * rand(N, n);
    D = abs(C .* leader - X);
    x_gwo = x_gwo + (leader - A .* D);
end
x_gwo = x_gwo / 3;
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
d = struct('PopSize', 30, 'MaxIter', 100, 'c1', 1.5, 'c2', 1.5, 'c3', 1, 'gwo_power', 2, ...
           'wmin', 0.4, 'wmax', 0.9, 'alpha0', 0.9, 'alpha_drop', 0, 'alpha_hold', 0, ...
           'c3_ramp', false, 'pack_ramp', 0, ...
           'vmax', 0.2, 'Seeds', zeros(0, nvar), 'SeedFraction', 0.3, ...
           'SeedJitter', 0.05, 'UseParallel', false, 'Checkpoint', '', ...
           'Resume', false, 'CheckpointTag', [], 'RandomSeed', [], 'Verbose', true);
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
if ~isempty(opts.Seeds) && size(opts.Seeds, 2) ~= nvar
    error('hybrid_fopso_gwo:seeds', 'Seeds must have %d columns', nvar);
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
%SAVE_STATE  Checkpoint the optimiser state, in MAT format under Octave too.
if exist('OCTAVE_VERSION', 'builtin')
    save('-mat7-binary', file, 'state');
else
    save(file, 'state');
end
end
