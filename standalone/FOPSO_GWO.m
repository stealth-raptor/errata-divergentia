function results = FOPSO_GWO(mode, seed, iterations)
%FOPSO_GWO  The FOPID tuned by FOPSO-GWO, this work's fractional-order PSO / grey-wolf hybrid.
%
%   results = FOPSO_GWO()                 the stored controller (random seed 6, the best of
%                                         seeds 1-8): cost, metrics, torque, plots
%   results = FOPSO_GWO('tune', seed)     run FOPSO-GWO from the FOPID with that random seed
%   results = FOPSO_GWO('tune', seed, n)  ... for n iterations instead of 100
%
%   Jiang, Zhang & Liu, "Trajectory tracking control of a 6-DOF robotic arm
%   based on improved FOPID", Int. J. Dyn. Control 13:137 (2025), tune the
%   FOPID (Eq. 9) with FBPA.  This work keeps FBPA's fractional-order
%   velocity (Eq. 26) and replaces its beetle-antennae term, which has no
%   effect at the paper's step size, by a grey-wolf (GWO) pull towards the
%   three best solutions (RUN_FOPSO_GWO).
%
%   The cost is this work's whole-controller cost (WHOLE_COST): tracking
%   against the paper's FBPA-FOPID, within the torque that the paper's own
%   controllers need on every joint.  Both of the paper's experiments on
%   its arm (Table 2, Eq. 21):
%     step   1 rad on every joint at t = 1 s           -> Table 3, Figs 6-12
%     sine   sin(1.5 t) rad on every joint for 5 s      -> Table 4, Figs 13-19
%   one gain set for both.  Budget as the paper's FBPA: 30 particles x 100
%   iterations, 3030 cost evaluations; the start is the paper's FOPID.
%
%   FOPSO-GWO is the first stage of the proposed FOPSO-GWO-CC
%   (FOPSO_GWO_CC.m), which improves on it on four of the five metrics.
%
%   Run time of 'tune': 3030 cost evaluations of 2-4 simulations each.  Use
%   the compiled simulation (build_mex.m and simulate_mex.c from the
%   repository, on the path): about 6 minutes.  Without it a simulation
%   takes seconds, so the full run takes many hours; try a few iterations
%   first, e.g. FOPSO_GWO('tune', 6, 3).  MATLAB's random numbers differ
%   from Octave's for the same seed, so a MATLAB run is a new sample, not
%   a copy of the stored one (the stored seed 6 was run in Octave).
%
%   results: gains, cost, itae, summary = [overshoot, adjustment time, peak
%   time, sine MSE, sine sum |tau|], the step and sine runs (t, r, q, u on
%   the 0.01 s grid), their metrics, the torque (effort) and the share of
%   each torque cap used (cap_use); for 'tune' the convergence (info).
%
%   Self-contained: the arm, controller, simulation and metrics are copies
%   of the repository's model files, the optimiser and the cost of its
%   hybrid_fopso_gwo.m, tune_fopid_hybrid.m and fopid_fitness.m; no
%   toolbox needed.
%
%   See also FOPSO_GWO_CC, FBPA_FOPID, FOPID_CONVENTIONAL, PID_CONVENTIONAL.

if nargin < 1 || isempty(mode), mode = 'tuned'; end
if nargin < 2 || isempty(seed), seed = 6; end
if nargin < 3 || isempty(iterations), iterations = 100; end
P = arm_parameters();
K = whole_cost_constants();
switch lower(mode)
    case 'tuned'
        fprintf('\nFOPSO-GWO: the stored controller (random seed 6, the best of seeds 1-8; stored cost 1.41564)\n');
        results = report_tuned('FOPSO-GWO', P, tuned_gains(), K);
    case 'tune'
        B = search_space();
        z_start = min(max(encode(fopid_start_gains(), B), 0), 1);
        cost = @(z) whole_cost(P, decode(z, B), K, true);
        rng(seed);
        [z_best, info] = run_fopso_gwo(cost, z_start, iterations);
        fprintf('\nFOPSO-GWO, random seed %d, %d iterations: best cost %.6g\n', seed, iterations, info.cost);
        results = report_tuned('FOPSO-GWO', P, decode(z_best, B), K, info);
        results.info = info;
    otherwise
        error('FOPSO_GWO:mode', 'mode must be ''tuned'' or ''tune''');
end
end

function g = tuned_gains()
%TUNED_GAINS  The FOPID tuned by FOPSO-GWO, random seed 6 (repository:
%   results/fopso_gwo_gains.mat, whole-controller cost 1.41564).
g.Kp     = [110.72184318697178 0.20543957877119498 0.033457337433226042 590.00312944229563 33.955373516894106 3392.616097811368]';
g.Ki     = [42.814444185152098 132.46035874383378 0.72649277291578462 43.964733451892172 4.4197720216728209 0.00043289756606391396]';
g.Kd     = [156.0617438234122 989.13385770545108 890.04115140458964 9.6225696080837988 171.51333398414212 0.36500541416620341]';
g.lambda = [1.5338872604827252 0.07579255484342115 1.377235222258302 0.07648682955853589 0.52510008371723238 0.050093224556129912]';
g.mu     = [0.82392484118460385 0.51723094125952362 0.52885618073695473 0.97275798529106672 0.73216755921113152 1.6365358055818044]';
end

% =========================================================================
%  FOPSO-GWO, this work's optimiser (repository: hybrid_fopso_gwo.m with
%  its final settings)
% =========================================================================

function [z_best, info] = run_fopso_gwo(cost, z_start, MaxIter)
%RUN_FOPSO_GWO  Fractional-order PSO hybridised with the grey wolf optimiser.
%   FBPA's velocity equation (Eq. 26) with the beetle term replaced by a
%   grey-wolf term:
%     v(k+1) = (w-1+a) v(k) + a(1-a)/2 v(k-1) + a(1-a)(2-a)/6 v(k-2)
%              + a(1-a)(2-a)(3-a)/24 v(k-3)          fractional memory (Eq. 25)
%              + c1 r1 (pbest - x) + c2 r2 (gbest - x)                 PSO
%              + c3 r3 (x_gwo - x)                                     GWO
%     x(k+1) = x(k) + v(k+1), kept inside the unit box
%   x_gwo is the grey-wolf estimate of the prey from the three best personal
%   bests, the alpha, beta and delta wolves (GREY_WOLF_TARGET).
%   Settings, chosen on development seeds 101-104 against plain PSO, never
%   on the test seeds 1-8 (README D10-D11):
%     c1 = c2 = 1.5, c3 = 1   the three pulls add up to PSO's 4
%     a = 0.9, held           the paper's Eq. 27 lowers it to 0.4, which
%                             drains the velocity memory (0.86 -> 0.03)
%     w = 0.9 -> 0.4 linearly; a_g = 2 (1 - k/MaxIter)^2; |v| <= 0.2
%   30 particles; 30 + 30 MaxIter evaluations (3030 for MaxIter = 100).
N = 30;
c1 = 1.5;  c2 = 1.5;  c3 = 1;
wmin = 0.4;  wmax = 0.9;
a = 0.9;
vmax = 0.2;

nvar = numel(z_start);
X = initial_swarm(N, z_start);
fprintf('FOPSO-GWO: %d particles x %d iterations; evaluating the initial swarm ...\n', N, MaxIter);
F = evaluate(cost, X);
V = zeros(N, nvar);  V1 = V;  V2 = V;  V3 = V;
pbest = X;  pf = F;
[gf, gi] = min(F);
g = X(gi, :);
evals = N;
history = nan(1, MaxIter);
evals_at = nan(1, MaxIter);
fprintf('  initial best %.6g\n', gf);
g1 = a * (1 - a) / 2;                    % Grunwald-Letnikov weights
g2 = g1 * (2 - a) / 3;
g3 = g2 * (3 - a) / 4;
for k = 1:MaxIter
    tau = k / MaxIter;
    w   = wmax - (wmax - wmin) * tau;
    a_g = 2 * (1 - tau)^2;

    memory = (w - 1 + a) * V + g1 * V1 + g2 * V2 + g3 * V3;
    x_gwo = grey_wolf_target(X, pbest, pf, a_g);
    gbest = repmat(g, N, 1);
    Vn = memory ...
        + c1 * rand(N, nvar) .* (pbest - X) ...
        + c2 * rand(N, nvar) .* (gbest - X) ...
        + c3 * rand(N, nvar) .* (x_gwo - X);
    Vn = min(max(Vn, -vmax), vmax);

    X = X + Vn;
    out = X < 0 | X > 1;
    X = min(max(X, 0), 1);
    Vn(out) = 0;                         % stop at the wall
    V3 = V2;  V2 = V1;  V1 = V;  V = Vn;

    F = evaluate(cost, X);
    evals = evals + N;
    better = F < pf;
    pbest(better, :) = X(better, :);
    pf(better) = F(better);
    [fmin, imin] = min(pf);
    if fmin < gf
        gf = fmin;
        g = pbest(imin, :);
    end
    history(k) = gf;
    evals_at(k) = evals;
    fprintf('  iteration %3d/%d  best %.6g  (%d evaluations)\n', k, MaxIter, gf, evals);
end
z_best = g;
info.cost = gf;
info.history = history;
info.evaluations = evals_at;
end

function x_gwo = grey_wolf_target(X, pbest, pf, a_g)
%GREY_WOLF_TARGET  The grey-wolf estimate of the prey for every particle:
%     D_L = |C .* L - x|,  X_L = L - A .* D_L,   L in {alpha, beta, delta}
%     A = 2 a_g r - a_g,  C = 2 r',  x_gwo = (X_alpha + X_beta + X_delta) / 3
%   Early on |A| can exceed 1 and the term explores around and past the
%   leaders; as a_g falls to 0 it refines the three best basins.
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

% =========================================================================
%  ITAE (paper Eq. 29) and the torque a controller needs (repository:
%  fopid_fitness.m, control_effort.m)
% =========================================================================

function v = itae(out)
%ITAE  Integral of t * sum_j |e_j(t)| dt (Eq. 29), trapezoidal, on the
%   samples of OUT.
v = trapz(out.t, out.t .* sum(abs(out.r - out.q), 1));
end

function out = on_grid(out)
%ON_GRID  The samples on the paper's 0.01 s grid of a run logged faster.
every = round(0.01 / (out.t(2) - out.t(1)));
if every <= 1, return; end
k = 1:every:numel(out.t);
out.t = out.t(k);  out.r = out.r(:, k);  out.q = out.q(:, k);  out.u = out.u(:, k);
end

function e = control_effort(out, reference)
%CONTROL_EFFORT  Torque per joint (1x6) of a run logged at the control rate.
%     peak   max |tau|
%     sum    sum of |tau| on the paper's 0.01 s grid
%     tv     total variation, sum |tau(k+1) - tau(k)| at 1 kHz
%   For the step these leave out the derivative kick, the first 50 ms after
%   the step at t = 1 s, which is reported on its own:
%     kick   max |tau| within those 50 ms (zero for the sine)
u = out.u;
t = out.t;
dt = t(2) - t(1);
every = max(1, round(0.01 / dt));
switch reference
    case 'step'
        in_kick = t >= 1 - 1e-9 & t < 1 + 0.05 - 1e-9;
        keep = ~in_kick;
        e.kick = max(abs(u(:, in_kick)), [], 2)';
    case 'sine'
        keep = true(size(t));
        e.kick = zeros(1, size(u, 1));
    otherwise
        error('control_effort:ref', 'unknown reference ''%s''', reference);
end
grid = false(size(t));
grid(1:every:end) = true;
e.peak = max(abs(u(:, keep)), [], 2)';
e.sum  = sum(abs(u(:, keep & grid)), 2)';
du = abs(diff(u, 1, 2));
e.tv   = sum(du(:, keep(2:end) & keep(1:end-1)), 2)';
end

% =========================================================================
%  The whole-controller cost (README D7; repository: fopid_fitness.m with
%  Fitness 'whole', and its references from tune_fopid_hybrid.m)
% =========================================================================

function [J, raw, R] = whole_cost(P, gains, K, robust)
%WHOLE_COST  Tracking within the torque the paper's own controllers use.
%   Tracking alone rewards ever stiffer controllers, so this cost balances
%   it against the torque it takes.  Both experiments are simulated with
%   the torque logged at the control rate (1 kHz); the paper's metrics are
%   taken on its 0.01 s grid.
%     raw = [ITAE step, ITAE sine, overshoot, adjustment time, peak time,
%            sine MSE, sine sum |tau|]
%   and ratio = raw ./ K.ref, the adjustment and peak times measured from
%   the step at t = 1 s (from t = 0 they all start at 1 s and the ratio
%   would hardly move).  K.ref holds the paper's FBPA-FOPID: its Table 3
%   and 4 values and, for the ITAEs, its reproduction.
%     tracking  ratios 1-6, each floored at 0.5: no extra credit for
%               being more than twice as good on one metric
%     effort    sum |tau| and total variation of tau in both runs (the
%               step's 50 ms derivative kick left out), relative to the
%               reproduced FBPA-FOPID
%     J = (sum(w_track .* tracking) + sum(effort)) / (sum(w_track) + 4)
%         + 2 * sum over the five paper metrics of max(0, ratio - 1)
%         + 10 * sum over joints of max(0, peak / cap - 1)   for the kick,
%                the step after the kick and the sine run, and of
%                max(0, overshoot / 33.07 % - 1)
%   so every paper metric must beat the paper's FBPA-FOPID table, and no
%   joint may need more torque than the paper's own controllers need there
%   (24 caps per controller, K.cap_*).  Unstable candidates score 1e3-2e3;
%   with robust = true an ill-conditioned closed loop gets 10 added.
%   R returns the runs (on the 0.01 s grid), their metrics and torque.
raw = nan(1, 7);
R = [];
full_step = simulate_arm(P, gains, 'step', 1e-3, 5);
if full_step.diverged
    J = diverged_cost(full_step, 0);
    return;
end
full_sine = simulate_arm(P, gains, 'sine', 1e-3, 5);
if full_sine.diverged
    J = diverged_cost(full_sine, 0.5);
    return;
end
step = on_grid(full_step);
sine = on_grid(full_sine);
es = control_effort(full_step, 'step');
en = control_effort(full_sine, 'sine');
ms = step_metrics(step);
mn = sine_metrics(sine);
raw = [itae(step), itae(sine), ms.mean, mn.mean];

num = raw;  den = K.ref;
num(4:5) = num(4:5) - 1;                     % times after the step at t = 1 s
den(4:5) = den(4:5) - 1;
ratio = max(num, 0) ./ max(den, eps);
track = max(ratio(1:6), K.floor);
eff = [mean(es.sum) / K.effort_ref(1), mean(en.sum) / K.effort_ref(2), ...
       mean(es.tv)  / K.effort_ref(3), mean(en.tv)  / K.effort_ref(4)];
over = sum(max(0, es.kick ./ K.cap_kick - 1)) + sum(max(0, es.peak ./ K.cap_step - 1)) ...
       + sum(max(0, en.peak ./ K.cap_sine - 1)) + sum(max(0, ms.overshoot(:)' / K.os_cap - 1));
J = (sum(K.w_track .* track) + sum(K.w_effort .* eff)) / (sum(K.w_track) + sum(K.w_effort)) ...
    + K.regret * sum(max(0, ratio(3:7) - 1)) + K.cap_penalty * over;
if robust && sensitivity(P, gains, 1e-3, full_step, full_sine) > 1e-6
    J = J + 10;
end
R.step = step;  R.sine = sine;
R.step_metrics = ms;  R.sine_metrics = mn;
R.effort = struct('step', es, 'sine', en);
end

function K = whole_cost_constants()
%WHOLE_COST_CONSTANTS  The fixed references of the whole-controller cost,
%   exactly as every reported tuning run used them.  The repository
%   measures them (tune_fopid_hybrid.m, WHOLE_REFERENCES) from the
%   reproductions of the paper's three controllers:
%     cap_*       on each joint, the largest peak torque any of PID, FOPID
%                 and FBPA-FOPID needs (every identified gain set), in the
%                 kick (first 50 ms after the step), the rest of the step
%                 run and the sine run [Nm]
%     os_cap      the worst joint's overshoot in the paper's own FBPA-FOPID
%                 step curves (Figs 6-11) [%]
%     effort_ref  sum |tau| and total variation of the reproduced
%                 FBPA-FOPID: step sum, sine sum, step tv, sine tv [Nm]
%     ref         ITAE step and sine of the reproduced FBPA-FOPID, then the
%                 paper's FBPA-FOPID Tables 3-4: overshoot 22.1 %,
%                 adjustment time 1.43 s, peak time 1.09 s, MSE 3.7e-3,
%                 sum |tau| 2.3154e4
K.cap_kick = [1112945.044476124 210541.76064953001 150222.20033704062 ...
              10901.415755919243 45516.919989003676 50848.439774552615];
K.cap_step = [1523.7391915331837 1495.479468385015 754.32620078048728 ...
              159.66329230716013 60.897716310826489 638.99128636290573];
K.cap_sine = [1669.4169406826688 1902.3890078125041 627.24171713231226 ...
              495.06241839168416 480.82127243345695 64.291241775508993];
K.os_cap = 33.074219999999997;
K.effort_ref = [6342.6987597849447 10144.675395325123 934.72073423560744 1444.7790457855272];
K.ref = [1.1582686404952576 2.4396181225667917 22.1 1.43 1.09 3.7e-3 2.3154e4];
K.floor = 0.5;
K.w_track = [0.5 0.5 1 1 1 1];               % ITAE step, sine; four paper metrics
K.w_effort = [1 1 1 1];
K.regret = 2;
K.cap_penalty = 10;
end

% =========================================================================
%  The search space and the starting swarm (repository: tune_fopid_hybrid.m,
%  the initialisation shared by pso.m, fbpa.m and hybrid_fopso_gwo.m)
% =========================================================================

function B = search_space()
%SEARCH_SPACE  All 30 FOPID parameters at once (the joints are coupled),
%   mapped onto the unit box [0, 1]^30:
%     z = [Kp(1:6) Ki(1:6) Kd(1:6) lambda(1:6) mu(1:6)]
%   with the gains on a log10 scale: log10 Kp in [-2, 5], log10 Ki in
%   [-4, 5], log10 Kd in [-1, 3]; lambda, mu in [0.05, 1.95].  Every gain
%   of the FOPID start controller lies inside.
B.lo = kron([-2 -4 -1 0.05 0.05], ones(1, 6));
B.hi = kron([ 5  5  3 1.95 1.95], ones(1, 6));
B.is_log = [true(1, 18) false(1, 12)];
B.n = 30;
end

function z = encode(g, B)
%ENCODE  Gains -> point of the unit box.
x = [g.Kp(:); g.Ki(:); g.Kd(:); g.lambda(:); g.mu(:)]';
x(B.is_log) = log10(max(x(B.is_log), realmin));
z = (x - B.lo) ./ (B.hi - B.lo);
end

function g = decode(z, B)
%DECODE  Point of the unit box -> gains.
x = B.lo + z .* (B.hi - B.lo);
x(B.is_log) = 10 .^ x(B.is_log);
g.Kp     = x(1:6)';
g.Ki     = x(7:12)';
g.Kd     = x(13:18)';
g.lambda = x(19:24)';
g.mu     = x(25:30)';
end

function X = initial_swarm(N, z_start)
%INITIAL_SWARM  N random particles in the unit box; the first is the start
%   controller and the next round(0.3 N) are copies of it jittered with
%   standard deviation 0.05.  For the same random seed every optimiser of
%   the repository (PSO, FBPA, FOPSO-GWO, FOPSO-GWO-CC) starts from this
%   same swarm.
nvar = numel(z_start);
X = rand(N, nvar);
X(1, :) = z_start;
for i = 1:min(N - 1, round(0.3 * N))
    X(1 + i, :) = min(max(z_start + 0.05 * randn(1, nvar), 0), 1);
end
end

function F = evaluate(cost, X)
%EVALUATE  Cost of every row of X; a non-finite cost counts as 1e12.
F = zeros(size(X, 1), 1);
for i = 1:size(X, 1)
    F(i) = cost(X(i, :));
end
F(~isfinite(F)) = 1e12;
end

function d = sensitivity(P, gains, log_dt, step, sine)
%SENSITIVITY  How far the response moves when every gain changes by a
%   relative 1e-10.  A well-conditioned closed loop moves ~1e-9 rad; one
%   chattering in a round-off-sensitive regime ~1e-3 rad, and its results
%   would not reproduce on another machine.
g = gains;
for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
    g.(f{1}) = gains.(f{1}) * (1 + 1e-10);
end
a = simulate_arm(P, g, 'step', log_dt, 5);
b = simulate_arm(P, g, 'sine', log_dt, 5);
if a.diverged || b.diverged
    d = Inf;
    return;
end
d = max(max(abs(a.q(:) - step.q(:))), max(abs(b.q(:) - sine.q(:))));
end

function J = diverged_cost(out, survived_before)
%DIVERGED_COST  An unstable candidate scores 1e3-2e3: lower the longer it
%   held on (survived_before = 0.5 credits a completed step run when the
%   sine run diverged).
frac = survived_before + 0.5 * out.t(end) / 5;
J = 1e3 + 1e3 * (1 - frac);
end

function g = fopid_start_gains()
%FOPID_START_GAINS  The start controller of every tuning run: the paper's
%   FOPID, with the one gain set identified from both of its experiments'
%   published curves (repository: controller_gains('FOPID', 'shared')).
g.Kp     = [466.5048899 0.2050011905 0.07599433869 124.5656189 2.134765482 28.46272228]';
g.Ki     = [348.1945821 2206.077613 54.67759987 11.92085781 2.285310734 0.009220016116]';
g.Kd     = [49.95111633 645.0353382 591.4914619 0.4691240383 1.473705321 0.1510568454]';
g.lambda = [1.662623676 0.6526495045 1.95 0.3800523434 0.7206369078 0.05006636685]';
g.mu     = [1.425361662 0.5509011254 0.3845477049 1.453723122 0.7627629958 1.817543914]';
end

% =========================================================================
%  The arm: the paper's Table 2 and Eq. 21 (repository: robot_params.m,
%  robot_dynamics.m)
% =========================================================================

function P = arm_parameters()
%ARM_PARAMETERS  The 6-DOF arm of the paper.
%   Masses, principal inertias and link lengths: Table 2, as printed.
%   Kinematics: the UR5 of Fig. 3, standard Denavit-Hartenberg.
%   Not published, chosen as in the repository (README, D1):
%     centre of mass at each DH frame origin (Robotics Toolbox default);
%     gravity g = 0 (the published responses are flat before the step);
%     no friction (Eq. 21 has the term but no coefficients).
P.m = [2.0 2.5 5.7 3.9 2.5 2.5];                               % kg
P.Idiag = [1.0 1.0 1.0                                         % kg m^2, about the COM
           4.0 4.0 4.0
           6.0 6.0 4.0
           5.5 5.5 4.0
           4.0 4.0 4.0
           4.0 4.0 4.0] .* [1e-3; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];
L = [0.128 0.612 0.571 0.164 0.115 0.092];                     % m
d     = [L(1)  0     0     L(4)  L(5)  L(6)];
a     = [0    -L(2) -L(3)  0     0     0   ];
alpha = [pi/2  0     0     pi/2 -pi/2  0   ];
P.ca = cos(alpha);
P.sa = sin(alpha);
P.pstar = [a; d .* P.sa; d .* P.ca];   % origin of frame i seen from frame i-1, in frame i
P.rc = zeros(3, 6);                    % centre of mass of link i, in frame i
P.g  = 0;                              % gravity [m/s^2], along -z0
P.fc = zeros(6, 1);                    % Coulomb friction
P.b  = zeros(6, 1);                    % viscous friction
end

function [M, h] = arm_dynamics(P, q, qd)
%ARM_DYNAMICS  M(q) qdd + C(q, qd) qd + G(q) + tau_f(qd) = tau  (Eq. 21).
%   Recursive Newton-Euler (standard DH, Corke's formulation), which gives
%   the same M, C and G as the paper's Lagrange form (Eqs. 10-20):
%     h             = RNEA(q, qd, qdd = 0) with gravity, plus friction
%     column j of M = RNEA(q, qd = 0, qdd = e_j) without gravity
%   The seven recursions run at once as the columns of 3x7 arrays.
QD  = [zeros(6, 6), qd(:)];
QDD = [eye(6), zeros(6, 1)];
w  = zeros(3, 7);                 % angular velocity
wd = zeros(3, 7);                 % angular acceleration
vd = zeros(3, 7);                 % linear acceleration of the frame origin
vd(3, 7) = P.g;                   % gravity as an upward base acceleration
Fx = zeros(6, 7); Fy = Fx; Fz = Fx;      % force on the COM of each link
Nx = Fx;          Ny = Fx; Nz = Fx;      % moment about the COM
ct = cos(q);  st = sin(q);
Rs = zeros(3, 3, 6);
for i = 1:6                       % outward recursion
    R = [ct(i), -st(i)*P.ca(i),  st(i)*P.sa(i)
         st(i),  ct(i)*P.ca(i), -ct(i)*P.sa(i)
         0,      P.sa(i),        P.ca(i)];
    Rs(:, :, i) = R;
    Rt = R.';
    qdi = QD(i, :);
    wd = Rt * [wd(1,:) + w(2,:).*qdi;  wd(2,:) - w(1,:).*qdi;  wd(3,:) + QDD(i,:)];
    w  = Rt * [w(1,:);  w(2,:);  w(3,:) + qdi];
    p = P.pstar(:, i);
    c1 = w(2,:)*p(3) - w(3,:)*p(2);  c2 = w(3,:)*p(1) - w(1,:)*p(3);  c3 = w(1,:)*p(2) - w(2,:)*p(1);
    vd = Rt * vd + [wd(2,:)*p(3) - wd(3,:)*p(2) + w(2,:).*c3 - w(3,:).*c2
                    wd(3,:)*p(1) - wd(1,:)*p(3) + w(3,:).*c1 - w(1,:).*c3
                    wd(1,:)*p(2) - wd(2,:)*p(1) + w(1,:).*c2 - w(2,:).*c1];
    r = P.rc(:, i);
    c1 = w(2,:)*r(3) - w(3,:)*r(2);  c2 = w(3,:)*r(1) - w(1,:)*r(3);  c3 = w(1,:)*r(2) - w(2,:)*r(1);
    m = P.m(i);
    Fx(i,:) = m * (vd(1,:) + wd(2,:)*r(3) - wd(3,:)*r(2) + w(2,:).*c3 - w(3,:).*c2);
    Fy(i,:) = m * (vd(2,:) + wd(3,:)*r(1) - wd(1,:)*r(3) + w(3,:).*c1 - w(1,:).*c3);
    Fz(i,:) = m * (vd(3,:) + wd(1,:)*r(2) - wd(2,:)*r(1) + w(1,:).*c2 - w(2,:).*c1);
    I = P.Idiag(i, :);
    Nx(i,:) = I(1)*wd(1,:) + (I(3) - I(2)) * w(2,:).*w(3,:);
    Ny(i,:) = I(2)*wd(2,:) + (I(1) - I(3)) * w(3,:).*w(1,:);
    Nz(i,:) = I(3)*wd(3,:) + (I(2) - I(1)) * w(1,:).*w(2,:);
end
tau = zeros(6, 7);                % inward recursion
f = zeros(3, 7);  n = zeros(3, 7);
for i = 6:-1:1
    if i < 6
        R = Rs(:, :, i+1);
        f = R * f;
        n = R * n;
    end
    p  = P.pstar(:, i);
    pr = p + P.rc(:, i);
    F  = [Fx(i,:); Fy(i,:); Fz(i,:)];
    n = n + [Nx(i,:); Ny(i,:); Nz(i,:)] ...
          + [p(2)*f(3,:) - p(3)*f(2,:);   p(3)*f(1,:) - p(1)*f(3,:);   p(1)*f(2,:) - p(2)*f(1,:)] ...
          + [pr(2)*F(3,:) - pr(3)*F(2,:); pr(3)*F(1,:) - pr(1)*F(3,:); pr(1)*F(2,:) - pr(2)*F(1,:)];
    f = f + F;
    z = Rs(3, :, i);              % joint axis z_{i-1} in frame i
    tau(i, :) = z * n;
end
M = tau(:, 1:6);
M = 0.5 * (M + M.');
h = tau(:, 7) + P.fc .* sign(qd(:)) + P.b .* qd(:);
end

function qdd = accel(P, q, qd, u)
%ACCEL  Forward dynamics, qdd = M(q) \ (tau - h(q, qd)).
[M, h] = arm_dynamics(P, q, qd);
qdd = M \ (u - h);
end

% =========================================================================
%  The FOPID controller, paper Eq. 9 (repository: fopid_controller.m,
%  fopid_update.m, fractional_operator.m)
% =========================================================================

function C = fopid_create(gains, dt)
%FOPID_CREATE  Six independent FOPID joint controllers,
%       u = Kp e + Ki D^(-lambda) e + Kd D^(mu) e        (Eq. 9)
%   gains: 6x1 fields Kp, Ki, Kd, lambda, mu (lambda = mu = 1 is a PID).
%   Each order is split into an integer part, applied as a plain integrator
%   or backward difference, and a remainder in [0, 1), realised by an
%   Oustaloup filter (OUSTALOUP).  No derivative filter, anti-windup or
%   torque saturation, as in the paper.
C.Kp = gains.Kp(:);
C.Ki = gains.Ki(:);
C.Kd = gains.Kd(:);
C.dt = dt;
n_i = floor(gains.lambda(:) + 1e-12);
n_d = floor(gains.mu(:)     + 1e-12);
if any(n_i > 1) || any(n_d > 1)
    error('fopid:order', 'lambda and mu must be below 2');
end
C.Fi = oustaloup(-(gains.lambda(:) - n_i), dt);   % integral branch
C.Fd = oustaloup(  gains.mu(:)     - n_d,  dt);   % derivative branch
C.use_integrator = n_i >= 1;
C.use_difference = n_d >= 1;
C.integral = zeros(6, 1);
C.prev_d   = [];
end

function [u, C] = fopid_update(C, e)
%FOPID_UPDATE  One control sample: e = r - q (6x1) -> joint torques u (6x1).
[yi, C.Fi] = filter_step(C.Fi, e);                % Ki * D^(-lambda) e
C.integral = C.integral + yi * C.dt;
term_i = yi;
term_i(C.use_integrator) = C.integral(C.use_integrator);
[yd, C.Fd] = filter_step(C.Fd, e);                % Kd * D^(mu) e
if isempty(C.prev_d)
    C.prev_d = yd;                                % no kick on the first sample
end
dyd = (yd - C.prev_d) / C.dt;
C.prev_d = yd;
term_d = yd;
term_d(C.use_difference) = dyd(C.use_difference);
u = C.Kp .* e + C.Ki .* term_i + C.Kd .* term_d;
end

function F = oustaloup(order, dt)
%OUSTALOUP  Discrete approximation of s^order (|order| < 1) for each joint.
%   Oustaloup's recursive approximation over [1e-3, 1e3] rad/s with N = 5
%   (11 poles and zeros, the FOMCON defaults; the paper gives no settings),
%   in parallel form  s^a ~= K + sum_k r_k / (s + p_k), every first-order
%   section discretised exactly with a zero-order hold at dt.
wb = 1e-3;  wh = 1e3;  N = 5;
M = 2*N + 1;
k = -N:N;
a = order(:);
if any(abs(a) >= 1)
    error('oustaloup:range', 'order must lie in (-1, 1)');
end
zeros_ = wb * (wh/wb).^((k + N + 0.5*(1 - a)) / M);
poles  = wb * (wh/wb).^((k + N + 0.5*(1 + a)) / M);
F.K = wh.^a;
F.r = zeros(numel(a), M);
for j = 1:M
    num = prod(zeros_ - poles(:, j), 2);
    den = prod(poles(:, [1:j-1 j+1:M]) - poles(:, j), 2);
    F.r(:, j) = F.K .* num ./ den;
end
F.A = exp(-poles * dt);
F.B = (1 - F.A) ./ poles;
F.x = zeros(numel(a), M);
end

function [y, F] = filter_step(F, e)
%FILTER_STEP  One zero-order-hold step of an Oustaloup filter bank.
y = F.K .* e + sum(F.r .* F.x, 2);
F.x = F.A .* F.x + F.B .* e;
end

% =========================================================================
%  One experiment of the paper (repository: simulate_closed_loop.m)
% =========================================================================

function out = simulate_arm(P, gains, reference, log_dt, abort_err)
%SIMULATE_ARM  The arm under the six FOPID joint controllers, for 5 s.
%   out = SIMULATE_ARM(P, gains, 'step' | 'sine', log_dt, abort_err)
%     'step'  1 rad on every joint at t = 1 s       (paper Figs 6-12)
%     'sine'  sin(1.5 t) rad on every joint         (paper Figs 13-19)
%   The controller runs at dt = 1 ms with a zero-order hold, and the arm is
%   integrated with one RK4 step per sample, from rest at q = 0.  q and u
%   are logged every log_dt seconds (default 0.01 s, the paper's grid).  The
%   run stops early, out.diverged = true, once any |error| exceeds
%   abort_err (default Inf) or the state leaves 1e3 rad.
%   If SIMULATE_MEX is on the path (the compiled copy of this loop; build
%   it with build_mex.m from the repository) it is used: same results,
%   several hundred times faster.
if nargin < 4, log_dt = 0.01; end
if nargin < 5, abort_err = Inf; end
dt = 1e-3;
Nt = round(5 / dt) + 1;
t  = (0:Nt-1) * dt;
every = round(log_dt / dt);
switch reference
    case 'step', r = repmat(double(t >= 1 - 1e-12), 6, 1);
    case 'sine', r = repmat(sin(1.5 * t), 6, 1);
    otherwise,   error('simulate_arm:ref', 'unknown reference ''%s''', reference);
end
C = fopid_create(gains, dt);

if exist('simulate_mex') == 3 %#ok<EXIST>
    [qo, uo, nlogged, diverged, kstop] = simulate_mex(P.pstar, P.rc, P.m, P.Idiag, P.ca, P.sa, ...
        P.g, P.fc, P.b, r, dt, every, C.Kp, C.Ki, C.Kd, C.Fi.K, C.Fi.r, C.Fi.A, C.Fi.B, ...
        C.Fd.K, C.Fd.r, C.Fd.A, C.Fd.B, double(C.use_integrator), double(C.use_difference), ...
        abort_err);
    keep = 1:size(qo, 2);
    if diverged
        keep = 1:max(nlogged, 1);
        if isinf(abort_err)
            warning('simulate_arm:unstable', 'closed loop diverged at t = %.3f s', t(kstop));
        end
    end
    out.t = t(1 + (keep - 1) * every);
    out.r = r(:, 1 + (keep - 1) * every);
    out.q = qo(:, keep);
    out.u = uo(:, keep);
    out.diverged = logical(diverged);
    return;
end

q  = zeros(6, 1);
qd = zeros(6, 1);
nlog = floor((Nt - 1) / every) + 1;
out.t = t(1:every:Nt);
out.r = r(:, 1:every:Nt);
out.q = zeros(6, nlog);
out.u = zeros(6, nlog);
out.diverged = false;
j = 0;
for k = 1:Nt
    e = r(:, k) - q;
    if max(abs(e)) > abort_err
        out.diverged = true;
        break;
    end
    [u, C] = fopid_update(C, e);
    if mod(k - 1, every) == 0
        j = (k - 1) / every + 1;
        out.q(:, j) = q;
        out.u(:, j) = u;
    end
    if k == Nt, break; end
    k1q = qd;              k1v = accel(P, q,            qd,  u);   % RK4, torque held
    k2q = qd + dt/2*k1v;   k2v = accel(P, q + dt/2*k1q, k2q, u);
    k3q = qd + dt/2*k2v;   k3v = accel(P, q + dt/2*k2q, k3q, u);
    k4q = qd + dt*k3v;     k4v = accel(P, q + dt*k3q,   k4q, u);
    q  = q  + dt/6 * (k1q + 2*k2q + 2*k3q + k4q);
    qd = qd + dt/6 * (k1v + 2*k2v + 2*k3v + k4v);
    if any(~isfinite(q)) || any(~isfinite(qd)) || any(abs(q) > 1e3)
        out.diverged = true;
        if isinf(abort_err)
            warning('simulate_arm:unstable', 'closed loop diverged at t = %.3f s', t(k));
        end
        break;
    end
end
if out.diverged
    keep = 1:max(j, 1);
    out.t = out.t(keep);
    out.r = out.r(:, keep);
    out.q = out.q(:, keep);
    out.u = out.u(:, keep);
end
end

% =========================================================================
%  The paper's metrics, Tables 3 and 4 (repository: performance_metrics.m)
% =========================================================================

function m = step_metrics(out)
%STEP_METRICS  Table 3, per joint and averaged over the six joints, on the
%   0.01 s grid: overshoot 100 (max q - 1) [%]; adjustment time, the last
%   exit from a +-5 % band, and peak time, both measured from t = 0 [s]
%   (as the paper's 1.09-1.39 s peak times for a step at t = 1 s require).
%   A joint that never settles counts as the 5 s horizon.
m.overshoot       = zeros(6, 1);
m.adjustment_time = zeros(6, 1);
m.peak_time       = zeros(6, 1);
for j = 1:6
    y = out.q(j, :);
    [ymax, k] = max(y);
    m.overshoot(j) = max(0, 100 * (ymax - 1));
    m.peak_time(j) = out.t(k);
    m.adjustment_time(j) = settling_time(out.t, y, 1, 0.05);
end
ts = m.adjustment_time;
ts(isnan(ts)) = out.t(end);
m.mean = [mean(m.overshoot), mean(ts), mean(m.peak_time)];
end

function m = sine_metrics(out)
%SINE_METRICS  Table 4, per joint and averaged, on the 0.01 s grid: mean
%   squared tracking error [rad^2] and sum of |tau| [Nm].
e = out.r - out.q;
m.mse    = mean(e.^2, 2);
m.torque = sum(abs(out.u), 2);
m.mean   = [mean(m.mse), mean(m.torque)];
end

function ts = settling_time(t, y, yfinal, band)
%SETTLING_TIME  First time after which y stays within +-band*yfinal (the
%   exit located by linear interpolation; NaN if it never settles).
outside = abs(y - yfinal) > band * abs(yfinal);
k = find(outside, 1, 'last');
if isempty(k)
    ts = t(1);
elseif k == numel(t)
    ts = NaN;
else
    lvl = yfinal + sign(y(k) - yfinal) * band * abs(yfinal);
    ts = t(k) + (lvl - y(k)) / (y(k+1) - y(k)) * (t(k+1) - t(k));
end
end

% =========================================================================
%  Report of a controller tuned with the whole-controller cost
% =========================================================================

function results = report_tuned(name, P, gains, K, info)
%REPORT_TUNED  Simulate a tuned FOPID, print its cost, the paper's metrics
%   against the paper's FBPA-FOPID, the torque against the 24 caps, and
%   plot the responses (and the convergence, if INFO is given).
[J, raw, R] = whole_cost(P, gains, K, true);
if isempty(R)
    error('report_tuned:unstable', 'the controller does not complete both experiments');
end
summary = raw(3:7);
fprintf('\n%s: whole-controller cost %.6g\n', name, J);
fprintf('  ITAE (Eq. 29): step %.4g, sine %.4g (the reproduced FBPA-FOPID: %.4g, %.4g)\n', ...
        raw(1), raw(2), K.ref(1), K.ref(2));
print_metrics({'Paper FBPA-FOPID, Tables 3 and 4', K.ref(3:7)
               ['FOPID tuned by ' name],            summary});
fprintf('  Times after the step at t = 1 s: adjustment %.3f s, peak %.3f s (paper FBPA-FOPID %.2f, %.2f)\n', ...
        summary(2) - 1, summary(3) - 1, K.ref(4) - 1, K.ref(5) - 1);
print_per_joint(R.step_metrics, R.sine_metrics);

es = R.effort.step;  en = R.effort.sine;
use = [es.kick ./ K.cap_kick; es.peak ./ K.cap_step; en.peak ./ K.cap_sine; ...
       R.step_metrics.overshoot(:)' / K.os_cap];
fprintf('\nShare of each per-joint cap used (1 = at the cap)  J1     J2     J3     J4     J5     J6\n');
labels = {'kick: peak torque, first 50 ms', 'step: peak torque after the kick', ...
          'sine: peak torque', 'step: overshoot'};
for i = 1:4
    fprintf('  %-48s%s\n', labels{i}, sprintf(' %6.2f', use(i, :)));
end
fprintf('  within all 24 caps: %s (largest share %.2f)\n', mat2str(all(use(:) <= 1)), max(use(:)));
fprintf('  torque at 1 kHz, mean over the joints: step sum %.4g Nm (kick excluded), sine sum %.4g Nm\n', ...
        mean(es.sum), mean(en.sum));
fprintf('\nGains:');
print_gains(gains);
plot_runs(name, R.step, R.sine);
if nargin >= 5 && ~isempty(info)
    figure('Name', [name ': convergence'], 'NumberTitle', 'off');
    semilogy(info.evaluations, info.history, 'k.-');
    xlabel('cost evaluations');  ylabel('best cost');  grid on;
    title([name ' on the whole-controller cost']);
end
results = struct('gains', gains, 'cost', J, 'itae', raw(1:2), 'summary', summary, ...
                 'step', R.step, 'sine', R.sine, 'step_metrics', R.step_metrics, ...
                 'sine_metrics', R.sine_metrics, 'effort', R.effort, 'cap_use', use);
end

% =========================================================================
%  Report and plots
% =========================================================================

function print_metrics(rows)
%PRINT_METRICS  The five metrics of the paper's Tables 3 and 4 (averages
%   over the six joints), one row per {label, [overshoot, adjustment time,
%   peak time, sine MSE, sine sum |tau|]}.
fprintf('\n%-38s %10s %10s %10s %11s %12s\n', '', 'Overshoot', 'Adjust.', 'Peak', 'Sine MSE', 'Sine sum');
fprintf('%-38s %10s %10s %10s %11s %12s\n', '', '(%)', 'time (s)', 'time (s)', '(rad^2)', '|tau| (Nm)');
for i = 1:size(rows, 1)
    fprintf('%-38s %10.1f %10.2f %10.2f %11.2e %12.4g\n', rows{i, 1}, rows{i, 2});
end
end

function print_per_joint(ms, mn)
%PRINT_PER_JOINT  The same metrics joint by joint.
fprintf('\n%-24s %10s %10s %10s %10s %10s %10s\n', 'Per joint', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6');
fprintf('%-24s%s\n', 'Overshoot (%)',       sprintf(' %10.1f', ms.overshoot));
fprintf('%-24s%s\n', 'Adjustment time (s)', sprintf(' %10.2f', ms.adjustment_time));
fprintf('%-24s%s\n', 'Peak time (s)',       sprintf(' %10.2f', ms.peak_time));
fprintf('%-24s%s\n', 'Sine MSE (rad^2)',    sprintf(' %10.2e', mn.mse));
fprintf('%-24s%s\n', 'Sine sum |tau| (Nm)', sprintf(' %10.4g', mn.torque));
end

function plot_runs(name, step, sine)
%PLOT_RUNS  Joint angles against the reference in both experiments (as the
%   paper's Figs 6-11 and 13-18), and the joint torques (Figs 12 and 19).
figure('Name', [name ': step response'], 'NumberTitle', 'off');
for j = 1:6
    subplot(3, 2, j);
    plot(step.t, step.r(j, :), 'r--', step.t, step.q(j, :), 'b-');
    title(sprintf('%s, joint %d', name, j));
    xlabel('t (s)');  ylabel('q (rad)');  grid on;
end
figure('Name', [name ': sine tracking'], 'NumberTitle', 'off');
for j = 1:6
    subplot(3, 2, j);
    plot(sine.t, sine.r(j, :), 'r--', sine.t, sine.q(j, :), 'b-');
    title(sprintf('%s, joint %d', name, j));
    xlabel('t (s)');  ylabel('q (rad)');  grid on;
end
figure('Name', [name ': joint torques'], 'NumberTitle', 'off');
subplot(2, 1, 1);
plot(step.t, step.u);
title(sprintf('%s, step response: joint torques', name));
xlabel('t (s)');  ylabel('\tau (Nm)');  grid on;
legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
subplot(2, 1, 2);
plot(sine.t, sine.u);
title(sprintf('%s, sine tracking: joint torques', name));
xlabel('t (s)');  ylabel('\tau (Nm)');  grid on;
legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
end

function print_gains(g)
%PRINT_GAINS  A gain set, ready to paste into MATLAB.
fprintf('\n');
for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
    fprintf('g.%-6s = [%s]'';\n', f{1}, strtrim(sprintf('%.17g ', g.(f{1}))));
end
end
