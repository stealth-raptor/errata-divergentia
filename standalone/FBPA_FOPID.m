function results = FBPA_FOPID(mode, seed, iterations)
%FBPA_FOPID  The paper's FBPA-FOPID: a FOPID tuned by FBPA, its fractional-order beetle antennae PSO.
%
%   results = FBPA_FOPID()                 the paper's FBPA-FOPID, reproduced: the FOPID with
%                                          the gains identified from its published
%                                          FBPA-FOPID curves, one set per experiment
%                                          (default; every reported result uses these)
%   results = FBPA_FOPID('shared')         the same, one gain set for both experiments
%   results = FBPA_FOPID('tuned')          the FOPID tuned by FBPA below, as stored in
%                                          the repository (random seed 1, the best of 1-4)
%   results = FBPA_FOPID('tune', seed)     run FBPA from the FOPID with that random seed
%   results = FBPA_FOPID('tune', seed, n)  ... for n iterations instead of the paper's 100
%
%   Jiang, Zhang & Liu, "Trajectory tracking control of a 6-DOF robotic arm
%   based on improved FOPID", Int. J. Dyn. Control 13:137 (2025).
%
%   The controller is the paper's FOPID (Eq. 9), u = Kp e + Ki D^(-lambda) e
%   + Kd D^(mu) e, one per joint at 1 kHz; its 30 parameters (5 per joint)
%   are tuned by FBPA (paper Sect. 3, its Sect. 4 settings; RUN_FBPA) on the
%   paper's fitness, the ITAE of the step response (Eq. 29; PAPER_COST).
%   Both experiments of the paper on its arm (Table 2, Eq. 21):
%     step   1 rad on every joint at t = 1 s           -> Table 3, Figs 6-12
%     sine   sin(1.5 t) rad on every joint for 5 s      -> Table 4, Figs 13-19
%   It prints the metrics of Tables 3 and 4 next to the paper's values, and
%   joint by joint, and plots the responses and the torques.
%
%   Two FBPA-FOPIDs.  The paper publishes neither its FBPA-FOPID gains nor
%   enough detail to rerun its tuning exactly (search space, start, seed),
%   so:
%     * the reproduction ('separate', 'shared'): gains identified from the
%       paper's own FBPA-FOPID curves, read from the vector graphics of its
%       PDF, matched in the fully coupled closed loop (repository README,
%       D6).  This is the FBPA-FOPID of every comparison with the paper.
%     * FBPA as an optimiser ('tuned', 'tune'): this file's FBPA, run from
%       the paper's FOPID on the paper's fitness, with the same search
%       space and starting swarm as every other optimiser in the
%       repository.  Seed 1 in Octave gave the stored 'tuned' controller,
%       cost 0.0955 (the FOPID scores 1).
%
%   Run time of 'tune': 9030 cost evaluations of 2-4 simulations each.  Use
%   the compiled simulation (build_mex.m and simulate_mex.c from the
%   repository, on the path): about 15 minutes.  Without it a simulation
%   takes seconds, so the full run takes a day or more; try a few
%   iterations first, e.g. FBPA_FOPID('tune', 1, 3).  MATLAB's random
%   numbers differ from Octave's for the same seed, so a MATLAB run is a
%   new sample, not a copy of the stored one.
%
%   results: gains, step and sine runs (t, r, q, u on the 0.01 s grid),
%   step_metrics, sine_metrics, summary = [overshoot, adjustment time, peak
%   time, sine MSE, sine sum |tau|]; for 'tuned' and 'tune' also cost, and
%   for 'tune' the convergence (info).
%
%   Self-contained: the arm, controller, simulation and metrics are copies
%   of the repository's model files, the optimiser and the cost of its
%   fbpa.m, tune_fopid_hybrid.m and fopid_fitness.m; no toolbox needed.
%
%   See also PID_CONVENTIONAL, FOPID_CONVENTIONAL, FOPSO_GWO, FOPSO_GWO_CC.

if nargin < 1 || isempty(mode), mode = 'separate'; end
if nargin < 2 || isempty(seed), seed = 1; end
if nargin < 3 || isempty(iterations), iterations = 100; end
paper = [22.1 1.43 1.09 0.37e-2 2.3154e4];      % the paper's Tables 3 and 4
P = arm_parameters();

switch lower(mode)
    case {'separate', 'shared'}
        if strcmpi(mode, 'separate')
            gains.step = fbpa_identified_gains('step');
            gains.sine = fbpa_identified_gains('sine');
            label = 'one gain set per experiment';
        else
            gains.step = fbpa_identified_gains('shared');
            gains.sine = gains.step;
            label = 'one gain set for both experiments';
        end
        step = simulate_arm(P, gains.step, 'step');
        sine = simulate_arm(P, gains.sine, 'sine');
        ms = step_metrics(step);
        mn = sine_metrics(sine);
        summary = [ms.mean, mn.mean];
        fprintf('\nFBPA-FOPID on the paper''s arm (gains identified from its FBPA-FOPID curves, %s)\n', label);
        print_metrics({'Paper, Tables 3 and 4', paper
                       'This simulation',       summary});
        print_per_joint(ms, mn);
        plot_runs('FBPA-FOPID', step, sine);
        results = struct('gains', gains, 'step', step, 'sine', sine, ...
                         'step_metrics', ms, 'sine_metrics', mn, 'summary', summary);
        return;

    case 'tuned'
        gains = fbpa_tuned_gains();
        label = 'the stored FBPA run, random seed 1';
        info = [];

    case 'tune'
        B = search_space();
        start = fopid_start_gains();
        [~, itae_ref] = paper_cost(P, start, [], false);    % the FOPID scores 1
        z_start = min(max(encode(start, B), 0), 1);
        cost = @(z) paper_cost(P, decode(z, B), itae_ref, true);
        rng(seed);
        [z_best, info] = run_fbpa(cost, z_start, iterations);
        gains = decode(z_best, B);
        label = sprintf('this run, random seed %d, %d iterations', seed, iterations);

    otherwise
        error('FBPA_FOPID:mode', 'mode must be ''separate'', ''shared'', ''tuned'' or ''tune''');
end

% a tuned controller: one gain set for both experiments
start = fopid_start_gains();
[~, itae_ref] = paper_cost(P, start, [], false);
J = paper_cost(P, gains, itae_ref, true);
step = simulate_arm(P, gains, 'step');
sine = simulate_arm(P, gains, 'sine');
ms = step_metrics(step);
mn = sine_metrics(sine);
summary = [ms.mean, mn.mean];
fprintf('\nFOPID tuned by FBPA on the paper''s fitness (%s)\n', label);
fprintf('  cost: step ITAE %.4g, %.6g x that of the FOPID start (%.4g)\n', itae(step), J, itae_ref);
if strcmpi(mode, 'tuned')
    fprintf('  (the stored run''s cost is 0.0954847; the same value confirms this file)\n');
end
print_metrics({'Paper FBPA-FOPID, Tables 3 and 4', paper
               'This controller',                  summary});
print_per_joint(ms, mn);
fprintf('\nGains:');
print_gains(gains);
plot_runs('FBPA-tuned FOPID', step, sine);
if ~isempty(info)
    figure('Name', 'FBPA: convergence', 'NumberTitle', 'off');
    semilogy(info.evaluations, info.history, 'k.-');
    xlabel('cost evaluations');  ylabel('best cost (FOPID = 1)');  grid on;
    title('FBPA on the paper''s fitness (ITAE, Eq. 29)');
end
results = struct('gains', gains, 'cost', J, 'step', step, 'sine', sine, ...
                 'step_metrics', ms, 'sine_metrics', mn, 'summary', summary, 'info', info);
end

% =========================================================================
%  The gains
% =========================================================================

function g = fbpa_identified_gains(set)
%FBPA_IDENTIFIED_GAINS  The paper's FBPA-FOPID, identified from its curves
%   (repository: controller_gains.m).  The comments give the rms difference
%   to the paper's published curves, joints 1-6 [rad].
switch set
    case 'step'
        % step 0.026 0.027 0.038 0.053 0.052 0.044   (mean 0.040)
        g.Kp     = [0.03019615241 0.003117763082 36.34195347 99.23208699 0.01844205206 1632.51821]';
        g.Ki     = [48.97499014 21.90373185 14.33853659 0.0002432365723 760.6432845 29730.60498]';
        g.Kd     = [14.08304669 917.4419973 81.67353576 8.897366238 9.229806161 0.229492629]';
        g.lambda = [0.0500283687 1.949451944 0.4205795346 0.1690771851 0.900550279 1.582303849]';
        g.mu     = [0.9998708848 0.514832496 0.9999230055 0.4464245556 0.784430861 1.747201606]';
    case 'sine'
        % sine 0.038 0.018 0.050 0.015 0.061 0.015   (mean 0.033)
        g.Kp     = [364.8420664 3.333513883 0.00254491802 116.4486735 107646.9101 0.3871261489]';
        g.Ki     = [581.954497 1900.905882 372.0667211 0.0003336347483 9221.445046 1157.612758]';
        g.Kd     = [169.7245658 1543.395685 520.1945999 5.553126174 0.1540965712 186.6467437]';
        g.lambda = [1.591565168 1.090930414 1.948322993 1.307296612 1.391655871 1.94005557]';
        g.mu     = [0.8397908885 0.4538816423 0.7757818084 1.405946266 1.910186661 0.5412488322]';
    case 'shared'
        % step 0.075 0.015 0.024 0.034 0.064 0.045   (mean 0.043)
        % sine 0.055 0.022 0.065 0.018 0.062 0.018   (mean 0.040)
        g.Kp     = [597.8824431 0.1434815132 455.9707133 53.45665593 0.04140659648 542.9101952]';
        g.Ki     = [446.7583618 1822.211343 0.01137724837 75.30711814 680.6070125 0.5540232669]';
        g.Kd     = [56.89198356 261.9893742 103.7086648 13.27766929 6.314529721 359.7143173]';
        g.lambda = [1.948766589 0.05392475073 0.8307652713 0.1919073142 0.4245580453 1.185441396]';
        g.mu     = [1.430396425 0.8295674254 0.9714340539 0.6204829484 1.285828505 0.5981791625]';
end
end

function g = fbpa_tuned_gains()
%FBPA_TUNED_GAINS  The FOPID tuned by RUN_FBPA on the paper's fitness,
%   random seed 1 (repository: results/fbpa_gains.mat, cost 0.0954847).
g.Kp     = [1902.9762545851943 99971.789401343689 0.043850895758055619 12002.955242569173 2155.1285087100582 14829.47101437592]';
g.Ki     = [269.10578482599095 1.8429752217221538 16.978497619617652 210.47282663896064 0.30115912697411573 0.00022265444109529825]';
g.Kd     = [69.429226800363125 722.11603927699196 997.7715579717692 0.61479554611539367 52.521855282587175 0.20798864719959465]';
g.lambda = [1.798082264675557 0.60810665738775804 1.9493048115947371 0.64488317089701275 0.11776705259680137 0.058500278642983697]';
g.mu     = [1.1554530263278102 0.39487050774277987 0.59649338793266227 1.8001679208265153 0.92088675401581954 1.7618953066083056]';
end

% =========================================================================
%  FBPA, the paper's optimiser, Sect. 3 with its Sect. 4 settings
%  (repository: fbpa.m)
% =========================================================================

function [z_best, info] = run_fbpa(cost, z_start, MaxIter)
%RUN_FBPA  Fractional-order beetle antennae PSO.  Every beetle is a particle
%   whose velocity has the fractional memory of FO-PSO (Grunwald-Letnikov,
%   four terms, Eq. 25) plus a beetle antennae search (BAS) step:
%     v(k+1) = (w-1+a) v(k) + a(1-a)/2 v(k-1) + a(1-a)(2-a)/6 v(k-2)
%              + a(1-a)(2-a)(3-a)/24 v(k-3)
%              + c1 r1 (pbest - x) + c2 r2 (gbest - x) + c3 r3 v_l      Eq. 26
%     x(k+1) = x(k) + v(k+1)                                            Eq. 22
%   The beetle senses the cost with two antennae on either side of x along
%   a random unit direction b and steps towards the better one:
%     v_l = -step * b * sign(J(x + b d/2) - J(x - b d/2)),   d = step / 5
%     step <- 0.95 step after every iteration, from 1e-4
%   Schedules: w = 0.9 - 0.5 k/MaxIter (Eq. 23 printed as increasing, but
%   described as linearly decreasing); fractional order a = 0.9 - 0.5
%   k/MaxIter (Eq. 27).  Paper settings: 30 beetles, 100 iterations,
%   c1 = c2 = c3 = 2, |v| <= 1.  Not stated in the paper, so chosen: the
%   antenna length step/5 (the original BAS code), the search space and
%   the starting swarm (the same as every other optimiser here).
%   Each iteration costs 3 evaluations per beetle (two antennae and the
%   move): 30 + 100 * 90 = 9030 in all.
N = 30;
c1 = 2;  c2 = 2;  c3 = 2;
wmin = 0.4;  wmax = 0.9;
alpha0 = 0.9;  alpha_drop = 0.5;
vmax = 1;
step = 1e-4;  eta = 0.95;  c_ant = 5;

nvar = numel(z_start);
X = initial_swarm(N, z_start);
fprintf('FBPA: %d beetles x %d iterations; evaluating the initial swarm ...\n', N, MaxIter);
F = evaluate(cost, X);
V = zeros(N, nvar);  V1 = V;  V2 = V;  V3 = V;
pbest = X;  pf = F;
[gf, gi] = min(F);
g = X(gi, :);
evals = N;
history = nan(1, MaxIter);
evals_at = nan(1, MaxIter);
fprintf('  initial best %.6g\n', gf);
for k = 1:MaxIter
    tau = k / MaxIter;
    w = wmax - (wmax - wmin) * tau;
    a = alpha0 - alpha_drop * tau;

    % beetle antennae: sense left and right, step towards the better side
    b = 2 * rand(N, nvar) - 1;
    b = b ./ (sqrt(sum(b.^2, 2)) + eps);
    d = step / c_ant;
    x_left  = min(max(X + b * d / 2, 0), 1);
    x_right = min(max(X - b * d / 2, 0), 1);
    f_left  = evaluate(cost, x_left);
    f_right = evaluate(cost, x_right);
    v_beetle = -step * b .* sign(f_left - f_right);
    evals = evals + 2 * N;

    % velocity, Eq. 26
    g1 = a * (1 - a) / 2;
    g2 = g1 * (2 - a) / 3;
    g3 = g2 * (3 - a) / 4;
    memory = (w - 1 + a) * V + g1 * V1 + g2 * V2 + g3 * V3;
    gbest = repmat(g, N, 1);
    Vn = memory ...
        + c1 * rand(N, nvar) .* (pbest - X) ...
        + c2 * rand(N, nvar) .* (gbest - X) ...
        + c3 * rand(N, nvar) .* v_beetle;
    Vn = min(max(Vn, -vmax), vmax);

    % position, Eq. 22, kept inside the unit box
    X = X + Vn;
    out = X < 0 | X > 1;
    X = min(max(X, 0), 1);
    Vn(out) = 0;
    V3 = V2;  V2 = V1;  V1 = V;  V = Vn;
    step = eta * step;

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

% =========================================================================
%  The paper's fitness, Eq. 29 (repository: fopid_fitness.m with
%  Fitness 'itae', the cost FBPA tuned with)
% =========================================================================

function [J, itae_step] = paper_cost(P, gains, itae_ref, robust)
%PAPER_COST  ITAE of the step response (Eq. 29) relative to the FOPID start
%   controller, so the FOPID scores 1 and J < 1 is an improvement:
%       J = ITAE_step(gains) / itae_ref
%   A candidate must also complete the sine run: the step-only fitness
%   cannot see it otherwise.  Runs stop once any |error| exceeds 5 rad; an
%   unstable candidate scores 1e3-2e3 (DIVERGED_COST).  With robust = true
%   a closed loop that is numerically ill-conditioned (SENSITIVITY above
%   1e-6 rad) gets 10 added.  itae_ref = [] returns J = NaN and only the
%   ITAE.
step = simulate_arm(P, gains, 'step', 0.01, 5);
itae_step = NaN;
if step.diverged
    J = diverged_cost(step, 0);
    return;
end
sine = simulate_arm(P, gains, 'sine', 0.01, 5);
if sine.diverged
    J = diverged_cost(sine, 0.5);
    return;
end
itae_step = itae(step);
if isempty(itae_ref)
    J = NaN;
    return;
end
J = max(itae_step, 0) / max(itae_ref, eps);
if robust && sensitivity(P, gains, 0.01, step, sine) > 1e-6
    J = J + 10;
end
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
