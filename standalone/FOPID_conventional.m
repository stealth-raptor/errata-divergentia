function results = FOPID_conventional(gain_set)
%FOPID_CONVENTIONAL  The paper's FOPID with fixed gains: six fractional-order PID joint controllers.
%
%   results = FOPID_CONVENTIONAL()          one gain set per experiment (default; every
%                                           reported result uses these)
%   results = FOPID_CONVENTIONAL('shared')  one gain set for both experiments
%   results = FOPID_CONVENTIONAL('table')   gains fitted to the paper's Tables 3 and 4
%
%   Jiang, Zhang & Liu, "Trajectory tracking control of a 6-DOF robotic arm
%   based on improved FOPID", Int. J. Dyn. Control 13:137 (2025): the
%   paper's second baseline, its Eq. 9,
%
%       u = Kp e + Ki D^(-lambda) e + Kd D^(mu) e,
%
%   one per joint, sampled at 1 kHz, before any optimiser tunes it.  The
%   fractional operators are Oustaloup approximations over [1e-3, 1e3]
%   rad/s, N = 5 (the FOMCON defaults; the paper gives no settings),
%   discretised exactly with a zero-order hold; the integer part of an
%   order above 1 is a plain integrator or backward difference.
%   Both of the paper's experiments on its arm (Table 2, Eq. 21):
%     step   1 rad on every joint at t = 1 s           -> Table 3, Figs 6-12
%     sine   sin(1.5 t) rad on every joint for 5 s      -> Table 4, Figs 13-19
%   It prints the metrics of Tables 3 and 4 next to the paper's values, and
%   joint by joint, and plots the responses and the torques.
%
%   Gains.  The paper publishes none.  These were identified from the
%   paper's own FOPID curves, read from the vector graphics of its PDF: the
%   six joint trajectories were matched in the fully coupled closed loop
%   (repository README, D6).  One set fitted to each experiment's curves
%   (default) and one fitted to both ('shared'; it is also the start
%   controller of every tuning run in FBPA_FOPID, FOPSO_GWO and
%   FOPSO_GWO_CC).  They show the published curves are attainable; they
%   need not be the authors' gains.
%
%   'table': gains fitted to the paper's Tables 3 and 4 instead of its
%   curves (repository: tools/fit_table_gains.m), one set per experiment.
%   Every table metric, the sine torque included, comes within 4 % of the
%   paper's value; the responses then follow the published curves less
%   closely, because the paper's tables disagree with its own figures.
%
%   results: gains (.step, .sine), step and sine runs (t, r, q, u on the
%   0.01 s grid), step_metrics, sine_metrics, and summary = [overshoot,
%   adjustment time, peak time, sine MSE, sine sum |tau|].
%
%   Self-contained: the arm, the controller, the simulation and the
%   metrics below are copies of the repository's robot_params.m,
%   robot_dynamics.m, fopid_controller.m, fopid_update.m,
%   fractional_operator.m, simulate_closed_loop.m and
%   performance_metrics.m, and give the numbers of its results/summary.md.
%   Runs in seconds; no toolbox needed.
%
%   See also PID_CONVENTIONAL, FBPA_FOPID, FOPSO_GWO, FOPSO_GWO_CC.

if nargin < 1 || isempty(gain_set), gain_set = 'separate'; end
switch lower(gain_set)
    case 'separate'
        gains.step = fopid_gains('step');
        gains.sine = fopid_gains('sine');
        label = 'gains identified from its FOPID curves, one set per experiment';
    case 'shared'
        gains.step = fopid_gains('shared');
        gains.sine = gains.step;
        label = 'gains identified from its FOPID curves, one set for both experiments';
    case 'table'
        gains.step = fopid_gains('table_step');
        gains.sine = fopid_gains('table_sine');
        label = 'gains fitted to the paper''s Tables 3 and 4, one set per experiment';
    otherwise
        error('FOPID_conventional:gains', 'gain_set must be ''separate'', ''shared'' or ''table''');
end

P = arm_parameters();
step = simulate_arm(P, gains.step, 'step');
sine = simulate_arm(P, gains.sine, 'sine');
ms = step_metrics(step);
mn = sine_metrics(sine);
summary = [ms.mean, mn.mean];

fprintf('\nFOPID on the paper''s arm (%s)\n', label);
print_metrics({'Paper, Tables 3 and 4', [31.2 1.89 1.33 0.88e-2 2.5686e4]
               'This simulation',       summary});
print_per_joint(ms, mn);
plot_runs('FOPID', step, sine);

results = struct('gains', gains, 'step', step, 'sine', sine, ...
                 'step_metrics', ms, 'sine_metrics', mn, 'summary', summary);
end

% =========================================================================
%  The gains, identified from the paper's FOPID curves (repository:
%  controller_gains.m)
% =========================================================================

function g = fopid_gains(set)
%FOPID_GAINS  6x1 Kp, Ki, Kd, lambda, mu of the six joints.  The comments
%   give the rms difference to the paper's published curves, joints 1-6 [rad].
switch set
    case 'step'
        % step 0.118 0.038 0.034 0.024 0.077 0.054   (mean 0.058)
        g.Kp     = [0.2246029026 0.001008584286 233.0620586 129.1202032 0.1001314098 37.9669364]';
        g.Ki     = [1011.942297 4092.232668 66.46026957 0.0001000343441 2.323429273 655.2171066]';
        g.Kd     = [111.6222949 488.4447765 310.7293829 2.102304702 0.8775511388 0.1470426384]';
        g.lambda = [0.3762673819 0.586740784 0.7319898437 1.939554884 0.09077580848 1.70607341]';
        g.mu     = [1.28267767 0.7291342749 0.6235574115 1.223482714 0.7142570757 1.807796772]';
    case 'sine'
        % sine 0.050 0.046 0.069 0.016 0.108 0.019   (mean 0.051)
        g.Kp     = [495.9935847 0.002705183396 0.1236514463 153.4029937 15.17725195 28.57833231]';
        g.Ki     = [532.711622 2432.873877 323.629477 0.0210062648 5.179202505 71.34826142]';
        g.Kd     = [82.03897077 114.6546778 450.4413379 11.68525819 0.9272306454 1.964879599]';
        g.lambda = [1.774002479 0.269229394 1.949843658 1.07017743 1.947748298 0.8092217149]';
        g.mu     = [1.227019335 1.347895571 0.7744312048 1.387471886 1.750055386 1.274988856]';
    case 'shared'
        % step 0.095 0.034 0.041 0.051 0.141 0.055   (mean 0.070)
        % sine 0.062 0.032 0.076 0.017 0.157 0.030   (mean 0.062)
        g.Kp     = [466.5048899 0.2050011905 0.07599433869 124.5656189 2.134765482 28.46272228]';
        g.Ki     = [348.1945821 2206.077613 54.67759987 11.92085781 2.285310734 0.009220016116]';
        g.Kd     = [49.95111633 645.0353382 591.4914619 0.4691240383 1.473705321 0.1510568454]';
        g.lambda = [1.662623676 0.6526495045 1.95 0.3800523434 0.7206369078 0.05006636685]';
        g.mu     = [1.425361662 0.5509011254 0.3845477049 1.453723122 0.7627629958 1.817543914]';
    case 'table_step'
        % fitted to the paper's Table 3 (repository: tools/fit_table_gains.m):
        %   difference to the table: overshoot -0.2 %, adjustment time -1.0 %, peak time -0.4 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.111 0.030 0.036 0.039 0.134 0.049 (mean 0.066)
        g.Kp     = [0.0726716506 0.0008032584069 162.6435932 96.18511154 0.03043655715 42.53066185]';
        g.Ki     = [497.1617953 902.9688649 33.0192936 5.537714495e-05 3.647945877 772.5041199]';
        g.Kd     = [98.58027759 646.5258216 423.6556653 0.5457432347 0.6691814401 0.5729749254]';
        g.lambda = [0.3754040622 0.2611844844 0.8039260167 1.723889285 0.07006399855 1.651613513]';
        g.mu     = [1.068451924 0.5363865362 0.4470969717 0.9252874564 1.002307105 1.51263575]';
    case 'table_sine'
        % fitted to the paper's Table 4 (repository: tools/fit_table_gains.m):
        %   difference to the table: MSE -1.1 %, sum |tau| -2.7 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.088 0.171 0.085 0.031 0.152 0.020 (mean 0.091)
        g.Kp     = [321.7752078 0.1231100107 0.05836118968 859.152216 9.142294228 746.1960559]';
        g.Ki     = [37.32154111 1052.933141 108.2221626 0.06956013386 2.106717055 462.4845256]';
        g.Kd     = [154.3270594 32.1043395 388.530668 0.1246469781 3.789108757 0.4865877342]';
        g.lambda = [1.880114215 0.05 1.819921257 0.7963817842 1.834408652 1.066205993]';
        g.mu     = [0.6461293791 0.06952214393 0.8732949818 1.762281308 0.9733297157 0.8735492287]';
end
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
