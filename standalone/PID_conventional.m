function results = PID_conventional(gain_set)
%PID_CONVENTIONAL  The conventional controller: six independent PID joint controllers.
%
%   results = PID_CONVENTIONAL()          one gain set per experiment (default; every
%                                         reported result uses these)
%   results = PID_CONVENTIONAL('shared')  one gain set for both experiments
%   results = PID_CONVENTIONAL('table')   gains fitted to the paper's Tables 3 and 4
%
%   Jiang, Zhang & Liu, "Trajectory tracking control of a 6-DOF robotic arm
%   based on improved FOPID", Int. J. Dyn. Control 13:137 (2025): the
%   paper's baseline controller, its Eq. 9 with lambda = mu = 1,
%
%       u(k) = Kp e(k) + Ki sum_{i<=k} e(i) dt + Kd (e(k) - e(k-1)) / dt,
%
%   one per joint, sampled at 1 kHz, on the paper's arm (Table 2, Eq. 21),
%   in both of its experiments:
%     step   1 rad on every joint at t = 1 s           -> Table 3, Figs 6-12
%     sine   sin(1.5 t) rad on every joint for 5 s      -> Table 4, Figs 13-19
%   It prints the metrics of Tables 3 and 4 next to the paper's values, and
%   joint by joint, and plots the responses and the torques.
%
%   Gains.  The paper publishes none.  These were identified from the
%   paper's own PID curves, read from the vector graphics of its PDF: the
%   six joint trajectories were matched in the fully coupled closed loop
%   (repository README, D6).  The paper does not say whether its two
%   experiments used the same gains, so there is one set fitted to each
%   experiment's curves (default) and one fitted to both ('shared').  They
%   show the published curves are attainable; they need not be the
%   authors' gains.
%
%   'table': gains fitted to the paper's Tables 3 and 4 instead of its
%   curves (repository: tools/fit_table_gains.m), one set per experiment.
%   Every table metric, the sine torque included, comes within 10 % of the
%   paper's value; the responses then follow the published curves less
%   closely, because the paper's tables disagree with its own figures.
%
%   results: gains (.step, .sine), step and sine runs (t, r, q, u on the
%   0.01 s grid), step_metrics, sine_metrics, and summary = [overshoot,
%   adjustment time, peak time, sine MSE, sine sum |tau|].
%
%   Self-contained: the arm, the simulation and the metrics below are
%   copies of the repository's robot_params.m, robot_dynamics.m,
%   simulate_closed_loop.m and performance_metrics.m, and give the numbers
%   of its results/summary.md.  Runs in seconds; no toolbox needed.
%
%   See also FOPID_CONVENTIONAL, FBPA_FOPID, FOPSO_GWO, FOPSO_GWO_CC.

if nargin < 1 || isempty(gain_set), gain_set = 'separate'; end
switch lower(gain_set)
    case 'separate'
        gains.step = pid_gains('step');
        gains.sine = pid_gains('sine');
        label = 'gains identified from its PID curves, one set per experiment';
    case 'shared'
        gains.step = pid_gains('shared');
        gains.sine = gains.step;
        label = 'gains identified from its PID curves, one set for both experiments';
    case 'table'
        gains.step = pid_gains('table_step');
        gains.sine = pid_gains('table_sine');
        label = 'gains fitted to the paper''s Tables 3 and 4, one set per experiment';
    otherwise
        error('PID_conventional:gains', 'gain_set must be ''separate'', ''shared'' or ''table''');
end

P = arm_parameters();
step = simulate_arm(P, gains.step, 'step');
sine = simulate_arm(P, gains.sine, 'sine');
ms = step_metrics(step);
mn = sine_metrics(sine);
summary = [ms.mean, mn.mean];

fprintf('\nPID on the paper''s arm (%s)\n', label);
print_metrics({'Paper, Tables 3 and 4', [54.6 2.44 1.39 2.27e-2 3.7422e4]
               'This simulation',       summary});
print_per_joint(ms, mn);
plot_runs('PID', step, sine);

results = struct('gains', gains, 'step', step, 'sine', sine, ...
                 'step_metrics', ms, 'sine_metrics', mn, 'summary', summary);
end

% =========================================================================
%  The gains, identified from the paper's PID curves (repository:
%  controller_gains.m)
% =========================================================================

function g = pid_gains(set)
%PID_GAINS  6x1 Kp, Ki, Kd of the six joints.  The comments give the rms
%   difference to the paper's published curves, joints 1-6 [rad].
switch set
    case 'step'
        % step 0.168 0.083 0.049 0.046 0.142 0.065   (mean 0.092)
        g.Kp = [97.37514954 1564.876986 359.2803819 102.4754746 4.088352959 56.88093831]';
        g.Ki = [557.4767198 10486.36353 163.0551406 6.115185187 0.9552286351 310.5752354]';
        g.Kd = [159.9754331 208.9663973 149.8627569 2.754135176 0.1769806667 2.982132753]';
    case 'sine'
        % sine 0.124 0.064 0.105 0.025 0.143 0.021   (mean 0.080)
        g.Kp = [85.75329619 1083.287035 386.0742159 8.5825545 4.746903194 8.473415177]';
        g.Ki = [308.9360215 5037.557484 0.0001003450182 50.57169418 2.164146914 178.3005691]';
        g.Kd = [70.71407657 194.9763253 56.34833611 233.4242584 1.855891971 10.40962032]';
    case 'shared'
        % step 0.157 0.050 0.076 0.104 0.187 0.076   (mean 0.108)
        % sine 0.143 0.050 0.118 0.046 0.218 0.021   (mean 0.099)
        g.Kp = [75.45276223 1504.044829 696.2823467 69.66649056 4.7024463 6389.310235]';
        g.Ki = [565.531096 3041.237729 71.11348748 188.2343736 0.6565698329 1305.181802]';
        g.Kd = [137.1522896 102.8120137 113.9384517 1.02441153 0.4189171128 10.20444477]';
    case 'table_step'
        % fitted to the paper's Table 3, every metric within 10 % (repository: tools/fit_table_gains.m):
        %   difference to the table: overshoot -9.0 %, adjustment time +9.0 %, peak time +4.9 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.177 0.067 0.049 0.046 0.155 0.061 (mean 0.093)
        g.Kp = [179.5346724 1582.327205 372.9408767 100.7142999 3.99902397 140.7491131]';
        g.Ki = [1217.021456 8356.023527 124.9820399 4.031584892 0.7844714445 559.0830427]';
        g.Kd = [218.1809165 183.0850197 132.3721848 3.023725067 0.1942174349 1.248639522]';
    case 'table_sine'
        % fitted to the paper's Table 4, every metric within 10 % (repository: tools/fit_table_gains.m):
        %   difference to the table: MSE -8.9 %, sum |tau| -8.9 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.143 0.060 0.195 0.023 0.177 0.021 (mean 0.103)
        g.Kp = [71.85762901 10435.14108 845.8230593 186.2325337 7.587992021 6.990320416]';
        g.Ki = [298.3944056 27631.60421 0.009513614826 13.11856888 2.704779655 1477.431058]';
        g.Kd = [69.3834926 154.3642649 0.5739346068 65.15412446 3.923319473 59.51609494]';
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
%  The PID controller, paper Eq. 9 with lambda = mu = 1 (repository:
%  fopid_controller.m and fopid_update.m, which reduce to exactly this)
% =========================================================================

function C = pid_create(gains, dt)
%PID_CREATE  Six independent PID joint controllers sampled every dt,
%       u(k) = Kp e(k) + Ki sum_{i<=k} e(i) dt + Kd (e(k) - e(k-1)) / dt
%   No derivative filter, anti-windup or torque saturation, as in the paper.
C.Kp = gains.Kp(:);
C.Ki = gains.Ki(:);
C.Kd = gains.Kd(:);
C.dt = dt;
C.integral = zeros(6, 1);
C.prev_e   = [];
end

function [u, C] = pid_update(C, e)
%PID_UPDATE  One control sample: e = r - q (6x1) -> joint torques u (6x1).
C.integral = C.integral + e * C.dt;               % rectangular integration
if isempty(C.prev_e)
    C.prev_e = e;                                 % no kick on the first sample
end
de = (e - C.prev_e) / C.dt;                       % backward difference
C.prev_e = e;
u = C.Kp .* e + C.Ki .* C.integral + C.Kd .* de;
end

% =========================================================================
%  One experiment of the paper (repository: simulate_closed_loop.m)
% =========================================================================

function out = simulate_arm(P, gains, reference)
%SIMULATE_ARM  The arm under the six PID joint controllers, for 5 s.
%   out = SIMULATE_ARM(P, gains, 'step' | 'sine')
%     'step'  1 rad on every joint at t = 1 s       (paper Figs 6-12)
%     'sine'  sin(1.5 t) rad on every joint         (paper Figs 13-19)
%   The controller runs at dt = 1 ms with a zero-order hold, and the arm is
%   integrated with one RK4 step per sample, from rest at q = 0.  q and u
%   are logged every 0.01 s, the paper's grid.
dt = 1e-3;
Nt = round(5 / dt) + 1;
t  = (0:Nt-1) * dt;
every = round(0.01 / dt);
switch reference
    case 'step', r = repmat(double(t >= 1 - 1e-12), 6, 1);
    case 'sine', r = repmat(sin(1.5 * t), 6, 1);
    otherwise,   error('simulate_arm:ref', 'unknown reference ''%s''', reference);
end
C = pid_create(gains, dt);
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
    [u, C] = pid_update(C, e);
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
        warning('simulate_arm:unstable', 'closed loop diverged at t = %.3f s', t(k));
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
