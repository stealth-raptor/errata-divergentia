function out = simulate_closed_loop(P, gains, reference, opt)
%SIMULATE_CLOSED_LOOP  Run the arm under (FO)PID control for one experiment.
%
%   out = SIMULATE_CLOSED_LOOP(P, gains, reference, opt)
%     P          robot parameters from ROBOT_PARAMS
%     gains      6x1 fields Kp, Ki, Kd, lambda, mu
%     reference  'step'  1 rad on every joint at t = 1 s
%                'sine'  sin(1.5 t) rad on every joint
%     opt        optional: dt (1e-3), T (5), log_dt (0.01), t_step (1), omega (1.5),
%                abort_err (Inf): stop early once any |error| exceeds this
%                value; used by the gain tuner (FOPID_FITNESS) to discard
%                unstable candidates quickly;
%                use_mex (true when SIMULATE_MEX has been built): run the
%                compiled C version of this loop (BUILD_MEX), the same model
%                and results (tools/check_mex.m), several hundred times faster
%
%   Returns, sampled every log_dt seconds:
%     out.t   1xN time vector
%     out.r   6xN reference
%     out.q   6xN joint angles
%     out.u   6xN joint torques (controller output)
%     out.diverged  true if the run stopped early (abort_err exceeded, or the
%                   state became non-finite or exceeded 1e3 rad); the outputs
%                   then hold only the samples actually run
%
%   The controllers run at the sample time dt with a zero-order hold and
%   the plant is integrated between samples with a fourth-order Runge-Kutta
%   step.  The arm starts at rest at q = 0.
%
%   Logging interval: the paper's figures are drawn from signals logged
%   every 0.01 s (the published step reference rises from 0 at t = 0.99 s to
%   1 at t = 1.00 s, and the published torque curves have one vertex every
%   0.01 s).  The paper's metrics are computed on those samples: the MSE of
%   Table 4 recomputed from the published curves on this grid reproduces the
%   table to within 0.7 %.  Everything is therefore logged at 0.01 s.
%
%   See also ROBOT_DYNAMICS, FOPID_CONTROLLER, PERFORMANCE_METRICS.

if nargin < 4, opt = struct(); end
if ~isfield(opt, 'dt'),     opt.dt = 1e-3;   end
if ~isfield(opt, 'T'),      opt.T = 5;       end
if ~isfield(opt, 'log_dt'), opt.log_dt = 0.01; end
if ~isfield(opt, 't_step'), opt.t_step = 1;  end
if ~isfield(opt, 'omega'),  opt.omega = 1.5; end
if ~isfield(opt, 'abort_err'), opt.abort_err = Inf; end
if ~isfield(opt, 'use_mex'),   opt.use_mex = exist('simulate_mex') == 3; end %#ok<EXIST>

dt = opt.dt;
Nt = round(opt.T / dt) + 1;
t  = (0:Nt-1) * dt;
every = round(opt.log_dt / dt);

switch reference
    case 'step', r = repmat(double(t >= opt.t_step - 1e-12), 6, 1);
    case 'sine', r = repmat(sin(opt.omega * t), 6, 1);
    otherwise,   error('simulate_closed_loop:ref', 'unknown reference "%s"', reference);
end

C  = fopid_controller(gains, dt);

if opt.use_mex
    [qo, uo, nlogged, diverged, kstop] = simulate_mex(P.pstar, P.rc, P.m, P.Idiag, P.ca, P.sa, ...
        P.g, P.fc, P.b, r, dt, every, C.Kp, C.Ki, C.Kd, C.Fi.K, C.Fi.r, C.Fi.A, C.Fi.B, ...
        C.Fd.K, C.Fd.r, C.Fd.A, C.Fd.B, double(C.use_integrator), double(C.use_difference), ...
        opt.abort_err);
    keep = 1:size(qo, 2);
    if diverged
        keep = 1:max(nlogged, 1);
        if isinf(opt.abort_err)
            warning('simulate_closed_loop:unstable', 'closed loop diverged at t = %.3f s', t(kstop));
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
j = 0;                                  % number of samples logged so far

for k = 1:Nt
    e = r(:, k) - q;
    if max(abs(e)) > opt.abort_err
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

    % fourth-order Runge-Kutta step with the torque held constant
    k1q = qd;              k1v = accel(P, q,            qd,  u);
    k2q = qd + dt/2*k1v;   k2v = accel(P, q + dt/2*k1q, k2q, u);
    k3q = qd + dt/2*k2v;   k3v = accel(P, q + dt/2*k2q, k3q, u);
    k4q = qd + dt*k3v;     k4v = accel(P, q + dt*k3q,   k4q, u);
    q  = q  + dt/6 * (k1q + 2*k2q + 2*k3q + k4q);
    qd = qd + dt/6 * (k1v + 2*k2v + 2*k3v + k4v);

    if any(~isfinite(q)) || any(~isfinite(qd)) || any(abs(q) > 1e3)
        out.diverged = true;
        if isinf(opt.abort_err)         % not a tuning run: worth a warning
            warning('simulate_closed_loop:unstable', 'closed loop diverged at t = %.3f s', t(k));
        end
        break;
    end
end

if out.diverged                         % keep only the samples actually run
    keep = 1:max(j, 1);
    out.t = out.t(keep);
    out.r = out.r(:, keep);
    out.q = out.q(:, keep);
    out.u = out.u(:, keep);
end
end

% ------------------------------------------------------------------------
function qdd = accel(P, q, qd, u)
%ACCEL  Forward dynamics:  qdd = M(q) \ (tau - h(q, qd)).
[M, h] = robot_dynamics(P, q, qd);
qdd = M \ (u - h);
end
