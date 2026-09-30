function out = simulate_closed_loop(P, gains, reference, opt)
%SIMULATE_CLOSED_LOOP  Run the arm under (FO)PID control for one experiment.
%
%   out = SIMULATE_CLOSED_LOOP(P, gains, reference, opt)
%     P          robot parameters from ROBOT_PARAMS
%     gains      6x1 fields Kp, Ki, Kd, lambda, mu
%     reference  'step'  1 rad on every joint at t = 1 s
%                'sine'  sin(1.5 t) rad on every joint
%     opt        optional: dt (default 1e-3), T (default 5), t_step, omega,
%                abort_err (default Inf): stop early once any |error| exceeds
%                this value or the state stops being finite; used by the
%                gain tuner to discard unstable candidates quickly;
%                use_mex (default false): use the compiled SIMULATE_MEX
%                (build it with BUILD_MEX) instead of this .m loop
%
%   Returns
%     out.t   1xNt time vector
%     out.r   6xNt reference
%     out.q   6xNt joint angles
%     out.u   6xNt joint torques
%     out.diverged  true if the run was stopped by abort_err
%
%   The controllers run at the sample time dt with a zero-order hold, and the
%   plant is integrated between samples with a fourth-order Runge-Kutta step.
%   The arm starts at rest at q = 0, as in the paper.
%
%   SIMULATE_MEX is a C port of exactly this loop (with FOPID_UPDATE and
%   ROBOT_DYNAMICS); it gives the same results about 100x faster, which is
%   what makes gain tuning practical, particularly under Octave.
%
%   See also ROBOT_DYNAMICS, FOPID_CONTROLLER, PERFORMANCE_METRICS.

if nargin < 4, opt = struct(); end
if ~isfield(opt, 'dt'),     opt.dt = 1e-3;   end
if ~isfield(opt, 'T'),      opt.T = 5;       end
if ~isfield(opt, 't_step'), opt.t_step = 1;  end
if ~isfield(opt, 'omega'),  opt.omega = 1.5; end
if ~isfield(opt, 'abort_err'), opt.abort_err = Inf; end
if ~isfield(opt, 'use_mex'),   opt.use_mex = false; end   % not yet verified against the .m loop

dt = opt.dt;
t  = 0:dt:opt.T;
Nt = numel(t);

switch reference
    case 'step', r = repmat(double(t >= opt.t_step - 1e-12), 6, 1);
    case 'sine', r = repmat(sin(opt.omega * t), 6, 1);
    otherwise,   error('simulate_closed_loop:ref', 'unknown reference "%s"', reference);
end

C  = fopid_controller(gains, dt);

if opt.use_mex
    [qo, uo, n, div] = simulate_mex(P.R_fixed, P.p_fixed, P.I, P.g, r, dt, ...
        C.Kp, C.Ki, C.Kd, C.Fi.K, C.Fi.r, C.Fi.A, C.Fi.B, ...
        C.Fd.K, C.Fd.r, C.Fd.A, C.Fd.B, ...
        double(C.use_integrator), double(C.use_difference), opt.abort_err);
    keep = 1:max(n, 1);
    out.t = t(keep);
    out.r = r(:, keep);
    out.q = qo(:, keep);
    out.u = uo(:, keep);
    out.diverged = logical(div);
    return;
end

q  = zeros(6, 1);
qd = zeros(6, 1);

out.t = t;
out.r = r;
out.q = zeros(6, Nt);
out.u = zeros(6, Nt);
out.diverged = false;

for k = 1:Nt
    e = r(:, k) - q;
    if ~all(isfinite(q)) || ~all(isfinite(qd)) || max(abs(e)) > opt.abort_err
        out.diverged = true;
        break;
    end
    [u, C] = fopid_update(C, e);

    out.q(:, k) = q;
    out.u(:, k) = u;
    if k == Nt, break; end

    % fourth-order Runge-Kutta step with the torque held constant
    [k1q, k1v] = derivative(P, q,            qd,            u);
    [k2q, k2v] = derivative(P, q + dt/2*k1q, qd + dt/2*k1v, u);
    [k3q, k3v] = derivative(P, q + dt/2*k2q, qd + dt/2*k2v, u);
    [k4q, k4v] = derivative(P, q + dt*k3q,   qd + dt*k3v,   u);
    q  = q  + dt/6 * (k1q + 2*k2q + 2*k3q + k4q);
    qd = qd + dt/6 * (k1v + 2*k2v + 2*k3v + k4v);
end

if out.diverged                      % keep only the samples actually run
    keep = 1:max(k - 1, 1);
    out.t = out.t(keep);
    out.r = out.r(:, keep);
    out.q = out.q(:, keep);
    out.u = out.u(:, keep);
end
end

% ------------------------------------------------------------------------
function [dq, dqd] = derivative(P, q, qd, u)
%DERIVATIVE  Forward dynamics:  qdd = M(q) \ (tau - h(q, qd)).
[M, h] = robot_dynamics(P, q, qd);
dq  = qd;
dqd = M \ (u - h);
end
