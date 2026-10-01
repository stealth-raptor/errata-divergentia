function e = control_effort(out, reference, t_kick)
%CONTROL_EFFORT  How hard a controller drives the joints, per joint.
%
%   e = CONTROL_EFFORT(out, reference)   out from SIMULATE_CLOSED_LOOP logged
%                                        at the control rate (opt.log_dt = dt)
%   e = CONTROL_EFFORT(out, 'step', t_kick)
%
%   The paper's metrics say how well the joints track; these say what that
%   costs in torque.  All are 1x6, one entry per joint:
%     peak   max |tau|                                              [Nm]
%     sum    sum of |tau| on the paper's 0.01 s grid, as Table 4    [Nm]
%     tv     total variation, sum of |tau(k+1) - tau(k)| at the
%            control rate: how much the torque moves, which grows
%            with chattering and noise amplification               [Nm]
%   For the step reference these exclude the derivative kick: the first
%   t_kick seconds (default 0.05) after the step at t = 1 s, where the
%   fractional derivative of the ideal 1 rad step drives every controller
%   with a D-term to 1e4-1e6 Nm.  The kick is reported on its own:
%     kick   max |tau| within the kick window                      [Nm]
%   (zero for the sine reference, which starts smoothly from rest).
%
%   See also PERFORMANCE_METRICS, FOPID_FITNESS.

if nargin < 3, t_kick = 0.05; end
u = out.u;
t = out.t;
dt = t(2) - t(1);
every = max(1, round(0.01 / dt));            % the paper's 0.01 s grid
switch reference
    case 'step'
        in_kick = t >= 1 - 1e-9 & t < 1 + t_kick - 1e-9;
        keep = ~in_kick;
        e.kick = max(abs(u(:, in_kick)), [], 2)';
    case 'sine'
        keep = true(size(t));
        e.kick = zeros(1, size(u, 1));
    otherwise
        error('control_effort:ref', 'unknown reference "%s"', reference);
end
grid = false(size(t));
grid(1:every:end) = true;
e.peak = max(abs(u(:, keep)), [], 2)';
e.sum  = sum(abs(u(:, keep & grid)), 2)';
du = abs(diff(u, 1, 2));
e.tv   = sum(du(:, keep(2:end) & keep(1:end-1)), 2)';
end
