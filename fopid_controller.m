function C = fopid_controller(gains, dt)
%FOPID_CONTROLLER  Create six independent PID / FOPID joint controllers.
%
%   C = FOPID_CONTROLLER(gains, dt)
%     gains  struct with 6x1 fields Kp, Ki, Kd, lambda, mu
%     dt     controller sample time [s]
%
%   Control law per joint (Eq. 9 of the paper):
%
%       u = Kp * e  +  Ki * D^(-lambda) e  +  Kd * D^(mu) e
%
%   lambda = mu = 1 gives a classical PID, so the same code covers both
%   controllers.  The fractional operators are realised by
%   FRACTIONAL_OPERATOR; the integer parts of the orders are handled here as
%   a plain integrator and a backward difference.
%
%   No derivative filter, no anti-windup and no torque saturation are used,
%   matching the paper's controller structure.
%
%   Advance the controller one sample with FOPID_UPDATE.
%
%   See also FOPID_UPDATE, FRACTIONAL_OPERATOR.

C.Kp = gains.Kp(:);
C.Ki = gains.Ki(:);
C.Kd = gains.Kd(:);
C.dt = dt;

% Split each order into an integer part and a remainder in [0, 1):
%   D^(-lambda) = (1/s)^n_i * s^(-f_i),   D^(mu) = s^n_d * s^(f_d)
% The remainders are realised by the Oustaloup filters, the integer parts by
% the plain integrator and the backward difference in FOPID_UPDATE.
n_i = floor(gains.lambda(:) + 1e-12);
n_d = floor(gains.mu(:)     + 1e-12);
if any(n_i > 1) || any(n_d > 1)
    error('fopid_controller:order', 'lambda and mu must be below 2');
end
C.Fi = fractional_operator(-(gains.lambda(:) - n_i), dt);   % integral branch
C.Fd = fractional_operator(  gains.mu(:)     - n_d,  dt);   % derivative branch

C.use_integrator = n_i >= 1;   % joints whose lambda reaches 1
C.use_difference = n_d >= 1;   % joints whose mu reaches 1
C.integral = zeros(6, 1);
C.prev_d   = [];
end
