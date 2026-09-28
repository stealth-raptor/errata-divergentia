function [u, C] = fopid_update(C, e)
%FOPID_UPDATE  Advance the controllers by one sample and return the torques.
%
%   [u, C] = FOPID_UPDATE(C, e)
%     C  controller struct from FOPID_CONTROLLER (state is carried in C)
%     e  6x1 tracking error r - q
%     u  6x1 joint torques
%
%   See also FOPID_CONTROLLER.

% ---- integral branch:  Ki * D^(-lambda) e -------------------------------
[yi, C.Fi] = filter_step(C.Fi, e);
C.integral = C.integral + yi * C.dt;            % integer part of the order
term_i = yi;
term_i(C.use_integrator) = C.integral(C.use_integrator);

% ---- derivative branch:  Kd * D^(mu) e ----------------------------------
[yd, C.Fd] = filter_step(C.Fd, e);
if isempty(C.prev_d)
    C.prev_d = yd;                              % no kick on the first sample
end
diff = (yd - C.prev_d) / C.dt;                  % integer part of the order
C.prev_d = yd;
term_d = yd;
term_d(C.use_difference) = diff(C.use_difference);

u = C.Kp .* e + C.Ki .* term_i + C.Kd .* term_d;
end

% ------------------------------------------------------------------------
function [y, F] = filter_step(F, e)
%FILTER_STEP  One zero-order-hold step of the Oustaloup filter bank.
y = F.K .* e + sum(F.r .* F.x, 2);
F.x = F.A .* F.x + F.B .* e;
end
