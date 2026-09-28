function [M, h] = robot_dynamics(P, q, qd)
%ROBOT_DYNAMICS  Rigid-body dynamics of the arm:  M(q) qdd + h(q, qd) = tau.
%
%   [M, h] = ROBOT_DYNAMICS(P, q, qd)
%     P      parameters from ROBOT_PARAMS
%     q, qd  6x1 joint angles and velocities
%     M      6x6 joint-space inertia matrix
%     h      6x1 Coriolis, centrifugal and gravity torques
%
%   Both come from the recursive Newton-Euler algorithm:
%     h          = RNEA(q, qd, qdd = 0)          with gravity
%     column j of M = RNEA(q, qd = 0, qdd = e_j) without gravity
%   which is the standard way of extracting the inertia matrix column by
%   column.  This is the same model as the Lagrange formulation in the paper
%   and was verified against MATLAB's Robotics System Toolbox (~1e-14).
%
%   See also ROBOT_PARAMS.

h = rnea(P, q, qd, zeros(6,1), P.g);

M = zeros(6, 6);
z = zeros(6, 1);
for j = 1:6
    e = z;  e(j) = 1;
    M(:, j) = rnea(P, q, z, e, 0);
end
M = 0.5 * (M + M');          % symmetrise (removes round-off asymmetry)
end

% ------------------------------------------------------------------------
function tau = rnea(P, q, qd, qdd, g)
%RNEA  Recursive Newton-Euler inverse dynamics for a serial chain.
%   Spatial vectors are stacked as [angular; linear].  Gravity enters as a
%   fictitious upward acceleration of the base.

w = zeros(3, 1);  v = zeros(3, 1);          % velocity of the previous body
aw = zeros(3, 1); av = [0; 0; g];           % acceleration of the previous body
n = zeros(3, 6);  f = zeros(3, 6);          % moments and forces per body
W = zeros(3, 6);  V = zeros(3, 6);

% ---- outward recursion: velocities, accelerations, body forces ----------
for i = 1:6
    R = P.R_fixed(:, :, i)';                 % parent -> body, fixed part
    p = P.p_fixed(:, i);
    c = cos(q(i));  s = sin(q(i));

    w_p = rotate(R, c, s, w);                       % transform to body i
    v_p = rotate(R, c, s, v - cross(p, w));
    aw_p = rotate(R, c, s, aw);
    av_p = rotate(R, c, s, av - cross(p, aw));

    w = w_p + [0; 0; qd(i)];                        % add the joint motion
    v = v_p;
    aw = aw_p + [0; 0; qdd(i)] + cross(w_p, [0; 0; qd(i)]);
    av = av_p + cross(v_p, [0; 0; qd(i)]);

    Iv = P.I(:, :, i) * [w; v];
    Ia = P.I(:, :, i) * [aw; av];
    n(:, i) = Ia(1:3) + cross(w, Iv(1:3)) + cross(v, Iv(4:6));
    f(:, i) = Ia(4:6) + cross(w, Iv(4:6));
    W(:, i) = w;  V(:, i) = v;
end

% ---- inward recursion: joint torques ------------------------------------
tau = zeros(6, 1);
nn = n(:, 6);  ff = f(:, 6);
for i = 6:-1:1
    tau(i) = nn(3);                                  % torque about the joint axis
    if i > 1
        R = P.R_fixed(:, :, i);
        p = P.p_fixed(:, i);
        c = cos(q(i));  s = sin(q(i));
        nn_p = R * rotz(c, s, nn);                   % transform back to the parent
        ff_p = R * rotz(c, s, ff);
        nn = n(:, i-1) + nn_p + cross(p, ff_p);
        ff = f(:, i-1) + ff_p;
    end
end
end

% ------------------------------------------------------------------------
function y = rotate(R, c, s, x)
%ROTATE  Parent -> body:  Rz(q)' * (R * x).
x = R * x;
y = [ c*x(1) + s*x(2); -s*x(1) + c*x(2); x(3)];
end

function y = rotz(c, s, x)
%ROTZ  Body -> parent (rotation part only):  Rz(q) * x.
y = [c*x(1) - s*x(2); s*x(1) + c*x(2); x(3)];
end
