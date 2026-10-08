function [M, h] = robot_dynamics(P, q, qd)
%ROBOT_DYNAMICS  Rigid-body dynamics of the arm (Eq. 21 of the paper):
%
%       M(q) qdd + C(q, qd) qd + G(q) + tau_f(qd) = tau
%
%   [M, h] = ROBOT_DYNAMICS(P, q, qd)
%     P      parameters from ROBOT_PARAMS
%     q, qd  6x1 joint angles and velocities
%     M      6x6 joint-space inertia matrix
%     h      6x1 = C(q, qd) qd + G(q) + tau_f(qd)
%
%   Computed with the recursive Newton-Euler algorithm (standard DH, as in
%   Corke's Robotics Toolbox), which gives the same M, C and G as the
%   Lagrange formulation of the paper (Eqs. 10-20):
%     h             = RNEA(q, qd, qdd = 0) with gravity, plus friction
%     column j of M = RNEA(q, qd = 0, qdd = e_j) without gravity
%   All seven recursions are run at once as the columns of 3x7 arrays, and
%   every cross product is written out, because the cost of interpreted
%   code is per statement, not per floating-point operation.
%
%   Verified against an independent Jacobian-based M(q) and by energy
%   conservation (tools/verify_dynamics.m).
%
%   See also ROBOT_PARAMS.

% batch columns 1..6: qd = 0, qdd = e_j, no gravity  -> M(:, j)
% batch column  7   : qd,     qdd = 0,  gravity      -> h
QD  = [zeros(6, 6), qd(:)];
QDD = [eye(6), zeros(6, 1)];

w  = zeros(3, 7);                 % angular velocity of the previous link
wd = zeros(3, 7);                 % angular acceleration
vd = zeros(3, 7);                 % linear acceleration of the frame origin
vd(3, 7) = P.g;                   % gravity as an upward base acceleration

Fx = zeros(6, 7); Fy = Fx; Fz = Fx;      % force on the COM of each link
Nx = Fx;          Ny = Fx; Nz = Fx;      % moment about the COM
ct = cos(q);  st = sin(q);
Rs = zeros(3, 3, 6);

% ---- outward recursion ---------------------------------------------------
for i = 1:6
    R = [ct(i), -st(i)*P.ca(i),  st(i)*P.sa(i)
         st(i),  ct(i)*P.ca(i), -ct(i)*P.sa(i)
         0,      P.sa(i),        P.ca(i)];
    Rs(:, :, i) = R;
    Rt = R.';
    qdi = QD(i, :);

    % angular velocity and acceleration of link i, in frame i
    wd = Rt * [wd(1,:) + w(2,:).*qdi;  wd(2,:) - w(1,:).*qdi;  wd(3,:) + QDD(i,:)];
    w  = Rt * [w(1,:);  w(2,:);  w(3,:) + qdi];

    % linear acceleration of the frame origin: wd x p + w x (w x p) + Rt vd
    p = P.pstar(:, i);
    c1 = w(2,:)*p(3) - w(3,:)*p(2);  c2 = w(3,:)*p(1) - w(1,:)*p(3);  c3 = w(1,:)*p(2) - w(2,:)*p(1);
    vd = Rt * vd + [wd(2,:)*p(3) - wd(3,:)*p(2) + w(2,:).*c3 - w(3,:).*c2
                    wd(3,:)*p(1) - wd(1,:)*p(3) + w(3,:).*c1 - w(1,:).*c3
                    wd(1,:)*p(2) - wd(2,:)*p(1) + w(1,:).*c2 - w(2,:).*c1];

    % acceleration of the COM, then Newton and Euler for the link
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

% ---- inward recursion ------------------------------------------------------
tau = zeros(6, 7);
f = zeros(3, 7);  n = zeros(3, 7);
for i = 6:-1:1
    if i < 6
        R = Rs(:, :, i+1);
        f = R * f;                 % force and moment from link i+1, in frame i
        n = R * n;
    end
    p  = P.pstar(:, i);
    pr = p + P.rc(:, i);
    F  = [Fx(i,:); Fy(i,:); Fz(i,:)];
    n = n + [Nx(i,:); Ny(i,:); Nz(i,:)] ...
          + [p(2)*f(3,:) - p(3)*f(2,:);   p(3)*f(1,:) - p(1)*f(3,:);   p(1)*f(2,:) - p(2)*f(1,:)] ...
          + [pr(2)*F(3,:) - pr(3)*F(2,:); pr(3)*F(1,:) - pr(1)*F(3,:); pr(1)*F(2,:) - pr(2)*F(1,:)];
    f = f + F;
    z = Rs(3, :, i);               % joint axis z_{i-1} expressed in frame i
    tau(i, :) = z * n;
end

M = tau(:, 1:6);
M = 0.5 * (M + M.');
h = tau(:, 7) + P.fc .* sign(qd(:)) + P.b .* qd(:);
end
