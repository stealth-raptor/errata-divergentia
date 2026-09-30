function verify_dynamics()
%VERIFY_DYNAMICS  Self-checks of ROBOT_DYNAMICS (run from the repository root).
%
%   1. M(q) against an independent Jacobian formula
%        M = sum_i m_i Jv_i' Jv_i + Jw_i' R_i I_i R_i' Jw_i
%   2. M symmetric positive definite
%   3. conservation of kinetic + potential energy under zero torque, with
%      gravity switched on so that G(q) is tested as well
%   for every centre-of-mass placement.

addpath(fileparts(fileparts(mfilename('fullpath'))));
rand('seed', 1);
for com = {'distal', 'middle', 'proximal'}
    P = robot_params(struct('com', com{1}, 'g', 9.81));
    q = 4 * rand(6, 1) - 2;
    [M, ~] = robot_dynamics(P, q, zeros(6, 1));
    err = max(max(abs(M - jacobian_mass(P, q))));

    x = q;  v = 4 * rand(6, 1) - 2;  dt = 1e-4;  E0 = energy(P, x, v);
    f = @(x, v) acc(P, x, v);
    for k = 1:2000
        k1q = v;             k1v = f(x, v);
        k2q = v + dt/2*k1v;  k2v = f(x + dt/2*k1q, k2q);
        k3q = v + dt/2*k2v;  k3v = f(x + dt/2*k2q, k3q);
        k4q = v + dt*k3v;    k4v = f(x + dt*k3q,   k4q);
        x = x + dt/6*(k1q + 2*k2q + 2*k3q + k4q);
        v = v + dt/6*(k1v + 2*k2v + 2*k3v + k4v);
    end
    drift = abs(energy(P, x, v) - E0) / abs(E0);
    fprintf('%-8s  |M - M_jacobian| = %.1e   min eig(M) = %.3e   energy drift (0.2 s) = %.1e\n', ...
            com{1}, err, min(eig(M)), drift);
    assert(err < 1e-12 && min(eig(M)) > 0 && drift < 1e-8);
end
fprintf('robot_dynamics: all checks passed\n');
end

function a = acc(P, q, qd)
[M, h] = robot_dynamics(P, q, qd);
a = -(M \ h);
end

function [T, Rs] = fk(P, q)
T = zeros(4, 4, 7);  T(:, :, 1) = eye(4);
for i = 1:6
    ct = cos(q(i)); st = sin(q(i)); ca = P.ca(i); sa = P.sa(i);
    A = [ct -st*ca st*sa P.a(i)*ct; st ct*ca -ct*sa P.a(i)*st; 0 sa ca P.d(i); 0 0 0 1];
    T(:, :, i+1) = T(:, :, i) * A;
end
end

function M = jacobian_mass(P, q)
T = fk(P, q);
M = zeros(6);
for i = 1:6
    Ti = T(:, :, i+1);
    c = Ti(1:3, 1:3) * P.rc(:, i) + Ti(1:3, 4);
    Jv = zeros(3, 6);  Jw = zeros(3, 6);
    for j = 1:i
        z = T(1:3, 3, j);  o = T(1:3, 4, j);
        Jv(:, j) = cross(z, c - o);  Jw(:, j) = z;
    end
    Iw = Ti(1:3, 1:3) * diag(P.Idiag(i, :)) * Ti(1:3, 1:3)';
    M = M + P.m(i) * (Jv' * Jv) + Jw' * Iw * Jw;
end
end

function E = energy(P, q, qd)
T = fk(P, q);
[M, ~] = robot_dynamics(P, q, zeros(6, 1));
V = 0;
for i = 1:6
    c = T(1:3, 1:3, i+1) * P.rc(:, i) + T(1:3, 4, i+1);
    V = V + P.m(i) * P.g * c(3);
end
E = 0.5 * qd' * M * qd + V;
end
