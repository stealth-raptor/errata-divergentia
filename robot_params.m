function P = robot_params()
%ROBOT_PARAMS  Parameters of the 6-DOF UR5 arm used in the simulation.
%
%   P = ROBOT_PARAMS() returns everything the dynamics needs:
%     P.m, P.Idiag   link masses and inertias   (paper, Table 2)
%     P.d, P.a, P.alpha   standard DH parameters (UR5)
%     P.I            6x6 spatial inertia of each link about its own frame
%     P.g            gravity magnitude
%
%   Modelling choices, stated explicitly:
%     * Masses and inertias are taken from Table 2 of the paper.
%     * Link lengths are the real UR5 DH values.  Table 2 lists UR10 lengths,
%       which are inconsistent with the UR5 the paper studies; with those the
%       arm cannot physically reach the published torque values.
%     * Each link's centre of mass sits at its proximal joint axis.
%     * Gravity is compensated (equivalently: excluded), because the
%       published step responses stay exactly at zero before the step.
%     * Joint friction is zero; the paper gives the friction model but no
%       coefficient values.
%
%   See also ROBOT_DYNAMICS.

% ---- link masses and inertias (Table 2 of the paper) --------------------
P.m = [2.0 2.5 5.7 3.9 2.5 2.5];                       % kg
P.Idiag = [1.0 1.0 1.0                                 % kg m^2, about the COM
           4.0 4.0 4.0
           6.0 6.0 4.0
           5.5 5.5 4.0
           4.0 4.0 4.0
           4.0 4.0 4.0] .* [1e-3; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];

% ---- UR5 kinematics, standard Denavit-Hartenberg ------------------------
L = [0.089159 0.425 0.39225 0.10915 0.09465 0.0823];   % m
P.d     = [L(1)  0     0     L(4)  L(5)  L(6)];
P.a     = [0    -L(2) -L(3)  0     0     0   ];
P.alpha = [pi/2  0     0     pi/2 -pi/2  0   ];
P.L     = L;

% ---- gravity ------------------------------------------------------------
P.g = 0;              % gravity compensated; set to 9.81 to include it

% ---- body frames and spatial inertias -----------------------------------
% Body frame i has its origin on joint axis i and rotates with link i.
% The fixed part of the transform from link i-1 to link i is Rx(alpha) and
% the offset [a; 0; d] of the previous joint.
P.R_fixed = zeros(3, 3, 6);
P.p_fixed = zeros(3, 6);
P.I       = zeros(6, 6, 6);
for i = 1:6
    if i == 1
        R = eye(3);
        p = [0; 0; 0];
    else
        R = rotx(P.alpha(i-1));
        p = [P.a(i-1); 0; P.d(i-1)];
    end
    P.R_fixed(:, :, i) = R;
    P.p_fixed(:, i)    = p;

    com = [0; 0; 0];                                   % COM at the joint axis
    Rdh = rotx(P.alpha(i));                            % DH frame within body frame
    Icom = Rdh * diag(P.Idiag(i, :)) * Rdh';
    P.I(:, :, i) = spatial_inertia(P.m(i), com, Icom);
end
end

% ------------------------------------------------------------------------
function R = rotx(t)
R = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
end

function I = spatial_inertia(m, c, Icom)
%SPATIAL_INERTIA  6x6 inertia about a frame whose origin is offset by -c from the COM.
C = [0 -c(3) c(2); c(3) 0 -c(1); -c(2) c(1) 0];
I = [Icom + m*(C*C'), m*C; m*C', m*eye(3)];
end
