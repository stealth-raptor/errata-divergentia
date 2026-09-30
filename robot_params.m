function P = robot_params(opt)
%ROBOT_PARAMS  Parameters of the 6-DOF arm, taken from the paper (Table 2).
%
%   P = ROBOT_PARAMS()       the model used for the reproduction
%   P = ROBOT_PARAMS(opt)    with any of these fields overridden:
%       opt.lengths   'table2' (default) or 'ur5'
%       opt.com       'distal' (default), 'middle' or 'proximal'
%       opt.g         gravity [m/s^2], default 0
%       opt.fc, opt.b friction coefficients (6x1), default 0
%
%   Returns what ROBOT_DYNAMICS needs:
%     P.m, P.Idiag     link masses and principal inertias about the COM (Table 2)
%     P.d, P.a, P.alpha  standard Denavit-Hartenberg parameters of the UR arm
%     P.pstar          origin of frame i relative to frame i-1, in frame i
%     P.rc             centre of mass of link i, in frame i
%     P.g              gravity magnitude (acts along -z0)
%     P.fc, P.b        Coulomb and viscous friction, tau_f = fc sgn(qd) + b qd
%
%   What the paper gives, and what it leaves open:
%     * Masses, lengths and inertia diagonals: Table 2, used as printed.
%       The lengths are the paper's "UR5" values; they coincide with the
%       UR10 DH parameters, which is noted in docs/audit_report.md but not
%       changed: the argument previously used to replace them (a torque
%       floor) assumed 5001 torque samples, whereas the paper's figures show
%       it logged 501 (0.01 s).
%     * Kinematic structure: the UR5 of Fig. 3, standard DH with
%       alpha = [pi/2 0 0 pi/2 -pi/2 0].
%     * Centre of mass: not published.  Default: at the origin of each DH
%       frame (r = 0), the Robotics Toolbox default; the published curves
%       are matched best with it (docs/audit_report.md, model selection).
%     * Gravity: the model contains G(q) (Eq. 21) but g = 0 by default.
%       The published step responses are exactly flat before the step
%       (Figs 6-11, drawn as a single straight segment over 0-0.99 s),
%       which an uncompensated arm under PID could not do.
%     * Friction: Eq. 21 has tau_f = fc sgn(qd) + b qd but no coefficient
%       values are given, so both are zero.
%
%   See also ROBOT_DYNAMICS.

if nargin < 1, opt = struct(); end
if ~isfield(opt, 'lengths'), opt.lengths = 'table2'; end
if ~isfield(opt, 'com'),     opt.com = 'distal';     end
if ~isfield(opt, 'g'),       opt.g = 0;              end
if ~isfield(opt, 'fc'),      opt.fc = zeros(6, 1);   end
if ~isfield(opt, 'b'),       opt.b = zeros(6, 1);    end

% ---- Table 2 of the paper ----------------------------------------------
P.m = [2.0 2.5 5.7 3.9 2.5 2.5];                           % kg
P.Idiag = [1.0 1.0 1.0                                     % kg m^2, about the COM
           4.0 4.0 4.0
           6.0 6.0 4.0
           5.5 5.5 4.0
           4.0 4.0 4.0
           4.0 4.0 4.0] .* [1e-3; 1e-2; 1e-2; 1e-2; 1e-2; 1e-2];

switch lower(opt.lengths)
    case 'table2', L = [0.128 0.612 0.571 0.164 0.115 0.092];            % m, Table 2
    case 'ur5',    L = [0.089159 0.425 0.39225 0.10915 0.09465 0.0823];  % m, real UR5
    otherwise,     error('robot_params:lengths', 'unknown lengths ''%s''', opt.lengths);
end
P.L = L;

% ---- UR kinematics, standard Denavit-Hartenberg --------------------------
P.d     = [L(1)  0     0     L(4)  L(5)  L(6)];
P.a     = [0    -L(2) -L(3)  0     0     0   ];
P.alpha = [pi/2  0     0     pi/2 -pi/2  0   ];
P.ca = cos(P.alpha);
P.sa = sin(P.alpha);

% origin of frame i seen from frame i-1, expressed in frame i
P.pstar = [P.a; P.d .* P.sa; P.d .* P.ca];

% ---- centre of mass of each link, in its own DH frame --------------------
switch lower(opt.com)
    case 'distal',   frac = 0;      % at the frame origin (joint i+1 end)
    case 'middle',   frac = 0.5;    % half-way along the link
    case 'proximal', frac = 1;      % on the axis of joint i
    otherwise, error('robot_params:com', 'unknown COM placement ''%s''', opt.com);
end
P.rc = -frac * P.pstar;
P.com = lower(opt.com);
P.lengths = lower(opt.lengths);

% ---- gravity and friction -------------------------------------------------
P.g  = opt.g;
P.fc = opt.fc(:);
P.b  = opt.b(:);
end
