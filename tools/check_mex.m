function check_mex()
%CHECK_MEX  Verify that SIMULATE_MEX and the .m loop of SIMULATE_CLOSED_LOOP agree.
%
%   Runs every gain set of CONTROLLER_GAINS (and FOPSO-GWO if tuned) through
%   both experiments with use_mex = true and false, plus an unstable
%   candidate with abort_err = 5 to check the early-abort path, and reports
%   the largest differences and the speed-up.  Run from the repository root
%   after BUILD_MEX.  The .m runs take about 40 s each in Octave.

addpath(fileparts(fileparts(mfilename('fullpath'))));
if exist('simulate_mex') ~= 3 %#ok<EXIST>
    error('check_mex:missing', 'simulate_mex is not built; run build_mex first');
end
P = robot_params();
sets = {'PID', 'shared'; 'PID', 'step'; 'PID', 'sine'; ...
        'FOPID', 'shared'; 'FOPID', 'step'; 'FOPID', 'sine'};
if exist(fullfile('results', 'fopso_gwo_gains.mat'), 'file')
    sets(end+1, :) = {'FOPSO_GWO', 'shared'};
end
worst_q = 0;  worst_u = 0;  t_m = 0;  t_c = 0;
for i = 1:size(sets, 1)
    g = controller_gains(sets{i, 1}, sets{i, 2});
    for x = {'step', 'sine'}
        tic; a = simulate_closed_loop(P, g, x{1}, struct('use_mex', false)); t_m = t_m + toc;
        tic; b = simulate_closed_loop(P, g, x{1}, struct('use_mex', true));  t_c = t_c + toc;
        dq = max(abs(a.q(:) - b.q(:)));
        du = max(abs(a.u(:) - b.u(:))) / max(1, max(abs(a.u(:))));
        fprintf('%-9s %-6s %-4s  max|dq| = %.1e rad   max|du|/max|u| = %.1e\n', ...
                sets{i, 1}, sets{i, 2}, x{1}, dq, du);
        worst_q = max(worst_q, dq);
        worst_u = max(worst_u, du);
    end
end

% early abort: a destabilised FOPID must stop at the same sample in both
g = controller_gains('FOPID');
g.Kd = -g.Kd;
o = struct('abort_err', 5);
o.use_mex = false;  a = simulate_closed_loop(P, g, 'step', o);
o.use_mex = true;   b = simulate_closed_loop(P, g, 'step', o);
same = isequal(a.diverged, b.diverged) && numel(a.t) == numel(b.t);
fprintf('abort path: diverged %d/%d, %d/%d samples, max|dq| = %.1e\n', ...
        a.diverged, b.diverged, numel(a.t), numel(b.t), max(abs(a.q(:) - b.q(:))));
assert(same && a.diverged, 'the abort path differs');

fprintf('speed-up %.0fx (%.1f s for the .m runs, %.2f s compiled)\n', t_m / t_c, t_m, t_c);
% positions to 1e-8 rad (the round-off floor of the least well-conditioned
% gain set, FOPID/sine, is ~5e-9 rad); torques to 1e-5 relative, since the
% fractional derivatives (mu up to 1.8) amplify that floor
assert(worst_q < 1e-8 && worst_u < 1e-5, 'simulate_mex and the .m loop disagree');
fprintf('check_mex: simulate_mex and the .m loop agree (max |dq| %.1e rad, max |du|/max|u| %.1e)\n', ...
        worst_q, worst_u);
end
