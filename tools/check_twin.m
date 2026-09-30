function check_twin()
%CHECK_TWIN  Verify that the Octave model and the Python identification twin agree.
%
%   Run from the repository root after
%       python3 tools/ur_twin_reference.py
%   which simulates the gains of CONTROLLER_GAINS with tools/ur_twin.py and
%   writes data/twin_reference.csv.  This function repeats the same runs
%   with SIMULATE_CLOSED_LOOP and reports the largest difference.

addpath(fileparts(fileparts(mfilename('fullpath'))));
ref = dlmread(fullfile('data', 'twin_reference.csv'), ',', 1, 0);
P = robot_params();
worst = 0;
k = 0;
for c = {'PID', 'FOPID'}
    g = controller_gains(c{1});
    for x = {'step', 'sine'}
        out = simulate_closed_loop(P, g, x{1});
        k = k + 1;
        R = ref(ref(:, 1) == k, 3:8).';
        d = max(abs(out.q(:) - R(:)));
        fprintf('%-6s %-4s  max |q_octave - q_twin| = %.2e rad\n', c{1}, x{1}, d);
        worst = max(worst, d);
    end
end
assert(worst < 1e-8, 'Octave model and Python twin disagree');
fprintf('check_twin: Octave and the Python twin agree (max difference %.1e rad)\n', worst);
end
