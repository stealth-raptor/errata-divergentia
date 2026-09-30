function out = paper_curves(controller, reference)
%PAPER_CURVES  The published curves of the paper, as a simulation result.
%
%   out = PAPER_CURVES(controller, reference)
%     controller  'PID', 'FOPID' or 'FBPA'
%     reference   'step' (Figs 6-12) or 'sine' (Figs 13-19)
%
%   Returns the same fields as SIMULATE_CLOSED_LOOP, on the paper's 0.01 s
%   logging grid, so that PERFORMANCE_METRICS and the plots can be applied
%   to the paper's own data:
%     out.t   1x501, out.r, out.q, out.e, out.u   6x501
%
%   The curves were read exactly from the vector graphics of the published
%   PDF by tools/extract_paper_curves.py (calibrated on each panel's grid
%   lines; the sine reference is recovered as sin(1.5 t) to 1e-3 rad rms).
%   Panel (a) gives q, panel (b) gives e, Figs 12/19 give u.
%
%   See also SIMULATE_CLOSED_LOOP, PERFORMANCE_METRICS.

here = fileparts(mfilename('fullpath'));
base = fullfile(here, 'data', 'paper_grid', sprintf('%s_%s_', reference, upper(controller)));
Q = dlmread([base 'q.csv'],   ',', 1, 0);
E = dlmread([base 'e.csv'],   ',', 1, 0);
U = dlmread([base 'tau.csv'], ',', 1, 0);

out.t = Q(:, 1).';
out.q = Q(:, 2:7).';
out.e = E(:, 2:7).';
out.u = U(:, 2:7).';
switch reference
    case 'step', out.r = repmat(double(out.t >= 1 - 1e-9), 6, 1);
    case 'sine', out.r = repmat(sin(1.5 * out.t), 6, 1);
end
end
