function fit = fit_table_gains(controller, experiment, opts)
%FIT_TABLE_GAINS  PID / FOPID gains that reproduce the paper's Tables 3 and 4.
%
%   fit = FIT_TABLE_GAINS(controller, experiment)
%   fit = FIT_TABLE_GAINS(controller, experiment, opts)
%     controller  'PID' or 'FOPID'
%     experiment  'step' (Table 3: overshoot, adjustment time, peak time)
%                 or 'sine' (Table 4: MSE and, with opts.Torque, sum |tau|)
%
%   The paper publishes no gains.  CONTROLLER_GAINS holds gains identified
%   from the paper's published curves (sets 'step', 'sine', 'shared').  Their
%   averaged metrics differ from the paper's tables by up to a third, partly
%   because the tables disagree with the paper's own figures
%   (docs/audit_report.md, Sect. 3).  This tool re-identifies the gains of one
%   experiment against the table instead:
%
%     minimise   curve rms, relative to the curve-identified gains
%     subject to |metric / table value - 1| <= Tol for every table metric
%
%   (the constraint as a heavy quadratic penalty), with CMA-ES (CMAES)
%   started from the curve-identified gains, each gain searched within
%   +-opts.Decades decades of its start on a log scale (lambda, mu of the
%   FOPID in [0.05, 1.95]; the PID keeps lambda = mu = 1).  Candidates that
%   diverge or are numerically ill-conditioned (a relative gain change of
%   1e-10 moving the response by more than 1e-6 rad) are rejected.
%
%   Options: Tol (0.04), Decades (2), Torque (false: the sine fit leaves
%   sum |tau| free; true: it is fitted too), MaxEvals (3000), Sigma0 (0.08),
%   RandomSeed (1), Verbose (true), Start (a gain struct to start the
%   search from, e.g. an earlier fit, instead of the curve-identified gains;
%   the search box and the curve reference stay those of the latter).
%
%   fit: gains, metrics (the table's metrics, averaged over the joints),
%   target, error (relative, per metric), curve_rms (per joint, rad),
%   curve_rms_start, tv (total variation of tau at the control rate, mean
%   over the joints), cost, options.  The gains print ready to paste into
%   CONTROLLER_GAINS.
%
%   See also CONTROLLER_GAINS, PAPER_CURVES, CMAES, PERFORMANCE_METRICS.

if nargin < 3, opts = struct(); end
d = struct('Tol', 0.04, 'Decades', 2, 'Torque', false, 'MaxEvals', 3000, ...
           'Sigma0', 0.08, 'RandomSeed', 1, 'Verbose', true, 'Start', []);
for f = fieldnames(d)'
    if ~isfield(opts, f{1}), opts.(f{1}) = d.(f{1}); end
end
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);

table.PID   = [54.6 2.44 1.39 2.27e-2 3.7422e4];   % paper, Tables 3 and 4
table.FOPID = [31.2 1.89 1.33 0.88e-2 2.5686e4];
controller = upper(controller);
switch experiment
    case 'step', idx = 1:3;
    case 'sine', idx = 4;  if opts.Torque, idx = 4:5; end
    otherwise, error('fit_table_gains:experiment', 'experiment must be ''step'' or ''sine''');
end
target = table.(controller)(idx);

P = robot_params();
g0 = controller_gains(controller, experiment);
paper = paper_curves(controller, experiment);
fractional = strcmp(controller, 'FOPID');

% search space: log10 Kp, Ki, Kd within +-Decades of the start; lambda, mu
x0 = log10([g0.Kp(:); g0.Ki(:); g0.Kd(:)])';
lo = x0 - opts.Decades;  hi = x0 + opts.Decades;
if fractional
    x0 = [x0, g0.lambda(:)', g0.mu(:)'];
    lo = [lo, 0.05 * ones(1, 12)];
    hi = [hi, 1.95 * ones(1, 12)];
end
n = numel(x0);
decode = @(z) to_gains(lo + z .* (hi - lo), fractional);
z0 = (x0 - lo) ./ (hi - lo);
if ~isempty(opts.Start)
    s0 = opts.Start;
    xs = log10([s0.Kp(:); s0.Ki(:); s0.Kd(:)])';
    if fractional, xs = [xs, s0.lambda(:)', s0.mu(:)']; end
    z0 = min(max((xs - lo) ./ (hi - lo), 0), 1);
end

[~, m0, rms0] = evaluate(P, g0, experiment, paper, idx, target, opts.Tol, 1);
cost = @(z) evaluate(P, decode(z), experiment, paper, idx, target, opts.Tol, mean(rms0));

if opts.Verbose
    fprintf('%s %s: start %s, table %s, curve rms %.4f rad\n', controller, experiment, ...
            mat2str(m0, 4), mat2str(target, 4), mean(rms0));
end
copts = struct('MaxEvals', opts.MaxEvals, 'Sigma0', opts.Sigma0, 'Mean0', z0, ...
               'RandomSeed', opts.RandomSeed, 'Verbose', opts.Verbose);
[z, info] = cmaes(cost, n, copts);

g = decode(z);
[J, m, rms, tv] = evaluate(P, g, experiment, paper, idx, target, opts.Tol, mean(rms0));
fit = struct('controller', controller, 'experiment', experiment, 'gains', g, ...
             'metrics', m, 'target', target, 'error', m ./ target - 1, ...
             'curve_rms', rms, 'curve_rms_start', rms0, 'tv', tv, 'cost', J, ...
             'start_metrics', m0, 'options', opts, 'info', info);
if opts.Verbose
    fprintf('\n%s %s fitted to the table: cost %.4g\n', controller, experiment, J);
    fprintf('  metrics %s, table %s, error %s %%\n', mat2str(m, 4), mat2str(target, 4), ...
            mat2str(round(1000 * fit.error) / 10));
    fprintf('  curve rms per joint %s (mean %.4f; before %.4f)\n', mat2str(rms', 3), ...
            mean(rms), mean(rms0));
    fprintf('  torque total variation (mean over joints) %.4g\n', tv);
    for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
        fprintf('        gains.%-6s = [%s]'';\n', f{1}, strtrim(sprintf('%.10g ', g.(f{1}))));
    end
end
end

% ------------------------------------------------------------------------
function [J, m, rms, tv] = evaluate(P, g, experiment, paper, idx, target, tol, rms_ref)
%EVALUATE  Table penalty plus relative curve rms of one gain set.
o = struct('abort_err', 5);
out = simulate_closed_loop(P, g, experiment, o);
m = nan(1, numel(idx));  rms = nan(6, 1);  tv = NaN;
if out.diverged
    J = 1e3 + 1e3 * (1 - out.t(end) / 5);
    return;
end
switch experiment
    case 'step', s = performance_metrics(out, 'step');  all5 = [s.mean, NaN, NaN];
    case 'sine', s = performance_metrics(out, 'sine');  all5 = [NaN, NaN, NaN, s.mean];
end
m = all5(idx);
rms = sqrt(mean((out.q - paper.q).^2, 2));
err = m ./ target - 1;
J = 100 * sum(max(0, abs(err) - tol).^2) + 0.01 * mean(rms) / rms_ref;
if nargout > 3
    fast = struct('log_dt', 1e-3);
    full = simulate_closed_loop(P, g, experiment, fast);
    tv = mean(sum(abs(diff(full.u, 1, 2)), 2));
end
% numerically ill-conditioned closed loops chatter and do not reproduce
g2 = g;
for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
    g2.(f{1}) = g.(f{1}) * (1 + 1e-10);
end
out2 = simulate_closed_loop(P, g2, experiment, o);
if out2.diverged || max(abs(out2.q(:) - out.q(:))) > 1e-6
    J = J + 10;
end
end

function g = to_gains(x, fractional)
%TO_GAINS  Search vector -> gain struct.
g.Kp = 10 .^ x(1:6)';
g.Ki = 10 .^ x(7:12)';
g.Kd = 10 .^ x(13:18)';
if fractional
    g.lambda = x(19:24)';
    g.mu     = x(25:30)';
else
    g.lambda = ones(6, 1);
    g.mu     = ones(6, 1);
end
end
