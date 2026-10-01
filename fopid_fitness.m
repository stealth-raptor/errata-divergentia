function [J, raw] = fopid_fitness(P, gains, ref, fit)
%FOPID_FITNESS  Cost of one gain set for the FO-PSO/GWO tuner.
%
%   [J, raw] = FOPID_FITNESS(P, gains, ref, fit)
%     P      robot parameters from ROBOT_PARAMS
%     gains  6x1 fields Kp, Ki, Kd, lambda, mu
%     ref    raw vector of a reference controller (normally the existing
%            FOPID), used to normalise the cost; [] returns J = NaN and only
%            computes raw
%     fit    struct with
%              weights       1x8 weights on the entries of raw (below); a
%                            1x7 vector is padded with a zero weight on the
%                            step peak torque
%              regret        extra weight on every metric that ends up worse
%                            than the reference (pushes the search towards
%                            gain sets that beat FOPID on every count)
%              abort_err     stop a simulation once any |error| exceeds this
%              robust        if true, a candidate that completes both runs is
%                            simulated again with all gains x (1 + 1e-10);
%                            if any joint angle then moves by more than
%                            1e-6 rad the closed loop is numerically
%                            ill-conditioned (it chatters in a round-off-
%                            sensitive regime, and its results would not
%                            be reproducible across platforms), and 10 is
%                            added to J (default false)
%              time_offset   subtracted from the adjustment and peak times
%                            (entries 4-5) of raw and ref before their ratio
%                            is taken (default 0); with 1, the step instant,
%                            the ratio compares response times after the
%                            step instead of times from t = 0, which all
%                            start at 1 s
%
%   raw = [ITAE_step, ITAE_sine, overshoot, adjustment time, peak time,
%          sine MSE, sine torque, step peak torque]
%
%   The step peak torque is max|tau| over the step run, averaged over the
%   joints. It is not one of the paper's metrics, but without it the tuner
%   cannot see the torque spike at the step instant, which the summed sine
%   torque hardly registers.
%
%   ITAE is the paper's fitness function, Eq. 29,
%       ITAE = integral of  t * sum_j |e_j(t)|  dt,
%   evaluated here on both experiments; the other five entries are the
%   paper's own performance figures from PERFORMANCE_METRICS. The cost is the
%   weighted mean of the ratios raw ./ ref, so the reference itself scores
%   exactly 1 and any J < 1 is an improvement on it:
%
%       J = sum(w .* raw./ref) / sum(w)  +  regret * sum(max(0, raw./ref - 1))
%
%   (the regret sum runs over entries 3-8, the five paper metrics and the
%   peak torque, where their weight is non-zero). A run that
%   diverges scores 1e3 + 1e3 * (fraction of the 5 s it failed to survive),
%   so unstable candidates are still ranked by how long they held on.
%
%   See also TUNE_FOPID_HYBRID, SIMULATE_CLOSED_LOOP, PERFORMANCE_METRICS.

if nargin < 4 || isempty(fit), fit = struct(); end
if ~isfield(fit, 'weights'),   fit.weights = [1 1 1 1 0.5 1 1 0]; end
if numel(fit.weights) == 7,    fit.weights = [fit.weights 0]; end
if ~isfield(fit, 'regret'),    fit.regret = 1; end
if ~isfield(fit, 'abort_err'), fit.abort_err = 5; end
if ~isfield(fit, 'time_offset'), fit.time_offset = 0; end
if ~isfield(fit, 'robust'),    fit.robust = false; end

opt.abort_err = fit.abort_err;
T = 5;
raw = nan(1, 8);

step = simulate_closed_loop(P, gains, 'step', opt);
if step.diverged
    J = diverged_cost(step, T, 0);
    return;
end
sine = simulate_closed_loop(P, gains, 'sine', opt);
if sine.diverged
    J = diverged_cost(sine, T, 0.5);
    return;
end

ms = performance_metrics(step, 'step');
mn = performance_metrics(sine, 'sine');
raw = [itae(step), itae(sine), ms.mean, mn.mean, mean(max(abs(step.u), [], 2))];

if isempty(ref)
    J = NaN;
    return;
end

num = raw;  den = ref;
num(4:5) = num(4:5) - fit.time_offset;
den(4:5) = den(4:5) - fit.time_offset;
ratio = max(num, 0) ./ max(den, eps);
judged = 3:8;
judged = judged(fit.weights(judged) > 0);
J = sum(fit.weights .* ratio) / sum(fit.weights) ...
    + fit.regret * sum(max(0, ratio(judged) - 1));
if fit.robust && sensitivity(P, gains, opt, step, sine) > 1e-6
    J = J + 10;
end
end

% ------------------------------------------------------------------------
function d = sensitivity(P, gains, opt, step, sine)
%SENSITIVITY  Response change for a relative change of 1e-10 in every gain.
%   A well-conditioned closed loop moves by ~1e-9 rad; one that chatters in a
%   round-off-sensitive regime by ~1e-3 rad whatever the size of the change.
g = gains;
for f = {'Kp', 'Ki', 'Kd', 'lambda', 'mu'}
    g.(f{1}) = gains.(f{1}) * (1 + 1e-10);
end
a = simulate_closed_loop(P, g, 'step', opt);
b = simulate_closed_loop(P, g, 'sine', opt);
if a.diverged || b.diverged
    d = Inf;
    return;
end
d = max(max(abs(a.q(:) - step.q(:))), max(abs(b.q(:) - sine.q(:))));
end

% ------------------------------------------------------------------------
function v = itae(out)
%ITAE  Integral of time-weighted absolute error summed over the joints.
v = trapz(out.t, out.t .* sum(abs(out.r - out.q), 1));
end

function J = diverged_cost(out, T, survived_before)
%DIVERGED_COST  Large cost, lower the longer the run held on.
%   survived_before credits a step run that finished before the sine run
%   diverged.
frac = survived_before + 0.5 * out.t(end) / T;
J = 1e3 + 1e3 * (1 - frac);
end
