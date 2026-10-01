function [J, raw, effort] = fopid_fitness(P, gains, ref, fit)
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
%              whole         [] (default), or a struct that turns the cost
%                            into the whole-controller cost below, with
%                            fields floor, weights (1x4), ref (step_sum,
%                            sine_sum, step_tv, sine_tv), cap (kick, step,
%                            sine: 1x6 each), os_cap (scalar, %) and
%                            cap_penalty
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
%   Whole-controller cost (fit.whole set).  Tracking alone rewards ever
%   stiffer controllers; this cost balances it against the torque it takes
%   (CONTROL_EFFORT, simulated at the control rate):
%     tracking  the ratios of entries 1-6 (both ITAEs and four paper
%               metrics), each floored at fit.whole.floor: a metric better
%               than floor x its reference earns no more credit
%     effort    sum |tau| and total variation of tau in both experiments
%               (the step's derivative kick excluded), relative to
%               fit.whole.ref, not floored
%     J = weighted mean of tracking (fit.weights(1:6)) and effort
%         (fit.whole.weights) + regret * (paper metrics worse than ref)
%         + cap_penalty * sum over joints of max(0, peak/cap - 1) for the
%           kick, the step after the kick and the sine run, and of
%           max(0, overshoot/os_cap - 1): the mean overshoot of Table 3
%           must not hide one joint that overshoots far more
%   effort returns the CONTROL_EFFORT of both runs (fields step, sine).
%
%   See also TUNE_FOPID_HYBRID, SIMULATE_CLOSED_LOOP, PERFORMANCE_METRICS.

if nargin < 4 || isempty(fit), fit = struct(); end
if ~isfield(fit, 'weights'),   fit.weights = [1 1 1 1 0.5 1 1 0]; end
if numel(fit.weights) == 7,    fit.weights = [fit.weights 0]; end
if ~isfield(fit, 'regret'),    fit.regret = 1; end
if ~isfield(fit, 'abort_err'), fit.abort_err = 5; end
if ~isfield(fit, 'time_offset'), fit.time_offset = 0; end
if ~isfield(fit, 'robust'),    fit.robust = false; end
if ~isfield(fit, 'whole'),     fit.whole = []; end
whole = ~isempty(fit.whole);

opt.abort_err = fit.abort_err;
if whole, opt.log_dt = 1e-3; end             % torque at the control rate
T = 5;
raw = nan(1, 8);
effort = [];

step_full = simulate_closed_loop(P, gains, 'step', opt);
if step_full.diverged
    J = diverged_cost(step_full, T, 0);
    return;
end
sine_full = simulate_closed_loop(P, gains, 'sine', opt);
if sine_full.diverged
    J = diverged_cost(sine_full, T, 0.5);
    return;
end
step = on_grid(step_full);                   % the paper's 0.01 s samples
sine = on_grid(sine_full);
if whole
    effort.step = control_effort(step_full, 'step');
    effort.sine = control_effort(sine_full, 'sine');
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
if whole
    W = fit.whole;
    es = effort.step;  en = effort.sine;
    track = max(ratio(1:6), W.floor);
    eff = [mean(es.sum) / W.ref.step_sum, mean(en.sum) / W.ref.sine_sum, ...
           mean(es.tv)  / W.ref.step_tv,  mean(en.tv)  / W.ref.sine_tv];
    over = sum(max(0, es.kick ./ W.cap.kick - 1)) + sum(max(0, es.peak ./ W.cap.step - 1)) ...
           + sum(max(0, en.peak ./ W.cap.sine - 1)) + sum(max(0, ms.overshoot(:)' / W.os_cap - 1));
    J = (sum(fit.weights(1:6) .* track) + sum(W.weights .* eff)) / (sum(fit.weights(1:6)) + sum(W.weights)) ...
        + fit.regret * sum(max(0, ratio(3:7) - 1)) + W.cap_penalty * over;
else
    judged = 3:8;
    judged = judged(fit.weights(judged) > 0);
    J = sum(fit.weights .* ratio) / sum(fit.weights) ...
        + fit.regret * sum(max(0, ratio(judged) - 1));
end
if fit.robust && sensitivity(P, gains, opt, step_full, sine_full) > 1e-6
    J = J + 10;
end
end

% ------------------------------------------------------------------------
function out = on_grid(out)
%ON_GRID  The samples on the paper's 0.01 s grid of a run logged faster.
every = round(0.01 / (out.t(2) - out.t(1)));
if every <= 1, return; end
k = 1:every:numel(out.t);
out.t = out.t(k);  out.r = out.r(:, k);  out.q = out.q(:, k);  out.u = out.u(:, k);
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
