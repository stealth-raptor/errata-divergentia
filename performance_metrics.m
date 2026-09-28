function m = performance_metrics(out, reference)
%PERFORMANCE_METRICS  Metrics of the paper's Tables 3 (step) and 4 (sine).
%
%   m = PERFORMANCE_METRICS(out, reference)  with out from
%   SIMULATE_CLOSED_LOOP.
%
%   Step reference ('step'), per joint and averaged over the six joints:
%     overshoot        100 * (max q - 1)                  [%]
%     adjustment_time  time to stay inside +-2 % of the reference, measured
%                      from t = 0 (MATLAB's stepinfo definition)   [s]
%     peak_time        time of the largest excursion, from t = 0   [s]
%
%   Sine reference ('sine'), per joint and averaged:
%     mse              mean squared tracking error          [rad^2]
%     torque           sum of |tau| over the logged samples [Nm]
%
%   Two conventions are worth stating, because the paper does not:
%     * times are measured from t = 0, not from the step instant, which is
%       the only reading consistent with the paper's peak times of 1.09-1.39 s
%       for a step applied at 1 s;
%     * the torque metric is a sum over samples, so it scales with the
%       logging rate; 5001 samples (5 s at 1 ms) are used here.
%
%   See also SIMULATE_CLOSED_LOOP.

switch reference
    case 'step'
        m.overshoot       = zeros(6, 1);
        m.adjustment_time = zeros(6, 1);
        m.peak_time       = zeros(6, 1);
        for j = 1:6
            y  = out.q(j, :);
            si = stepinfo(y, out.t, 1, 0);
            m.overshoot(j)       = max(0, 100 * (max(y) - 1));
            m.adjustment_time(j) = si.SettlingTime;
            m.peak_time(j)       = si.PeakTime;
        end
        % a joint that never settles is counted at the 5 s horizon
        ts = m.adjustment_time;
        ts(isnan(ts)) = out.t(end);
        m.mean = [mean(m.overshoot), mean(ts), mean(m.peak_time)];

    case 'sine'
        e = out.r - out.q;
        m.mse    = mean(e.^2, 2);
        m.torque = sum(abs(out.u), 2);
        m.mean   = [mean(m.mse), mean(m.torque)];

    otherwise
        error('performance_metrics:ref', 'unknown reference "%s"', reference);
end
end
