function m = performance_metrics(out, reference, band)
%PERFORMANCE_METRICS  Metrics of the paper's Tables 3 (step) and 4 (sine).
%
%   m = PERFORMANCE_METRICS(out, reference)        out from SIMULATE_CLOSED_LOOP
%   m = PERFORMANCE_METRICS(out, 'step', band)     settling band (default 0.05)
%
%   Step reference ('step'), per joint and averaged over the six joints:
%     overshoot        100 * (max q - 1)                               [%]
%     adjustment_time  last time |q - 1| leaves the band, from t = 0   [s]
%     peak_time        time of the maximum, from t = 0                 [s]
%
%   Sine reference ('sine'), per joint and averaged:
%     mse              mean of e^2 over the logged samples             [rad^2]
%     torque           sum of |tau| over the logged samples            [Nm]
%
%   All metrics are computed on the 0.01 s samples the paper logged.  The
%   paper defines none of them precisely; the choices below are the ones its
%   own published curves support (docs/audit_report.md, Sect. 3):
%     * times are measured from t = 0, as the peak times of 1.09-1.39 s for a
%       step applied at t = 1 s require;
%     * MSE and torque on the 0.01 s grid: Table 4's MSE recomputed this
%       way from the published curves agrees with the table to 0.7 %;
%     * the adjustment (settling) time uses a +-5 % band: recomputed from
%       the published step curves (PID / FOPID / FBPA-FOPID) it gives
%       2.32 / 1.95 / 1.52 s against the table's 2.44 / 1.89 / 1.43 s, where
%       the 2 % band gives 3.12 / 2.13 / 1.68 s (joint 5 under PID never
%       enters the 2 % band in the paper).
%   A joint that never settles is counted at the 5 s horizon.
%
%   See also SIMULATE_CLOSED_LOOP.

if nargin < 3, band = 0.05; end

switch reference
    case 'step'
        m.overshoot       = zeros(6, 1);
        m.adjustment_time = zeros(6, 1);
        m.peak_time       = zeros(6, 1);
        for j = 1:6
            y = out.q(j, :);
            [ymax, k] = max(y);
            m.overshoot(j) = max(0, 100 * (ymax - 1));
            m.peak_time(j) = out.t(k);
            m.adjustment_time(j) = settling_time(out.t, y, 1, band);
        end
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

% ------------------------------------------------------------------------
function ts = settling_time(t, y, yfinal, band)
%SETTLING_TIME  First time after which y stays within +-band*yfinal.
%   The exit from the band is located by linear interpolation between
%   samples; NaN if y is still outside the band at the end.
outside = abs(y - yfinal) > band * abs(yfinal);
k = find(outside, 1, 'last');
if isempty(k)
    ts = t(1);
elseif k == numel(t)
    ts = NaN;
else
    % interpolate the crossing of the band edge between samples k and k+1
    lvl = yfinal + sign(y(k) - yfinal) * band * abs(yfinal);
    ts = t(k) + (lvl - y(k)) / (y(k+1) - y(k)) * (t(k+1) - t(k));
end
end
