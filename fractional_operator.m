function F = fractional_operator(order, dt)
%FRACTIONAL_OPERATOR  Discrete realisation of the fractional operator s^order.
%
%   F = FRACTIONAL_OPERATOR(order, dt) builds, for each of the six joints, a
%   filter that approximates s^order(i) at sample time dt.
%
%   Method: Oustaloup's recursive approximation over [1e-3, 1e3] rad/s with
%   N = 5, i.e. 11 poles and zeros,
%
%       s^a  ~=  K + sum_k  r_k / (s + p_k),
%
%   written in parallel (partial-fraction) form so every first-order section
%   can be discretised exactly with a zero-order hold.  That keeps the fast
%   poles near 1e3 rad/s stable at dt = 1 ms.
%
%   The order must lie in (-1, 1): FOPID_CONTROLLER splits any integer part
%   off first and applies it as an extra integration or a backward
%   difference.  An order of exactly zero yields K = 1 and r = 0, i.e. a
%   pass-through, so integer-order PID needs no special case.
%
%   Output fields (all 6-by-1 or 6-by-11):
%     F.K      direct feed-through gain of the filter
%     F.A, F.B state-update coefficients (exact ZOH)
%     F.r      output weights of the filter states
%     F.x      filter states, initialised to zero
%
%   See also FOPID_CONTROLLER, FOPID_UPDATE.

wb = 1e-3;  wh = 1e3;  N = 5;              % fitting band and order
M = 2*N + 1;
k = -N:N;

a = order(:);
if any(abs(a) >= 1)
    error('fractional_operator:range', 'order must lie in (-1, 1); split off the integer part first');
end

zeros_ = wb * (wh/wb).^((k + N + 0.5*(1 - a)) / M);
poles  = wb * (wh/wb).^((k + N + 0.5*(1 + a)) / M);
F.K = wh.^a;

F.r = zeros(numel(a), M);
for j = 1:M
    num = prod(zeros_ - poles(:, j), 2);                    % prod_i (z_i - p_j)
    den = prod(poles(:, [1:j-1 j+1:M]) - poles(:, j), 2);   % prod_{i~=j} (p_i - p_j)
    F.r(:, j) = F.K .* num ./ den;
end

F.A = exp(-poles * dt);                    % exact zero-order-hold update
F.B = (1 - F.A) ./ poles;
F.x = zeros(numel(a), M);
end
