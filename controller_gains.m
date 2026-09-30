function gains = controller_gains(controller)
%CONTROLLER_GAINS  Gains of the six joint controllers, identified from the paper.
%
%   gains = CONTROLLER_GAINS('PID')    classical PID  (lambda = mu = 1)
%   gains = CONTROLLER_GAINS('FOPID')  fractional-order PID
%
%   Each field is 6x1, one entry per joint.  The paper publishes no gains.
%   These were identified from the paper's own published curves: all six
%   joint trajectories of the step experiment (Figs 6-11) and of the sine
%   experiment (Figs 13-18), read from the PDF's vector graphics, were matched
%   by least squares in the fully coupled closed loop, one gain set per
%   controller for both experiments (tools/identify_gains.py, plant:
%   table2 lengths, COM distal, g = 0).
%
%   The joint-1 step and sine curves of the paper cannot both be matched by
%   any gain set (docs/audit_report.md, Sect. 4); these gains are the best
%   compromise over all 12 curves of each controller.
%
%   See also FOPID_CONTROLLER, ROBOT_PARAMS.

switch upper(controller)
    case 'PID'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.157 0.050 0.076 0.104 0.187 0.076
        %   sine 0.143 0.050 0.118 0.046 0.218 0.021
        gains.Kp     = [75.45276223 1504.044829 696.2823467 69.66649056 4.7024463 6389.310235]';
        gains.Ki     = [565.531096 3041.237729 71.11348748 188.2343736 0.6565698329 1305.181802]';
        gains.Kd     = [137.1522896 102.8120137 113.9384517 1.02441153 0.4189171128 10.20444477]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'FOPID'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.095 0.034 0.041 0.051 0.141 0.055
        %   sine 0.062 0.032 0.076 0.017 0.157 0.030
        gains.Kp     = [466.5048899 0.2050011905 0.07599433869 124.5656189 2.134765482 28.46272228]';
        gains.Ki     = [348.1945821 2206.077613 54.67759987 11.92085781 2.285310734 0.009220016116]';
        gains.Kd     = [49.95111633 645.0353382 591.4914619 0.4691240383 1.473705321 0.1510568454]';
        gains.lambda = [1.662623676 0.6526495045 1.95 0.3800523434 0.7206369078 0.05006636685]';
        gains.mu     = [1.425361662 0.5509011254 0.3845477049 1.453723122 0.7627629958 1.817543914]';
    otherwise
        error('controller_gains:name', 'unknown controller ''%s''', controller);
end
end
