function gains = controller_gains(controller, experiment)
%CONTROLLER_GAINS  Gains of the six joint controllers, identified from the paper.
%
%   gains = CONTROLLER_GAINS(controller)              one gain set for both experiments
%   gains = CONTROLLER_GAINS(controller, experiment)  gain set for one experiment
%     controller  'PID' (lambda = mu = 1), 'FOPID', 'FBPA' or 'FOPSO_GWO'
%     experiment  'step', 'sine', or 'shared' (the default)
%
%   Each field is 6x1, one entry per joint.  The paper publishes no gains.
%   All of them were identified from the paper's own published curves, read
%   from the PDF's vector graphics: the six joint trajectories of the step
%   experiment (Figs 6-11) and/or of the sine experiment (Figs 13-18) were
%   matched by least squares in the fully coupled closed loop
%   (tools/identify_gains.py; plant: table2 lengths, COM distal, g = 0).
%
%   'shared'      one gain set fitted to both experiments at once.  This is
%                 what the paper implies (it describes one PID and one FOPID
%                 controller), but its joint-1 step and sine curves cannot
%                 both be matched by any gain set (docs/audit_report.md, 4).
%   'step','sine' a gain set fitted to that experiment's curves only.  The
%                 paper does not say whether its two experiments used the
%                 same gains; separate sets match each experiment more
%                 closely (docs/audit_report.md, 4.3).  MAIN uses these by
%                 default.
%
%   'FBPA'        the FOPID re-tuned by the paper's FBPA, and
%   'FOPSO_GWO'   by this work's FO-PSO / grey-wolf hybrid (TUNE_FOPID_HYBRID),
%                 read from results/fbpa_gains.mat and results/fopso_gwo_gains.mat;
%                 one gain set for both experiments, so EXPERIMENT is ignored.
%
%   See also FOPID_CONTROLLER, ROBOT_PARAMS, MAIN, TUNE_FOPID_HYBRID.

if nargin < 2, experiment = 'shared'; end
if any(strcmpi(controller, {'FBPA', 'FOPSO_GWO'}))
    file = tuned_file(controller);
    if ~exist(file, 'file')
        error('controller_gains:untuned', 'no tuned gains in %s; run tune_fopid_hybrid first', file);
    end
    S = load(file, 'gains');
    gains = S.gains;
    return;
end

switch [upper(controller) '/' lower(experiment)]
    case 'PID/shared'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.157 0.050 0.076 0.104 0.187 0.076   (mean 0.108)
        %   sine 0.143 0.050 0.118 0.046 0.218 0.021   (mean 0.099)
        gains.Kp     = [75.45276223 1504.044829 696.2823467 69.66649056 4.7024463 6389.310235]';
        gains.Ki     = [565.531096 3041.237729 71.11348748 188.2343736 0.6565698329 1305.181802]';
        gains.Kd     = [137.1522896 102.8120137 113.9384517 1.02441153 0.4189171128 10.20444477]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'PID/step'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.168 0.083 0.049 0.046 0.142 0.065   (mean 0.092)
        gains.Kp     = [97.37514954 1564.876986 359.2803819 102.4754746 4.088352959 56.88093831]';
        gains.Ki     = [557.4767198 10486.36353 163.0551406 6.115185187 0.9552286351 310.5752354]';
        gains.Kd     = [159.9754331 208.9663973 149.8627569 2.754135176 0.1769806667 2.982132753]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'PID/sine'
        % rms vs the published curves, joints 1-6 [rad]:
        %   sine 0.124 0.064 0.105 0.025 0.143 0.021   (mean 0.080)
        gains.Kp     = [85.75329619 1083.287035 386.0742159 8.5825545 4.746903194 8.473415177]';
        gains.Ki     = [308.9360215 5037.557484 0.0001003450182 50.57169418 2.164146914 178.3005691]';
        gains.Kd     = [70.71407657 194.9763253 56.34833611 233.4242584 1.855891971 10.40962032]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'FOPID/shared'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.095 0.034 0.041 0.051 0.141 0.055   (mean 0.070)
        %   sine 0.062 0.032 0.076 0.017 0.157 0.030   (mean 0.062)
        gains.Kp     = [466.5048899 0.2050011905 0.07599433869 124.5656189 2.134765482 28.46272228]';
        gains.Ki     = [348.1945821 2206.077613 54.67759987 11.92085781 2.285310734 0.009220016116]';
        gains.Kd     = [49.95111633 645.0353382 591.4914619 0.4691240383 1.473705321 0.1510568454]';
        gains.lambda = [1.662623676 0.6526495045 1.95 0.3800523434 0.7206369078 0.05006636685]';
        gains.mu     = [1.425361662 0.5509011254 0.3845477049 1.453723122 0.7627629958 1.817543914]';
    case 'FOPID/step'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.118 0.038 0.034 0.024 0.077 0.054   (mean 0.058)
        gains.Kp     = [0.2246029026 0.001008584286 233.0620586 129.1202032 0.1001314098 37.9669364]';
        gains.Ki     = [1011.942297 4092.232668 66.46026957 0.0001000343441 2.323429273 655.2171066]';
        gains.Kd     = [111.6222949 488.4447765 310.7293829 2.102304702 0.8775511388 0.1470426384]';
        gains.lambda = [0.3762673819 0.586740784 0.7319898437 1.939554884 0.09077580848 1.70607341]';
        gains.mu     = [1.28267767 0.7291342749 0.6235574115 1.223482714 0.7142570757 1.807796772]';
    case 'FOPID/sine'
        % rms vs the published curves, joints 1-6 [rad]:
        %   sine 0.050 0.046 0.069 0.016 0.108 0.019   (mean 0.051)
        gains.Kp     = [495.9935847 0.002705183396 0.1236514463 153.4029937 15.17725195 28.57833231]';
        gains.Ki     = [532.711622 2432.873877 323.629477 0.0210062648 5.179202505 71.34826142]';
        gains.Kd     = [82.03897077 114.6546778 450.4413379 11.68525819 0.9272306454 1.964879599]';
        gains.lambda = [1.774002479 0.269229394 1.949843658 1.07017743 1.947748298 0.8092217149]';
        gains.mu     = [1.227019335 1.347895571 0.7744312048 1.387471886 1.750055386 1.274988856]';
    otherwise
        error('controller_gains:name', 'unknown controller/experiment ''%s/%s''', controller, experiment);
end
end

% ------------------------------------------------------------------------
function file = tuned_file(controller)
%TUNED_FILE  Where TUNE_FOPID_HYBRID saves the gains of CONTROLLER.
file = fullfile(fileparts(mfilename('fullpath')), 'results', [lower(controller) '_gains.mat']);
end
