function gains = controller_gains(controller, experiment)
%CONTROLLER_GAINS  Gains of the six joint controllers, identified from the paper.
%
%   gains = CONTROLLER_GAINS(controller)              one gain set for both experiments
%   gains = CONTROLLER_GAINS(controller, experiment)  gain set for one experiment
%     controller  'PID' (lambda = mu = 1), 'FOPID', 'FBPA' (the FBPA-FOPID),
%                 'PSO' (the PSO-FOPID), 'FOPSO_GWO', 'FOPSO_GWO_CMA' or 'FOPSO_GWO_CC'
%     experiment  'step', 'sine', or 'shared' (the default); for PID and FOPID
%                 also 'table_step' and 'table_sine'
%
%   Each field is 6x1, one entry per joint.  The paper publishes no gains.
%   All of them were identified from the paper's own published curves, read
%   from the PDF's vector graphics: the six joint trajectories of the step
%   experiment (Figs 6-11) and/or of the sine experiment (Figs 13-18) of
%   that controller were matched by least squares in the fully coupled
%   closed loop
%   (tools/identify_gains.py on branch brand-new-day, not needed here: the
%   identified gains are written out below; plant: table2 lengths, COM
%   distal, g = 0).
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
%   'table_step', 'table_sine'  (PID and FOPID) gain sets fitted to the
%                 paper's Tables 3 and 4 instead of its curves: every table
%                 metric of that experiment within 10 % of the paper's value,
%                 and otherwise as close to the published curves as possible
%                 (tools/fit_table_gains.m).  The paper's tables disagree with
%                 its own figures (docs/audit_report.md, 3), so these sets
%                 match the curves less closely than 'step' and 'sine'.  MAIN
%                 uses them with MAIN('table').  They are not used by the
%                 tuning cost, whose torque caps come from the sets above.
%
%   'FOPSO_GWO'   the FOPID tuned by this work's FO-PSO / grey-wolf hybrid,
%   'FOPSO_GWO_CMA' by that hybrid with its CMA-ES refinement,
%   'FOPSO_GWO_CC' by that hybrid with cooperative coevolution, and
%   'PSO'         by plain PSO (the paper's improved PSO), all with the same
%                 whole-controller cost (TUNE_FOPID_HYBRID), read from
%                 results/<name in lower case>_gains.mat;
%                 one gain set for both experiments, so EXPERIMENT is ignored.
%                 (FBPA re-run as an optimiser, TUNE_FOPID_HYBRID with
%                 Optimizer 'FBPA', writes results/fbpa_gains.mat; that is not
%                 a gain set of this function: 'FBPA' here reproduces the
%                 paper's FBPA-FOPID curves.)
%
%   See also FOPID_CONTROLLER, ROBOT_PARAMS, MAIN, TUNE_FOPID_HYBRID.

if nargin < 2, experiment = 'shared'; end
if any(strcmpi(controller, {'FOPSO_GWO', 'FOPSO_GWO_CMA', 'FOPSO_GWO_CC', 'PSO'}))
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
    case 'FBPA/shared'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.075 0.015 0.024 0.034 0.064 0.045   (mean 0.043)
        %   sine 0.055 0.022 0.065 0.018 0.062 0.018   (mean 0.040)
        gains.Kp     = [597.8824431 0.1434815132 455.9707133 53.45665593 0.04140659648 542.9101952]';
        gains.Ki     = [446.7583618 1822.211343 0.01137724837 75.30711814 680.6070125 0.5540232669]';
        gains.Kd     = [56.89198356 261.9893742 103.7086648 13.27766929 6.314529721 359.7143173]';
        gains.lambda = [1.948766589 0.05392475073 0.8307652713 0.1919073142 0.4245580453 1.185441396]';
        gains.mu     = [1.430396425 0.8295674254 0.9714340539 0.6204829484 1.285828505 0.5981791625]';
    case 'FBPA/step'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step 0.026 0.027 0.038 0.053 0.052 0.044   (mean 0.040)
        gains.Kp     = [0.03019615241 0.003117763082 36.34195347 99.23208699 0.01844205206 1632.51821]';
        gains.Ki     = [48.97499014 21.90373185 14.33853659 0.0002432365723 760.6432845 29730.60498]';
        gains.Kd     = [14.08304669 917.4419973 81.67353576 8.897366238 9.229806161 0.229492629]';
        gains.lambda = [0.0500283687 1.949451944 0.4205795346 0.1690771851 0.900550279 1.582303849]';
        gains.mu     = [0.9998708848 0.514832496 0.9999230055 0.4464245556 0.784430861 1.747201606]';
    case 'FBPA/sine'
        % rms vs the published curves, joints 1-6 [rad]:
        %   sine 0.038 0.018 0.050 0.015 0.061 0.015   (mean 0.033)
        gains.Kp     = [364.8420664 3.333513883 0.00254491802 116.4486735 107646.9101 0.3871261489]';
        gains.Ki     = [581.954497 1900.905882 372.0667211 0.0003336347483 9221.445046 1157.612758]';
        gains.Kd     = [169.7245658 1543.395685 520.1945999 5.553126174 0.1540965712 186.6467437]';
        gains.lambda = [1.591565168 1.090930414 1.948322993 1.307296612 1.391655871 1.94005557]';
        gains.mu     = [0.8397908885 0.4538816423 0.7757818084 1.405946266 1.910186661 0.5412488322]';
    case 'PID/table_step'
        % fitted to the paper's Table 3, every metric within 10 % (tools/fit_table_gains.m):
        %   difference to the table: overshoot -9.0 %, adjustment time +9.0 %, peak time +4.9 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.177 0.067 0.049 0.046 0.155 0.061 (mean 0.093)
        gains.Kp     = [179.5346724 1582.327205 372.9408767 100.7142999 3.99902397 140.7491131]';
        gains.Ki     = [1217.021456 8356.023527 124.9820399 4.031584892 0.7844714445 559.0830427]';
        gains.Kd     = [218.1809165 183.0850197 132.3721848 3.023725067 0.1942174349 1.248639522]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'PID/table_sine'
        % fitted to the paper's Table 4, every metric within 10 % (tools/fit_table_gains.m):
        %   difference to the table: MSE -8.9 %, sum |tau| -8.9 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.143 0.060 0.195 0.023 0.177 0.021 (mean 0.103)
        gains.Kp     = [71.85762901 10435.14108 845.8230593 186.2325337 7.587992021 6.990320416]';
        gains.Ki     = [298.3944056 27631.60421 0.009513614826 13.11856888 2.704779655 1477.431058]';
        gains.Kd     = [69.3834926 154.3642649 0.5739346068 65.15412446 3.923319473 59.51609494]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'FOPID/table_step'
        % fitted to the paper's Table 3, every metric within 10 % (tools/fit_table_gains.m):
        %   difference to the table: overshoot -6.4 %, adjustment time +3.1 %, peak time +1.6 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.154 0.022 0.035 0.020 0.042 0.052 (mean 0.054)
        gains.Kp     = [0.2428467899 0.0001745928079 373.7463299 116.4299858 0.349293042 922.5859329]';
        gains.Ki     = [2227.115749 2568.528305 25.06357956 6.348857e-05 2.424867885 97.41858185]';
        gains.Kd     = [248.5994423 264.0250904 237.1439936 2.120218293 0.536405423 0.4603233828]';
        gains.lambda = [0.7548443081 0.310490346 1.195627167 1.656903638 0.05 1.818377887]';
        gains.mu     = [0.9877559563 0.853443053 0.6901203267 1.267785367 0.833341585 1.584901476]';
    case 'FOPID/table_sine'
        % fitted to the paper's Table 4, every metric within 10 % (tools/fit_table_gains.m):
        %   difference to the table: MSE -5.8 %, sum |tau| -6.1 %
        %   rms vs the published curves, joints 1-6 [rad]: 0.078 0.070 0.083 0.037 0.181 0.023 (mean 0.079)
        gains.Kp     = [394.5555484 0.003395151533 0.2309153858 68.93418034 30.13204845 24.15012276]';
        gains.Ki     = [150.938369 6609.911518 279.9029384 0.02807010791 27.56053449 104.7784735]';
        gains.Kd     = [37.61202758 62.21387153 333.4871648 8.431156516 0.4761258611 2.351305179]';
        gains.lambda = [1.556452311 0.2470736675 1.95 1.074198346 1.901656402 1.017116354]';
        gains.mu     = [1.184094273 1.002370887 0.8109766075 1.338965676 1.38206359 1.043985186]';
    otherwise
        error('controller_gains:name', 'unknown controller/experiment ''%s/%s''', controller, experiment);
end
end

% ------------------------------------------------------------------------
function file = tuned_file(controller)
%TUNED_FILE  Where TUNE_FOPID_HYBRID saves the gains of CONTROLLER.
file = fullfile(fileparts(mfilename('fullpath')), 'results', [lower(controller) '_gains.mat']);
end
