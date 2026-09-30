function gains = controller_gains(controller)
%CONTROLLER_GAINS  Tuned gains for the six joints.
%
%   gains = CONTROLLER_GAINS('PID')    classical PID  (lambda = mu = 1)
%   gains = CONTROLLER_GAINS('FOPID')  fractional-order PID
%   gains = CONTROLLER_GAINS('FOPSO_GWO')  FOPID tuned by the FO-PSO/GWO
%                                      hybrid, read from the file written by
%                                      TUNE_FOPID_HYBRID
%
%   Each field is 6x1, one entry per joint. The paper publishes no gains,
%   so these were obtained by fitting the simulation to the five published
%   performance figures (Table 3 step metrics and Table 4 sine metrics)
%   with a pattern-search optimiser. They reproduce the paper's baseline;
%   they are not claimed to be the authors' own gains.
%
%   See also FOPID_CONTROLLER, TUNE_FOPID_HYBRID.

switch upper(controller)
    case 'PID'
        gains.Kp     = [1906.83287409 16414.224263 200.5441424 6.3709646543 0.0001024 5.12e-05]';
        gains.Ki     = [37142.627424 116756.308154 1.06971964794 39.2432698709 0.0004096 0.364191623176]';
        gains.Kd     = [108.012158148 80.5156634327 25.0680178 2.5183449 110.84343339 0.64]';
        gains.lambda = [1 1 1 1 1 1]';
        gains.mu     = [1 1 1 1 1 1]';
    case 'FOPID'
        gains.Kp     = [2.77841720184 15531.2064014 189.755691059 3389.92448755 96.891300498 27.2429911607]';
        gains.Ki     = [507.116505583 0.0504098584914 0.109522940904 6187.2877089 0.0005592359375 8.8422965596]';
        gains.Kd     = [176.047189996 174.999508748 38.021255025 3.8196332254 1.94140625 0.970703125]';
        gains.lambda = [1.859375 1.859375 1.859375 1.859375 1.859375 1.859375]';
        gains.mu     = [1 1 1.0625 1 1.125 1]';
    case 'FOPSO_GWO'
        file = fopso_gwo_file();
        if ~exist(file, 'file')
            error('controller_gains:untuned', ...
                  'no tuned gains in %s; run tune_fopid_hybrid first', file);
        end
        S = load(file, 'gains');
        gains = S.gains;
    otherwise
        error('controller_gains:name', 'unknown controller ''%s''', controller);
end
end

% ------------------------------------------------------------------------
function file = fopso_gwo_file()
%FOPSO_GWO_FILE  Where TUNE_FOPID_HYBRID saves its gains.
file = fullfile(fileparts(mfilename('fullpath')), 'results', 'fopso_gwo_gains.mat');
end
