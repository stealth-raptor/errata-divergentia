"""Write controller_gains.m from identification results (identify_gains.py JSON).

Usage:
  python3 tools/write_controller_gains.py \\
      --shared PID_shared.json FOPID_shared.json FBPA_shared.json \\
      --step   PID_step.json   FOPID_step.json   FBPA_step.json \\
      --sine   PID_sine.json   FOPID_sine.json   FBPA_sine.json
"""
import argparse, json, os

HERE = os.path.dirname(os.path.abspath(__file__))


def vec(v):
    return '[' + ' '.join('%.10g' % x for x in v) + "]'"


def rms_line(R, kind):
    v = R['rms_' + kind]
    return f"        %   {kind} " + ' '.join('%.3f' % x for x in v) + '   (mean %.3f)' % (sum(v) / len(v))


def block(key, R, kinds):
    g = R['gains']
    lines = '\n'.join(rms_line(R, k) for k in kinds)
    return f"""    case '{key}'
        % rms vs the published curves, joints 1-6 [rad]:
{lines}
        gains.Kp     = {vec(g['Kp'])};
        gains.Ki     = {vec(g['Ki'])};
        gains.Kd     = {vec(g['Kd'])};
        gains.lambda = {vec(g['lambda'])};
        gains.mu     = {vec(g['mu'])};
"""


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    for k in ('shared', 'step', 'sine'):
        ap.add_argument('--' + k, nargs=3, required=True, metavar=('PID_JSON', 'FOPID_JSON', 'FBPA_JSON'))
    a = ap.parse_args()
    R = {k: [json.load(open(f)) for f in getattr(a, k)] for k in ('shared', 'step', 'sine')}
    meta = R['shared'][0]['meta']
    blocks = ''
    for c, name in enumerate(('PID', 'FOPID', 'FBPA')):
        blocks += block(f'{name}/shared', R['shared'][c], ('step', 'sine'))
        blocks += block(f'{name}/step', R['step'][c], ('step',))
        blocks += block(f'{name}/sine', R['sine'][c], ('sine',))
    src = f"""function gains = controller_gains(controller, experiment)
%CONTROLLER_GAINS  Gains of the six joint controllers, identified from the paper.
%
%   gains = CONTROLLER_GAINS(controller)              one gain set for both experiments
%   gains = CONTROLLER_GAINS(controller, experiment)  gain set for one experiment
%     controller  'PID' (lambda = mu = 1), 'FOPID', 'FBPA' (the FBPA-FOPID),
%                 'PSO' (the PSO-FOPID), 'FOPSO_GWO', 'FOPSO_GWO_CMA' or 'FOPSO_GWO_CC'
%     experiment  'step', 'sine', or 'shared' (the default)
%
%   Each field is 6x1, one entry per joint.  The paper publishes no gains.
%   All of them were identified from the paper's own published curves, read
%   from the PDF's vector graphics: the six joint trajectories of the step
%   experiment (Figs 6-11) and/or of the sine experiment (Figs 13-18) of
%   that controller were matched by least squares in the fully coupled
%   closed loop
%   (tools/identify_gains.py; plant: {meta['lengths']} lengths, COM {meta['com']}, g = {meta['g']:g}).
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
if any(strcmpi(controller, {{'FOPSO_GWO', 'FOPSO_GWO_CMA', 'FOPSO_GWO_CC', 'PSO'}}))
    file = tuned_file(controller);
    if ~exist(file, 'file')
        error('controller_gains:untuned', 'no tuned gains in %s; run tune_fopid_hybrid first', file);
    end
    S = load(file, 'gains');
    gains = S.gains;
    return;
end

switch [upper(controller) '/' lower(experiment)]
{blocks}    otherwise
        error('controller_gains:name', 'unknown controller/experiment ''%s/%s''', controller, experiment);
end
end

% ------------------------------------------------------------------------
function file = tuned_file(controller)
%TUNED_FILE  Where TUNE_FOPID_HYBRID saves the gains of CONTROLLER.
file = fullfile(fileparts(mfilename('fullpath')), 'results', [lower(controller) '_gains.mat']);
end
"""
    open(os.path.join(HERE, '..', 'controller_gains.m'), 'w').write(src)
    print('wrote controller_gains.m')
