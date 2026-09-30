"""Write controller_gains.m from identification results (identify_gains.py JSON).

Usage: python3 tools/write_controller_gains.py fit_PID.json fit_FOPID.json
"""
import json, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))


def vec(v):
    return '[' + ' '.join('%.10g' % x for x in v) + "]'"


def block(name, R):
    g = R['gains']
    rs = ' '.join('%.3f' % v for v in R['rms_step'])
    rn = ' '.join('%.3f' % v for v in R['rms_sine'])
    return f"""    case '{name}'
        % rms vs the published curves, joints 1-6 [rad]:
        %   step {rs}
        %   sine {rn}
        gains.Kp     = {vec(g['Kp'])};
        gains.Ki     = {vec(g['Ki'])};
        gains.Kd     = {vec(g['Kd'])};
        gains.lambda = {vec(g['lambda'])};
        gains.mu     = {vec(g['mu'])};
"""


if __name__ == '__main__':
    pid, fopid = (json.load(open(f)) for f in sys.argv[1:3])
    meta = pid['meta']
    src = f"""function gains = controller_gains(controller)
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
%   {meta['lengths']} lengths, COM {meta['com']}, g = {meta['g']:g}).
%
%   The joint-1 step and sine curves of the paper cannot both be matched by
%   any gain set (docs/audit_report.md, Sect. 4); these gains are the best
%   compromise over all 12 curves of each controller.
%
%   See also FOPID_CONTROLLER, ROBOT_PARAMS.

switch upper(controller)
{block('PID', pid)}{block('FOPID', fopid)}    otherwise
        error('controller_gains:name', 'unknown controller ''%s''', controller);
end
end
"""
    open(os.path.join(HERE, '..', 'controller_gains.m'), 'w').write(src)
    print('wrote controller_gains.m')
