function tuned = tune_fopid_hybrid(opts)
%TUNE_FOPID_HYBRID  Tune the six FOPID controllers with FOPSO-GWO or the paper's FBPA.
%
%   tuned = TUNE_FOPID_HYBRID()        FOPSO-GWO, paper budget: 30 particles x 100 iterations
%   tuned = TUNE_FOPID_HYBRID(opts)    override any option below
%
%   Searches all 30 FOPID parameters at once (Kp, Ki, Kd, lambda, mu for each
%   of the six joints, as in the paper), starting from the existing FOPID
%   gains, with
%     Optimizer = 'FOPSO-GWO'  (default) HYBRID_FOPSO_GWO, this work's
%                              optimiser -> results/fopso_gwo_gains.mat
%     Optimizer = 'FBPA'       FBPA, the paper's optimiser, with the paper's
%                              settings and, by default, its fitness (step
%                              ITAE, Eq. 29) -> results/fbpa_gains.mat
%   CONTROLLER_GAINS('FOPSO_GWO') and CONTROLLER_GAINS('FBPA') read those
%   files, and MAIN then adds the tuned controllers to the comparison and the
%   figures.  The two optimisers share the search space, the seed and, for
%   the same RandomSeed, the initial swarm.
%
%   Search space. The gains span eight orders of magnitude, so Kp, Ki and Kd
%   are searched on a log10 scale; lambda and mu linearly. Everything is
%   mapped onto the unit box for the optimiser.
%
%   Cost. FOPID_FITNESS, normalised by the existing FOPID so that the
%   baseline scores exactly 1.  opts.Fitness selects
%     'composite'  (the default for FOPSO-GWO) ITAE of both experiments plus
%                  the five paper metrics, with an extra penalty for any
%                  metric worse than FOPID
%     'torque'     as 'composite', but sine torque weighted 3x and the step
%                  peak torque (the spike at the step instant) added with
%                  weight 2, both also penalised if worse than the baseline
%     'itae'       the paper's own fitness: step ITAE only (Eq. 29); the
%                  default for FBPA.  Candidates must still complete the
%                  sine run, since the paper uses the same gains for it
%
%   The paper's FBPA-FOPID, reproduced with the paper's optimiser and fitness:
%     tune_fopid_hybrid(struct('Optimizer', 'FBPA'))
%
%   Target.  By default (Target = 'baseline') every metric is scored
%   relative to the FOPID baseline.  With Target = 'FBPA' the five paper
%   metrics are scored relative to the paper's FBPA-FOPID (Table 3/4:
%   overshoot 22.1 %, adjustment time 1.43 s, peak time 1.09 s, MSE 3.7e-3,
%   torque 2.3154e4), so the regret term penalises every metric that is not
%   yet better than FBPA's, and the adjustment and peak times are compared
%   after the step instant (t - 1 s; from t = 0 they all start at 1 s).  The
%   ITAE terms stay relative to the FOPID baseline.  The reported metrics are
%   unchanged.  Aiming to beat FBPA on every metric:
%     tune_fopid_hybrid(struct('Target', 'FBPA'))
%   Target = 'FBPA-all' scores against every FBPA-FOPID available: for each
%   paper metric, the better of the paper's value and that of the FBPA re-run
%   on this arm (results/fbpa_gains.mat, from Optimizer = 'FBPA'), so the
%   result must beat both to score below 1 on it.
%
%   Reducing torque from an already tuned controller, comparing against the
%   original FOPID but starting from the FO-PSO/GWO gains:
%     tune_fopid_hybrid(struct('Fitness', 'torque', 'Start', {{'FOPSO_GWO'}}, ...
%                              'PopSize', 12, 'MaxIter', 10))
%
%   Runtime. One cost evaluation is two full 5 s simulations; the first
%   iteration prints an ETA.  With the paper's budget (3000 evaluations;
%   FBPA's antennae triple that) and the compiled simulation (BUILD_MEX)
%   FOPSO-GWO takes about 6 min and FBPA about 15 min; without it this is a
%   long, overnight-scale run, so:
%     * opts.UseParallel = true evaluates the swarm with PARFOR (Parallel
%       Computing Toolbox; on by default when the toolbox is installed);
%     * a checkpoint is written after every iteration, and an interrupted
%       run resumes from it when called again with the same options (a
%       checkpoint from a run with other options is ignored with a warning)
%       (delete results/<optimizer>_checkpoint.mat to start afresh; it is
%       removed automatically once a run completes);
%     * unstable candidates are aborted as soon as an error passes 5 rad;
%     * a quick trial, e.g. tune_fopid_hybrid(struct('PopSize', 12,
%       'MaxIter', 15)), already improves on the baseline because the
%       baseline is in the swarm.
%
%   Options (besides those of HYBRID_FOPSO_GWO or FBPA, which are passed through):
%     Optimizer    'FOPSO-GWO' (default) or 'FBPA'
%     Fitness      'composite', 'torque' or 'itae'; default 'composite' for
%                  FOPSO-GWO and 'itae' (the paper's) for FBPA
%     Weights      1x8 weights of FOPID_FITNESS (overrides Fitness)
%     Target       'baseline' (default), 'FBPA' or 'FBPA-all', see above
%     Robust       penalise numerically ill-conditioned closed loops (see
%                  FOPID_FITNESS); default true when SIMULATE_MEX is built,
%                  since it doubles the cost of a candidate
%     Regret       extra weight on metrics worse than the reference (default
%                  1, or 2 with Target = 'FBPA' or 'FBPA-all')
%     Baseline     CONTROLLER_GAINS name the cost is normalised by (it
%                  scores 1), default 'FOPID'
%     Start        cell array of CONTROLLER_GAINS names put into the initial
%                  swarm, default {Baseline}
%     Bounds       struct with 1x5 fields lo, hi on [log10 Kp, log10 Ki,
%                  log10 Kd, lambda, mu], applied to every joint; the
%                  default [-2 -4 -1 0.05 0.05] ... [5 5 3 1.95 1.95] contains
%                  every gain of the identified FOPID, so the seed enters the
%                  swarm unclipped
%     OutFile      where the tuned gains are saved, default
%                  results/<optimizer>_gains.mat; an existing file is first
%                  copied to <name>_prev.mat
%     Checkpoint   default results/<optimizer>_checkpoint.mat
%
%   See also HYBRID_FOPSO_GWO, FBPA, FOPID_FITNESS, CONTROLLER_GAINS, MAIN.

here = fileparts(mfilename('fullpath'));
cd(here);
addpath(here);                       % so PARFOR workers find the model files
if ~exist('results', 'dir'), mkdir('results'); end

if nargin < 1, opts = struct(); end
user_regret = isfield(opts, 'Regret');
opts = set_defaults(opts);

P    = robot_params();
base = controller_gains(opts.Baseline);
B    = search_space(opts.Bounds);

switch lower(opts.Fitness)
    case 'composite', fit.weights = [1 1 1 1 0.5 1 1 0];  fit.regret = opts.Regret;
    case 'torque',    fit.weights = [1 1 1 1 0.5 1 3 2];  fit.regret = opts.Regret;
    case 'itae',      fit.weights = [1 0 0 0 0 0 0 0];    fit.regret = 0;
    otherwise, error('tune_fopid_hybrid:fitness', 'unknown fitness ''%s''', opts.Fitness);
end
fit.abort_err = 5;
fit.time_offset = 0;
fit.robust = opts.Robust;
switch upper(opts.Target)
    case 'BASELINE'
    case {'FBPA', 'FBPA-ALL'}
        fit.time_offset = 1;                     % the step instant
        if strcmpi(opts.Fitness, 'composite'), fit.weights = [0.5 0.5 1 1 1 1 1 0]; end
        if ~user_regret, fit.regret = 2; end
    otherwise
        error('tune_fopid_hybrid:target', 'unknown target ''%s''', opts.Target);
end
if ~isempty(opts.Weights), fit.weights = opts.Weights; end
if numel(fit.weights) == 7, fit.weights = [fit.weights 0]; end

fprintf('Evaluating the %s baseline ...\n', opts.Baseline);
[~, base_raw] = fopid_fitness(P, base, [], fit);
if any(~isfinite(base_raw))
    error('tune_fopid_hybrid:baseline', 'the %s baseline does not complete both runs', opts.Baseline);
end
print_metrics(opts.Baseline, base_raw);

ref = base_raw;                                  % what the cost is normalised by
ref_name = opts.Baseline;
paper_fbpa = [22.1 1.43 1.09 3.7e-3 2.3154e4];   % paper Tables 3 and 4, FBPA-FOPID
switch upper(opts.Target)
    case 'FBPA'
        ref(3:7) = paper_fbpa;
        ref_name = 'FBPA target';
    case 'FBPA-ALL'
        file = fullfile('results', 'fbpa_gains.mat');
        if ~exist(file, 'file')
            error('tune_fopid_hybrid:target', ...
                  'Target ''FBPA-all'' needs %s; run tune_fopid_hybrid(struct(''Optimizer'', ''FBPA'')) first', file);
        end
        F = load(file, 'metrics');
        ref(3:7) = min(paper_fbpa, F.metrics(3:7));
        ref_name = 'FBPA target';
end

cost = @(z) fopid_fitness(P, decode(z, B), ref, fit);

if isempty(opts.Start), opts.Start = {opts.Baseline}; end
if ischar(opts.Start), opts.Start = {opts.Start}; end
z_seed = zeros(numel(opts.Start), B.n);
for i = 1:numel(opts.Start)
    z_seed(i, :) = encode(controller_gains(opts.Start{i}), B);
    if any(z_seed(i, :) < 0 | z_seed(i, :) > 1)
        warning('tune_fopid_hybrid:seed', ...
                'the %s gains lie partly outside the search bounds; the seed is clipped', opts.Start{i});
    end
end

opt_opts = rmfield(opts, {'Optimizer', 'Fitness', 'Weights', 'Regret', 'Baseline', 'Start', ...
                          'Bounds', 'OutFile', 'Target', 'Robust'});
opt_opts.Seeds = min(max(z_seed, 0), 1);
% a checkpoint is only resumed if it was written for the same cost function
opt_opts.CheckpointTag = {opts.Optimizer, fit.weights, fit.regret, fit.time_offset, fit.robust, ...
                          B.lo, B.hi, ref, opts.Start};

fprintf('%s: %d particles x %d iterations, 30 parameters, fitness ''%s'', start %s\n', ...
        opts.Optimizer, opt_opts.PopSize, opt_opts.MaxIter, opts.Fitness, strjoin(opts.Start, ' + '));
if strcmpi(opts.Optimizer, 'FBPA')
    [z_best, info] = fbpa(cost, B.n, opt_opts);
else
    [z_best, info] = hybrid_fopso_gwo(cost, B.n, opt_opts);
end

gains = decode(z_best, B);
[J, raw] = fopid_fitness(P, gains, ref, fit);
fprintf('\nBest cost %.4f (%s = 1)\n', J, ref_name);
print_comparison(opts.Optimizer, opts.Baseline, base_raw, raw);
if ~strcmpi(opts.Target, 'baseline')
    print_comparison(opts.Optimizer, 'FBPA target', ref, raw);
end

tuned.optimizer = opts.Optimizer;
tuned.fitness_name = opts.Fitness;
tuned.gains    = gains;
tuned.cost     = J;
tuned.metrics  = raw;
tuned.baseline = base_raw;
tuned.reference = ref;
tuned.target   = opts.Target;
tuned.info     = info;
tuned.fitness  = fit;
tuned.bounds   = B;
if exist(opts.OutFile, 'file')
    [d, n, e] = fileparts(opts.OutFile);
    prev = fullfile(d, [n '_prev' e]);
    copyfile(opts.OutFile, prev);
    fprintf('Previous gains kept in %s\n', prev);
end
save_mat(opts.OutFile, tuned);
if ~isempty(opt_opts.Checkpoint) && exist(opt_opts.Checkpoint, 'file')
    delete(opt_opts.Checkpoint);     % finished: the next call starts afresh
end
fprintf('\nSaved the tuned gains to %s; run MAIN to compare and plot.\n', opts.OutFile);
end

% ------------------------------------------------------------------------
function B = search_space(bounds)
%SEARCH_SPACE  Per-parameter bounds of the 30-dimensional search.
%   Layout of the parameter vector: [Kp(1:6) Ki(1:6) Kd(1:6) lambda(1:6) mu(1:6)],
%   the first three groups as log10.
lo = kron(bounds.lo(:)', ones(1, 6));
hi = kron(bounds.hi(:)', ones(1, 6));
B.lo = lo;
B.hi = hi;
B.is_log = [true(1, 18) false(1, 12)];
B.n = 30;
end

function z = encode(g, B)
x = [g.Kp(:); g.Ki(:); g.Kd(:); g.lambda(:); g.mu(:)]';
x(B.is_log) = log10(max(x(B.is_log), realmin));
z = (x - B.lo) ./ (B.hi - B.lo);
end

function g = decode(z, B)
x = B.lo + z .* (B.hi - B.lo);
x(B.is_log) = 10 .^ x(B.is_log);
g.Kp     = x(1:6)';
g.Ki     = x(7:12)';
g.Kd     = x(13:18)';
g.lambda = x(19:24)';
g.mu     = x(25:30)';
end

% ------------------------------------------------------------------------
function print_metrics(name, raw)
fprintf(['  %s: ITAE step %.4g, ITAE sine %.4g, overshoot %.1f %%, adjustment %.2f s, ' ...
         'peak %.2f s, MSE %.3e, torque %.4g, step peak torque %.4g\n'], name, raw);
end

function print_comparison(optimizer, name, ref, raw)
labels = {'ITAE step', 'ITAE sine', 'Step overshoot (%)', 'Step adjustment time (s)', ...
          'Step peak time (s)', 'Sine MSE (rad^2)', 'Sine torque (Nm)', ...
          'Step peak torque (Nm)'};
fprintf('\n%-26s %12s %12s %9s\n', 'Metric', name, optimizer, 'change');
for k = 1:8
    fprintf('%-26s %12.4g %12.4g %+8.1f%%\n', labels{k}, ref(k), raw(k), 100 * (raw(k) / ref(k) - 1));
end
end

% ------------------------------------------------------------------------
function opts = set_defaults(opts)
% PARFOR needs MATLAB's Parallel Computing Toolbox; Octave runs it serially
has_pct = ~exist('OCTAVE_VERSION', 'builtin') && ~isempty(ver('parallel')) ...
          && license('test', 'Distrib_Computing_Toolbox');
if ~isfield(opts, 'Optimizer'), opts.Optimizer = 'FOPSO-GWO'; end
switch upper(strrep(opts.Optimizer, '_', '-'))
    case 'FOPSO-GWO', opts.Optimizer = 'FOPSO-GWO';  fitness = 'composite';
    case 'FBPA',      opts.Optimizer = 'FBPA';       fitness = 'itae';
    otherwise, error('tune_fopid_hybrid:optimizer', 'unknown optimizer ''%s''', opts.Optimizer);
end
stem = lower(strrep(opts.Optimizer, '-', '_'));
d = struct('PopSize', 30, 'MaxIter', 100, 'Fitness', fitness, 'Weights', [], ...
           'Regret', 1, 'Baseline', 'FOPID', 'Start', {{}}, 'Target', 'baseline', ...
           'Robust', exist('simulate_mex') == 3, ...                       %#ok<EXIST>
           'Bounds', struct('lo', [-2 -4 -1 0.05 0.05], 'hi', [5 5 3 1.95 1.95]), ...
           'UseParallel', has_pct, ...
           'Checkpoint', fullfile('results', [stem '_checkpoint.mat']), ...
           'Resume', true, ...
           'OutFile', fullfile('results', [stem '_gains.mat']));
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
end

% ------------------------------------------------------------------------
function save_mat(file, S)
%SAVE_MAT  Save the fields of S as variables, in MAT format under Octave too.
if exist('OCTAVE_VERSION', 'builtin')
    save('-mat7-binary', file, '-struct', 'S');
else
    save(file, '-struct', 'S');
end
end
