function tuned = tune_fopid_hybrid(opts)
%TUNE_FOPID_HYBRID  Tune the six FOPID controllers with the FO-PSO/GWO hybrid.
%
%   tuned = TUNE_FOPID_HYBRID()        paper budget: 30 particles x 100 iterations
%   tuned = TUNE_FOPID_HYBRID(opts)    override any option below
%
%   Searches all 30 FOPID parameters at once (Kp, Ki, Kd, lambda, mu for each
%   of the six joints, as in the paper) with HYBRID_FOPSO_GWO, starting from
%   the existing FOPID gains, and saves the best gain set to
%   results/fopso_gwo_gains.mat. CONTROLLER_GAINS('FOPSO_GWO') reads that
%   file, and MAIN then adds the tuned controller to the comparison and the
%   figures.
%
%   Search space. The gains span eight orders of magnitude, so Kp, Ki and Kd
%   are searched on a log10 scale; lambda and mu linearly. Everything is
%   mapped onto the unit box for the optimiser.
%
%   Cost. FOPID_FITNESS, normalised by the existing FOPID so that the
%   baseline scores exactly 1.  opts.Fitness selects
%     'composite'  (default) ITAE of both experiments plus the five paper
%                  metrics, with an extra penalty for any metric worse than
%                  FOPID
%     'itae'       the paper's own fitness: step ITAE only (Eq. 29)
%
%   Runtime. One cost evaluation is two full 5 s simulations; the first
%   iteration prints an ETA. With the paper's budget (3000 evaluations) this
%   is a long, overnight-scale run, so:
%     * opts.UseParallel = true evaluates the swarm with PARFOR (Parallel
%       Computing Toolbox; on by default when the toolbox is installed);
%     * a checkpoint is written after every iteration, and an interrupted
%       run resumes from it when called again with the same options (a
%       checkpoint from a run with other options is ignored with a warning)
%       (delete results/fopso_gwo_checkpoint.mat to start afresh; it is
%       removed automatically once a run completes);
%     * unstable candidates are aborted as soon as an error passes 5 rad;
%     * a quick trial, e.g. tune_fopid_hybrid(struct('PopSize', 12,
%       'MaxIter', 15)), already improves on the baseline because the
%       baseline is in the swarm.
%
%   Options (besides those of HYBRID_FOPSO_GWO, which are passed through):
%     Fitness      'composite' or 'itae'
%     Weights      1x7 weights of FOPID_FITNESS (overrides Fitness)
%     Regret       extra weight on metrics worse than FOPID (composite only)
%     Baseline     name for CONTROLLER_GAINS to start from, default 'FOPID'
%     Bounds       struct with 1x5 fields lo, hi on [log10 Kp, log10 Ki,
%                  log10 Kd, lambda, mu], applied to every joint
%     OutFile      where the tuned gains are saved
%
%   See also HYBRID_FOPSO_GWO, FOPID_FITNESS, CONTROLLER_GAINS, MAIN.

here = fileparts(mfilename('fullpath'));
cd(here);
addpath(here);                       % so PARFOR workers find the model files
if ~exist('results', 'dir'), mkdir('results'); end

if nargin < 1, opts = struct(); end
opts = set_defaults(opts);

P    = robot_params();
base = controller_gains(opts.Baseline);
B    = search_space(opts.Bounds);

switch lower(opts.Fitness)
    case 'composite', fit.weights = [1 1 1 1 0.5 1 1];  fit.regret = opts.Regret;
    case 'itae',      fit.weights = [1 0 0 0 0 0 0];    fit.regret = 0;
    otherwise, error('tune_fopid_hybrid:fitness', 'unknown fitness ''%s''', opts.Fitness);
end
if ~isempty(opts.Weights), fit.weights = opts.Weights; end
fit.abort_err = 5;

fprintf('Evaluating the %s baseline ...\n', opts.Baseline);
[~, ref] = fopid_fitness(P, base, [], fit);
if any(~isfinite(ref))
    error('tune_fopid_hybrid:baseline', 'the %s baseline does not complete both runs', opts.Baseline);
end
print_metrics(opts.Baseline, ref);

cost = @(z) fopid_fitness(P, decode(z, B), ref, fit);

z_seed = encode(base, B);
if any(z_seed < 0 | z_seed > 1)
    warning('tune_fopid_hybrid:seed', ...
            'the %s gains lie partly outside the search bounds; the seed is clipped', opts.Baseline);
end

opt_opts = rmfield(opts, {'Fitness', 'Weights', 'Regret', 'Baseline', 'Bounds', 'OutFile'});
opt_opts.Seeds = min(max(z_seed, 0), 1);
% a checkpoint is only resumed if it was written for the same cost function
opt_opts.CheckpointTag = {fit.weights, fit.regret, B.lo, B.hi, ref};

fprintf('FO-PSO/GWO: %d particles x %d iterations, 30 parameters, fitness ''%s''\n', ...
        opt_opts.PopSize, opt_opts.MaxIter, opts.Fitness);
[z_best, info] = hybrid_fopso_gwo(cost, B.n, opt_opts);

gains = decode(z_best, B);
[J, raw] = fopid_fitness(P, gains, ref, fit);
fprintf('\nBest cost %.4f (%s = 1)\n', J, opts.Baseline);
print_comparison(opts.Baseline, ref, raw);

tuned.gains    = gains;
tuned.cost     = J;
tuned.metrics  = raw;
tuned.baseline = ref;
tuned.info     = info;
tuned.fitness  = fit;
tuned.bounds   = B;
save(opts.OutFile, '-struct', 'tuned');
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
         'peak %.2f s, MSE %.3e, torque %.4g\n'], name, raw);
end

function print_comparison(name, ref, raw)
labels = {'ITAE step', 'ITAE sine', 'Step overshoot (%)', 'Step adjustment time (s)', ...
          'Step peak time (s)', 'Sine MSE (rad^2)', 'Sine torque (Nm)'};
fprintf('\n%-26s %12s %12s %9s\n', 'Metric', name, 'FOPSO-GWO', 'change');
for k = 1:7
    fprintf('%-26s %12.4g %12.4g %+8.1f%%\n', labels{k}, ref(k), raw(k), 100 * (raw(k) / ref(k) - 1));
end
end

% ------------------------------------------------------------------------
function opts = set_defaults(opts)
has_pct = ~isempty(ver('parallel')) && license('test', 'Distrib_Computing_Toolbox');
d = struct('PopSize', 30, 'MaxIter', 100, 'Fitness', 'composite', 'Weights', [], ...
           'Regret', 1, 'Baseline', 'FOPID', ...
           'Bounds', struct('lo', [-1 -4 -1 0.5 0.5], 'hi', [5 5 3 1.95 1.6]), ...
           'UseParallel', has_pct, ...
           'Checkpoint', fullfile('results', 'fopso_gwo_checkpoint.mat'), ...
           'Resume', true, ...
           'OutFile', fullfile('results', 'fopso_gwo_gains.mat'));
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
end
