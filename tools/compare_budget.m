function compare_budget(seeds, optimizers, maxiter)
%COMPARE_BUDGET  PSO and FOPSO-GWO-CMA with the same, larger budget.
%
%   COMPARE_BUDGET(seeds, optimizers, maxiter)
%     seeds       random seeds, e.g. 1:8
%     optimizers  cell of 'PSO', 'FOPSO-GWO-CMA' (default both)
%     maxiter     iterations of 30 particles (default 300): a budget of
%                 30 * (maxiter + 1) cost evaluations, 9030 for 300, the
%                 budget of the paper's FBPA (three evaluations per particle
%                 and iteration, 30 x 100)
%
%   Both under the whole-controller cost, as COMPARE_OPTIMIZERS, with the
%   same seed controller and initial swarm for the same random seed.
%     PSO            runs maxiter iterations (its inertia schedule over all of
%                    them).
%     FOPSO-GWO-CMA  runs the swarm for the 100 iterations of FOPSO-GWO's own
%                    budget (SwarmFraction 100/maxiter), which repeats the
%                    FOPSO-GWO run of COMPARE_OPTIMIZERS with that seed
%                    evaluation for evaluation, and spends the rest on
%                    CMA-ES: hunts from the pack's three leaders, then the
%                    best one refined.
%   The swarm's 3030 evaluations do not find the basins' bottoms: refining
%   good controllers with CMA-ES for some 6000 evaluations lowers the cost
%   by up to half (the refinement probes in the README).
%
%   Results go to results/budget_runs/<optimizer>_whole_it<maxiter>_seed<k>.mat.
%
%   See also COMPARE_OPTIMIZERS, HYBRID_FOPSO_CMA, PSO.

if nargin < 2 || isempty(optimizers), optimizers = {'PSO', 'FOPSO-GWO-CMA'}; end
if nargin < 3 || isempty(maxiter), maxiter = 300; end
if ischar(optimizers), optimizers = {optimizers}; end
root = fileparts(fileparts(mfilename('fullpath')));
outdir = fullfile(root, 'results', 'budget_runs');
if ~exist(outdir, 'dir'), [~, ~] = mkdir(outdir); end   % no error if a parallel run made it

for s = seeds
    for o = optimizers
        stem = sprintf('%s_whole_it%d_seed%d', lower(strrep(o{1}, '-', '_')), maxiter, s);
        file = fullfile(outdir, [stem '.mat']);
        if exist(file, 'file')
            fprintf('%s exists, skipped\n', file);
            continue;
        end
        opts = struct('Optimizer', o{1}, 'Fitness', 'whole', 'RandomSeed', s, 'MaxIter', maxiter, ...
                      'OutFile', file, 'Checkpoint', fullfile(outdir, [stem '_checkpoint.mat']));
        if strcmp(o{1}, 'FOPSO-GWO-CMA')
            opts.SwarmFraction = 100 / maxiter;
        end
        fprintf('\n===== %s =====\n', stem);
        tune_fopid_hybrid(opts);
    end
end
end
