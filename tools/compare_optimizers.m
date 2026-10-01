function compare_optimizers(seeds, optimizers, costs)
%COMPARE_OPTIMIZERS  FBPA against FOPSO-GWO under the same cost, seeds and budget.
%
%   COMPARE_OPTIMIZERS()                       seeds 1-4, both optimisers, both costs
%   COMPARE_OPTIMIZERS(seeds, optimizers, costs)
%     seeds       random seeds, e.g. 1:4
%     optimizers  cell of 'FBPA', 'FOPSO-GWO'
%     costs       cell of
%                   'paper'  the paper's fitness: step ITAE (Eq. 29)
%                   'fbpa'   this work's cost: ITAE of both experiments and
%                            the five paper metrics scored against the
%                            paper's FBPA-FOPID (TUNE_FOPID_HYBRID, Target 'FBPA')
%
%   Every optimiser x cost x seed is one TUNE_FOPID_HYBRID run with the
%   paper's budget (30 particles x 100 iterations), the same search space,
%   the same seed (the identified FOPID) and, for the same random seed, the
%   same initial swarm.  Each result is saved to
%   results/optimizer_runs/<optimizer>_<cost>_seed<k>.mat; MAIN tabulates
%   them and plots the convergence.  The best run per optimiser under its
%   own cost (FBPA: 'paper', FOPSO-GWO: 'fbpa') is what MAIN shows as the
%   FBPA-FOPID and FOPSO-GWO controllers; copy it to results/fbpa_gains.mat
%   or results/fopso_gwo_gains.mat (the same file the stand-alone
%   TUNE_FOPID_HYBRID call with that RandomSeed writes).
%
%   Runtime with the compiled simulation: about 6 min per FOPSO-GWO run and
%   15 min per FBPA run (its antennae triple the evaluations).  The runs
%   are independent, so several Octave processes can share the work, e.g.
%   one per seed:  octave --eval "addpath tools; compare_optimizers(2)"
%
%   See also TUNE_FOPID_HYBRID, FBPA, HYBRID_FOPSO_GWO, MAIN.

if nargin < 1 || isempty(seeds),      seeds = 1:4; end
if nargin < 2 || isempty(optimizers), optimizers = {'FBPA', 'FOPSO-GWO'}; end
if nargin < 3 || isempty(costs),      costs = {'paper', 'fbpa'}; end

root = fileparts(fileparts(mfilename('fullpath')));
outdir = fullfile(root, 'results', 'optimizer_runs');
if ~exist(outdir, 'dir'), mkdir(outdir); end

for s = seeds
    for c = costs
        for o = optimizers
            stem = sprintf('%s_%s_seed%d', lower(strrep(o{1}, '-', '_')), c{1}, s);
            file = fullfile(outdir, [stem '.mat']);
            if exist(file, 'file')
                fprintf('%s exists, skipped\n', file);
                continue;
            end
            opts = struct('Optimizer', o{1}, 'RandomSeed', s, 'OutFile', file, ...
                          'Checkpoint', fullfile(outdir, [stem '_checkpoint.mat']));
            switch c{1}
                case 'paper', opts.Fitness = 'itae';
                case 'fbpa',  opts.Fitness = 'composite';  opts.Target = 'FBPA';
                otherwise, error('compare_optimizers:cost', 'unknown cost ''%s''', c{1});
            end
            fprintf('\n===== %s =====\n', stem);
            tune_fopid_hybrid(opts);
        end
    end
end
end
