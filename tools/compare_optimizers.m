function compare_optimizers(seeds, optimizers, costs)
%COMPARE_OPTIMIZERS  FBPA against FOPSO-GWO under the same cost, seeds and budget.
%
%   COMPARE_OPTIMIZERS()                       seeds 1-4, both optimisers, all costs
%   COMPARE_OPTIMIZERS(seeds, optimizers, costs)
%     seeds       random seeds, e.g. 1:4
%     optimizers  cell of 'FBPA', 'FOPSO-GWO'
%     costs       cell of
%                   'paper'     the paper's fitness: step ITAE (Eq. 29)
%                   'fbpa'      this work's cost: ITAE of both experiments and
%                               the five paper metrics scored against the
%                               paper's FBPA-FOPID (TUNE_FOPID_HYBRID,
%                               Target 'FBPA')
%                   'fbpa_all'  the same, scored against the better of the
%                               paper's FBPA-FOPID and the FBPA re-run in
%                               results/fbpa_gains.mat, metric by metric
%                               (Target 'FBPA-all'; needs that file, and is
%                               left out of the default costs without it)
%
%   Every optimiser x cost x seed is one TUNE_FOPID_HYBRID run with the
%   paper's budget (30 particles x 100 iterations), the same search space,
%   the same seed (the identified FOPID) and, for the same random seed, the
%   same initial swarm.  Each result is saved to
%   results/optimizer_runs/<optimizer>_<cost>_seed<k>.mat; MAIN tabulates
%   them and plots the convergence.  MAIN shows as FBPA-FOPID and FOPSO-GWO
%   whatever is in results/fbpa_gains.mat and results/fopso_gwo_gains.mat;
%   in this repository those are copies of the best FBPA run under the
%   paper's fitness and the best FOPSO-GWO run under 'fbpa_all' (the same
%   files the stand-alone TUNE_FOPID_HYBRID call with that RandomSeed
%   writes).  Run the 'fbpa_all' cost after results/fbpa_gains.mat is in
%   place.
%
%   Runtime with the compiled simulation: about 6 min per FOPSO-GWO run and
%   10 min per FBPA run (its antennae triple the evaluations).  The runs
%   are independent, so several Octave processes can share the work, e.g.
%   one per seed:  octave --eval "addpath tools; compare_optimizers(2)"
%
%   See also TUNE_FOPID_HYBRID, FBPA, HYBRID_FOPSO_GWO, MAIN.

if nargin < 1 || isempty(seeds),      seeds = 1:4; end
if nargin < 2 || isempty(optimizers), optimizers = {'FBPA', 'FOPSO-GWO'}; end
if nargin < 3 || isempty(costs)
    costs = {'paper', 'fbpa'};
    if exist(fullfile('results', 'fbpa_gains.mat'), 'file'), costs{end+1} = 'fbpa_all'; end
end

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
                case 'paper',    opts.Fitness = 'itae';
                case 'fbpa',     opts.Fitness = 'composite';  opts.Target = 'FBPA';
                case 'fbpa_all', opts.Fitness = 'composite';  opts.Target = 'FBPA-all';
                otherwise, error('compare_optimizers:cost', 'unknown cost ''%s''', c{1});
            end
            fprintf('\n===== %s =====\n', stem);
            tune_fopid_hybrid(opts);
        end
    end
end
end
