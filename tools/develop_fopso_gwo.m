function develop_fopso_gwo(seeds, candidates, cost)
%DEVELOP_FOPSO_GWO  The runs that chose FOPSO-GWO's settings, on development seeds.
%
%   DEVELOP_FOPSO_GWO()                          seeds 101-104, all candidates, cost 'fbpa'
%   DEVELOP_FOPSO_GWO(seeds, candidates, cost)
%
%   FOPSO-GWO's first settings (c1 = c2 = c3 = 1, fractional order
%   0.9 -> 0.4) lost to plain PSO on the tracking costs (TOOLS/ABLATE_OPTIMIZERS).
%   The candidates below were run on seeds 101-104, which the comparison
%   (TOOLS/COMPARE_OPTIMIZERS, seeds 1-8) never uses, exactly as in that
%   comparison (budget, seed controller, initial swarms, cost):
%
%     V1   the first settings: c = 1 / 1 / 1, order 0.9 -> 0.4 (Eq. 27)
%     A    c = 2 / 2 / 0.5, order held at 0.9
%     B    c = 2 / 2 / 1,   order 0.9
%     C    c = 2 / 2 / 0.5, order 0.9 -> 0.4
%     E    A with a_g falling linearly (gwo_power 1)
%     G    C with the order held at 0.9 for the first 60 % of the run
%     H    c = 2 / 2 / 1.5 with the grey-wolf pull growing from 0 (c3_ramp)
%     L    c = 1.5 / 1.5 / 1, order 0.9 (the final settings)
%     M    c = 1.75 / 1.75 / 0.5, order 0.9
%     N    c = 2 / 1 / 1, order 0.9 (PSO's cognitive pull kept)
%     R    L, starting as PSO (pack_ramp 1: the grey-wolf share grows over the run)
%     R2   L, starting as PSO, grey-wolf share complete at mid-run (pack_ramp 0.5)
%     S    L with half the swarm leaving the pack and pulled as in PSO (explorers 0.5)
%     H1   FOPSO-GWO-CMA (HYBRID_FOPSO_CMA): L for 60 % of the iterations,
%          then CMA-ES from the best point, shaped by the elite
%     H1I  H1 with an identity start covariance
%     H3   H1 with a short CMA-ES hunt from each of three leaders first
%     H4   H1 with the swarm for 40 % of the iterations
%     H5   the swarm cut short before it converges (FullSchedule), hunts
%          from three leaders, identity start covariance
%     H6   H3 with the swarm for 40 % of the iterations
%     PSO  plain PSO, the reference
%
%   First all of them under cost 'fbpa', then the best of those and three
%   designed against the paper's fitness (N, R, R2, S) under cost 'paper'.  The
%   rule fixed before the last round: take the candidate whose mean beats
%   PSO's under both costs by the largest margin on the weaker of the two.
%   That is L (paper -19 %, fbpa -9 %); MAIN reports the table (Sect. 4).
%
%   Results go to results/dev_runs/<candidate>_<cost>_seed<k>.mat.
%
%   See also HYBRID_FOPSO_GWO, COMPARE_OPTIMIZERS, ABLATE_OPTIMIZERS.

L = {'c1', 1.5, 'c2', 1.5, 'c3', 1, 'alpha_drop', 0};
all_candidates = { ...
    'V1',  'FOPSO-GWO', struct('c1', 1,    'c2', 1,    'c3', 1,   'alpha_drop', 0.5)
    'A',   'FOPSO-GWO', struct('c1', 2,    'c2', 2,    'c3', 0.5, 'alpha_drop', 0)
    'B',   'FOPSO-GWO', struct('c1', 2,    'c2', 2,    'c3', 1,   'alpha_drop', 0)
    'C',   'FOPSO-GWO', struct('c1', 2,    'c2', 2,    'c3', 0.5, 'alpha_drop', 0.5)
    'E',   'FOPSO-GWO', struct('c1', 2,    'c2', 2,    'c3', 0.5, 'alpha_drop', 0, 'gwo_power', 1)
    'G',   'FOPSO-GWO', struct('c1', 2,    'c2', 2,    'c3', 0.5, 'alpha_drop', 0.5, 'alpha_hold', 0.6)
    'H',   'FOPSO-GWO', struct('c1', 2,    'c2', 2,    'c3', 1.5, 'alpha_drop', 0, 'c3_ramp', true)
    'L',   'FOPSO-GWO', struct(L{:})
    'M',   'FOPSO-GWO', struct('c1', 1.75, 'c2', 1.75, 'c3', 0.5, 'alpha_drop', 0)
    'N',   'FOPSO-GWO', struct('c1', 2,    'c2', 1,    'c3', 1,   'alpha_drop', 0)
    'R',   'FOPSO-GWO', struct(L{:}, 'pack_ramp', 1)
    'R2',  'FOPSO-GWO', struct(L{:}, 'pack_ramp', 0.5)
    'S',   'FOPSO-GWO', struct(L{:}, 'explorers', 0.5)
    'H1',  'FOPSO-GWO-CMA', struct()
    'H1I', 'FOPSO-GWO-CMA', struct('ShapeFromElite', false)
    'H3',  'FOPSO-GWO-CMA', struct('Leaders', 3)
    'H4',  'FOPSO-GWO-CMA', struct('SwarmFraction', 0.4)
    'H5',  'FOPSO-GWO-CMA', struct('Leaders', 3, 'FullSchedule', true, 'ShapeFromElite', false)
    'H6',  'FOPSO-GWO-CMA', struct('Leaders', 3, 'SwarmFraction', 0.4)
    'PSO', 'PSO',       struct()};
if nargin < 1 || isempty(seeds),      seeds = 101:104; end
if nargin < 2 || isempty(candidates), candidates = all_candidates(:, 1)'; end
if nargin < 3 || isempty(cost),       cost = 'fbpa'; end
if ischar(candidates), candidates = {candidates}; end

root = fileparts(fileparts(mfilename('fullpath')));
outdir = fullfile(root, 'results', 'dev_runs');
if ~exist(outdir, 'dir'), [~, ~] = mkdir(outdir); end   % no error if a parallel run made it

for s = seeds
    for c = candidates
        i = find(strcmp(all_candidates(:, 1), c{1}), 1);
        if isempty(i), error('develop_fopso_gwo:candidate', 'unknown candidate ''%s''', c{1}); end
        stem = sprintf('%s_%s_seed%d', c{1}, cost, s);
        file = fullfile(outdir, [stem '.mat']);
        if exist(file, 'file')
            fprintf('%s exists, skipped\n', file);
            continue;
        end
        opts = all_candidates{i, 3};
        opts.Optimizer = all_candidates{i, 2};
        opts.RandomSeed = s;
        opts.OutFile = file;
        opts.Checkpoint = fullfile(outdir, [stem '_checkpoint.mat']);
        switch cost
            case 'paper',    opts.Fitness = 'itae';
            case 'fbpa',     opts.Fitness = 'composite';  opts.Target = 'FBPA';
            case 'fbpa_all', opts.Fitness = 'composite';  opts.Target = 'FBPA-all';
            case 'whole',    opts.Fitness = 'whole';
            otherwise, error('develop_fopso_gwo:cost', 'unknown cost ''%s''', cost);
        end
        fprintf('\n===== %s =====\n', stem);
        tune_fopid_hybrid(opts);
    end
end
end
