function ablate_optimizers(seeds, variants, cost)
%ABLATE_OPTIMIZERS  Why plain PSO beats FBPA and FOPSO-GWO: one change at a time.
%
%   ABLATE_OPTIMIZERS()                        seeds 1-4, all variants, cost 'fbpa'
%   ABLATE_OPTIMIZERS(seeds, variants, cost)
%
%   Each variant is one of the three optimisers with one setting changed,
%   run exactly like TOOLS/COMPARE_OPTIMIZERS (same budget, seed controller,
%   initial swarms and cost):
%     fopso_gwo_c2   FOPSO-GWO with c1 = c2 = c3 = 2 (PSO's and FBPA's), not 1
%     fopso_c2       FO-PSO alone: FOPSO-GWO without the grey-wolf term
%                    (c3 = 0), c1 = c2 = 2: the fractional velocity memory of
%                    Eq. 25 against PSO's plain inertia, nothing else
%     fbpa_v02       FBPA with |v| <= 0.2 of each range (as PSO and FOPSO-GWO)
%                    instead of the paper's |v| <= 1, the whole range here
%     pso_c1         PSO with c1 = c2 = 1 (FOPSO-GWO's), not 2
%   cost: 'paper', 'fbpa', 'fbpa_all' or 'whole', as in COMPARE_OPTIMIZERS.
%
%   Results go to results/ablation_runs/<variant>_<cost>_seed<k>.mat; MAIN
%   reports them next to the unmodified optimisers.
%
%   See also COMPARE_OPTIMIZERS, PSO, FBPA, HYBRID_FOPSO_GWO.

all_variants = {'fopso_gwo_c2', 'FOPSO-GWO', struct('c1', 2, 'c2', 2, 'c3', 2)
                'fopso_c2',     'FOPSO-GWO', struct('c1', 2, 'c2', 2, 'c3', 0)
                'fbpa_v02',     'FBPA',      struct('vmax', 0.2)
                'pso_c1',       'PSO',       struct('c1', 1, 'c2', 1)};
if nargin < 1 || isempty(seeds),    seeds = 1:4; end
if nargin < 2 || isempty(variants), variants = all_variants(:, 1)'; end
if nargin < 3 || isempty(cost),     cost = 'fbpa'; end
if ischar(variants), variants = {variants}; end

root = fileparts(fileparts(mfilename('fullpath')));
outdir = fullfile(root, 'results', 'ablation_runs');
if ~exist(outdir, 'dir'), [~, ~] = mkdir(outdir); end   % no error if a parallel run made it

for s = seeds
    for v = variants
        i = find(strcmp(all_variants(:, 1), v{1}), 1);
        if isempty(i), error('ablate_optimizers:variant', 'unknown variant ''%s''', v{1}); end
        stem = sprintf('%s_%s_seed%d', v{1}, cost, s);
        file = fullfile(outdir, [stem '.mat']);
        if exist(file, 'file')
            fprintf('%s exists, skipped\n', file);
            continue;
        end
        opts = all_variants{i, 3};
        opts.Optimizer = all_variants{i, 2};
        opts.RandomSeed = s;
        opts.OutFile = file;
        opts.Checkpoint = fullfile(outdir, [stem '_checkpoint.mat']);
        switch cost
            case 'paper',    opts.Fitness = 'itae';
            case 'fbpa',     opts.Fitness = 'composite';  opts.Target = 'FBPA';
            case 'fbpa_all', opts.Fitness = 'composite';  opts.Target = 'FBPA-all';
            case 'whole',    opts.Fitness = 'whole';
            otherwise, error('ablate_optimizers:cost', 'unknown cost ''%s''', cost);
        end
        fprintf('\n===== %s =====\n', stem);
        tune_fopid_hybrid(opts);
    end
end
end
