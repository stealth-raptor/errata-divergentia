function compare_weighted(seeds, optimizers, effort_weights, tag)
%COMPARE_WEIGHTED  PSO and FOPSO-GWO-CC under the whole cost with other effort weights.
%
%   COMPARE_WEIGHTED(seeds, optimizers, effort_weights, tag)
%     seeds           random seeds, e.g. 1:8
%     optimizers      cell of 'PSO', 'FOPSO-GWO', 'FOPSO-GWO-CC' (default PSO and
%                     FOPSO-GWO-CC)
%     effort_weights  weights of the whole cost's four effort terms (sum |tau|
%                     step, sine; total variation step, sine); default
%                     [1 3 1 1], the paper's torque metric (the sine sum) three
%                     times as heavy as in the whole cost
%     tag             folder name under results/ (default 'torque_runs')
%
%   Everything else as COMPARE_OPTIMIZERS with the whole cost: the same
%   budget, caps, seed controller and initial swarms for the same random
%   seed, so the optimisers are compared under one cost that asks more of
%   the torque.  Results go to results/<tag>/<optimizer>_whole_seed<k>.mat.
%
%   See also COMPARE_OPTIMIZERS, TUNE_FOPID_HYBRID (EffortWeights).

if nargin < 2 || isempty(optimizers), optimizers = {'PSO', 'FOPSO-GWO-CC'}; end
if nargin < 3 || isempty(effort_weights), effort_weights = [1 3 1 1]; end
if nargin < 4 || isempty(tag), tag = 'torque_runs'; end
if ischar(optimizers), optimizers = {optimizers}; end
root = fileparts(fileparts(mfilename('fullpath')));
outdir = fullfile(root, 'results', tag);
if ~exist(outdir, 'dir'), [~, ~] = mkdir(outdir); end   % no error if a parallel run made it

for s = seeds
    for o = optimizers
        stem = sprintf('%s_whole_seed%d', lower(strrep(o{1}, '-', '_')), s);
        file = fullfile(outdir, [stem '.mat']);
        if exist(file, 'file')
            fprintf('%s exists, skipped\n', file);
            continue;
        end
        opts = struct('Optimizer', o{1}, 'Fitness', 'whole', 'RandomSeed', s, ...
                      'EffortWeights', effort_weights, 'OutFile', file, ...
                      'Checkpoint', fullfile(outdir, [stem '_checkpoint.mat']));
        fprintf('\n===== %s (effort weights %s) =====\n', stem, mat2str(effort_weights));
        tune_fopid_hybrid(opts);
    end
end
end
