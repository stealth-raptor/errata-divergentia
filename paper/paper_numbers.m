function paper_numbers()
%PAPER_NUMBERS  The number of paper/main.tex that no stored file holds.
%
%   PAPER_NUMBERS()   prints the fitness of the initial FOPID controller of
%   every tuning run (Sect. 4.2 of the paper: 35.4).  Every other number of
%   the paper is in results/table_gains/summary.md (written by main('table'))
%   or in the stored tuning runs of results/optimizer_runs/ (see
%   paper/README.md).  Run from any folder.
%
%   See also FOPID_FITNESS.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
tuned = load(fullfile(root, 'results', 'optimizer_runs', 'fopso_gwo_whole_seed6.mat'));
J0 = fopid_fitness(robot_params(), controller_gains('FOPID'), tuned.reference, tuned.fitness);
fprintf('fitness of the initial FOPID controller: %.4g\n', J0);
fprintf('fitness of the reported controller:      %.4g (stored %.4g)\n', ...
        fopid_fitness(robot_params(), tuned.gains, tuned.reference, tuned.fitness), tuned.cost);
end
