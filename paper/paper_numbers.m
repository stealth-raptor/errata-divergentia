function paper_numbers()
%PAPER_NUMBERS  The numbers of paper/main.tex that no stored file holds.
%
%   PAPER_NUMBERS()   prints them; run from any folder.  Every other number
%   of the paper is in results/table_gains/summary.md (written by
%   main('table')) or in the stored tuning runs of results/optimizer_runs/
%   and results/ablation_runs/ (see paper/README.md).
%
%   Computed here:
%     * J_W of the start controller of every tuning run (Sect. 6.1);
%     * the sum of |tau| that perfect tracking of the sinusoid needs, by
%       inverse dynamics along the reference, on the 0.01 s grid (Sect. 6.4);
%     * the control effort of the best FOPSO-GWO run tuned for the step ITAE
%       alone, and the torque limits it exceeds (Table 9, last column),
%       next to that of FOPSO-GWO-FOPID for comparison.
%
%   See also FOPID_FITNESS, CONTROL_EFFORT.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
runs = fullfile(root, 'results', 'optimizer_runs');
P = robot_params();

tuned = load(fullfile(runs, 'fopso_gwo_whole_seed6.mat'));
J0 = fopid_fitness(P, controller_gains('FOPID'), tuned.reference, tuned.fitness);
fprintf('J_W of the start controller: %.4g\n', J0);

t = 0:0.01:5;                                % the logging grid of the metrics
tau = zeros(6, numel(t));
for k = 1:numel(t)
    q = sin(1.5 * t(k)) * ones(6, 1);
    [M, h] = robot_dynamics(P, q, 1.5 * cos(1.5 * t(k)) * ones(6, 1));
    tau(:, k) = M * (-2.25 * sin(1.5 * t(k)) * ones(6, 1)) + h;
end
fprintf('perfect tracking of the sinusoid: sum |tau| %.4g N m (first 0.5 s %.4g, after %.4g)\n', ...
        mean(sum(abs(tau), 2)), mean(sum(abs(tau(:, t < 0.5)), 2)), mean(sum(abs(tau(:, t >= 0.5)), 2)));

cap = tuned.fitness.whole.cap;
for name = {'fopso_gwo_whole_seed6', 'fopso_gwo_paper_seed7'}
    m = load(fullfile(runs, [name{1} '.mat']));
    step = simulate_closed_loop(P, m.gains, 'step', struct('log_dt', 1e-3));
    sine = simulate_closed_loop(P, m.gains, 'sine', struct('log_dt', 1e-3));
    es = control_effort(step, 'step');
    en = control_effort(sine, 'sine');
    fprintf('\n%s (stored cost %.4g)\n', name{1}, m.cost);
    fprintf('  kick peak %.4g, step peak after kick %.4g, sine peak %.4g N m\n', ...
            max(es.kick), max(es.peak), max(en.peak));
    fprintf('  step sum %.4g, sine sum %.4g, step TV %.4g, sine TV %.4g N m\n', ...
            mean(es.sum), mean(en.sum), mean(es.tv), mean(en.tv));
    fprintf('  torque limits exceeded: kick %d, step %d, sine %d of 6 each; worst ratio %.3g / %.3g / %.3g\n', ...
            sum(es.kick(:) > cap.kick(:)), sum(es.peak(:) > cap.step(:)), sum(en.peak(:) > cap.sine(:)), ...
            max(es.kick(:) ./ cap.kick(:)), max(es.peak(:) ./ cap.step(:)), max(en.peak(:) ./ cap.sine(:)));
end
end
