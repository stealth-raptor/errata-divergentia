function plot_paper_figures(results, outdir)
%PLOT_PAPER_FIGURES  Reproduce the paper's figures for PID, FOPID and FOPSO-GWO.
%
%   PLOT_PAPER_FIGURES(results, outdir)  with results from MAIN.
%
%   The layout follows the source paper:
%
%     Figs 6-11   step response, one figure per joint, two panels:
%                 (a) position tracking trajectory   y label "jointN /rad"
%                 (b) tracking error                 y label "jointN err/rad"
%     Fig 12      driving torque with step response, 3x2 grid, "tauN / Nm"
%     Figs 13-18  sine response, one figure per joint, same two panels
%     Fig 19      driving torque with sine response, 3x2 grid, "jointN tau/N*m"
%
%   Colours are the paper's: reference ("dir") red, PID green, FOPID blue;
%   in the error panels of the sine figures and in Fig 19 the paper draws
%   FOPID in red, which is kept.  All signals are plotted at the paper's
%   0.01 s logging interval, so the step reference rises between t = 0.99 s
%   and 1.00 s exactly as in the published figures.
%
%   The paper's figures also carry an FBPA-FOPID curve.  In its place, when
%   results has a FOPSO_GWO entry (the FOPID re-tuned by TUNE_FOPID_HYBRID),
%   that controller is drawn as a third, magenta curve in Figs 6-19.
%
%   In addition, compare_step.png and compare_sine.png overlay every
%   published curve (thick black, read from the paper's vector graphics)
%   and the reproduction (thin colour), joint by joint, with the rms
%   difference in each title.
%
%   Works in Octave (gnuplot or Qt toolkit) and MATLAB.
%
%   See also MAIN, PAPER_CURVES.

if nargin < 2, outdir = 'results'; end
if ~exist(outdir, 'dir'), mkdir(outdir); end

RED     = [1 0 0];
GREEN   = [0 1 0];
BLUE    = [0 0 1];
MAGENTA = [0.85 0 0.85];

has_hybrid = isfield(results, 'FOPSO_GWO');
names = {'PID', 'FOPID'};
if has_hybrid, names{end+1} = 'FOPSO-GWO'; end

% ---- Figs 6-11 (step) and 13-18 (sine): per joint, two panels ------------
experiments = {'step', 6; 'sine', 13};
for ex = 1:size(experiments, 1)
    name  = experiments{ex, 1};
    first = experiments{ex, 2};
    run_pid   = results.PID.(name);
    run_fopid = results.FOPID.(name);
    fopid_err = BLUE;
    if strcmp(name, 'sine'), fopid_err = RED; end      % as printed in the paper
    for j = 1:6
        fig = figure('Visible', 'off', 'Position', [100 100 1100 380], 'Color', 'w');

        subplot(1, 2, 1); hold on; grid on; box on;
        plot(run_pid.t,   run_pid.r(j, :),   'Color', RED,   'LineWidth', 1);
        plot(run_pid.t,   run_pid.q(j, :),   'Color', GREEN, 'LineWidth', 1);
        plot(run_fopid.t, run_fopid.q(j, :), 'Color', BLUE,  'LineWidth', 1);
        if has_hybrid
            run_h = results.FOPSO_GWO.(name);
            plot(run_h.t, run_h.q(j, :), 'Color', MAGENTA, 'LineWidth', 1);
        end
        style_axis(sprintf('joint%d /rad', j));
        legend([{'dir'}, names], 'Location', 'northeast');
        xlabel('time / s');
        title('(a) position tracking trajectory');

        subplot(1, 2, 2); hold on; grid on; box on;
        plot(run_pid.t,   run_pid.r(j, :)   - run_pid.q(j, :),   'Color', GREEN,     'LineWidth', 1);
        plot(run_fopid.t, run_fopid.r(j, :) - run_fopid.q(j, :), 'Color', fopid_err, 'LineWidth', 1);
        if has_hybrid
            plot(run_h.t, run_h.r(j, :) - run_h.q(j, :), 'Color', MAGENTA, 'LineWidth', 1);
        end
        style_axis(sprintf('joint%d err/rad', j));
        legend(names, 'Location', 'northeast');
        xlabel('time / s');
        title('(b) tracking error');

        save_png(fig, fullfile(outdir, sprintf('fig%02d_joint%d_%s.png', first + j - 1, j, name)));
    end
end

% ---- Fig 12 (step torque) and Fig 19 (sine torque): 3x2 grids ------------
torques = {'step', 12, 'tau%d / Nm', BLUE
           'sine', 19, 'joint%d tau/N*m', RED};
for tq = 1:size(torques, 1)
    name = torques{tq, 1};
    fig = figure('Visible', 'off', 'Position', [100 100 1100 900], 'Color', 'w');
    for j = 1:6
        subplot(3, 2, j); hold on; grid on; box on;
        plot(results.PID.(name).t,   results.PID.(name).u(j, :),   'Color', GREEN,          'LineWidth', 1);
        plot(results.FOPID.(name).t, results.FOPID.(name).u(j, :), 'Color', torques{tq, 4}, 'LineWidth', 1);
        if has_hybrid
            plot(results.FOPSO_GWO.(name).t, results.FOPSO_GWO.(name).u(j, :), 'Color', MAGENTA, 'LineWidth', 1);
        end
        style_axis(sprintf(torques{tq, 3}, j));
        xlabel('time / s');
        legend(names, 'Location', 'northeast');
    end
    save_png(fig, fullfile(outdir, sprintf('fig%02d_%s_torque.png', torques{tq, 2}, name)));
end

% ---- published curve vs reproduction, joint by joint ---------------------
for ex = {'step', 'sine'}
    name = ex{1};
    fig = figure('Visible', 'off', 'Position', [50 50 1200 1500], 'Color', 'w');
    ctrl = {'PID', GREEN; 'FOPID', BLUE};
    for c = 1:2
        sim = results.(ctrl{c, 1}).(name);
        pap = paper_curves(ctrl{c, 1}, name);
        for j = 1:6
            subplot(6, 2, 2*(j-1) + c); hold on; grid on; box on;
            plot(pap.t, pap.r(j, :), 'Color', [1 .6 .6], 'LineWidth', 0.8);
            plot(pap.t, pap.q(j, :), 'k', 'LineWidth', 3);
            plot(sim.t, sim.q(j, :), '-', 'Color', ctrl{c, 2}, 'LineWidth', 1.2);
            rms_err = sqrt(mean((sim.q(j, :) - pap.q(j, :)).^2));
            title(sprintf('%s joint %d (%s):  rms difference %.3f rad', ctrl{c, 1}, j, name, rms_err));
            ylabel(sprintf('joint%d /rad', j));
            xlim([0 5]);
            light_grid();
            if j == 1, legend({'dir', 'paper', 'this work'}, 'Location', 'southeast'); end
            if j == 6, xlabel('time / s'); end
        end
    end
    save_png(fig, fullfile(outdir, sprintf('compare_%s.png', name)));
end

fprintf('  wrote 16 figures to %s/ (figs 6-19, compare_step, compare_sine)\n', outdir);
end

% ------------------------------------------------------------------------
function style_axis(ylab)
%STYLE_AXIS  Common axis styling: 0-5 s span with half-second ticks.
ylabel(ylab);
xlim([0 5]);
set(gca, 'XTick', 0:0.5:5, 'FontSize', 9);
light_grid();
end

function light_grid()
%LIGHT_GRID  Pale grid lines as in the paper (property names differ between
%   MATLAB and Octave versions, so failures are ignored).
try, set(gca, 'GridColor', [0.8 0.8 0.8]); catch, end
try, set(gca, 'GridAlpha', 0.6); catch, end
end

function save_png(fig, file)
print(fig, file, '-dpng', '-r150');
close(fig);
end
