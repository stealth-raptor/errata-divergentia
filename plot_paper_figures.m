function plot_paper_figures(results, outdir)
%PLOT_PAPER_FIGURES  Reproduce the paper's figures for PID and FOPID.
%
%   PLOT_PAPER_FIGURES(results, outdir)  with results from MAIN.
%
%   The layout follows the source paper exactly:
%
%     Figs 6-11   step response, one figure per joint, two panels:
%                 (a) position tracking trajectory   y label "jointN /rad"
%                 (b) tracking error                 y label "jointN err/rad"
%     Fig 12      driving torque with step response, 3x2 grid, "tauN / Nm"
%     Figs 13-18  sine response, one figure per joint, same two panels
%     Fig 19      driving torque with sine response, 3x2 grid, "jointN tau/N*m"
%
%   Colour convention, also taken from the paper: the reference ("dir") is
%   red, PID green and FOPID blue.  In the error panels of the sine figures
%   the paper reuses red for FOPID, because no reference is drawn there; that
%   inconsistency is kept so the figures match the published ones.
%
%   The paper's own figures additionally carry an FBPA-FOPID curve.  In its
%   place, when results has a FOPSO_GWO entry (the FOPID tuned by
%   TUNE_FOPID_HYBRID), that controller is drawn as a third, magenta curve.
%   Every curve is labelled in every panel.
%
%   See also MAIN.

if nargin < 2, outdir = 'results'; end
if ~exist(outdir, 'dir'), mkdir(outdir); end

RED   = [1 0 0];
GREEN = [0 1 0];
BLUE  = [0 0 1];
MAGENTA = [0.85 0 0.85];

has_hybrid = isfield(results, 'FOPSO_GWO');
names = {'PID', 'FOPID'};
if has_hybrid, names{end+1} = 'FOPSO-GWO'; end

% ---- Figs 6-11 (step) and 13-18 (sine): per joint, two panels ------------
experiments = {'step', 6, 'with step response'
               'sine', 13, 'with sine response'};

for ex = 1:size(experiments, 1)
    name   = experiments{ex, 1};
    first  = experiments{ex, 2};
    for j = 1:6
        run_pid   = results.PID.(name);
        run_fopid = results.FOPID.(name);

        fig = figure('Visible', 'off', 'Position', [100 100 1100 380], 'Color', 'w');

        % (a) position tracking trajectory
        subplot(1, 2, 1); hold on; grid on; box on;
        plot(run_pid.t,   run_pid.r(j, :), 'Color', RED,   'LineWidth', 1);
        plot(run_pid.t,   run_pid.q(j, :), 'Color', GREEN, 'LineWidth', 1);
        plot(run_fopid.t, run_fopid.q(j, :), 'Color', BLUE, 'LineWidth', 1);
        if has_hybrid
            run_h = results.FOPSO_GWO.(name);
            plot(run_h.t, run_h.q(j, :), 'Color', MAGENTA, 'LineWidth', 1);
        end
        style_axis(sprintf('joint%d /rad', j));
        legend([{'dir'}, names], 'Location', 'northeast');
        xlabel({'time / s', '', '(a) position tracking trajectory'});

        % (b) tracking error
        subplot(1, 2, 2); hold on; grid on; box on;
        err_colour_fopid = BLUE;
        if strcmp(name, 'sine')
            err_colour_fopid = RED;          % as printed in the paper
        end
        plot(run_pid.t,   run_pid.r(j, :)   - run_pid.q(j, :),   'Color', GREEN, 'LineWidth', 1);
        plot(run_fopid.t, run_fopid.r(j, :) - run_fopid.q(j, :), 'Color', err_colour_fopid, 'LineWidth', 1);
        if has_hybrid
            plot(run_h.t, run_h.r(j, :) - run_h.q(j, :), 'Color', MAGENTA, 'LineWidth', 1);
        end
        style_axis(sprintf('joint%d err/rad', j));
        legend(names, 'Location', 'northeast');
        xlabel({'time / s', '', '(b) tracking error'});

        num = first + j - 1;
        file = fullfile(outdir, sprintf('fig%02d_joint%d_%s.png', num, j, name));
        exportgraphics(fig, file, 'Resolution', 200);
        close(fig);
    end
end

% ---- Fig 12 (step torque) and Fig 19 (sine torque): 3x2 grids ------------
torques = {'step', 12, '\tau%d / Nm'
           'sine', 19, 'joint%d tau/N*m'};

for tq = 1:size(torques, 1)
    name = torques{tq, 1};
    fig = figure('Visible', 'off', 'Position', [100 100 1100 900], 'Color', 'w');
    for j = 1:6
        subplot(3, 2, j); hold on; grid on; box on;
        plot(results.PID.(name).t,   results.PID.(name).u(j, :),   'Color', GREEN, 'LineWidth', 1);
        plot(results.FOPID.(name).t, results.FOPID.(name).u(j, :), 'Color', BLUE,  'LineWidth', 1);
        if has_hybrid
            plot(results.FOPSO_GWO.(name).t, results.FOPSO_GWO.(name).u(j, :), 'Color', MAGENTA, 'LineWidth', 1);
        end
        style_axis(sprintf(torques{tq, 3}, j));
        xlabel('time / s');
        legend(names, 'Location', 'northeast');
    end
    file = fullfile(outdir, sprintf('fig%02d_%s_torque.png', torques{tq, 2}, name));
    exportgraphics(fig, file, 'Resolution', 200);
    close(fig);
end

fprintf('  wrote 14 figures to %s/ (figs 6-11, 12, 13-18, 19)\n', outdir);
end

% ------------------------------------------------------------------------
function style_axis(ylab)
%STYLE_AXIS  Common axis styling: 0-5 s span with half-second ticks.
ylabel(ylab);
xlim([0 5]);
xticks(0:0.5:5);
set(gca, 'FontSize', 9, 'GridAlpha', 0.15);
end
