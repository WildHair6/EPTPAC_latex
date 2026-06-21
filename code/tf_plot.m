%% Case 2 prescribed-time sweep plot
% Run this script from the code/ folder.

clearvars -except sim_dt t_total tf
close all;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

data_folder = 'saved_data';
plots_folder = 'tf_comparison_plots';

if ~exist(data_folder, 'dir')
    error('Cannot find data folder: %s', data_folder);
end
if ~exist(plots_folder, 'dir')
    mkdir(plots_folder);
end

tf_values = [110, 120, 130, 140];

fontName = 'Times New Roman';
fontSize = 14;
lw = 1.8;

figWidth = 16;
figHeight = 10.5;

colors = [
    1.0, 0.8, 0.0;
    1.0, 0.0, 0.0;
    0.0, 0.0, 1.0;
    0.0, 0.0, 0.0
];
lineStyles = {'-', '--', ':', '-.'};

t_base = load_time_vector(data_folder);

%% Load data and compute norm
qe_norm = cell(1, numel(tf_values));
qe_time = cell(1, numel(tf_values));

for tf_idx = 1:numel(tf_values)
    tf_now = tf_values(tf_idx);

    [qe, t_qe] = load_case_matrix(data_folder, sprintf('q_e_tf%g.mat', tf_now), ...
        {'q_tar_e', 'qe', 'q_e', 'sigma_e'}, t_base);
    qe = hold_after_tf(qe, t_qe, tf_now);

    qe_time{tf_idx} = t_qe;
    qe_norm{tf_idx} = sqrt(sum(qe.^2, 2));
end

%% MRP tracking-error norm
fig = figure('Color', 'w', 'Units', 'centimeters', ...
    'Position', [5, 5, figWidth, figHeight]);

axPos = [0.12, 0.27, 0.80, 0.65];
ax = axes('Position', axPos);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

legend_handles = gobjects(1, numel(tf_values));
legend_labels = cell(1, numel(tf_values));

for tf_idx = 1:numel(tf_values)
    tf_now = tf_values(tf_idx);
    legend_handles(tf_idx) = plot(ax, qe_time{tf_idx}, qe_norm{tf_idx}, ...
        'LineWidth', lw, ...
        'Color', colors(tf_idx, :), ...
        'LineStyle', lineStyles{tf_idx});
    legend_labels{tf_idx} = sprintf('$T_f=%d~\\mathrm{s}$', tf_now);
end

add_tf_lines(ax, tf_values);

xlabel(ax, 'Time (s)', 'FontName', fontName, 'FontSize', fontSize);
ylabel(ax, '$||\sigma_{e,\mathrm{tar}}||$', ...
    'FontName', fontName, 'FontSize', fontSize);
set(ax, 'FontName', fontName, 'FontSize', fontSize);
xlim(ax, [0, max(t_base)]);
ylim(ax, [0, 1.08 * max(cellfun(@max, qe_norm))]);

zoomX = [105, 145];
zoomBoxY = [0, 0.05];
zoomInsetY = [0, 0.001];
add_zoom_box(ax, zoomX, zoomBoxY);

ax_inset = axes('Position', [0.55, 0.58, 0.34, 0.29]);
hold(ax_inset, 'on'); grid(ax_inset, 'on'); box(ax_inset, 'on');
for tf_idx = 1:numel(tf_values)
    plot(ax_inset, qe_time{tf_idx}, qe_norm{tf_idx}, ...
        'LineWidth', 1.4, ...
        'Color', colors(tf_idx, :), ...
        'LineStyle', lineStyles{tf_idx});
end
add_tf_lines(ax_inset, tf_values);
set(ax_inset, 'FontName', fontName, 'FontSize', fontSize - 3);
xlim(ax_inset, zoomX);
ylim(ax_inset, zoomInsetY);

add_inset_arrow(ax, ax_inset, zoomX, zoomBoxY);
add_bottom_legend(ax, legend_handles, legend_labels, fontName, fontSize, axPos);

save_figure(fig, plots_folder, 'qe_norm_comparison');

fprintf('Case 2 MRP norm figure saved in: %s\n', plots_folder);

%% Helper functions
function t = load_time_vector(data_folder)
    time_file = fullfile(data_folder, 't_sim.mat');
    if exist(time_file, 'file') ~= 2
        error('Cannot find %s', time_file);
    end

    S = load(time_file);
    if isfield(S, 't')
        t = S.t(:);
    elseif isfield(S, 't_sim')
        t = S.t_sim(:);
    elseif isfield(S, 'tout')
        t = S.tout(:);
    else
        error('No recognized time vector in %s', time_file);
    end
end

function [M, t] = load_case_matrix(data_folder, filename, preferred_names, t_base)
    data_path = fullfile(data_folder, filename);
    if exist(data_path, 'file') ~= 2
        error('Cannot find %s', data_path);
    end

    S = load(data_path);
    M = pick_matrix(S, preferred_names);
    if size(M, 2) < 3
        error('%s does not contain a matrix with at least three columns.', data_path);
    end
    M = M(:, 1:3);
    if size(M, 1) >= 2
        M(1, :) = M(2, :);
    end

    if numel(t_base) == size(M, 1)
        t = t_base;
    else
        t = linspace(t_base(1), t_base(end), size(M, 1)).';
    end
end

function M = pick_matrix(S, preferred_names)
    for idx = 1:numel(preferred_names)
        name = preferred_names{idx};
        if isfield(S, name)
            X = S.(name);
            if isnumeric(X) && ismatrix(X)
                M = X;
                return;
            end
        end
    end

    fields = fieldnames(S);
    for idx = 1:numel(fields)
        X = S.(fields{idx});
        if isnumeric(X) && ismatrix(X) && size(X, 2) >= 3
            M = X;
            return;
        end
    end

    error('No numeric matrix found in data file.');
end

function M = hold_after_tf(M, t, tf_now)
    idx = find(t >= tf_now, 1, 'first');
    if ~isempty(idx)
        M(idx:end, :) = repmat(M(idx, :), numel(t) - idx + 1, 1);
    end
end

function add_tf_lines(ax, tf_values)
    for idx = 1:numel(tf_values)
        xline(ax, tf_values(idx), '--', ...
            'Color', [0.55 0.55 0.55], ...
            'LineWidth', 0.9, ...
            'Alpha', 0.45, ...
            'HandleVisibility', 'off');
    end
end

function add_zoom_box(ax, zoomX, zoomY)
    rectangle('Parent', ax, ...
        'Position', [zoomX(1), zoomY(1), diff(zoomX), diff(zoomY)], ...
        'EdgeColor', [0.85 0.0 0.0], ...
        'LineStyle', '--', ...
        'LineWidth', 1.2, ...
        'FaceColor', 'none', ...
        'HandleVisibility', 'off');
end

function add_inset_arrow(ax, ax_inset, zoomX, zoomY)
    axPos = get(ax, 'Position');
    insetPos = get(ax_inset, 'Position');
    xLim = xlim(ax);
    yLim = ylim(ax);

    targetX = axPos(1) + axPos(3) * (mean(zoomX) - xLim(1)) / diff(xLim);
    targetY = axPos(2) + axPos(4) * (zoomY(2) - yLim(1)) / diff(yLim);
    startX = insetPos(1) + 0.5 * insetPos(3);
    startY = insetPos(2);

    annotation('arrow', [startX, targetX], [startY, targetY], ...
        'Color', [0.85 0.0 0.0], ...
        'LineWidth', 1.0);
end

function add_bottom_legend(ax, legend_handles, legend_labels, fontName, fontSize, axPos)
    lgd = legend(ax, legend_handles, legend_labels, ...
        'Interpreter', 'latex', ...
        'FontName', fontName, ...
        'FontSize', fontSize - 1, ...
        'Orientation', 'horizontal', ...
        'Box', 'on');
    set(lgd, 'Units', 'normalized');
    set(lgd, 'Position', [axPos(1), 0.035, axPos(3), 0.055]);
end

function save_figure(fig, folder, filename)
    exportgraphics(fig, fullfile(folder, [filename, '.png']), 'Resolution', 600);
    exportgraphics(fig, fullfile(folder, [filename, '.pdf']), 'Resolution', 300);
    fprintf('Saved: %s\n', fullfile(folder, [filename, '.png']));
    fprintf('Saved: %s\n', fullfile(folder, [filename, '.pdf']));
end
