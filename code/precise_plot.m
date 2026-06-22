

%% ---------- Force LaTeX rendering globally ----------
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%% ---------- style settings (match your template) ----------
fontName = 'Times New Roman';
fontSize = 14;
lw       = 1.8;     % curve line width
lw_th    = 3.0;     % threshold line width (thicker)
fig_width_single  = 16;    % cm
fig_height_single = 11.2;  % cm

%% ---------- folders ----------
dataDir       = 'saved_data_precise';
output_folder = 'precise_compare';
if ~exist(output_folder,'dir'); mkdir(output_folder); end

file_prefix = 'CaseSweep';
file_suffix = '_log10';

%% ---------- eps1 list (NOW 4 cases) ----------
eps1_list = [1e-4, 1e-5, 1e-6, 1e-7];

%% ---------- design constants (use paper values; set eta to your sim value) ----------
eta  = 0.2;      % <<< set to your simulation value
Tp1  = 100;      % s
Tp2  = 15;       % s
dmax = 1e-2;     % N*m
Jmin = 200;      % lambda_min(J) for diag([200 250 300])

minFloor = 1e-16;  % avoid log10(0)

%% ---------- load time ----------
tFile = fullfile(dataDir, 't_sim.mat');
assert(exist(tFile,'file')==2, 'Cannot find %s', tFile);
St = load(tFile);

tf = 120;
labelStr = sprintf('$T_f=%.0f$', tf);

%% ---------- auto compute eps2_list ----------
eps2_list  = zeros(size(eps1_list));

exponent = (1 - eta) / (2 - eta);
term_pre = (2 * dmax * Tp2) / (Tp1 * (Jmin^(1 - eta/2)));
Gamma = term_pre^(1 / (2 - eta));

for i = 1:numel(eps1_list)
    eps1 = eps1_list(i);
    eps2 = Gamma * (eps1^exponent);
    eps2_list(i) = eps2;
end

fprintf('\n===== eps2 from scaling rule =====\n');
for k = 1:numel(eps1_list)
    fprintf('eps1 = %.0e  ->  eps2 = %.4e\n', eps1_list(k), eps2_list(k));
end
fprintf('=================================\n\n');

%% ---------- load qe & s, compute norms ----------
qe_norm = cell(numel(eps1_list),1);
s_norm  = cell(numel(eps1_list),1);

for k = 1:numel(eps1_list)
    eps1 = eps1_list(k);

    qeFile = fullfile(dataDir, sprintf('qe_eps1%g.mat', eps1));
    sFile  = fullfile(dataDir, sprintf('s_eps1%g.mat',  eps1));
    assert(exist(qeFile,'file')==2, 'Cannot find %s', qeFile);
    assert(exist(sFile,'file')==2,  'Cannot find %s', sFile);

    Sqe = load(qeFile);
    Ss  = load(sFile);

    qe = pick_Nx3_matrix(Sqe);  % N×3
    s  = pick_Nx3_matrix(Ss);   % N×3

    N = min([size(qe,1), size(s,1)]);
    qe = qe(1:N,:);
    s  = s(1:N,:);

    qe_n = sqrt(sum(qe.^2,2));
    s_n  = sqrt(sum(s.^2, 2));

    qe_n = max(qe_n, minFloor);
    s_n  = max(s_n,  minFloor);
    if numel(qe_n) >= 2, qe_n(1) = qe_n(2); end
    if numel(s_n)  >= 2, s_n(1)  = s_n(2);  end

    qe_norm{k} = qe_n;
    s_norm{k}  = s_n;
end

Nplot  = min(cellfun(@numel, qe_norm));
t_plot = linspace(0, tf, Nplot).';

% 4 colors: yellow, red, blue, black (fixed)
colorList = {
    [1.0 0.85 0.0];   % yellow
    [1.0 0.0  0.0];   % red
    [0.0 0.0  1.0];   % blue
    [0.0 0.0  0.0]    % black
};


%% ================== Figure 1: ||sigma_e|| (log10) ==================
figure('Color','w', 'Units','centimeters', ...
    'Position',[5, 5, fig_width_single, fig_height_single]);

ax = axes('Position', [0.12, 0.25, 0.80, 0.68]);
hold(ax, 'on');

lineStyles = {'-','--','-.',':'};   % NOW 4 styles
hLine = gobjects(numel(eps1_list),1);

for k = 1:numel(eps1_list)
    ls = lineStyles{1 + mod(k-1, numel(lineStyles))};
    c  = colorList{min(k, numel(colorList))};

    ylog = log10(qe_norm{k}(1:Nplot));
    hLine(k) = plot(ax, t_plot, ylog, ls, ...
        'LineWidth', lw, 'Color', c); hold on;

    yl = yline(ax, log10(eps1_list(k)), '--', ...
        'LineWidth', lw_th, 'Color', c);
    yl.HandleVisibility = 'off';
end

% xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
%     'Label', labelStr, ...
%     'LabelVerticalAlignment','bottom', ...
%     'LabelHorizontalAlignment','left');

grid on; box on;
xlabel(ax, 'Time (s)', 'FontName',fontName,'FontSize',fontSize);
ylabel(ax, '$\log_{10}(||\sigma_e||)$', ...
       'FontName',fontName,'FontSize',fontSize);

add_bottom_legend(ax, hLine, ...
    make_eps_labels(eps1_list), ...
    fontName, fontSize);

set(ax,'FontName',fontName,'FontSize',fontSize);
xlim(ax, [t_plot(1), t_plot(end)]);

filename_pdf = sprintf('%s_MRPErrorNorm%s.pdf', file_prefix, file_suffix);
filename_fig = sprintf('%s_MRPErrorNorm%s.fig', file_prefix, file_suffix);
exportgraphics(gcf, fullfile(output_folder, filename_pdf), 'Resolution', 300);
savefig(gcf, fullfile(output_folder, filename_fig));
fprintf('已保存: %s\n', fullfile(output_folder, filename_pdf));
fprintf('已保存: %s\n', fullfile(output_folder, filename_fig));

%% ================== Figure 2: ||s|| (log10) ==================
figure('Color','w', 'Units','centimeters', ...
    'Position',[5, 5, fig_width_single, fig_height_single]);

ax = axes('Position', [0.12, 0.25, 0.80, 0.68]);
hold(ax, 'on');

hLine2 = gobjects(numel(eps1_list),1);

for k = 1:numel(eps1_list)
    ls = lineStyles{1 + mod(k-1, numel(lineStyles))};

    c  = colorList{min(k, numel(colorList))};

    ylog = log10(s_norm{k}(1:Nplot));
    hLine2(k) = plot(ax, t_plot, ylog, ls, ...
        'LineWidth', lw, 'Color', c); hold on;

    yl = yline(ax, log10(eps2_list(k)), '--', ...
        'LineWidth', lw_th, 'Color', c);
    yl.HandleVisibility = 'off';
end

% xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
%     'Label', labelStr, ...
%     'LabelVerticalAlignment','bottom', ...
%     'LabelHorizontalAlignment','left');

grid on; box on;
xlabel(ax, 'Time (s)', 'FontName',fontName,'FontSize',fontSize);
ylabel(ax, '$\log_{10}(||s||)$', ...
       'FontName',fontName,'FontSize',fontSize);

add_bottom_legend(ax, hLine2, ...
    make_eps_labels(eps1_list), ...
    fontName, fontSize);

set(ax,'FontName',fontName,'FontSize',fontSize);
xlim(ax, [t_plot(1), t_plot(end)]);

filename_pdf = sprintf('%s_SlidingNorm%s.pdf', file_prefix, file_suffix);
filename_fig = sprintf('%s_SlidingNorm%s.fig', file_prefix, file_suffix);
exportgraphics(gcf, fullfile(output_folder, filename_pdf), 'Resolution', 300);
savefig(gcf, fullfile(output_folder, filename_fig));
fprintf('已保存: %s\n', fullfile(output_folder, filename_pdf));
fprintf('已保存: %s\n', fullfile(output_folder, filename_fig));

%% ================== helper: pick first numeric N×3 matrix ==================
function add_bottom_legend(ax, handles, labels, fontName, fontSize)
    axPos = get(ax, 'Position');
    lgd = legend(ax, handles, labels, ...
        'Interpreter', 'latex', ...
        'FontName', fontName, ...
        'FontSize', fontSize - 1, ...
        'Orientation', 'horizontal', ...
        'Box', 'on');
    set(lgd, 'Units', 'normalized');
    set(lgd, 'Position', [axPos(1), 0.035, axPos(3), 0.055]);
end

function labels = make_eps_labels(eps_list)
    labels = cell(size(eps_list));
    for idx = 1:numel(eps_list)
        exponent = round(log10(eps_list(idx)));
        labels{idx} = sprintf('$\\varepsilon_1=10^{%d}$', exponent);
    end
end

function M = pick_Nx3_matrix(S)
    fn = fieldnames(S);

    preferred = {'qe','q_e','sigma_e','sigmae','s','slide','sliding'};
    for p = 1:numel(preferred)
        name = preferred{p};
        if isfield(S, name)
            X = S.(name);
            if isnumeric(X) && ismatrix(X) && size(X,2)==3
                M = X; return;
            end
        end
    end

    for i = 1:numel(fn)
        X = S.(fn{i});
        if isnumeric(X) && ismatrix(X) && size(X,2)==3
            M = X; return;
        end
    end

    error('No numeric N×3 matrix found in this .mat file.');
end
