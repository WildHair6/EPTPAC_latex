%PLOT_CASE1_RESULTS  Plot five figures for Case 1 (tracking + constraints)
%
%   输入:
%       t         : 时间向量 (N x 1 或 1 x N)
%       sigma_ref : 参考 MRP (N x 3)
%       sigma_act : 实际 MRP (N x 3)
%       omega_ref : 参考角速度 (N x 3) [rad/s]
%       omega_act : 实际角速度 (N x 3) [rad/s]
%       tau_act   : 实际控制力矩 (N x 3) [N·m]
%       tau_max   : 力矩上限 (1 x 3 或 3 x 1)，按轴给
%       h_act     : 实际反作用飞轮角动量 (N x 3) [N·m·s]
%       h_max     : 角动量上限 (1 x 3 或 3 x 1)，按轴给
%
%   图的说明:
%     Fig 1: 参考姿态 vs 实际姿态 (MRP 三分量, 三个子图)
%     Fig 2: 姿态跟踪误差 MRP 三分量 (一张图)
%     Fig 3: 角速度跟踪误差三分量 (一张图)
%     Fig 4: 三轴控制力矩与上限对比 (三个子图)
%     Fig 5: 三轴角动量与上限对比 (三个子图)

% -------- 预处理 --------
t = 0:sim_dt:t_total;                % 确保是列向量
N = length(t);
sigma_ref = MRP_SEQ;
sigma_act = squeeze(qi)';
sigma_act(1,:) = sigma_act(2,:);
omega_ref = Omega_SEQ;
omega_act = squeeze(wb)';
omega_act(1,:) = omega_act(2,:);
tau_act = squeeze(RWT);
tau_act(1,:) = tau_act(2,:);
tau_max = [0.2;0.2;0.2];
h_act = squeeze(HW)';
h_max = [4;4;4];
s_p = squeeze(s);
sigma_e = squeeze(qe);
omega_e = squeeze(Omegae);


% ====== 滤波参数（按“秒”设定更直观）======
hampel_win_s = 0.30;   % Hampel窗口：0.10s（去尖峰）
nsigma       = 2;      % 判离群阈值（3~4 常用）
sg_win_s     = 1;   % SG平滑窗口：0.25s（越大越平滑）
sg_order     = 3;      % SG多项式阶数：2~3 常用

% ====== 对“用于画图”的数据做去突变+平滑 ======
sigma_act_f = despike_and_smooth(sigma_act, sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);
omega_act_f = despike_and_smooth(omega_act, sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);
tau_act_f   = despike_and_smooth(tau_act,   sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);
h_act_f     = despike_and_smooth(h_act,     sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);
sigma_e_f   = despike_and_smooth(sigma_e,   sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);
omega_e_f   = despike_and_smooth(omega_e,   sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);
s_p_f       = despike_and_smooth(s_p,       sim_dt, hampel_win_s, nsigma, sg_win_s, sg_order);

assert(size(sigma_ref,1) == N && size(sigma_act,1) == N, ...
    'sigma_ref / sigma_act 行数必须与 t 一致');
assert(size(omega_ref,1) == N && size(omega_act,1) == N, ...
    'omega_ref / omega_act 行数必须与 t 一致');
assert(size(tau_act,1) == N && size(h_act,1) == N, ...
    'tau_act / h_act 行数必须与 t 一致');

tau_max = tau_max(:).';   % 转成 1x3
h_max   = h_max(:).';

% 姿态 & 角速度误差
sigma_e = qe;     % N x 3
sigma_e(1,:) = sigma_e(2,:);
omega_e = Omegae;     % N x 3
omega_e(1,:) = omega_e(2,:);

% 创建输出文件夹
output_folder = 'plots';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

% 统一的文件名前缀和格式
file_prefix = 'Case1_Results';
file_suffix = sprintf('_tf%.1f', tf);  % 动态命名部分

% 图形尺寸设置
fig_width_single = 12*1.2;    % 单图宽度（厘米）
fig_height_single = 6*1.2;    % 单图高度（厘米）
fig_width_tiled = 14*1.2;     % 子图宽度（厘米）  
fig_height_tiled = 10*1.2;    % 子图高度（厘米）

% 统一字体
fontName = 'Times New Roman';
fontSize = 14;
lw       = 1.8;
labelStr = sprintf('$T_f = %.0f\\,\\mathrm{s}$', tf);

if exist('eps1', 'var')
    sigma_bound = eps1;
else
    sigma_bound = 1e-5;
end

if exist('eps2', 'var')
    s_bound = eps2;
else
    s_bound = 6.43e-5;
end

boundColor = [0.35 0.35 0.35];
tfColor = [0.45 0.45 0.45];
limitColor = [0.25 0.25 0.25];
minFloor = 1e-16;

%% ================== Figure 1: MRP ref vs actual ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_sigma = {'$\sigma_1$','$\sigma_2$','$\sigma_3$'};

% 创建 tiledlayout：3 行 1 列
tl = tiledlayout(3,1, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:3
    nexttile;
    plot(t, sigma_act_f(:,i), 'r-',  'LineWidth', lw); hold on;
    plot(t, sigma_ref(:,i), 'b--', 'LineWidth', lw);
    if i == 1
        xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2, ...
          'Label', labelStr, ...
          'LabelVerticalAlignment', 'top', ...
          'LabelHorizontalAlignment', 'right', ...
          'LabelOrientation', 'horizontal', ...
          'FontName', fontName, ...
          'FontSize', fontSize, ...
          'Interpreter', 'latex');
        xTf.HandleVisibility = 'off';
    else
        xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2);
        xTf.HandleVisibility = 'off';
    end

    grid on; box on;
    ylabel(labels_sigma{i}, 'Interpreter','latex', ...
           'FontName',fontName,'FontSize',fontSize);

    if i == 3
        xlabel('Time (s)', 'Interpreter','latex', ...
               'FontName',fontName,'FontSize',fontSize);
    end
    set(gca,'FontName',fontName,'FontSize',fontSize);
    xlim([t(1), t(end)]);
end

% % 添加图例到布局底部（不挤压子图）
% lgd = legend({'Reference','Actual'}, ...
%     'Interpreter','latex', ...
%     'FontName',fontName,'FontSize',fontSize, ...
%     'Orientation','horizontal');
% lgd.Layout.Tile = 'south';  % 关键：放在布局外底部

% 添加图例到布局底部（不挤压子图）
lgd = legend({'Actual','Reference'}, ...
    'Interpreter','latex', ...
    'FontName',fontName,'FontSize',fontSize, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';  % 关键：放在布局外底部

% 保存为PDF
filename = sprintf('%s_AttitudeComparison%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('已保存: %s\n', filename);

%% ================== Figure 2: OMGEA ref vs actual ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_omega = {'$\omega_1$','$\omega_2$','$\omega_3$'};

% 创建 tiledlayout：3 行 1 列
tl = tiledlayout(3,1, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:3
    nexttile;
    plot(t, omega_act_f(:,i), 'r-',  'LineWidth', lw); hold on;
    plot(t, omega_ref(:,i), 'b--', 'LineWidth', lw);
    if i == 1
        xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2, ...
          'Label', labelStr, ...
          'LabelVerticalAlignment', 'top', ...
          'LabelHorizontalAlignment', 'right', ...
          'LabelOrientation', 'horizontal', ...
          'FontName', fontName, ...
          'FontSize', fontSize, ...
          'Interpreter', 'latex');
        xTf.HandleVisibility = 'off';
    else
        xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2);
        xTf.HandleVisibility = 'off';
    end

    grid on; box on;
    ylabel(labels_omega{i}, 'Interpreter','latex', ...
           'FontName',fontName,'FontSize',fontSize);

    if i == 3
        xlabel('Time (s)', 'Interpreter','latex', ...
               'FontName',fontName,'FontSize',fontSize);
    end
    set(gca,'FontName',fontName,'FontSize',fontSize);
    xlim([t(1), t(end)]);
end

% % 添加图例到布局底部（不挤压子图）
% lgd = legend({'Reference','Actual'}, ...
%     'Interpreter','latex', ...
%     'FontName',fontName,'FontSize',fontSize, ...
%     'Orientation','horizontal');
% lgd.Layout.Tile = 'south';  % 关键：放在布局外底部

% 添加图例到布局底部（不挤压子图）
lgd = legend({'Actual','Reference'}, ...
    'Interpreter','latex', ...
    'FontName',fontName,'FontSize',fontSize, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';  % 关键：放在布局外底部

% 保存为PDF
filename = sprintf('%s_omega_AttitudeComparison%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('已保存: %s\n', filename);


%% ================== Figure 2: error responses ==================
%{
% Previous linear component plots. Keep this block if component-wise
% responses are needed again.

figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_single, fig_height_single]);
plot(t, sigma_e_f(:,1), 'r-',  'LineWidth', lw); hold on;
plot(t, sigma_e_f(:,2), 'b--', 'LineWidth', lw);
plot(t, sigma_e_f(:,3), 'k:',  'LineWidth', lw);
yline(sigma_bound, '--', 'Color', boundColor, 'LineWidth', 1.0);
yline(-sigma_bound, '--', 'Color', boundColor, 'LineWidth', 1.0);
xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2);
grid on; box on;
xlabel('Time (s)', 'Interpreter','latex', 'FontName',fontName,'FontSize',fontSize);
ylabel('$\sigma_e$', 'Interpreter','latex', 'FontName',fontName,'FontSize',fontSize);
legend({'$\sigma_{e1}$','$\sigma_{e2}$','$\sigma_{e3}$','$\pm\varepsilon_1$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, 'Location','best');
set(gca,'FontName',fontName,'FontSize',fontSize);
xlim([t(1), t(end)]);

figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_single, fig_height_single]);
plot(t, s_p_f(:,1), 'r-',  'LineWidth', lw); hold on;
plot(t, s_p_f(:,2), 'b--', 'LineWidth', lw);
plot(t, s_p_f(:,3), 'k:',  'LineWidth', lw);
yline(s_bound, '--', 'Color', boundColor, 'LineWidth', 1.0);
yline(-s_bound, '--', 'Color', boundColor, 'LineWidth', 1.0);
xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2);
grid on; box on;
xlabel('Time (s)', 'Interpreter','latex', 'FontName',fontName,'FontSize',fontSize);
ylabel('$s$', 'Interpreter','latex', 'FontName',fontName,'FontSize',fontSize);
legend({'$s_1$','$s_2$','$s_3$','$\pm\varepsilon_2$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, 'Location','best');
set(gca,'FontName',fontName,'FontSize',fontSize);
xlim([t(1), t(end)]);
%}

%% ================== Figure 2b: combined log-scale error norms ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_single, 1.35*fig_height_single]);
tl = tiledlayout(1,1, 'Padding','compact', 'TileSpacing','compact');
nexttile;
sigma_norm = sqrt(sum(sigma_e_f.^2, 2));
sigma_norm = max(sigma_norm, minFloor);
s_norm = sqrt(sum(s_p_f.^2, 2));
s_norm = max(s_norm, minFloor);
if numel(sigma_norm) >= 2
    sigma_norm(1) = sigma_norm(2);
end
if numel(s_norm) >= 2
    s_norm(1) = s_norm(2);
end

colorSigma = [0.12 0.34 0.70];
colorS = [0.86 0.45 0.08];
colorEps1 = colorSigma;
colorEps2 = colorS;

hSigNorm = plot(t, log10(sigma_norm), '-', 'Color', colorSigma, 'LineWidth', lw); hold on;
hSNorm = plot(t, log10(s_norm), '--', 'Color', colorS, 'LineWidth', lw);
hSigBound = yline(log10(sigma_bound), '--', 'Color', colorEps1, 'LineWidth', 2.0);
hSBound = yline(log10(s_bound), '--', 'Color', colorEps2, 'LineWidth', 2.0);
hSigBound.HandleVisibility = 'off';
hSBound.HandleVisibility = 'off';
epsLabelX = t(end)*0.06;
text(epsLabelX, log10(sigma_bound) + 0.08, '$\varepsilon_1$', ...
     'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
     'Color', [0 0 0], 'VerticalAlignment','bottom');
text(epsLabelX, log10(s_bound) - 0.08, '$\varepsilon_2$', ...
     'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
     'Color', [0 0 0], 'VerticalAlignment','top');
xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2);
xTf.HandleVisibility = 'off';

grid on; box on;
xlabel('Time (s)', 'Interpreter','latex', ...
       'FontName',fontName,'FontSize',fontSize);
ylabel('$\log_{10}(||\cdot||)$', ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize);
lgd = legend([hSigNorm, hSNorm], ...
       {'$\log_{10}(||\sigma_e||)$','$\log_{10}(||s||)$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
       'Orientation','horizontal');
lgd.Layout.Tile = 'south';
set(gca,'FontName',fontName,'FontSize',fontSize);
xlim([t(1), t(end)]);
allLogData = [log10(sigma_norm); log10(s_norm); log10(sigma_bound); log10(s_bound)];
yMin = min(allLogData);
yMax = max(allLogData);
yPad = max(0.35, 0.12*(yMax - yMin));
ylim([yMin - yPad, yMax + yPad]);
text(tf + 0.015*(t(end)-t(1)), yMax + 0.78*yPad, labelStr, ...
     'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
     'Color', tfColor, 'HorizontalAlignment','left', ...
     'VerticalAlignment','top');

filename = sprintf('%s_ErrorNorms%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('Saved: %s\n', filename);

%% ================== Figure 5: control torque vs limits ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_tau = {'$\tau_x$','$\tau_y$','$\tau_z$'};

% 使用 tiledlayout
tl = tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');

for i = 1:3
    nexttile;
    plot(t, tau_act_f(:,i), 'r-', 'LineWidth', lw); hold on;
    
    % 上下限（灰色虚线）
    yline(tau_max(i),  '--', 'Color', limitColor, 'LineWidth', 1.0);
    yline(-tau_max(i), '--', 'Color', limitColor, 'LineWidth', 1.0);
    if i == 1
        xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2, ...
          'Label', labelStr, ...
          'LabelVerticalAlignment', 'top', ...
          'LabelHorizontalAlignment', 'right', ...
          'LabelOrientation', 'horizontal', ...
          'FontName', fontName, ...
          'FontSize', fontSize, ...
          'Interpreter', 'latex');
        xTf.HandleVisibility = 'off';
    else
        xTf = xline(tf, 'Color', tfColor, 'LineStyle', '--', 'LineWidth', 1.2);
        xTf.HandleVisibility = 'off';
    end

    grid on; box on;
    ylabel(labels_tau{i}, 'Interpreter','latex', ...
           'FontName',fontName,'FontSize',fontSize);

    if i == 3
        xlabel('Time (s)', 'Interpreter','latex', ...
               'FontName',fontName,'FontSize',fontSize);
    end
    set(gca,'FontName',fontName,'FontSize',fontSize);
    xlim([t(1), t(end)]);
    ylim([-1.10*tau_max(i), 1.10*tau_max(i)]);
end

% 图例放在底部
lgd = legend({'$\tau_i$','${+}\tau_{\max}$','${-}\tau_{\max}$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
       'Orientation','horizontal');
lgd.Layout.Tile = 'south';

% 保存为PDF
filename = sprintf('%s_ControlTorque%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('已保存: %s\n', filename);

%% ================== Figure 6: wheel angular momentum vs limits ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_h = {'$h_x$','$h_y$','$h_z$'};

% 使用 tiledlayout
tl = tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');

for i = 1:3
    nexttile;
    plot(t, h_act_f(:,i), 'r-', 'LineWidth', lw); hold on;

    yline(h_max(i),  '--', 'Color', limitColor, 'LineWidth', 1.0);
    yline(-h_max(i), '--', 'Color', limitColor, 'LineWidth', 1.0);
    if i == 1
        xTf = xline(tf, '--', 'Color', tfColor, 'LineWidth', 1.2, ...
          'Label', labelStr, ...
          'LabelVerticalAlignment', 'top', ...
          'LabelHorizontalAlignment', 'right', ...
          'LabelOrientation', 'horizontal', ...
          'FontName', fontName, ...
          'FontSize', fontSize, ...
          'Interpreter', 'latex');
        xTf.HandleVisibility = 'off';
    else
        xTf = xline(tf, 'Color', tfColor, 'LineStyle', '--', 'LineWidth', 1.2);
        xTf.HandleVisibility = 'off';
    end

    grid on; box on;
    ylabel(labels_h{i}, 'Interpreter','latex', ...
           'FontName',fontName,'FontSize',fontSize);

    if i == 3
        xlabel('Time (s)', 'Interpreter','latex', ...
               'FontName',fontName,'FontSize',fontSize);
    end
    set(gca,'FontName',fontName,'FontSize',fontSize);
    xlim([t(1), t(end)]);
    ylim([-1.10*h_max(i), 1.10*h_max(i)]);
end

% 图例放在底部
lgd = legend({'$h_i$','${+}h_{\max}$','${-}h_{\max}$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
       'Orientation','horizontal');
lgd.Layout.Tile = 'south';

% 保存为PDF
filename = sprintf('%s_WheelMomentum%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('已保存: %s\n', filename);

fprintf('\n所有图片已保存到文件夹: %s\n', output_folder);

% 创建数据保存文件夹
data_folder = 'saved_data';
if ~exist(data_folder, 'dir')
    mkdir(data_folder);
end

% 保存变量到指定文件夹
filename = sprintf('RWT_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'tau_act_f');

filename = sprintf('q_e_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'q_tar_e');

filename = sprintf('h_act_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'h_act_f');

filename = sprintf('omega_act_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'omega_act');

filename = sprintf('se_tf%g.mat', tf);
save(fullfile(data_folder, filename), 's_p_f');

save(fullfile(data_folder, 't_sim'), 't');

fprintf('所有数据已保存到文件夹: %s\n', data_folder);

function X = ensureNx3(X, N, name)
    X = squeeze(X);
    if isvector(X)
        error('%s 不能是向量，期望 N×3', name);
    end

    % 尝试把 3×N 转成 N×3
    if size(X,1)==3 && size(X,2)==N
        X = X.';
    end

    if size(X,1)~=N || size(X,2)~=3
        error('%s 尺寸不对：当前 %dx%d，期望 %dx3', name, size(X,1), size(X,2), N);
    end
end

function Y = despike_and_smooth(X, dt, hampel_win_s, nsigma, sg_win_s, sg_order)
    % X: N×3
    N = size(X,1);

    hampel_win = max(3, round(hampel_win_s/dt));
    if mod(hampel_win,2)==0, hampel_win = hampel_win+1; end

    sg_win = max(5, round(sg_win_s/dt));
    if mod(sg_win,2)==0, sg_win = sg_win+1; end
    sg_win = min(sg_win, N-(mod(N+1,2))); % 防止窗口大于数据长度（粗略保护）

    Y = X;
    for k = 1:3
        % 1) 去尖峰：Hampel
        y1 = hampel(X(:,k), hampel_win, nsigma);

        % 2) 平滑：Savitzky-Golay
        y2 = smoothdata(y1, 'sgolay', sg_win, 'Degree', sg_order);

        Y(:,k) = y2;
    end
end
