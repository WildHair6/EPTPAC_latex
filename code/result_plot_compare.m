%PLOT_CASE1_RESULTS  Plot five figures for Case 1 (tracking + constraints)
%
%   杈撳叆:
%       t         : 鏃堕棿鍚戦噺 (N x 1 鎴?1 x N)
%       sigma_ref : 鍙傝€?MRP (N x 3)
%       sigma_act : 瀹為檯 MRP (N x 3)
%       omega_ref : 鍙傝€冭閫熷害 (N x 3) [rad/s]
%       omega_act : 瀹為檯瑙掗€熷害 (N x 3) [rad/s]
%       tau_act   : 瀹為檯鎺у埗鍔涚煩 (N x 3) [N路m]
%       tau_max   : 鍔涚煩涓婇檺 (1 x 3 鎴?3 x 1)锛屾寜杞寸粰
%       h_act     : 瀹為檯鍙嶄綔鐢ㄩ杞鍔ㄩ噺 (N x 3) [N路m路s]
%       h_max     : 瑙掑姩閲忎笂闄?(1 x 3 鎴?3 x 1)锛屾寜杞寸粰
%
%   鍥剧殑璇存槑:
%     Fig 1: 鍙傝€冨Э鎬?vs 瀹為檯濮挎€?(MRP 涓夊垎閲? 涓変釜瀛愬浘)
%     Fig 2: 濮挎€佽窡韪宸?MRP 涓夊垎閲?(涓€寮犲浘)
%     Fig 3: 瑙掗€熷害璺熻釜璇樊涓夊垎閲?(涓€寮犲浘)
%     Fig 4: 涓夎酱鎺у埗鍔涚煩涓庝笂闄愬姣?(涓変釜瀛愬浘)
%     Fig 5: 涓夎酱瑙掑姩閲忎笌涓婇檺瀵规瘮 (涓変釜瀛愬浘)

% -------- preprocessing --------
t = (0:sim_dt:t_total).';
sigma_ref = as_nx3(MRP_SEQ);
sigma_act = as_nx3(squeeze(qi));
omega_ref = as_nx3(Omega_SEQ);
omega_act = as_nx3(squeeze(wb));
tau_act = as_nx3(squeeze(RWT));
tau_max = [0.2;0.2;0.2];
h_act = as_nx3(squeeze(HW));
h_max = [4;4;4];
s_p = as_nx3(squeeze(s));

N = min([numel(t), size(sigma_ref,1), size(sigma_act,1), ...
    size(omega_ref,1), size(omega_act,1), size(tau_act,1), size(h_act,1)]);
if N < numel(t)
    warning('Workspace data are shorter than 0:sim_dt:t_total. Plotting only %.2f s.', t(N));
end

t = t(1:N);
sigma_ref = sigma_ref(1:N,:);
sigma_act = sigma_act(1:N,:);
omega_ref = omega_ref(1:N,:);
omega_act = omega_act(1:N,:);
tau_act = tau_act(1:N,:);
h_act = h_act(1:N,:);
s_p = s_p(1:min(N,size(s_p,1)),:);

if size(sigma_act,1) >= 2, sigma_act(1,:) = sigma_act(2,:); end
if size(omega_act,1) >= 2, omega_act(1,:) = omega_act(2,:); end
if size(tau_act,1) >= 2, tau_act(1,:) = tau_act(2,:); end

tau_max = tau_max(:).';
h_max   = h_max(:).';

sigma_e = as_nx3(qe);
omega_e = as_nx3(Omegae);
sigma_e = sigma_e(1:min(N,size(sigma_e,1)),:);
omega_e = omega_e(1:min(N,size(omega_e,1)),:);
if size(sigma_e,1) >= 2, sigma_e(1,:) = sigma_e(2,:); end
if size(omega_e,1) >= 2, omega_e(1,:) = omega_e(2,:); end

% 鍒涘缓杈撳嚭鏂囦欢澶?
output_folder = 'PTC_compare';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

% 缁熶竴鐨勬枃浠跺悕鍓嶇紑鍜屾牸寮?
file_prefix = 'PTC_compare';
file_suffix = sprintf('_t_total%.1f', t_total);  % 鍔ㄦ€佸懡鍚嶉儴鍒?

% 鍥惧舰灏哄璁剧疆
fig_width_single = 12*1.2;    % 鍗曞浘瀹藉害锛堝帢绫筹級
fig_height_single = 6*1.2;    % 鍗曞浘楂樺害锛堝帢绫筹級
fig_width_tiled = 14*1.2;     % 瀛愬浘瀹藉害锛堝帢绫筹級  
fig_height_tiled = 10*1.2;    % 瀛愬浘楂樺害锛堝帢绫筹級

% 缁熶竴瀛椾綋
fontName = 'Times New Roman';
fontSize = 14;
lw       = 1.8;

ref_time = get_ref_time_marker(t_total);
labelStr = sprintf('$T_f=%.0f\\,\\mathrm{s}$', tf);
if isfinite(ref_time)
    labelStr2 = sprintf('$T_{f,\\mathrm{ref}}=%.0f\\,\\mathrm{s}$', ref_time);
else
    labelStr2 = '';
end

savedata_folder = 'saved_data_PTCcompare';
filename_tau = sprintf('tau_t%g.mat', t_total);
file_fill = fullfile(savedata_folder,filename_tau);
tau_tap = load(file_fill);
tau_tap = as_nx3(tau_tap.tau_act);

filename_hw = sprintf('hw_t%g.mat', t_total);
file_fill = fullfile(savedata_folder,filename_hw);
hw_tap = load(file_fill);
hw_tap = as_nx3(hw_tap.h_act);

N_ref = min([numel(t), size(tau_tap,1), size(hw_tap,1)]);
if N_ref < numel(t)
    warning('Saved EPTPAC data are shorter than the Ref. [19] workspace time vector. Plotting only %.2f s.', t(N_ref));
    t = t(1:N_ref);
    sigma_ref = sigma_ref(1:N_ref,:);
    sigma_act = sigma_act(1:N_ref,:);
    omega_ref = omega_ref(1:N_ref,:);
    omega_act = omega_act(1:N_ref,:);
    tau_act = tau_act(1:N_ref,:);
    h_act = h_act(1:N_ref,:);
end
tau_tap = tau_tap(1:N_ref,:);
hw_tap = hw_tap(1:N_ref,:);


%% ================== Figure 1: MRP ref vs actual ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_sigma = {'$\sigma_1$','$\sigma_2$','$\sigma_3$'};
useRightSideMrpLabels = abs(t_total - 400) < 1e-9;

% 鍒涘缓 tiledlayout锛? 琛?1 鍒?
tl = tiledlayout(3,1, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:3
    nexttile;
    plot(t, sigma_ref(:,i), 'r-',  'LineWidth', lw); hold on;
    plot(t, sigma_act(:,i), 'b-.', 'LineWidth', lw);
    if i == 1
        if useRightSideMrpLabels
            xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
            add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
        else
            xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
              'Label', labelStr, ...
              'LabelVerticalAlignment', 'middle', ...
              'LabelHorizontalAlignment', 'left', ...
              'LabelOrientation', 'horizontal', ...
              'Interpreter', 'latex', ...
              'FontName', fontName, 'FontSize', fontSize);
            add_ref_time_line(gca, ref_time, labelStr2, true, fontName, fontSize, 'middle');
        end
    else
        xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
        add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
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
    if i == 1 && useRightSideMrpLabels
        add_time_marker_label_right(gca, tf, labelStr, 0.58, fontName, fontSize);
        add_time_marker_label_right(gca, ref_time, labelStr2, 0.58, fontName, fontSize);
    end
end

% % 娣诲姞鍥句緥鍒板竷灞€搴曢儴锛堜笉鎸ゅ帇瀛愬浘锛?
% lgd = legend({'Reference','Actual'}, ...
%     'Interpreter','latex', ...
%     'FontName',fontName,'FontSize',fontSize, ...
%     'Orientation','horizontal');
% lgd.Layout.Tile = 'south';  % 鍏抽敭锛氭斁鍦ㄥ竷灞€澶栧簳閮?

% 娣诲姞鍥句緥鍒板竷灞€搴曢儴锛堜笉鎸ゅ帇瀛愬浘锛?
lgd = legend({'EPTPAC','Ref.~[19]'}, ...
    'Interpreter','latex', ...
    'FontName',fontName,'FontSize',fontSize, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';  % 鍏抽敭锛氭斁鍦ㄥ竷灞€澶栧簳閮?

% 淇濆瓨涓篜DF
filename = sprintf('%s_Attitude_%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('宸蹭繚瀛? %s\n', filename);

%% ================== Figure 2: OMGEA ref vs actual ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_omega = {'$\omega_1$','$\omega_2$','$\omega_3$'};

% 鍒涘缓 tiledlayout锛? 琛?1 鍒?
tl = tiledlayout(3,1, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:3
    nexttile;
    plot(t, omega_ref(:,i), 'r-',  'LineWidth', lw); hold on;
    plot(t, omega_act(:,i), 'b-.', 'LineWidth', lw);
    if i == 1
        xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
          'Label', labelStr, ...
          'LabelVerticalAlignment', 'bottom', ...
          'LabelHorizontalAlignment', 'left', ...
          'LabelOrientation', 'horizontal', ...
          'Interpreter', 'latex', ...
          'FontName', fontName, 'FontSize', fontSize);
        add_ref_time_line(gca, ref_time, labelStr2, true, fontName, fontSize, 'bottom');
    else
        xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
        add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
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

% % 娣诲姞鍥句緥鍒板竷灞€搴曢儴锛堜笉鎸ゅ帇瀛愬浘锛?
% lgd = legend({'Reference','Actual'}, ...
%     'Interpreter','latex', ...
%     'FontName',fontName,'FontSize',fontSize, ...
%     'Orientation','horizontal');
% lgd.Layout.Tile = 'south';  % 鍏抽敭锛氭斁鍦ㄥ竷灞€澶栧簳閮?

% 娣诲姞鍥句緥鍒板竷灞€搴曢儴锛堜笉鎸ゅ帇瀛愬浘锛?
lgd = legend({'EPTPAC','Ref.~[19]'}, ...
    'Interpreter','latex', ...
    'FontName',fontName,'FontSize',fontSize, ...
    'Orientation','horizontal');
lgd.Layout.Tile = 'south';  % 鍏抽敭锛氭斁鍦ㄥ竷灞€澶栧簳閮?

% 淇濆瓨涓篜DF
filename = sprintf('%s_omega_%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('宸蹭繚瀛? %s\n', filename);

%% ================== Figure 3: control torque vs limits ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_tau = {'$\tau_x$','$\tau_y$','$\tau_z$'};

% 浣跨敤 tiledlayout
tl = tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');

for i = 1:3
    nexttile;
    plot(t, tau_tap(:,i), 'r-', 'LineWidth', lw);hold on;
    plot(t, tau_act(:,i), 'b-.', 'LineWidth', lw); 

    
    % 涓婁笅闄愶紙鐏拌壊铏氱嚎锛?
    yline(tau_max(i),  'k--', 'LineWidth', 1.0);
    yline(-tau_max(i), 'k--', 'LineWidth', 1.0);
    if i == 1
        xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
        add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
    else
        xline(tf, 'Color', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1.2);
        add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
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
    ylim(1.25 * [-tau_max(i), tau_max(i)]);
    if i == 1
        if abs(t_total - 300) < 1e-9
            add_time_marker_label_side(gca, tf, labelStr, 0.68, 'left', fontName, fontSize);
            add_time_marker_label_side(gca, ref_time, labelStr2, 0.68, 'left', fontName, fontSize);
        elseif abs(t_total - 500) < 1e-9
            add_time_marker_label_side(gca, tf, labelStr, 0.68, 'left', fontName, fontSize);
            add_time_marker_label_side(gca, ref_time, labelStr2, 0.68, 'left', fontName, fontSize);
        else
            add_time_marker_label_side(gca, tf, labelStr, 0.68, 'right', fontName, fontSize);
            add_time_marker_label_side(gca, ref_time, labelStr2, 0.68, 'right', fontName, fontSize);
        end
    end
end

% 鍥句緥鏀惧湪搴曢儴
lgd = legend({'EPTPAC','Ref.~[19]','${+}\tau_{\max}$','${-}\tau_{\max}$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
       'Orientation','horizontal');
lgd.Layout.Tile = 'south';

% 淇濆瓨涓篜DF
filename = sprintf('%s_ControlTorque%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('宸蹭繚瀛? %s\n', filename);

%% ================== Figure 6: wheel angular momentum vs limits ==================
figure('Color','w', 'Units', 'centimeters', 'Position', [5, 5, fig_width_tiled, fig_height_tiled]);
labels_h = {'$h_x$','$h_y$','$h_z$'};

% 浣跨敤 tiledlayout
tl = tiledlayout(3,1, 'Padding','compact', 'TileSpacing','compact');

for i = 1:3
    nexttile;
    plot(t, hw_tap(:,i), 'r-', 'LineWidth', lw); hold on;
    plot(t, h_act(:,i), 'b-.', 'LineWidth', lw);
    
    yline(h_max(i),  'k--', 'LineWidth', 1.0);
    yline(-h_max(i), 'k--', 'LineWidth', 1.0);
    if i == 1
        if abs(t_total - 500) < 1e-9
            xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
            add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
        else
            xline(tf, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
              'Label', labelStr, ...
              'LabelVerticalAlignment', 'bottom', ...
              'LabelHorizontalAlignment', 'left', ...
              'LabelOrientation', 'horizontal', ...
              'Interpreter', 'latex', ...
              'FontName', fontName, 'FontSize', fontSize);
            add_ref_time_line(gca, ref_time, labelStr2, true, fontName, fontSize, 'bottom');
        end
    else
        xline(tf, 'Color', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1.2);
        add_ref_time_line(gca, ref_time, labelStr2, false, fontName, fontSize);
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
    if i == 1 && abs(t_total - 500) < 1e-9
        add_time_marker_label_side(gca, tf, labelStr, 0.68, 'left', fontName, fontSize);
        add_time_marker_label_side(gca, ref_time, labelStr2, 0.68, 'left', fontName, fontSize);
    end
end

% 鍥句緥鏀惧湪搴曢儴
lgd = legend({'EPTPAC','Ref.~[19]','${+}h_{\max}$','${-}h_{\max}$'}, ...
       'Interpreter','latex','FontName',fontName,'FontSize',fontSize, ...
       'Orientation','horizontal');
lgd.Layout.Tile = 'south';

% 淇濆瓨涓篜DF
filename = sprintf('%s_WheelMomentum%s.pdf', file_prefix, file_suffix);
filepath = fullfile(output_folder, filename);
exportgraphics(gcf, filepath, 'Resolution', 300);
fprintf('宸蹭繚瀛? %s\n', filename);

fprintf('\n鎵€鏈夊浘鐗囧凡淇濆瓨鍒版枃浠跺す: %s\n', output_folder);

% % 鍒涘缓鏁版嵁淇濆瓨鏂囦欢澶?
% data_folder = 'saved_data_PTCcompare';
% if ~exist(data_folder, 'dir')
%     mkdir(data_folder);
% end
% 
% % 淇濆瓨鍙橀噺鍒版寚瀹氭枃浠跺す
% filename = sprintf('RWT_tf%g.mat', tf);
% save(fullfile(data_folder, filename), 'tau_act');
% 
% filename = sprintf('q_e_tf%g.mat', tf);
% save(fullfile(data_folder, filename), 'q_tar_e');
% 
% filename = sprintf('h_act_tf%g.mat', tf);
% save(fullfile(data_folder, filename), 'h_act');
% 
% filename = sprintf('omega_act_tf%g.mat', tf);
% save(fullfile(data_folder, filename), 'omega_act');
% 
% filename = sprintf('se_tf%g.mat', tf);
% save(fullfile(data_folder, filename), 's_e_plot');
% 
function X = as_nx3(X)
    X = squeeze(X);
    if isvector(X)
        X = X(:);
    elseif size(X,2) == 3
        % already N-by-3
    elseif size(X,1) == 3
        X = X.';
    else
        error('Expected a numeric array convertible to N-by-3.');
    end
end

function ref_time = get_ref_time_marker(t_total)
    if abs(t_total - 400) < 1e-9
        ref_time = 170;
    elseif abs(t_total - 300) < 1e-9
        ref_time = 260;
    elseif abs(t_total - 500) < 1e-9
        ref_time = 410;
    else
        ref_time = NaN;
    end
end

function add_ref_time_line(ax, ref_time, labelStr, showLabel, fontName, fontSize, labelVerticalAlignment)
    if ~isfinite(ref_time)
        return;
    end
    if nargin < 7
        labelVerticalAlignment = 'bottom';
    end

    if showLabel
        xline(ax, ref_time, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
            'Label', labelStr, ...
            'LabelVerticalAlignment', labelVerticalAlignment, ...
            'LabelHorizontalAlignment', 'left', ...
            'LabelOrientation', 'horizontal', ...
            'Interpreter', 'latex', ...
            'FontName', fontName, 'FontSize', fontSize);
    else
        xline(ax, ref_time, '--', 'Color', [0.3 0.3 0.3], 'LineWidth', 1.2);
    end
end

function add_time_marker_label_side(ax, marker_time, labelStr, yFraction, side, fontName, fontSize)
    if ~isfinite(marker_time)
        return;
    end

    xLimits = xlim(ax);
    yLimits = ylim(ax);
    xOffset = 0.012 * (xLimits(2) - xLimits(1));
    yText = yLimits(1) + yFraction * (yLimits(2) - yLimits(1));
    if strcmpi(side, 'left')
        xText = marker_time - xOffset;
        horizontalAlignment = 'right';
    else
        xText = marker_time + xOffset;
        horizontalAlignment = 'left';
    end

    text(ax, xText, yText, labelStr, ...
        'Interpreter', 'latex', ...
        'FontName', fontName, ...
        'FontSize', fontSize, ...
        'Color', [0.3 0.3 0.3], ...
        'HorizontalAlignment', horizontalAlignment, ...
        'VerticalAlignment', 'middle', ...
        'Clipping', 'on');
end

function add_time_marker_label_right(ax, marker_time, labelStr, yFraction, fontName, fontSize)
    if ~isfinite(marker_time)
        return;
    end

    xLimits = xlim(ax);
    yLimits = ylim(ax);
    xText = marker_time + 0.012 * (xLimits(2) - xLimits(1));
    yText = yLimits(1) + yFraction * (yLimits(2) - yLimits(1));
    text(ax, xText, yText, labelStr, ...
        'Interpreter', 'latex', ...
        'FontName', fontName, ...
        'FontSize', fontSize, ...
        'Color', [0.3 0.3 0.3], ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'middle', ...
        'Clipping', 'on');
end

% save(fullfile(data_folder, 't_sim'), 't');
% 
% fprintf('鎵€鏈夋暟鎹凡淇濆瓨鍒版枃浠跺す: %s\n', data_folder);
