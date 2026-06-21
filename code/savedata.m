
% 创建数据保存文件夹
data_folder = 'saved_data';
if ~exist(data_folder, 'dir')
    mkdir(data_folder);
end
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

% 保存变量到指定文件夹
filename = sprintf('RWT_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'tau_act');

filename = sprintf('q_e_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'q_tar_e');

filename = sprintf('h_act_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'h_act');

filename = sprintf('omega_act_tf%g.mat', tf);
save(fullfile(data_folder, filename), 'omega_act');

filename = sprintf('se_tf%g.mat', tf);
save(fullfile(data_folder, filename), 's_p');

save(fullfile(data_folder, 't_sim'), 't');

fprintf('所有数据已保存到文件夹: %s\n', data_folder);