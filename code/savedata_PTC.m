% 创建数据保存文件夹
% data_folder = 'saved_data_precise';
data_folder = 'saved_data_PTCcompare';
if ~exist(data_folder, 'dir')
    mkdir(data_folder);
end

tau_act = squeeze(RWT);
tau_act(1,:) = tau_act(2,:);
h_act = squeeze(HW)';

% 保存变量到指定文件夹
filename = sprintf('tau_t%g.mat', t_total);
save(fullfile(data_folder, filename), 'tau_act');

filename = sprintf('hw_t%g.mat', t_total);
save(fullfile(data_folder, filename), 'h_act');

save(fullfile(data_folder, 't_sim'), 'tout');

fprintf('所有数据已保存到文件夹: %s\n', data_folder);

for i = 1:length(q_tar_e)
    q_norm(i) = norm(q_tar_e(i,:));
end
q_norm = q_norm';