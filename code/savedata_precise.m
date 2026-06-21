% 创建数据保存文件夹
data_folder = 'saved_data_precise';
if ~exist(data_folder, 'dir')
    mkdir(data_folder);
end

% 保存变量到指定文件夹
filename = sprintf('s_eps1%g.mat', eps1);
save(fullfile(data_folder, filename), 's');

filename = sprintf('qe_eps1%g.mat', eps1);
save(fullfile(data_folder, filename), 'qe');

save(fullfile(data_folder, 't_sim'), 'tout');

fprintf('所有数据已保存到文件夹: %s\n', data_folder);