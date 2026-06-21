function [attitude_sequence, output] = Gauss_opt_mrp(s0, w0, sf, wf, I, torque_max, hw_max, t0, tf, max_angle_deviation_deg, sim_dt)
% 航天器姿态规划主函数（使用MRP和反作用飞轮）
% 输入参数:
%   s0 - 初始MRP (3×1向量)
%   w0 - 初始角速度 (3×1向量, rad/s)
%   sf - 目标MRP (3×1向量)
%   wf - 目标角速度 (3×1向量, rad/s)
%   I - 转动惯量矩阵 (3×3矩阵, kg*m^2)
%   torque_max - 最大控制力矩 (标量, N*m)
%   hw_max - 飞轮最大角动量 (标量, kg*m^2/s)
%   t0 - 初始时间 (标量, s)
%   tf - 终止时间 (标量, s)
%   sim_dt - 仿真步长 (标量, s)
% 输出参数:
%   attitude_sequence - 包含时间、MRP、角速度、控制力矩和飞轮角动量的结构体
%   output - GPOPS-II优化输出

% GPOPS-II setup
setup.name = 'Spacecraft-Attitude-Planning-RW';

setup.functions.endpoint   = @attitudeEndpoint;
setup.functions.continuous = @attitudeContinuous;

% 使用 SNOPT 求解器
setup.nlp.solver = 'snopt';

% 配置 SNOPT 选项
setup.nlp.snoptoptions = struct(...
    'Major_iteration_limit', 1000, ...
    'Minor_iteration_limit', 500, ...
    'Major_optimality_tolerance', 1e-4, ...
    'Major_feasibility_tolerance', 1e-6, ...
    'Superbasics_limit', 800, ...
    'Scale_option', 2, ... % 自动缩放
    'Print', 'yes' ...
);

setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';
setup.scales.method = 'automatic-bounds';
setup.method = 'RPMintegration';

% 网格设置
setup.mesh.method = 'hp-PattersonRao';
setup.mesh.tolerance = 1e-4;
setup.mesh.maxiterations = 5;
setup.mesh.colpointsmin = 4;
setup.mesh.colpointsmax = 10;
setup.mesh.phase.colpoints = 8*ones(1,3);
setup.mesh.phase.fraction = ones(1,3)/3;

% 计算最大角速度 (基于最大角动量和转动惯量)
w_max = hw_max ./ diag(I);

% 确保输入为列向量
s0 = s0(:);
w0 = w0(:);
sf = sf(:);
wf = wf(:);

% --- 去掉初始随机扰动，只使用初始容差 --- %
max_angle_deviation_rad = deg2rad(max_angle_deviation_deg);

% 状态边界
% 状态: [s1, s2, s3, w1, w2, w3, hw1, hw2, hw3]
state_min = [-ones(3,1); -w_max; -hw_max*ones(3,1)];
state_max = [ones(3,1); w_max; hw_max*ones(3,1)];

% 设置边界
% 标准状态边界
setup.bounds.phase.state.lower = state_min';
setup.bounds.phase.state.upper = state_max';

% 初始状态边界 
initial_tolerance_mrp = 0; % 数值容差
initial_tolerance_w = 0;       % 角速度容差
initial_tolerance_hw = 0;      % 飞轮角动量容差
s0 = s0 + max_angle_deviation_rad/4*randn(3,1);

% 修复维度问题：确保所有向量都是行向量
initial_state_lower = [s0; w0; zeros(3,1)]' - [initial_tolerance_mrp*ones(1,3), initial_tolerance_w*ones(1,3), initial_tolerance_hw*ones(1,3)];
initial_state_upper = [s0; w0; zeros(3,1)]' + [initial_tolerance_mrp*ones(1,3), initial_tolerance_w*ones(1,3), initial_tolerance_hw*ones(1,3)];

setup.bounds.phase.initialstate.lower = initial_state_lower;
setup.bounds.phase.initialstate.upper = initial_state_upper;

% 终端状态保持严格约束
final_state_lower = [sf; wf; zeros(3,1)]';
final_state_upper = [sf; wf; zeros(3,1)]';

setup.bounds.phase.finalstate.lower = final_state_lower;
setup.bounds.phase.finalstate.upper = final_state_upper;

% 控制边界 (飞轮力矩)
control_min = -torque_max * ones(3, 1);
control_max = torque_max * ones(3, 1);
setup.bounds.phase.control.lower = control_min';
setup.bounds.phase.control.upper = control_max';

% 积分约束
integral_min = -100;
integral_max = 100;
setup.bounds.phase.integral.lower = integral_min;
setup.bounds.phase.integral.upper = integral_max;

% 时间边界
setup.bounds.phase.initialtime.lower = t0;
setup.bounds.phase.initialtime.upper = t0;
setup.bounds.phase.finaltime.lower = tf;
setup.bounds.phase.finaltime.upper = tf;

% --- 初始猜测 --- %
num_points = 50;
time_guess = linspace(t0, tf, num_points)';

state_guess = zeros(num_points, 9);
% 状态猜测：从初始状态到目标状态
for i = 1:9
    if i <= 3
        % MRP部分：从 s0 线性插值到 sf
        state_guess(:, i) = linspace(s0(i), sf(i), num_points)';
    elseif i <= 6
        % 角速度部分：从 w0 线性插值到 wf
        state_guess(:, i) = linspace(w0(i-3), wf(i-3), num_points)';
    else
        % 飞轮角动量部分：从 0 到 0
        state_guess(:, i) = zeros(num_points, 1);
    end
end

control_guess = 0.1 * ones(num_points, 3); % 小值初始控制

setup.guess.phase.state = state_guess;
setup.guess.phase.control = control_guess;
setup.guess.phase.time = time_guess;
setup.guess.phase.integral = 0;

% 添加转动惯量矩阵作为辅助数据
setup.auxdata.I = I;

% Call Solver, GPOPS-II -------
%-----------------------------------
output = gpops2(setup);
%-----------------------------------

% 检查输出结构
if isstruct(output) && isfield(output, 'result')
    solution = output.result.solution;
    fprintf('优化成功完成!\n');
    
    % 提取解决方案
    time = solution.phase.time;
    state = solution.phase.state;
    control = solution.phase.control;
    
    % 验证初始状态
    fprintf('\n=== 初始状态验证 ===\n');
    initial_state_solution = state(1, :);
    desired_initial_state = [s0; w0; zeros(3,1)]';
    deviation = initial_state_solution - desired_initial_state;
    fprintf('初始MRP模长: %.6f\n', norm(initial_state_solution(1:3)));
    
    % --- 生成指定步长的姿态序列 --- %
    fprintf('\n=== 生成姿态序列 (步长: %.2f 秒) ===\n', sim_dt);
    attitude_sequence = generate_attitude_sequence(time, state, control, sim_dt, I, s0, w0);
    
    % 将MRP转换为四元数用于可视化（可选）
    attitude_sequence.quaternion = mrp2quat_array(attitude_sequence.MRP);
    
    % 保存姿态序列到文件
    save_attitude_sequence(attitude_sequence, 'attitude_sequence_rw.mat');
    fprintf('姿态序列已保存到 attitude_sequence_rw.mat\n');
    
else
    fprintf('优化失败或输出结构异常\n');
    if isstruct(output)
        fprintf('输出字段: %s\n', strjoin(fieldnames(output), ', '));
    end
    attitude_sequence = [];
end

end

%-------------------------------------------------------------------------%
function output = attitudeEndpoint(input)
    % 端点函数
    % 最小化控制努力 (积分控制量的平方)
    output.objective = input.phase.integral;
end

%-------------------------------------------------------------------------%
function output = attitudeContinuous(input)
    % 连续动力学函数（使用MRP和反作用飞轮）
    t = input.phase.time;
    x = input.phase.state; % [s1, s2, s3, w1, w2, w3, hw1, hw2, hw3]
    u = input.phase.control; % [u1, u2, u3] = -dot(hw)
    
    % 从辅助数据获取转动惯量
    I = input.auxdata.I;
    
    n = size(x, 1);
    s_dot = zeros(n, 3);
    w_dot = zeros(n, 3);
    hw_dot = zeros(n, 3);
    
    for i = 1:n
        s = x(i, 1:3)';
        w = x(i, 4:6)';
        hw = x(i, 7:9)';
        u_i = u(i, :)';
        
        % MRP动力学
        s_norm_sq = s' * s;
        B = 0.25 * ((1 - s_norm_sq) * eye(3) + 2 * skew(s) + 2 * (s * s'));
        s_dot(i, :) = (B * w)';
        
        % 角速度动力学（考虑飞轮角动量）
        % I * dot(w) + dot(hw) + w × (I*w + hw) = 0
        % 其中 dot(hw) = -u (因为 u = -dot(hw))
        % 所以: I * dot(w) = u - w × (I*w + hw)
        w_dot(i, :) = (I \ (u_i - cross(w, I*w + hw)))';
        
        % 飞轮角动量动力学
        hw_dot(i, :) = (-u_i)'; % u = -dot(hw)
    end
    
    % 动力学方程
    output.dynamics = [s_dot, w_dot, hw_dot];
    
    % 被积函数 (控制量的平方和)
    output.integrand = sum(u.^2, 2);
end

%-------------------------------------------------------------------------%
function S = skew(v)
    % 计算向量的斜对称矩阵
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end

%-------------------------------------------------------------------------%
function q_array = mrp2quat_array(MRP)
    % 将MRP数组转换为四元数数组
    n = size(MRP, 1);
    q_array = zeros(n, 4);
    
    for i = 1:n
        s = MRP(i, :)';
        s_norm_sq = s' * s;
        q_array(i, :) = [(1 - s_norm_sq); 2*s] / (1 + s_norm_sq);
        q_array(i, :) = q_array(i, :) / norm(q_array(i, :));
    end
end

function attitude_sequence = generate_attitude_sequence(time, state, control, sim_dt, I, s0, w0)
    % 生成指定步长的姿态序列
    % 输入:
    %   time - 原始时间序列
    %   state - 原始状态序列 [s1, s2, s3, w1, w2, w3, hw1, hw2, hw3]
    %   control - 原始控制序列 [u1, u2, u3]
    %   sim_dt - 期望的仿真步长
    %   I - 转动惯量矩阵
    %   s0 - 初始MRP
    %   w0 - 初始角速度
    % 输出:
    %   attitude_sequence - 结构体，包含时间、MRP、角速度、控制力矩和飞轮角动量序列
    
    % 创建新的时间序列
    new_time = (time(1):sim_dt:time(end))';
    
    % 确保包含终止时间
    if new_time(end) < time(end)
        new_time = [new_time; time(end)];
    end
    
    % 对控制力矩进行插值
    control_torque = zeros(length(new_time), 3);
    for i = 1:3
        control_torque(:, i) = interp1(time, control(:, i), new_time, 'pchip');
    end
    
    % 使用控制力矩进行动力学积分，得到姿态和角速度序列
    MRP = zeros(length(new_time), 3);
    angular_velocity = zeros(length(new_time), 3);
    wheel_momentum = zeros(length(new_time), 3);
    
    % 设置初始状态
    MRP(1, :) = s0';
    angular_velocity(1, :) = w0';
    wheel_momentum(1, :) = [0, 0, 0]; % 初始飞轮角动量为0
    
    % 使用四阶龙格-库塔法进行积分
    for i = 1:length(new_time)-1
        current_s = MRP(i, :)';
        current_w = angular_velocity(i, :)';
        current_hw = wheel_momentum(i, :)';
        current_u = control_torque(i, :)';
        
        % 计算k1
        k1_s = mrp_derivative(current_s, current_w);
        k1_w = angular_velocity_derivative_rw(current_w, current_hw, current_u, I);
        k1_hw = wheel_momentum_derivative(current_u);
        
        % 计算k2
        k2_s = mrp_derivative(current_s + 0.5 * sim_dt * k1_s, current_w + 0.5 * sim_dt * k1_w);
        k2_w = angular_velocity_derivative_rw(current_w + 0.5 * sim_dt * k1_w, current_hw + 0.5 * sim_dt * k1_hw, current_u, I);
        k2_hw = wheel_momentum_derivative(current_u);
        
        % 计算k3
        k3_s = mrp_derivative(current_s + 0.5 * sim_dt * k2_s, current_w + 0.5 * sim_dt * k2_w);
        k3_w = angular_velocity_derivative_rw(current_w + 0.5 * sim_dt * k2_w, current_hw + 0.5 * sim_dt * k2_hw, current_u, I);
        k3_hw = wheel_momentum_derivative(current_u);
        
        % 计算k4
        k4_s = mrp_derivative(current_s + sim_dt * k3_s, current_w + sim_dt * k3_w);
        k4_w = angular_velocity_derivative_rw(current_w + sim_dt * k3_w, current_hw + sim_dt * k3_hw, current_u, I);
        k4_hw = wheel_momentum_derivative(current_u);
        
        % 更新MRP、角速度和飞轮角动量
        next_s = current_s + (sim_dt / 6) * (k1_s + 2*k2_s + 2*k3_s + k4_s);
        next_w = current_w + (sim_dt / 6) * (k1_w + 2*k2_w + 2*k3_w + k4_w);
        next_hw = current_hw + (sim_dt / 6) * (k1_hw + 2*k2_hw + 2*k3_hw + k4_hw);
        
        MRP(i+1, :) = next_s';
        angular_velocity(i+1, :) = next_w';
        wheel_momentum(i+1, :) = next_hw';
    end
    
    % 创建输出结构体
    attitude_sequence.time = new_time;
    attitude_sequence.MRP = MRP;
    attitude_sequence.angular_velocity = angular_velocity;
    attitude_sequence.control_torque = control_torque;
    attitude_sequence.wheel_momentum = wheel_momentum;
end

%-------------------------------------------------------------------------%
function s_dot = mrp_derivative(s, w)
    % 计算MRP导数
    s_norm_sq = s' * s;
    B = 0.25 * ((1 - s_norm_sq) * eye(3) + 2 * skew(s) + 2 * (s * s'));
    s_dot = B * w;
end

%-------------------------------------------------------------------------%
function w_dot = angular_velocity_derivative_rw(w, hw, u, I)
    % 计算角速度导数（考虑飞轮）
    w_dot = I \ (u - cross(w, I*w + hw));
end

%-------------------------------------------------------------------------%
function hw_dot = wheel_momentum_derivative(u)
    % 计算飞轮角动量导数
    hw_dot = -u; % u = -dot(hw)
end

%-------------------------------------------------------------------------%
function save_attitude_sequence(attitude_sequence, filename)
    % 保存姿态序列到MAT文件
    time = attitude_sequence.time;
    MRP = attitude_sequence.MRP;
    angular_velocity = attitude_sequence.angular_velocity;
    control_torque = attitude_sequence.control_torque;
    wheel_momentum = attitude_sequence.wheel_momentum;
    
    % 检查是否有四元数字段
    if isfield(attitude_sequence, 'quaternion')
        quaternion = attitude_sequence.quaternion;
        save(filename, 'time', 'MRP', 'angular_velocity', 'control_torque', 'wheel_momentum', 'quaternion');
    else
        save(filename, 'time', 'MRP', 'angular_velocity', 'control_torque', 'wheel_momentum');
    end
end
