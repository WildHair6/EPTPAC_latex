clear
clc
euler0 = [30; -60; 30];
eulerf = [0; 0; 0];
% mrp0 = euler2mrp(deg2rad(euler0));
% mrpf = euler2mrp(deg2rad(eulerf));
mrp0 = [0.2, 0.3, -0.3];
mrpf = [0 0 0]';
% q0 = mrp2quat(mrp0)';
% qf = mrp2quat(mrpf)';
% mrp0 = [0.3;0.2;-0.4];
% mrpf = [0;0;0];

w0 = [0; 0; 0]; % 初始角速度 (rad/s)
wf = [0; 0; 0]; % 目标角速度 (rad/s)

I_B = diag([200, 250, 300]); % 转动惯量矩阵 (kg*m^2)
I_B_REAL = I_B*1.05;
torque_max = 0.2*0.8; % 最大力矩 (N*m)
ang_momentum_max = 4*0.9; % 最大角动量 (kg*m^2/s)
max_angle_deviation_deg = 0.00000;

t0 = 0; % 初始时间
% tf = 150; % 终止时间
% t_total = 300;
% tf = 300; % 终止时间
% t_total = 400;
tf = 300; % 终止时间
t_total = 500;
sim_dt = 0.01; % 仿真步长
eps1 = 1e-4;
lambda_max_J = max(eig(I_B));

[attitude_sequence, output] = Gauss_opt_mrp(mrp0, w0, mrpf, wf, I_B, torque_max, ...
    ang_momentum_max, t0, tf, max_angle_deviation_deg, sim_dt);


% [attitude_sequence, outtput] = Gauss_opt_time(mrp0, w0, mrpf, wf, I_B, torque_max, ang_momentum_max, ...
%     t0, tf, max_angle_deviation_deg, sim_dt) ;

% [attitude_sequence, output] = Gauss_opt_quat(q0, w0, qf, wf, I_B, ...
%                                 torque_max, ang_momentum_max, t0, tf, max_angle_deviation_deg,sim_dt);

MRP_SEQ = attitude_sequence.MRP;
MRP_REAL0 = MRP_SEQ(1,:);
MRP_REAL0 = MRP_REAL0';
Omega_SEQ = attitude_sequence.angular_velocity;
% dOmega_SEQ = attitude_sequence.alpha;
Tau_SEQ = attitude_sequence.control_torque;
HW_SEQ = attitude_sequence.wheel_momentum;

% %% ================== 第二次优化（末段修正） ==================
% t_refine_start = 5;     % 末段开始时间
% t_refine_horizon = tf - t_refine_start;    % 末段时长 (可以改成参数)
% 
% mrp_refine0 = MRP_SEQ(t_refine_start*100,:);   % 注意：索引 = 时间 / dt
% omega_refine0 = Omega_SEQ(t_refine_start*100,:);
% 
% [attitude_sequence2, output2] = Gauss_opt_mrp(mrp_refine0, omega_refine0, ...
%     mrpf, wf, I_B, torque_max, ang_momentum_max, ...
%     0, t_refine_horizon+sim_dt ...
%     , max_angle_deviation_deg, sim_dt);
% 
% % 前段取到 110s 之前，不包含拼接点
% MRP_SEQ  = [MRP_SEQ(1:(t_refine_start*100-1),:); attitude_sequence2.sigma];
% MRP_REAL0 = MRP_SEQ(1,:);
% MRP_REAL0 = MRP_REAL0';
% Omega_SEQ = [Omega_SEQ(1:(t_refine_start*100-1),:); attitude_sequence2.omega];
% Tau_SEQ   = [Tau_SEQ(1:(t_refine_start*100-1),:); attitude_sequence2.control_torque];

% 时间：前段到 110s 前一刻，后段自己带起点 0，要平移

T_extend = t_total - tf;   % 延长时间（秒），可按需要修改，比如 30s

% 原本优化序列时间（假设 attitude_sequence.time 是从 t0 到 tf）
time_opt = attitude_sequence.time(:);   % 列向量
% 如果你更信任 t0:sim_dt:tf，也可以直接用这个：
% time_opt = (t0:sim_dt:tf).';

% 延长时间段，从 tf+dt 一直到 tf+T_extend
time_ext = (time_opt(end) + sim_dt : sim_dt : time_opt(end) + T_extend).';

n_ext = length(time_ext);

% 末态（优化末点）的 MRP、角速度、飞轮动量等
MRP_last   = MRP_SEQ(end,:);      % 1x3
Omega_last = zeros(1,3);    % 1x3
Tau_last   = zeros(1,3);      % 1x3（理论上应接近 0）
% dOmega_last = zeros(1,3);  % 1x3（理论上应接近 0）
HW_last = HW_SEQ(end,:);

% 延长段设定：
% - 姿态、角速度：保持末态不变
% - 控制力矩、角加速度：为了“稳定”演示，这里设为 0
MRP_EXT    = repmat(MRP_last,   n_ext, 1);
Omega_EXT  = repmat(Omega_last, n_ext, 1);
Tau_EXT    = zeros(n_ext, 3);          % 末态保持，不再施加控制
% dOmega_EXT = zeros(n_ext, 3);          % 角加速度为 0
HW_EXT     = repmat(HW_last, n_ext, 1);

% 拼接优化段 + 延长段
time_total   = [time_opt;  time_ext];
MRP_SEQ    = [MRP_SEQ;   MRP_EXT];
Omega_SEQ  = [Omega_SEQ; Omega_EXT];
Tau_SEQ    = [Tau_SEQ;   Tau_EXT];
% dOmega_SEQ = [dOmega_SEQ; dOmega_EXT];
HW_SEQ     = [HW_SEQ; HW_EXT];



% figure;
% 
% % 绘制 MRP_total
% subplot(3,1,1);
% plot(time_total, MRP_SEQ);
% xlabel('Time (s)');
% ylabel('MRP');
% title('Modified Rodrigues Parameters (MRP)');
% legend('MRP_x', 'MRP_y', 'MRP_z');
% xlim([time_total(1), time_total(end)]);
% grid on;
% 
% % 绘制 Omega_total
% subplot(3,1,2);
% plot(time_total, Omega_SEQ);
% xlabel('Time (s)');
% ylabel('Angular Velocity (rad/s)');
% title('Angular Velocity (Omega)');
% legend('\omega_x', '\omega_y', '\omega_z');
% xlim([time_total(1), time_total(end)]);
% grid on;
% 
% % 绘制 Tau_total
% subplot(3,1,3);
% plot(time_total, Tau_SEQ);
% xlabel('Time (s)');
% ylabel('Control Torque (Nm)');
% title('Control Torque (Tau)');
% legend('\tau_x', '\tau_y', '\tau_z');
% xlim([time_total(1), time_total(end)]);
% grid on;



function mrp = euler2mrp(euler_angles)

    roll = euler_angles(1);
    pitch = euler_angles(2);
    yaw = euler_angles(3);
    
 
    R_x = [1, 0, 0;
           0, cos(roll), -sin(roll);
           0, sin(roll), cos(roll)];
       
    R_y = [cos(pitch), 0, sin(pitch);
           0, 1, 0;
           -sin(pitch), 0, cos(pitch)];
       
    R_z = [cos(yaw), -sin(yaw), 0;
           sin(yaw), cos(yaw), 0;
           0, 0, 1];
       
    R = R_z * R_y * R_x; % ZYX椤哄簭
    
    % 浠庢棆杞煩闃佃绠桵RP
    traceR = trace(R);
    denominator = 1 + traceR + 2*sqrt(1 + traceR);
    
    if denominator > 1e-10 % 閬垮厤闄や互闆?
        mrp = [R(2,3)-R(3,2); 
               R(3,1)-R(1,3); 
               R(1,2)-R(2,1)] / denominator;
    else
        k = sqrt((R(1,1)+1)/2);
        mrp = [k; 
               (R(1,2)+R(2,1))/(4*k); 
               (R(1,3)+R(3,1))/(4*k)];
    end
end

function q = mrp2quat(MRP)
% 将修正罗德里格斯参数(MRP)转换为四元数
% 输入:
%   MRP - 修正罗德里格斯参数 (N×3矩阵，每行是一个MRP向量 [σ1, σ2, σ3])
% 输出:
%   q - 四元数 (N×4矩阵，每行是一个四元数 [q0, q1, q2, q3])
%
% 转换公式:
%   q0 = (1 - |σ|?) / (1 + |σ|?)
%   q_vec = 2σ / (1 + |σ|?)

    % 确保输入是矩阵形式
    if isvector(MRP)
        MRP = MRP(:)'; % 转换为行向量
        if length(MRP) ~= 3
            error('MRP must be a 3-element vector or N×3 matrix');
        end
    end
    
    % 计算MRP的模长平方
    sigma_squared = sum(MRP.^2, 2);
    
    % 计算分母 (1 + |σ|?)
    denominator = 1 + sigma_squared;
    
    % 计算四元数标量部分
    q0 = (1 - sigma_squared) ./ denominator;
    
    % 计算四元数向量部分
    q_vec = 2 * MRP ./ denominator;
    
    % 组合成四元数 [q0, q1, q2, q3]
    q = [q0, q_vec];
    
    % 确保四元数是单位四元数
    q = q ./ vecnorm(q, 2, 2);
end
