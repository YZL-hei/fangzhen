 
clear; close all; clc;

%% 添加模块路径
addpath(pwd);

N = 6;                           % 追随者数量
tspan = [0, 20];                % 仿真时间 (调整为20秒)
use_true_leader = false;         % 控制模式选择：false=使用观测器指导跟踪，true=使用真实领导者状态

% ODE求解器配置
observer_restart_time = 6.5;    % 观测器重启时间
observer_period = 1;            % 观测器周期

A = [
    0 0 1 1 0 1;  % 节点1的连接
    0 0 1 1 1 1;  % 节点2的连接
    1 1 0 1 0 1;  % 节点3的连接
    0 1 1 0 1 1;  % 节点4的连接
    0 1 1 1 0 1;  % 节点5的连接
    1 1 0 0 1 0   % 节点6的连接
];

% 恢复阶段使用的特殊拓扑（6.5-7.5秒）
A_recovery = [
    0 0 1 0 1 1;  % 节点1的连接
    0 0 1 1 1 1;  % 节点2的连接
    0 1 0 1 1 0;  % 节点3的连接
    0 1 1 0 1 1;  % 节点4的连接
    0 1 1 1 0 1;  % 节点5的连接
    0 1 0 1 1 0   % 节点6的连接
];

% 领导者连接矩阵 B (N x 1): B(i) = 1 表示节点i能接收领导者信息
B = [1; 1; 1; 1; 0; 1];  % 节点1,2,3,4,6能接收领导者信息

% 创建拓扑配置
topology_config = struct();
topology_config.type = 'custom';
topology_config.N = N;
topology_config.A_normal = A;
topology_config.A_recovery = A_recovery;  % 恢复阶段拓扑
topology_config.B_normal = B;





% 系统参数（使用各模块的默认参数函数）
leader_params = create_default_leader_params();
follower_params = create_default_follower_params(N);
trajectory_params = create_default_trajectory_params();
control_params = struct();
% 控制增益现在解释为跟踪控制增益 [K1,K2]（适用于倒立摆模型的跟踪控制）
control_params.leader_gains = [0.5, 0.3];  % 领导者状态反馈增益

% 创建分段控制器增益配置
control_params.gain_segments = create_custom_control_gain_segments();

% 保留传统参数以向后兼容
control_params.follower_gains_1 = [950.8, 290.5];  % 追随者跟踪控制增益（第一阶段）
control_params.follower_gains_2 = [950.8, 290.5];  % 追随者跟踪控制增益（第二阶段）
control_params.coupling_gain_1 = 1;
control_params.coupling_gain_2 = 1;
control_params.consensus_gain_1 = 1;
control_params.consensus_gain_2 = 1;
control_params.gain_switch_time = 8;  % 在8秒时切换控制增益 (调整为1段攻击)

% 观测器参数
observer_params = create_default_observer_params(A, B, observer_restart_time);

% 设置自定义分段观测器增益
observer_params.gain_segments = create_custom_observer_gain_segments();

% 攻击配置
attack_config = create_custom_attack_config_with_phases(N, ...
    'Y1_phase1_edges', [2, 4; 3, 5], ...       
    'Y1_phase2_edges', [2, 3; 4, 5; 6, 1], ...  
    'Y2_isolated_nodes', [1], ...              
    'Y2_target_edges', [2, 6; 3, 5], ...       
    'Y1_type', 'edge_disconnect', ...           % Y1攻击类型：边断联
    'Y2_type', 'mixed_isolation');              % Y2攻击类型：混合孤立攻击

fprintf('系统参数初始化完成\n');

%% 初始条件
fprintf('\n设置初始条件...\n');

% 领导者初始状态（论文中的设定）
x0_leader = [1; 0.5];  % [p_0(0); v_0(0)] = [2; 1]

% 追随者初始状态（扩展到6个跟随者）
follower_initial_states = [2, 1, 0, 1, 2, 1.5;  % 追随者1-6的初始角度 [p_1(0), p_2(0), p_3(0), p_4(0), p_5(0), p_6(0)]
                          1, 0.5, 0, 0.5, 1, 0.8]; % 追随者1-6的初始角速度 [v_1(0), v_2(0), v_3(0), v_4(0), v_5(0), v_6(0)]

% 观测器初始状态（扩展到6个跟随者）
observer_initial_states = [0.2,  0.4, 0.8,  0.1, 0.3,  0.2;   % 观测器1-6的初始角度估计
                          0.4, 0.8,  0.8, 0.2,  0.5, 0.1];  % 观测器1-6的初始角速度估计

% 组装完整初始状态向量
x0 = [x0_leader; follower_initial_states(:); observer_initial_states(:)];

fprintf('初始条件设置完成\n');
fprintf('  领导者初始状态: [%.2f, %.2f]\n', x0_leader(1), x0_leader(2));



%% 准备ODE系统参数
ode_params = struct();
ode_params.N = N;
ode_params.leader_params = leader_params;
ode_params.follower_params = follower_params;
ode_params.trajectory_params = trajectory_params;
ode_params.observer_params = observer_params;
ode_params.control_params = control_params;
ode_params.use_true_leader = use_true_leader;
ode_params.topology_config = topology_config;  % 添加拓扑配置
ode_params.attack_config = attack_config;      % 添加攻击配置

%% 执行系统性分段ODE求解
fprintf('\n开始系统性分段ODE求解...\n');
tic;

% 1) 收集所有切换时刻
T = [tspan(1), tspan(2)];
T = [T, control_params.gain_segments.time_grid];
T = [T, observer_params.gain_segments.time_grid];
T = [T, observer_restart_time];
T = [T, attack_config.time_grid];
T = unique(T);
T = T(T >= tspan(1) & T <= tspan(2));
T = sort(T);

fprintf('切换时刻栅格: %s\n', mat2str(T, 3));
fprintf('总段数: %d\n', length(T)-1);

% 2) 动态调整容差与步长
min_segment_length = min(diff(T));
max_step_constraint = min_segment_length / 20;  % 最小段长的1/20
fprintf('最小段长: %.3f, MaxStep约束: %.6f\n', min_segment_length, max_step_constraint);

% 改进的选项设置 - 矢量化AbsTol
% 为26维状态向量设置不同的绝对容差：角度、角速度分别设置
abstol_vec = repmat([1e-6; 1e-6], 13, 1);  % 13个2维子系统（1领导者+6追随者+6观测器）
options_adaptive = odeset('RelTol', 1e-3, 'AbsTol', abstol_vec, ...
                         'MaxStep', max_step_constraint);

% 3) 逐段积分，自动切换求解器
t_all = [];
x_all = [];
x0k = x0;
solver_switches = 0;

for k = 1:length(T)-1
    segment_interval = [T(k), T(k+1)];
    fprintf('段 %d/%d: [%.3f, %.3f] (长度: %.3f)\n', k, length(T)-1, segment_interval(1), segment_interval(2), diff(segment_interval));
    
    % 使用ode45求解器
    try
        [tk, xk] = ode45(@(t,x) modular_ode_system(t, x, ode_params), segment_interval, x0k, options_adaptive);
        solver_used = 'ode45';
    catch ME
        % 如果ode45失败，报告错误但不切换求解器
        fprintf('  ode45求解器失败: %s\n', ME.message);
        fprintf('  尝试使用更保守的ode45设置...\n');
        options_conservative = odeset('RelTol', 1e-4, 'AbsTol', abstol_vec, 'MaxStep', max_step_constraint/10);
        [tk, xk] = ode45(@(t,x) modular_ode_system(t, x, ode_params), segment_interval, x0k, options_conservative);
        solver_used = 'ode45 (conservative)';
        solver_switches = solver_switches + 1;
    end
    
    % 检查数值健康性
    if any(isnan(xk(:))) || any(isinf(xk(:)))
        error('段 %d 出现NaN或Inf，积分失败', k);
    end
    
    % 累积结果
    if isempty(t_all)
        t_all = tk;
        x_all = xk;
    else
        t_all = [t_all; tk(2:end)];
        x_all = [x_all; xk(2:end, :)];
    end
    
    % 准备下一段初值并检查连续性
    x0k_next = xk(end, :)';
    if k > 1
        continuity_error = norm(x0k - x_all(end-1, :)');
        if continuity_error > 1e-10
            fprintf('  警告：段际连续性误差 %.2e\n', continuity_error);
        end
    end
    x0k = x0k_next;
    
    fprintf('  完成 - 求解器: %s, 时间点数: %d\n', solver_used, length(tk));
end

solve_time = toc;
fprintf('\n系统性分段ODE求解完成！\n');
fprintf('总用时: %.2f 秒\n', solve_time);
fprintf('总时间点数: %d\n', length(t_all));
fprintf('求解器切换次数: %d\n', solver_switches);

%% 后处理和分析
fprintf('\n开始后处理...\n');

% 重构状态变量
t = t_all;
x_leader = x_all(:, 1:2)';
x_followers = reshape(x_all(:, 3:2+2*N)', 2, N, length(t));
xhat_leader = reshape(x_all(:, 3+2*N:end)', 2, N, length(t));

% 计算派生量
attack_modes = zeros(length(t), 1);
observer_errors = zeros(2*N, length(t));
observer_gains_history = zeros(2*N, length(t));  % 记录观测器增益历史

for k = 1:length(t)
    current_t = t(k);
    
    % 使用攻击网络模块
    [A_t, B_t, attack_mode, attack_info] = attack_network_module(current_t, N, topology_config, attack_config);
    attack_modes(k) = attack_mode;

    % 观测误差和增益计算
    xl = x_leader(:, k);
    xhat_l = xhat_leader(:, :, k);
    
    % 计算观测误差（不使用绝对值）
    for i = 1:N
        observer_errors(i, k) = xl(1) - xhat_l(1, i);           % 角度观测误差
        observer_errors(i+N, k) = xl(2) - xhat_l(2, i);         % 角速度观测误差  
    end
    
    % 计算当前时刻的观测器增益（一次性计算所有观测器）
    try
        [~, ~, observer_gains_current] = observer_module(xhat_l, xl, current_t, A_t, B_t, observer_params);
        
        % 存储增益数据
        for i = 1:N
            observer_gains_history(i, k) = observer_gains_current(1, i);           % 角度增益
            observer_gains_history(i+N, k) = observer_gains_current(2, i);         % 角速度增益
        end
    catch ME
        fprintf('警告：在时间 %.3f 计算观测器增益时出错: %s\n', current_t, ME.message);
        % 使用默认增益值
        for i = 1:N
            observer_gains_history(i, k) = observer_params.l_gains(1);
            observer_gains_history(i+N, k) = observer_params.l_gains(2);
        end
    end
end

fprintf('后处理完成\n');

%% 保存观测器增益数据
fprintf('\n保存观测器增益数据...\n');

% 创建增益数据结构
observer_gains_data = struct();
observer_gains_data.time = t;
observer_gains_data.gains = observer_gains_history;
observer_gains_data.N = N;

% 按追随者组织增益数据
for i = 1:N
    observer_gains_data.follower(i).position_gain = observer_gains_history(i, :);
    observer_gains_data.follower(i).velocity_gain = observer_gains_history(i+N, :);
end

% 保存到MAT文件
save('observer_gains_data.mat', 'observer_gains_data');
fprintf('观测器增益数据已保存到 observer_gains_data.mat\n');

% 保存为CSV文件（便于Excel查看）
csv_filename = 'observer_gains_data.csv';
fid = fopen(csv_filename, 'w');
if fid ~= -1
    % 写入表头
    header = 'Time';
    for i = 1:N
        header = [header, sprintf(',Follower%d_Position_Gain,Follower%d_Velocity_Gain', i, i)];
    end
    fprintf(fid, '%s\n', header);
    
    % 写入数据
    for k = 1:length(t)
        row = sprintf('%.6f', t(k));
        for i = 1:N
            row = [row, sprintf(',%.6f,%.6f', ...
                observer_gains_history(i, k), ...
                observer_gains_history(i+N, k))];
        end
        fprintf(fid, '%s\n', row);
    end
    fclose(fid);
    fprintf('观测器增益数据已保存到 %s\n', csv_filename);
else
    fprintf('警告：无法创建CSV文件\n');
end

%% 性能统计
fprintf('\n性能统计:\n');
fprintf('解算用时: %.2f 秒\n', solve_time);
fprintf('时间点数: %d\n', length(t));
fprintf('平均时间步长: %.6f\n', mean(diff(t)));

% 观测器性能
max_obs_error = max(abs(observer_errors(:)));
mean_obs_error = mean(abs(observer_errors(:)));
fprintf('最大观测误差: %.6f\n', max_obs_error);
fprintf('平均观测误差: %.6f\n', mean_obs_error);

% 跟踪性能（不使用范数，存储原始误差向量）
tracking_errors = zeros(2*N, length(t));  % 改为2*N以存储位置和速度误差
for i = 1:N
    for k = 1:length(t)
        error_vector = x_followers(:, i, k) - x_leader(:, k);
        tracking_errors(i, k) = error_vector(1);           % 位置跟踪误差
        tracking_errors(i+N, k) = error_vector(2);         % 速度跟踪误差
    end
end
max_tracking_error = max(abs(tracking_errors(:)));
mean_tracking_error = mean(abs(tracking_errors(:)));
fprintf('最大跟踪误差: %.6f\n', max_tracking_error);
fprintf('平均跟踪误差: %.6f\n', mean_tracking_error);

%% 可视化和分析结果
fprintf('\n生成分析图表...\n');
simple_plot_module(t, x_leader, x_followers, xhat_leader, observer_errors, ...
                   attack_modes, N);

fprintf('\n=== 仿真完成 ===\n');

%% ========================================================================
%% 模块化ODE系统函数
%% ========================================================================

function dxdt = modular_ode_system(t, x, params)
    % 模块化的ODE系统函数
    %
    % 输入:
    %   t: 当前时间
    %   x: 状态向量 [x_leader; x_followers; xhat_leader]
    %   params: 包含所有参数的结构体
    %
    % 输出:
    %   dxdt: 状态导数向量
    
    try
        % 参数提取
        N = params.N;
        leader_params = params.leader_params;
        follower_params = params.follower_params;
        trajectory_params = params.trajectory_params;
        observer_params = params.observer_params;
        control_params = params.control_params;
        use_true_leader = params.use_true_leader;
        
        % 验证状态向量维度
        expected_dim = 2 + 2*N + 2*N;  % 领导者(2) + 追随者(2*N) + 观测器(2*N)
        if length(x) ~= expected_dim
            error('状态向量维度不匹配：期望 %d，实际 %d', expected_dim, length(x));
        end
        
        % 状态向量分解
        x_leader = x(1:2);
        x_followers = reshape(x(3:2+2*N), 2, N);
        xhat_leader = reshape(x(3+2*N:end), 2, N);
        
        % 数值保护
        max_bound = 1000;
        if any(abs(x) > max_bound)
            x = sign(x) .* min(abs(x), max_bound);
            x_leader = x(1:2);
            x_followers = reshape(x(3:2+2*N), 2, N);
            xhat_leader = reshape(x(3+2*N:end), 2, N);
        end
        
        % 获取攻击场景（使用攻击网络模块）
        [A_t, B_t, ~, ~] = attack_network_module(t, N, params.topology_config, params.attack_config);
        
        % 领导者动力学（使用领导者模块）
        [dx_leader, ~, ~] = leader_module(x_leader, t, leader_params, trajectory_params, control_params);
        
    % 观测器动力学（使用观测器模块）
    [dxhat_leader, ~, observer_gains] = observer_module(xhat_leader, x_leader, t, A_t, B_t, observer_params);
        
        % 追随者动力学（使用追随者模块）
        [dx_followers, ~] = follower_module(x_followers, xhat_leader, x_leader, t, A_t, follower_params, control_params, use_true_leader);
        
        % 组装状态导数
        dxdt = [dx_leader; dx_followers(:); dxhat_leader(:)];
        
        % 验证输出维度
        if length(dxdt) ~= expected_dim
            error('状态导数维度不匹配：期望 %d，实际 %d', expected_dim, length(dxdt));
        end
        
        % 数值保护
        max_derivative = 1000;
        dxdt = sign(dxdt) .* min(abs(dxdt), max_derivative);
        
    catch ME
        fprintf('ODE系统函数错误：%s\n', ME.message);
        fprintf('时间: %.6f, 状态向量长度: %d\n', t, length(x));
        % 返回零导数以避免积分器崩溃
        dxdt = zeros(size(x));
    end
end


  

function leader_params = create_default_leader_params()
    % 创建默认的领导者参数（倒立摆模型）
    leader_params = struct();
    
    % 倒立摆系统参数（论文中的数值）
    leader_params.I_0 = 7;      % 转动惯量
    leader_params.m_0 = 4;      % 质量
    leader_params.g = 9.8;      % 重力加速度
    leader_params.l_0 = 1;      % 摆长
    leader_params.C_0 = 10;     % 阻尼系数
    
    % 运行时写入
    leader_params.t = 0.0;
end

function follower_params = create_default_follower_params(N)
    % 默认参数：倒立摆模型参数（论文中的数值）
    follower_params = struct();
    for i = 1:N
        % 倒立摆系统参数（论文中的数值）
        follower_params(i).I_i = 7;      % 转动惯量
        follower_params(i).m_i = 4;      % 质量
        follower_params(i).g = 9.8;      % 重力加速度
        follower_params(i).l_i = 1;      % 摆长
        follower_params(i).C_i = 10;     % 阻尼系数
        
        % 跟随者索引（用于扰动计算）
        follower_params(i).follower_index = i;
        
        % 饱和限制参数
        follower_params(i).saturation_limit = 50.0;  % 控制信号饱和限制

        % 运行时写入
        follower_params(i).t = 0.0;
    end
end


function trajectory_params = create_default_trajectory_params()
    % 创建默认的轨迹参数
    trajectory_params = struct();
    trajectory_params.frequency = 0.1;
    trajectory_params.amplitude = 0.5;
    trajectory_params.phase = 0;
    
    % 添加generate_reference函数需要的字段
    trajectory_params.a = 0.0;  % 起始位置
    trajectory_params.b = 2.0;  % 目标位置（论文中的设定）
    trajectory_params.T = 20.0;  % 过渡时间
end

function observer_params = create_default_observer_params(A, B, restart_time)
    % 创建默认观测器参数
    observer_params = struct();
    observer_params.tp = 0.0;
    observer_params.te = 1.0;
    observer_params.A = A;
    observer_params.B = B;
    observer_params.l_gains = [48; 768];
    observer_params.alpha = 1;
    observer_params.restart_time = restart_time;
    
    % 创建默认的分段增益配置
    observer_params.gain_segments = create_default_gain_segments();
end



%% 攻击配置函数

function attack_config = create_custom_attack_config_with_phases(N, varargin)
    % 创建攻击配置
    
    % 解析参数
    p = inputParser;
    addParameter(p, 'Y1_phase1_edges', [], @isnumeric);
    addParameter(p, 'Y1_phase2_edges', [], @isnumeric);
    addParameter(p, 'Y2_isolated_nodes', [], @isnumeric);
    addParameter(p, 'Y2_target_edges', [], @isnumeric);
    addParameter(p, 'Y1_type', 'edge_disconnect', @ischar);
    addParameter(p, 'Y2_type', 'mixed_isolation', @ischar);
    parse(p, varargin{:});
    
    % 创建攻击配置
    attack_config = struct();
    
    % 时间表 (调整为20秒仿真时间，1段攻击)
    attack_config.time_schedule = struct();
    attack_config.time_schedule.Y2_phase = struct('start', 5, 'end', 6.5, 'type', 'Y2');
    attack_config.time_schedule.recovery_phase = struct('start', 6.5, 'end', 7.5, 'type', 'recovery');
    attack_config.time_schedule.Y1_phase1 = struct('start', 12, 'end', 16, 'type', 'Y1');
    
    % Y1攻击配置
    attack_config.Y1 = struct();
    attack_config.Y1.phase1_edges = p.Results.Y1_phase1_edges;
    attack_config.Y1.target_edges = p.Results.Y1_phase1_edges;
    attack_config.Y1.attack_type = p.Results.Y1_type;
    attack_config.Y1.description = 'Y1 Attack';
    attack_config.Y1.severity = 'Medium';
    
    % Y2攻击配置
    attack_config.Y2 = struct();
    attack_config.Y2.isolated_nodes = p.Results.Y2_isolated_nodes;
    attack_config.Y2.target_edges = p.Results.Y2_target_edges;
    attack_config.Y2.attack_type = p.Results.Y2_type;
    attack_config.Y2.description = 'Y2 Attack';
    attack_config.Y2.severity = 'High';
    
    % 导出时间栅格用于分段积分
    attack_config.time_grid = [];
    time_phases = fieldnames(attack_config.time_schedule);
    for i = 1:length(time_phases)
        phase = attack_config.time_schedule.(time_phases{i});
        attack_config.time_grid = [attack_config.time_grid, phase.start, phase.end];
    end

end

function gain_segments = create_default_gain_segments()

    gain_segments = struct();
    gain_segments.segments = [];  % 空的分段配置
end

function gain_segments = create_custom_observer_gain_segments()
    % 创建自定义分段观测器增益配置
    %
    % 输出:
    %   gain_segments: 分段观测器增益配置结构体
    
    gain_segments = struct();
    
    % 默认增益（用于未定义时间段）
    gain_segments.default_gains = [48; 768];
    
    % 定义分段增益配置
    % 每个分段包含：开始时间、结束时间、增益值
    segments = [];
    
    % 第一阶段：0-5秒 - 初始化阶段，使用较低增益
    segments(1).start_time = 0;
    segments(1).end_time = 1;
    segments(1).gains = [12; 192];  % 较低增益，平稳收敛
    segments(1).description = '初始化阶段';
    
    % 第二阶段：5-6.5秒 - Y2攻击期间，提高增益
    segments(2).start_time = 1;
    segments(2).end_time = 5;
    segments(2).gains = [48; 768];  % 提高增益以应对Y2攻击
    segments(2).description = 'Y2攻击期间';
    
    % 第三阶段：6.5-7.5秒 - 恢复阶段，节点1使用特殊增益
    segments(3).start_time = 6.5;
    segments(3).end_time = 7.5;
    segments(3).gains = [48; 768];  % 其他节点的标准增益
    segments(3).description = '恢复阶段';
    
    % 为节点1在恢复阶段设置特殊增益
    segments(3).node_specific_gains.node_1 = [1;1];  % 节点1使用更高的增益
    
    % 第四阶段：7.5-8秒 - 正常运行阶段
    segments(4).start_time = 7.5;
    segments(4).end_time = 8;
    segments(4).gains = [48; 768];  % 标准增益
    segments(4).description = '正常运行阶段';
    
    % 第五阶段：8-12秒 - Y1攻击期间，显著提高增益
    segments(5).start_time = 8;
    segments(5).end_time = 12;
    segments(5).gains = [48; 768];  % 高增益以应对Y1攻击
    segments(5).description = 'Y1攻击期间';
    
    % 第六阶段：12-20秒 - 攻击后恢复阶段
    segments(6).start_time = 12;
    segments(6).end_time = 20;
    segments(6).gains = [48; 768];  % 中等增益帮助恢复
    segments(6).description = '攻击后恢复阶段';
    
    gain_segments.segments = segments;
    
    % 导出时间栅格用于分段积分
    gain_segments.time_grid = [];
    for i = 1:length(segments)
        gain_segments.time_grid = [gain_segments.time_grid, segments(i).start_time, segments(i).end_time];
    end
    gain_segments.time_grid = unique(gain_segments.time_grid);
    
    % 添加调试信息
    gain_segments.num_segments = length(segments);
    gain_segments.total_time_span = [0, 20];
    gain_segments.description = '自定义分段观测器增益配置';
end

function control_gain_segments = create_custom_control_gain_segments()
    % 创建自定义分段控制器增益配置
    control_gain_segments = struct();
    control_gain_segments.default_follower_gains = [958, 295];
    control_gain_segments.default_coupling_gain = 1.0;
    control_gain_segments.default_consensus_gain = 1.0;

    segments = repmat(struct(), 1, 7);

    % 第一段：0-1秒 - 控制增益为0
    segments(1).start_time = 0;
    segments(1).end_time = 1;
    segments(1).follower_gains = [0, 0];
    segments(1).coupling_gain = 0;
    segments(1).consensus_gain = 0;
    segments(1).description = '第一段：0-1秒 - 控制增益为0';

    % 第二段：1-2.5秒 - 初始化阶段，增益线性上升至目标值
    segments(2).start_time = 1;
    segments(2).end_time = 2.5;
    segments(2).follower_gain_fun = @(time) ramp_follower_gains(time, [1, 2], [958, 295]);
    segments(2).coupling_gain = 1;
    segments(2).consensus_gain = 1;
    segments(2).description = '第二段：1-2.5秒 - 线性增益上升';

    % 第三段：2.5-6.5秒 - Y2攻击期间，提高增益
    segments(3).start_time = 2.5;
    segments(3).end_time = 6.5;
    segments(3).follower_gains = [958, 295];
    segments(3).coupling_gain = 1;
    segments(3).consensus_gain = 1;
    segments(3).description = '第三段：2.5-6.5秒 - Y2攻击期间';

    % 第四段：6.5-7.5秒 - 恢复阶段，节点1使用特殊控制增益
    segments(4).start_time = 6.5;
    segments(4).end_time = 7.5;
    segments(4).follower_gains = [280, 120];
    segments(4).coupling_gain = 1;
    segments(4).consensus_gain = 1;
    segments(4).description = '第四段：6.5-7.5秒 - 恢复阶段';
    segments(4).node_specific_control_gains.node_1.follower_gains = [280, 120];
    segments(4).node_specific_control_gains.node_1.coupling_gain = 1;
    segments(4).node_specific_control_gains.node_1.consensus_gain = 0;

    % 第五段：7.5-10.5秒 - 正常运行阶段
    segments(5).start_time = 7.5;
    segments(5).end_time = 10.5;
    segments(5).follower_gains = [958, 295];
    segments(5).coupling_gain = 1;
    segments(5).consensus_gain = 1;
    segments(5).description = '第五段：7.5-10.5秒 - 正常运行阶段';

    % 第六段：10.5-12秒 - Y1攻击期间，保持高增益
    segments(6).start_time = 10.5;
    segments(6).end_time = 12;
    segments(6).follower_gains = [958, 295];
    segments(6).coupling_gain = 1;
    segments(6).consensus_gain = 1;
    segments(6).description = '第六段：10.5-12秒 - Y1攻击期间';

    % 第七段：12-20秒 - 最终稳定阶段
    segments(7).start_time = 12;
    segments(7).end_time = 20;
    segments(7).follower_gains = [958, 290];
    segments(7).coupling_gain = 1.0;
    segments(7).consensus_gain = 1.0;
    segments(7).description = '第七段：12-20秒 - 最终稳定阶段';

    control_gain_segments.segments = segments;

    % 导出时间栅格用于分段积分
    time_points = arrayfun(@(s) [s.start_time, s.end_time], segments, 'UniformOutput', false);
    control_gain_segments.time_grid = unique([time_points{:}]);

    % 调试信息
    control_gain_segments.num_segments = numel(segments);
    control_gain_segments.total_time_span = [0, 20];
    control_gain_segments.description = '自定义分段控制器增益配置';
end

function gains = ramp_follower_gains(t, interval, target_gains)
    % 在线性区间内从零增益平滑过渡到目标增益
    if t <= interval(1)
        alpha = 0;
    elseif t >= interval(2)
        alpha = 1;
    else
        alpha = (t - interval(1)) / (interval(2) - interval(1));
    end

    gains = alpha * target_gains;
end
