

function [dx_followers, u_followers] = follower_module(x_followers, xhat_leader, x_leader, t, A_t, follower_params, control_params, use_true_leader)

    N = size(x_followers, 2);
    dx_followers = zeros(2, N);  % 改为2维状态向量
    u_followers  = zeros(N, 1);

    for i = 1:N
        % 控制输入（无参考轨迹）
        u_followers(i) = compute_follower_control(i, t, x_followers(:, i), xhat_leader, ...
                                                 x_leader, A_t, control_params, use_true_leader);
        % 将当前时间写入参数（供 d(t) 使用）
        follower_params(i).t = t;

        % 动力学：倒立摆模型
        dx_followers(:, i) = compute_follower_dynamics(x_followers(:, i), u_followers(i), follower_params(i), t);
    end
end

function u_follower = compute_follower_control(i, t, x_follower, xhat_leader, x_leader, A_t, control_params, use_true_leader)
    % 控制律：u = coupling_gain * (K*(x_leader - x_follower)) + 一致性注入
    % 说明：追随者跟踪领导者轨迹，使用倒立摆动力学模型

    % 增益切换（根据节点索引和时间）
    [Kvec, coupling_gain, consensus_gain] = select_control_gains(i, t, control_params);
    K1 = Kvec(1); K2 = Kvec(2);  % 只需要2个增益对应2维状态

    % 确定参考状态（领导者状态）
    if use_true_leader
        reference = x_leader;
    else
        reference = xhat_leader(:, i);
    end

    % 跟踪控制：-K*(x_leader - x_follower) 负反馈控制
    p_i = x_follower(1); v_i = x_follower(2);  % 角度和角速度
    ref_p = reference(1); ref_v = reference(2);  % 参考角度和角速度
    
    tracking_error = [ref_p - p_i; ref_v - v_i];
    tracking_control = (K1*tracking_error(1) + K2*tracking_error(2));

    % 一致性项（保留原实现：基于 xhat_leader 的相对位置信息）
    consensus_control = compute_consensus_control(i, xhat_leader, A_t, consensus_gain);

    % 组合输入
    u_follower = coupling_gain * tracking_control + consensus_control;
end


function [gains, coupling_gain, consensus_gain] = select_control_gains(i, t, control_params)
    % 检查是否有分段增益配置
    if isfield(control_params, 'gain_segments') && ~isempty(control_params.gain_segments)
        % 使用分段增益
        [gains, coupling_gain, consensus_gain] = select_segmented_control_gains(i, t, control_params.gain_segments);
    else
        % 使用传统增益切换
        if t < control_params.gain_switch_time
            gains = control_params.follower_gains_1;
            coupling_gain = control_params.coupling_gain_1;
            consensus_gain = control_params.consensus_gain_1;
        else
            gains = control_params.follower_gains_2;
            coupling_gain = control_params.coupling_gain_2;
            consensus_gain = control_params.consensus_gain_2;
        end
    end
end

function [gains, coupling_gain, consensus_gain] = select_segmented_control_gains(i, t, gain_segments)
    % 根据节点索引和时间选择对应的分段控制增益
    %
    % 输入:
    %   i: 节点索引
    %   t: 当前时间
    %   gain_segments: 分段控制增益配置结构体
    %
    % 输出:
    %   gains: 追随者控制增益 [K1, K2]
    %   coupling_gain: 耦合增益
    %   consensus_gain: 一致性增益
    
    % 遍历所有增益段，找到当前时间对应的段
    for j = 1:length(gain_segments.segments)
        segment = gain_segments.segments(j);
        
        % 检查是否在段内
        if t >= segment.start_time && t <= segment.end_time
            % 检查是否有时变增益函数
            if isfield(segment, 'follower_gain_fun') && ~isempty(segment.follower_gain_fun)
                % 使用时变增益函数
                gains = segment.follower_gain_fun(t);
            else
                % 使用固定增益
                gains = segment.follower_gains;
            end
            
            % 检查是否有节点特定控制增益
            if isfield(segment, 'node_specific_control_gains') && isfield(segment.node_specific_control_gains, sprintf('node_%d', i))
                % 使用节点特定控制增益
                node_gains = segment.node_specific_control_gains.(sprintf('node_%d', i));
                gains = node_gains.follower_gains;
                coupling_gain = node_gains.coupling_gain;
                consensus_gain = node_gains.consensus_gain;
                return;
            end
            
            coupling_gain = segment.coupling_gain;
            consensus_gain = segment.consensus_gain;
            return;
        end
    end
    
    % 如果没有找到对应的时间段，使用最后一个段的增益
    if ~isempty(gain_segments.segments)
        last_segment = gain_segments.segments(end);
        gains = last_segment.follower_gains;
        coupling_gain = last_segment.coupling_gain;
        consensus_gain = last_segment.consensus_gain;
    else
        % 如果没有定义任何分段，使用零增益
        gains = [0, 0];
        coupling_gain = 0;
        consensus_gain = 0;
    end
end

function consensus_control = compute_consensus_control(i, xhat_leader, A_t, consensus_gain)
    % 与原实现一致：利用邻居的 leader 估计的相对"位置"形成一致性驱动
    N = size(xhat_leader, 2);
    consensus_control = 0;
    neighbor_count = 0;

    for j = 1:N
        if A_t(i, j) == 1
            neighbor_error = xhat_leader(1, j) - xhat_leader(1, i);
            consensus_control = consensus_control + neighbor_error;
            neighbor_count = neighbor_count + 1;
        end
    end

    if neighbor_count > 0
        consensus_control = consensus_gain * consensus_control / neighbor_count;
    end
end

function dx_follower = compute_follower_dynamics(x_follower, u_follower, params, t)
    % 倒立摆动力学模型（论文中的跟随者模型）
    % x = [p_i; v_i] = [角度; 角速度]
    % 动力学: p_i = v_i, v_i = I_i^(-1)(-m_i*g*l_i*sin(p_i) - C_i*v_i + h_i(t) + τ_i)
    
    dx_follower = zeros(2,1);

    p_i = x_follower(1);  % 角度
    v_i = x_follower(2); % 角速度

    % 系统参数（论文中的数值）
    I_i = params.I_i;    % 转动惯量
    m_i = params.m_i;    % 质量
    g = params.g;        % 重力加速度
    l_i = params.l_i;    % 摆长
    C_i = params.C_i;    % 阻尼系数

    % 扰动项 h_i(t) = 0.8*sin(0.2*i*t + 0.1*i*π)
    i = params.follower_index;  % 跟随者索引
    h_i = 0.8 * sin(0.2 * i * t + 0.1 * i * pi);

    % 控制输入 τ_i
    tau_i = u_follower;

    % 动力学方程
    dx_follower(1) = v_i;  % p_i = v_i
    dx_follower(2) = I_i^(-1) * (-m_i * g * l_i * sin(p_i) - C_i * v_i + h_i + tau_i);  % v_i = I_i^(-1)(-m_i*g*l_i*sin(p_i) - C_i*v_i + h_i(t) + τ_i)
end

function V_followers = distributed_follower_control(x_follower, neighbor_estimates, leader_estimate, gains, coupling_gain, consensus_gain, varargin)
    % 兼容接口：此函数未在本模块主流程中调用，保留以避免外部依赖报错
    p = inputParser;
    addParameter(p, 'use_true_leader', false, @islogical);
    addParameter(p, 'true_leader_state', [], @isnumeric);
    parse(p, varargin{:});

    % 跟踪控制 + 一致性（与 compute_follower_control 一致的思想）
    K1 = gains(1); K2 = gains(2); K3 = gains(3);
    x1 = x_follower(1); x2 = x_follower(2); x3 = x_follower(3);
    
    % 确定参考状态
    if p.Results.use_true_leader && ~isempty(p.Results.true_leader_state)
        reference = p.Results.true_leader_state;
    else
        reference = leader_estimate;
    end
    
    % 跟踪控制（负反馈）
    ref1 = reference(1); ref2 = reference(2); ref3 = reference(3);
    tracking_error = [ref1 - x1; ref2 - x2; ref3 - x3];
    tracking_control = (K1*tracking_error(1) + K2*tracking_error(2) + K3*tracking_error(3));

    consensus_control = 0;
    if size(neighbor_estimates, 2) > 0
        neighbor_avg = mean(neighbor_estimates(1, :));
        consensus_control = consensus_gain * (neighbor_avg - leader_estimate(1));
    end

    V_followers = coupling_gain * tracking_control + consensus_control;
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

function saturated_output = apply_saturation(input_signal, saturation_limit)
    % 饱和函数：将输入信号限制在 [-saturation_limit, +saturation_limit] 范围内
    %
    % 输入:
    %   input_signal: 待饱和处理的输入信号
    %   saturation_limit: 饱和限制值（正数）
    %
    % 输出:
    %   saturated_output: 经过饱和处理的输出信号
    
    if input_signal > saturation_limit
        saturated_output = saturation_limit;
    elseif input_signal < -saturation_limit
        saturated_output = -saturation_limit;
    else
        saturated_output = input_signal;
    end
end


