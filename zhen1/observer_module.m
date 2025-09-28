%% 观测器模块
% 包含预定义时间观测器和相关函数

function [dxhat_leader, observer_errors, observer_gains] = observer_module(xhat_leader, x_leader, t, A_t, B_t, observer_params)
    % 观测器模块主函数
    %
    % 输入:
    %   xhat_leader: 观测器状态 [2 x N]
    %   x_leader: 真实领导者状态 [2 x 1]
    %   t: 当前时间
    %   A_t: 当前邻接矩阵
    %   B_t: 当前领导者连接矩阵
    %   observer_params: 观测器参数
    %
    % 输出:
    %   dxhat_leader: 观测器状态导数 [2 x N]
    %   observer_errors: 观测误差 [2 x N]
    %   observer_gains: 观测器增益 [2 x N]
    
    N = size(xhat_leader, 2);
    dxhat_leader = zeros(2, N);
    observer_errors = zeros(2, N);
    observer_gains = zeros(2, N);
    
    % 更新观测器参数
    observer_params_current = observer_params;
    observer_params_current.A = A_t;
    observer_params_current.B = B_t;
    
    for i = 1:N
        % 计算单个观测器
        [dxhat_dt, consensus_error, e_i, H_i] = compute_prescribed_time_observer(i, t, xhat_leader(:, i), ...
                                                                          xhat_leader, x_leader, observer_params_current);
        dxhat_leader(:, i) = dxhat_dt;
        observer_gains(:, i) = H_i;
        
        % 计算观测误差（不使用绝对值）
        observer_errors(:, i) = x_leader - xhat_leader(:, i);
    end
end

function [dxhat_dt, consensus_error, e_i, H] = compute_prescribed_time_observer(i, t, xhat_i, neighbor_states, leader_state, observer_params)
    % 计算预定义时间观测器
    %
    % 输入:
    %   i: 观测器索引
    %   t: 当前时间
    %   xhat_i: 第i个观测器状态
    %   neighbor_states: 所有邻居状态
    %   leader_state: 领导者状态
    %   observer_params: 观测器参数
    %
    % 输出:
    %   dxhat_dt: 观测器状态导数
    %   consensus_error: 一致性误差
    %   e_i: 观测误差
    %   H: 观测器增益 [3x1]
    
    % 基本参数提取
    tp = observer_params.tp;
    te = observer_params.te;
    A = observer_params.A;
    B = observer_params.B;
    l_gains = observer_params.l_gains;
    alpha = observer_params.alpha;
    restart_time = observer_params.restart_time;
    
    % 多周期支持
    [effective_tp, effective_te] = get_effective_time_params(t, tp, te, restart_time);
    
    % 计算一致性误差
    e_i = compute_consensus_error(i, xhat_i, neighbor_states, leader_state, A, B);
    
    % 计算观测器增益
    H = compute_observer_gain(t, effective_tp, effective_te, l_gains, alpha, observer_params.gain_segments, i);
    
    % 观测器动态方程
    dxhat_dt = compute_observer_dynamics(xhat_i, H, e_i);
    
    % 一致性误差
    consensus_error = abs(e_i);
end

function [effective_tp, effective_te] = get_effective_time_params(t, tp, te, restart_time)
    % 获取有效的时间参数（支持多周期）
    if t >= restart_time
        effective_tp = restart_time;
        effective_te = te;
    else
        effective_tp = tp;
        effective_te = te;
    end
end

function e_i = compute_consensus_error(i, xhat_i, neighbor_states, leader_state, A, B)
    % 计算一致性观测误差
    e_i = 0;
    
    % 追随者-追随者项
    for k = 1:size(neighbor_states, 2)
        if A(i, k) > 0
            e_i = e_i + A(i, k) * (neighbor_states(1, k) - xhat_i(1));
        end
    end
    
    % 追随者-领导者项
    if length(B) >= i && B(i) > 0
        e_i = e_i + B(i) * (leader_state(1) - xhat_i(1));
    end
end

function H = compute_observer_gain(t, tp, te, l_gains, alpha, gain_segments, node_id)
    % 计算观测器增益（支持分段增益和节点特定增益）
    %
    % 输入:
    %   t: 当前时间
    %   tp: 预定义时间起点
    %   te: 预定义时间长度
    %   l_gains: 基础观测器增益
    %   alpha: 指数参数
    %   gain_segments: 分段增益配置
    %   node_id: 节点ID（用于节点特定增益）
    %
    % 输出:
    %   H: 观测器增益向量
    
    % 选择当前时间段对应的增益
    current_gains = select_gain_segment(t, gain_segments, node_id);
    
    l0 = 2;  % 传统观测器基数
    h = compute_k2_function(t, tp, te);
    tf = tp + te;
    
    % 确定时间窗口标志 - 使用硬切换
    if t >= tp && t <= tf
        Upsilon = 1;  % 在预定义时间窗口内
    else
        Upsilon = 0;  % 在预定义时间窗口外
    end
    
    % 计算增益向量
    H = zeros(2, 1);
    for j = 1:2
        prescribed_term = Upsilon * h^(j);
        traditional_term = (1 - Upsilon) * l0^(j);
        H(j) = (prescribed_term+traditional_term) * current_gains(j);
    end
    

end

function current_gains = select_gain_segment(t, gain_segments, node_id)
    % 根据时间和节点ID选择对应的增益段
    %
    % 输入:
    %   t: 当前时间
    %   gain_segments: 分段增益配置结构体
    %   node_id: 节点ID（用于节点特定增益）
    %
    % 输出:
    %   current_gains: 当前时间段对应的增益 [2x1]
    
    % 检查是否定义了分段增益
    if ~isfield(gain_segments, 'segments') || isempty(gain_segments.segments)
        % 如果没有定义分段，使用默认增益
        current_gains = gain_segments.default_gains;
        return;
    end
    
    % 遍历所有增益段，找到当前时间对应的段
    for i = 1:length(gain_segments.segments)
        segment = gain_segments.segments(i);
        
        % 检查是否在段内
        if t >= segment.start_time && t <= segment.end_time
            % 检查是否有节点特定增益
            if isfield(segment, 'node_specific_gains') && isfield(segment.node_specific_gains, sprintf('node_%d', node_id))
                % 使用节点特定增益
                current_gains = segment.node_specific_gains.(sprintf('node_%d', node_id));
                return;
            else
                % 使用通用增益
                current_gains = segment.gains;
                return;
            end
        end
    end
    
    % 如果没有找到对应的时间段，使用默认增益
    current_gains = gain_segments.default_gains;
end

function h = compute_k2_function(t, tp, te)
    % 计算K2类函数（发散函数）
    %
    % 输入:
    %   t: 当前时间
    %   tp: 预定义时间起点
    %   te: 预定义时间长度
    %
    % 输出:
    %   h: K2函数值
    
    tf = tp + te;
    
    if t <= tp
        h = 0;  % 预定义时间开始前
    elseif t < tf
        d = (tf - t) + 1e-1;  % 避免除零，添加小量
        h = 1 / d;             % 发散函数
    else
        h = 1;  % 预定义时间结束后的稳定值
    end
    

end

function dxhat_dt = compute_observer_dynamics(xhat_i, H, e_i)
    % 计算观测器动态方程
    %
    % 输入:
    %   xhat_i: 观测器状态
    %   H: 观测器增益
    %   e_i: 观测误差
    %
    % 输出:
    %   dxhat_dt: 观测器状态导数
    
    dxhat_dt = zeros(2, 1);
    dxhat_dt(1) = xhat_i(2) + H(1) * e_i;  % 角度估计导数
    dxhat_dt(2) = H(2) * e_i;               % 角速度估计导数
    
    % 数值保护
    max_derivative = 1000;
    dxhat_dt = sign(dxhat_dt) .* min(abs(dxhat_dt), max_derivative);
end


function performance = analyze_observer_performance(observer_errors, t)
    % 分析观测器性能
    %
    % 输入:
    %   observer_errors: 观测误差历史 [3 x N x T]
    %   t: 时间向量
    %
    % 输出:
    %   performance: 性能分析结果
    
    performance = struct();
    
    % 基本统计
    performance.max_error = max(observer_errors(:));
    performance.mean_error = mean(observer_errors(:));
    performance.std_error = std(observer_errors(:));
    
    % 收敛时间分析
    convergence_threshold = 0.1;
    performance.convergence_time = find_convergence_time(observer_errors, t, convergence_threshold);
    
    % 稳态误差
    if length(t) > 100
        steady_state_errors = observer_errors(:, :, end-99:end);
        performance.steady_state_error = mean(steady_state_errors(:));
    else
        performance.steady_state_error = performance.mean_error;
    end
end

function convergence_time = find_convergence_time(observer_errors, t, threshold)
    % 找到收敛时间
    max_errors_over_time = squeeze(max(max(observer_errors, [], 1), [], 2));
    converged_indices = find(max_errors_over_time < threshold);
    
    if ~isempty(converged_indices)
        convergence_time = t(converged_indices(1));
    else
        convergence_time = inf;  % 未收敛
    end
end

function observer_params = create_default_observer_params(A, B, restart_time)
    % 创建默认观测器参数
    observer_params = struct();
    observer_params.tp = 0.0;
    observer_params.te = 1.0;
    observer_params.A = A;
    observer_params.B = B;
    observer_params.l_gains = [48; 768];  % 默认增益
    observer_params.alpha = 1;
    observer_params.restart_time = restart_time;
    observer_params.reset_on_restart = false;
    observer_params.switch_to_traditional = false;
    
    % 分段增益配置
    observer_params.gain_segments = create_default_gain_segments();
end

function gain_segments = create_default_gain_segments()
    % 创建默认的分段增益配置
    gain_segments = struct();
    gain_segments.default_gains = [48; 768];  % 默认增益
    gain_segments.segments = [];  % 空的分段配置
    gain_segments.time_grid = [];  % 空的时间栅格
end



function gain_segments = create_custom_gain_segments(segment1_gains, segment2_gains, segment3_gains, segment4_gains, ...
                                                   segment1_times, segment2_times, segment3_times, segment4_times)
    % 创建用户自定义的四段增益配置
    %
    % 输入:
    %   segment1_gains: 第一段增益 [l1; l2; l3]
    %   segment2_gains: 第二段增益 [l1; l2; l3]
    %   segment3_gains: 第三段增益 [l1; l2; l3]
    %   segment4_gains: 第四段增益 [l1; l2; l3]
    %   segment1_times: 第一段时间 [start_time, end_time]
    %   segment2_times: 第二段时间 [start_time, end_time]
    %   segment3_times: 第三段时间 [start_time, end_time]
    %   segment4_times: 第四段时间 [start_time, end_time]
    %
    % 输出:
    %   gain_segments: 分段增益配置结构体
    
    gain_segments = struct();
    
    % 默认增益（当时间不在任何定义段时使用）
    gain_segments.default_gains = [48; 768];
    
    % 定义四个增益段
    gain_segments.segments = struct();
    
    % 第一段
    gain_segments.segments(1).start_time = segment1_times(1);
    gain_segments.segments(1).end_time = segment1_times(2);
    gain_segments.segments(1).gains = segment1_gains;
    gain_segments.segments(1).description = '用户自定义第一段';
    
    % 第二段
    gain_segments.segments(2).start_time = segment2_times(1);
    gain_segments.segments(2).end_time = segment2_times(2);
    gain_segments.segments(2).gains = segment2_gains;
    gain_segments.segments(2).description = '用户自定义第二段';
    
    % 第三段
    gain_segments.segments(3).start_time = segment3_times(1);
    gain_segments.segments(3).end_time = segment3_times(2);
    gain_segments.segments(3).gains = segment3_gains;
    gain_segments.segments(3).description = '用户自定义第三段';
    
    % 第四段
    gain_segments.segments(4).start_time = segment4_times(1);
    gain_segments.segments(4).end_time = segment4_times(2);
    gain_segments.segments(4).gains = segment4_gains;
    gain_segments.segments(4).description = '用户自定义第四段';
end

function gain_analysis = analyze_observer_gains(t, observer_params)
    % 分析观测器增益变化
    H = compute_observer_gain(t, observer_params.tp, observer_params.te, ...
                             observer_params.l_gains, observer_params.alpha, observer_params.gain_segments);
    
    gain_analysis = struct();
    gain_analysis.time = t;
    gain_analysis.gains = H;
    gain_analysis.max_gain = max(H);
    gain_analysis.gain_norm = norm(H);
    
    % 检查是否在发散区间
    tf = observer_params.tp + observer_params.te;
    gain_analysis.in_divergent_region = (t >= observer_params.tp && t <= tf);
end
