%% 攻击模式模块
% 包含攻击场景和拓扑变化

function [A_t, B_t, attack_mode, attack_info] = attack_network_module(t, N, topology_config, attack_config)
    % 攻击模块主函数
    %
    % 输入:
    %   t: 当前时间
    %   N: 追随者数量
    %   topology_config: 网络拓扑配置
    %   attack_config: 攻击配置 (可选)
    %
    % 输出:
    %   A_t: 当前时刻的邻接矩阵
    %   B_t: 当前时刻的领导者连接矩阵
    %   attack_mode: 攻击模式
    %   attack_info: 攻击信息结构体
    
    % 将拓扑配置添加到攻击配置中，以便恢复阶段使用
    attack_config.topology_config = topology_config;
    
    % 获取攻击场景
    [attack_mode, attack_info] = get_attack_scenario(t, attack_config);
    
    % 生成受攻击的网络拓扑
    A_t = get_attacked_topology(attack_mode, attack_info, topology_config, attack_config);
    
    % 领导者连接矩阵（根据攻击类型修改）
    B_t = get_attacked_leader_connections(attack_mode, attack_info, topology_config, attack_config);
end


function [attack_mode, attack_info] = get_attack_scenario(t, attack_config)
    % 获取当前时刻的攻击场景
    %
    % 输入:
    %   t: 当前时间
    %   attack_config: 攻击配置
    %
    % 输出:
    %   attack_mode: 攻击模式 (0=正常, 1=Y1攻击, 2=Y2攻击, 3=恢复模式)
    %   attack_info: 攻击信息结构体
    
    attack_info = struct();
    
    % 安全检查：确保attack_config存在且包含必要字段
    if isempty(attack_config) || ~isfield(attack_config, 'time_schedule')
        % 如果没有攻击配置，返回正常状态
        attack_mode = 0;
        attack_info.description = '正常状态';
        attack_info.severity = 'None';
        attack_info.target_edges = [];
        attack_info.isolated_nodes = [];
        attack_info.attack_type = 'none';
        attack_info.time = t;
        attack_info.mode = attack_mode;
        attack_info.connectivity_score = 1.0;
        return;
    end
    
    % 检查Y1攻击时间段
    Y1_active = false;
    Y1_phase = 0;  % 0=无攻击, 1=第一阶段, 2=第二阶段
    if isfield(attack_config.time_schedule, 'Y1_phase1') && ...
       t >= attack_config.time_schedule.Y1_phase1.start && t < attack_config.time_schedule.Y1_phase1.end
        Y1_active = true;
        Y1_phase = 1;
    elseif isfield(attack_config.time_schedule, 'Y1_phase2') && ...
           t >= attack_config.time_schedule.Y1_phase2.start && t <= attack_config.time_schedule.Y1_phase2.end
        Y1_active = true;
        Y1_phase = 2;
    end
    
    % 检查Y2攻击时间段
    Y2_active = false;
    if isfield(attack_config.time_schedule, 'Y2_phase') && ...
       t >= attack_config.time_schedule.Y2_phase.start && t < attack_config.time_schedule.Y2_phase.end
        Y2_active = true;
    end
    
    % 检查攻击后恢复时间段
    recovery_active = false;
    if isfield(attack_config.time_schedule, 'recovery_phase') && ...
       t >= attack_config.time_schedule.recovery_phase.start && t < attack_config.time_schedule.recovery_phase.end
        recovery_active = true;
    end
    
    if recovery_active
        % 攻击后恢复模式
        attack_mode = 3;
        attack_info.description = '攻击后恢复阶段';
        attack_info.severity = 'Recovery';
        attack_info.attack_type = 'recovery';
        attack_info.recovery_progress = (t - attack_config.time_schedule.recovery_phase.start) / ...
                                       (attack_config.time_schedule.recovery_phase.end - attack_config.time_schedule.recovery_phase.start);
        
    elseif Y1_active
        % Y1攻击
        attack_mode = 1;
        attack_info.description = attack_config.Y1.description;
        attack_info.severity = attack_config.Y1.severity;
        attack_info.attack_type = attack_config.Y1.attack_type;
        attack_info.phase = Y1_phase;
        
        % 根据攻击阶段选择目标边
        if Y1_phase == 1
            % 第一阶段攻击
            if isfield(attack_config.Y1, 'phase1_edges') && ~isempty(attack_config.Y1.phase1_edges)
                attack_info.target_edges = attack_config.Y1.phase1_edges;
            else
                attack_info.target_edges = attack_config.Y1.target_edges;  % 向后兼容
            end
        elseif Y1_phase == 2
            % 第二阶段攻击
            if isfield(attack_config.Y1, 'phase2_edges') && ~isempty(attack_config.Y1.phase2_edges)
                attack_info.target_edges = attack_config.Y1.phase2_edges;
            else
                attack_info.target_edges = attack_config.Y1.target_edges;  % 向后兼容
            end
        end
        
    elseif Y2_active
        % Y2攻击
        attack_mode = 2;
        attack_info.description = attack_config.Y2.description;
        attack_info.severity = attack_config.Y2.severity;
        attack_info.isolated_nodes = attack_config.Y2.isolated_nodes;
        attack_info.attack_type = attack_config.Y2.attack_type;
        
        % 添加Y2攻击的额外边攻击信息
        if isfield(attack_config.Y2, 'target_edges') && ~isempty(attack_config.Y2.target_edges)
            attack_info.target_edges = attack_config.Y2.target_edges;
        else
            attack_info.target_edges = [];
        end
        
    else
        % 正常状态
        attack_mode = 0;
        attack_info.description = '正常状态';
        attack_info.severity = 'None';
        attack_info.target_edges = [];
        attack_info.isolated_nodes = [];
        attack_info.attack_type = 'none';
    end
    
    % 添加时间信息
    attack_info.time = t;
    attack_info.mode = attack_mode;
    
    % 计算连通性分数
    attack_info.connectivity_score = calculate_connectivity_score(attack_mode, attack_info);
end

function A_attacked = get_attacked_topology(attack_mode, attack_info, topology_config, attack_config)
    % 获取受攻击后的网络拓扑
    %
    % 输入:
    %   attack_mode: 攻击模式
    %   attack_info: 攻击信息
    %   topology_config: 拓扑配置
    %   attack_config: 攻击配置
    %
    % 输出:
    %   A_attacked: 受攻击后的邻接矩阵
    
    % 获取正常网络拓扑
    A_normal = topology_config.A_normal;
    
    if attack_mode == 0
        % 正常状态，无攻击
        A_attacked = A_normal;
        
    elseif attack_mode == 1
        % Y1攻击
        A_attacked = apply_Y1_attack(A_normal, attack_info, attack_config);
        
    elseif attack_mode == 2
        % Y2攻击
        A_attacked = apply_Y2_attack(A_normal, attack_info, attack_config);
        
    elseif attack_mode == 3
        % 攻击后恢复模式
        A_attacked = apply_recovery_attack(A_normal, attack_info, attack_config);
        
    else
        % 未知攻击模式，使用正常拓扑
        A_attacked = A_normal;
    end
    
    % 确保对角线为0
    A_attacked = A_attacked - diag(diag(A_attacked));
end

function B_attacked = get_attacked_leader_connections(attack_mode, attack_info, topology_config, attack_config)
    % 获取受攻击后的领导者连接矩阵
    %
    % 输入:
    %   attack_mode: 攻击模式
    %   attack_info: 攻击信息
    %   topology_config: 拓扑配置
    %   attack_config: 攻击配置
    %
    % 输出:
    %   B_attacked: 受攻击后的领导者连接矩阵
    
    % 获取正常领导者连接矩阵
    B_normal = topology_config.B_normal;
    
    % 检查特殊时间段：18-19秒，节点1不接收观测器信息
    current_time = attack_info.time;
    if current_time >= 18 && current_time < 19
        B_attacked = B_normal;
        B_attacked(1) = 0;  % 节点1断开领导者连接
        return;
    end
    
    if attack_mode == 0
        % 正常状态，无攻击
        B_attacked = B_normal;
        
    elseif attack_mode == 1
        % Y1攻击 - 边断联攻击，不影响领导者连接
        B_attacked = B_normal;
        
    elseif attack_mode == 2
        % Y2攻击 - 混合孤立和边攻击
        B_attacked = B_normal;
        
        % 孤立节点无法接收领导者信息
        if isfield(attack_info, 'isolated_nodes') && ~isempty(attack_info.isolated_nodes)
            for i = 1:length(attack_info.isolated_nodes)
                node = attack_info.isolated_nodes(i);
                if node >= 1 && node <= length(B_normal)
                    B_attacked(node) = 0;
                end
            end
        end
        
    elseif attack_mode == 3
        % 恢复阶段 - 观测器连接的不对称恢复
        B_attacked = apply_observer_recovery(B_normal, attack_info, attack_config);
        
    else
        % 未知攻击模式，使用正常连接
        B_attacked = B_normal;
    end
end

function B_recovered = apply_observer_recovery(B_normal, attack_info, attack_config)
    % 应用观测器连接的不对称恢复策略
    %
    % 输入:
    %   B_normal: 正常领导者连接矩阵
    %   attack_info: 攻击信息（包含恢复进度）
    %   attack_config: 攻击配置
    %
    % 输出:
    %   B_recovered: 恢复后的领导者连接矩阵
    
    B_recovered = B_normal;
    
    % 获取恢复进度 (0-1)
    recovery_progress = attack_info.recovery_progress;
    
    % 获取被攻击的节点（从Y2攻击配置中获取）
    if isfield(attack_config, 'Y2') && isfield(attack_config.Y2, 'isolated_nodes')
        isolated_nodes = attack_config.Y2.isolated_nodes;
    else
        isolated_nodes = [1];  % 默认恢复节点1
    end
    
    % 6.5秒开始：立即恢复被攻击节点对领导者的观测能力
    % （节点1能观测到领导者状态）
    for i = 1:length(isolated_nodes)
        node = isolated_nodes(i);
        if node >= 1 && node <= length(B_normal) && B_normal(node) == 1
            B_recovered(node) = 1;  % 立即恢复领导者观测
        end
    end
    
    % 注意：B矩阵只控制节点对领导者的观测，节点间的观测由A矩阵控制
    % 节点间观测的不对称恢复已在A矩阵的恢复逻辑中处理
end

function A_attacked = apply_Y1_attack(A_normal, attack_info, attack_config)
    % 应用Y1攻击 - 边断联攻击
    A_attacked = A_normal;
    
    switch attack_info.attack_type
        case 'edge_disconnect'
            % 指定边断联攻击
            if ~isempty(attack_info.target_edges)
                A_attacked = apply_edge_disconnect_attack(A_normal, attack_info.target_edges);
            else
                A_attacked = apply_random_link_failure(A_normal, attack_info.connectivity_reduction);
            end
            
        case 'random'
            % 随机链路失效攻击
            A_attacked = apply_random_link_failure(A_normal, attack_info.connectivity_reduction);
            
        otherwise
            % 默认边断联攻击
            if ~isempty(attack_info.target_edges)
                A_attacked = apply_edge_disconnect_attack(A_normal, attack_info.target_edges);
            else
                A_attacked = apply_random_link_failure(A_normal, attack_info.connectivity_reduction);
            end
    end
end

function A_attacked = apply_Y2_attack(A_normal, attack_info, attack_config)
    % 应用Y2攻击 - 混合孤立和边攻击
    A_attacked = A_normal;
    
    switch attack_info.attack_type
        case 'mixed_isolation'
            % 混合攻击：节点孤立 + 边断联
            % 首先应用节点孤立攻击
            if ~isempty(attack_info.isolated_nodes)
                A_attacked = apply_node_isolation_attack(A_attacked, attack_info.isolated_nodes);
            end
            
            % 然后应用额外的边断联攻击
            if ~isempty(attack_info.target_edges)
                A_attacked = apply_edge_disconnect_attack(A_attacked, attack_info.target_edges);
            end
            
        case 'node_isolation'
            % 传统节点孤立攻击
            if ~isempty(attack_info.isolated_nodes)
                A_attacked = apply_node_isolation_attack(A_normal, attack_info.isolated_nodes);
            else
                A_attacked = apply_network_partition_attack(A_normal, attack_info.connectivity_reduction);
            end
            
        case 'partition'
            % 网络分割攻击
            A_attacked = apply_network_partition_attack(A_normal, attack_info.connectivity_reduction);
            
        otherwise
            % 默认混合攻击
            if ~isempty(attack_info.isolated_nodes)
                A_attacked = apply_node_isolation_attack(A_normal, attack_info.isolated_nodes);
            end
            if ~isempty(attack_info.target_edges)
                A_attacked = apply_edge_disconnect_attack(A_attacked, attack_info.target_edges);
            end
    end
end

function A_attacked = apply_recovery_attack(A_normal, attack_info, attack_config)
    % 应用攻击后恢复模式 - 使用指定的恢复拓扑
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   attack_info: 攻击信息（包含恢复进度）
    %   attack_config: 攻击配置
    %
    % 输出:
    %   A_attacked: 恢复后的邻接矩阵
    
    % 检查是否有恢复阶段专用拓扑
    if isfield(attack_config, 'topology_config') && isfield(attack_config.topology_config, 'A_recovery')
        % 使用指定的恢复拓扑
        A_attacked = attack_config.topology_config.A_recovery;
        fprintf('使用恢复阶段专用拓扑\n');
    else
        % 使用原有的渐进恢复逻辑
        A_attacked = A_normal;
        
        % 获取恢复进度 (0-1)
        recovery_progress = attack_info.recovery_progress;
        
        % 获取被攻击的节点（从Y2攻击配置中获取）
        if isfield(attack_config, 'Y2') && isfield(attack_config.Y2, 'isolated_nodes')
            isolated_nodes = attack_config.Y2.isolated_nodes;
        else
            isolated_nodes = [1];  % 默认恢复节点1
        end
        
        % 对每个被攻击的节点应用恢复逻辑
        for i = 1:length(isolated_nodes)
            node = isolated_nodes(i);
            
            % 计算该节点的连接恢复程度
            % 使用平滑的恢复函数：从0到1的S型曲线
            if recovery_progress < 0.5
                % 前半段：缓慢开始
                connection_strength = 2 * recovery_progress^2;
            else
                % 后半段：快速完成
                connection_strength = 1 - 2 * (1 - recovery_progress)^2;
            end
            
            % 恢复该节点的连接
            A_attacked = apply_gradual_node_recovery(A_normal, A_attacked, node, connection_strength);
        end
        
        % 如果有额外的边断联攻击，也需要恢复
        if isfield(attack_config, 'Y2') && isfield(attack_config.Y2, 'target_edges') && ...
           ~isempty(attack_config.Y2.target_edges)
            A_attacked = apply_gradual_edge_recovery(A_normal, A_attacked, ...
                                                    attack_config.Y2.target_edges, recovery_progress);
        end
    end
end

function A_recovered = apply_gradual_node_recovery(A_normal, A_current, node, recovery_strength)
    % 逐渐恢复指定节点的连接（不对称恢复策略）
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   A_current: 当前邻接矩阵
    %   node: 要恢复的节点
    %   recovery_strength: 恢复强度 (0-1)
    %
    % 输出:
    %   A_recovered: 恢复后的邻接矩阵
    
    A_recovered = A_current;
    N = size(A_normal, 1);
    
    if node < 1 || node > N
        return;
    end
    
    % 不对称恢复策略：
    % 1. 节点1对其他节点的连接逐渐恢复（outgoing connections）
    % 2. 其他节点对节点1的连接只在完全恢复时（recovery_strength = 1）才恢复（incoming connections）
    
    for j = 1:N
        if j ~= node  % 排除自连接
            if A_normal(node, j) == 1  % 如果正常状态下有连接
                
                % 恢复节点1对其他节点的连接（outgoing）
                if recovery_strength >= 0.8  % 恢复强度达到80%时完全恢复
                    A_recovered(node, j) = 1;
                elseif recovery_strength >= 0.5  % 恢复强度达到50%时部分恢复
                    % 使用概率恢复
                    if rand() < recovery_strength
                        A_recovered(node, j) = 1;
                    end
                end
                
                % 恢复其他节点对节点1的连接（incoming）- 只在完全恢复时
                if recovery_strength >= 1.0  % 完全恢复时才恢复incoming连接
                    A_recovered(j, node) = 1;
                end
            end
        end
    end
end

function A_recovered = apply_gradual_edge_recovery(A_normal, A_current, target_edges, recovery_strength)
    % 逐渐恢复指定边的连接（不对称恢复策略）
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   A_current: 当前邻接矩阵
    %   target_edges: 目标边列表
    %   recovery_strength: 恢复强度 (0-1)
    %
    % 输出:
    %   A_recovered: 恢复后的邻接矩阵
    
    A_recovered = A_current;
    
    if isempty(target_edges)
        return;
    end
    
    % 恢复每条边（不对称策略）
    for i = 1:size(target_edges, 1)
        node1 = target_edges(i, 1);
        node2 = target_edges(i, 2);
        
        % 如果正常状态下有连接，则根据恢复强度决定是否恢复
        if A_normal(node1, node2) == 1
            % 检查是否涉及节点1
            if node1 == 1 || node2 == 1
                % 涉及节点1的边使用不对称恢复策略
                if node1 == 1
                    % 节点1到其他节点的连接逐渐恢复
                    if recovery_strength >= 0.8
                        A_recovered(node1, node2) = 1;
                    elseif recovery_strength >= 0.5
                        if rand() < recovery_strength
                            A_recovered(node1, node2) = 1;
                        end
                    end
                    
                    % 其他节点到节点1的连接只在完全恢复时恢复
                    if recovery_strength >= 1.0
                        A_recovered(node2, node1) = 1;
                    end
                else
                    % 其他节点到节点1的连接
                    if recovery_strength >= 1.0
                        A_recovered(node1, node2) = 1;
                    end
                    
                    % 节点1到其他节点的连接逐渐恢复
                    if recovery_strength >= 0.8
                        A_recovered(node2, node1) = 1;
                    elseif recovery_strength >= 0.5
                        if rand() < recovery_strength
                            A_recovered(node2, node1) = 1;
                        end
                    end
                end
            else
                % 不涉及节点1的边使用对称恢复策略
                if recovery_strength >= 0.7
                    A_recovered(node1, node2) = 1;
                    A_recovered(node2, node1) = 1;
                elseif recovery_strength >= 0.3
                    if rand() < recovery_strength
                        A_recovered(node1, node2) = 1;
                        A_recovered(node2, node1) = 1;
                    end
                end
            end
        end
    end
end

function A_attacked = apply_edge_disconnect_attack(A_normal, target_edges)
    % 应用指定边断联攻击
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   target_edges: 目标边列表 [node1, node2; node3, node4; ...]
    %
    % 输出:
    %   A_attacked: 受攻击后的邻接矩阵
    
    A_attacked = A_normal;
    N = size(A_normal, 1);
    
    % 验证目标边并断开连接
    for i = 1:size(target_edges, 1)
        node1 = target_edges(i, 1);
        node2 = target_edges(i, 2);
        
        % 验证节点编号
        if node1 >= 1 && node1 <= N && node2 >= 1 && node2 <= N && node1 ~= node2
            if A_attacked(node1, node2) == 1
                A_attacked(node1, node2) = 0;
                A_attacked(node2, node1) = 0;  % 保持对称性
            end
        end
    end
end

function A_attacked = apply_node_isolation_attack(A_normal, isolated_nodes)
    % 应用节点孤立攻击
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   isolated_nodes: 要孤立的节点列表 [node1, node2, ...]
    %
    % 输出:
    %   A_attacked: 受攻击后的邻接矩阵
    
    A_attacked = A_normal;
    N = size(A_normal, 1);
    
    % 验证孤立节点
    isolated_nodes = isolated_nodes(isolated_nodes >= 1 & isolated_nodes <= N);
    if isempty(isolated_nodes)
        return;
    end
    
    % 孤立指定节点（断开其所有连接）
    for i = 1:length(isolated_nodes)
        node = isolated_nodes(i);
        
        % 断开该节点的所有连接
        A_attacked(node, :) = 0;  % 该节点到其他节点的连接
        A_attacked(:, node) = 0;  % 其他节点到该节点的连接
    end
end

function A_attacked = apply_targeted_node_attack(A_normal, target_nodes, reduction)
    % 应用目标节点攻击
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   target_nodes: 目标节点列表
    %   reduction: 攻击强度
    %
    % 输出:
    %   A_attacked: 受攻击后的邻接矩阵
    
    A_attacked = A_normal;
    N = size(A_normal, 1);
    
    % 验证目标节点
    target_nodes = target_nodes(target_nodes >= 1 & target_nodes <= N);
    if isempty(target_nodes)
        fprintf('警告：没有有效的目标节点，使用随机攻击\n');
        A_attacked = apply_random_link_failure(A_normal, reduction);
        return;
    end
    
    % 计算要断开的连接数
    total_edges = sum(A_normal(:));
    edges_to_remove = round(total_edges * reduction);
    
    % 优先断开目标节点的连接
    target_edges = [];
    for i = 1:length(target_nodes)
        node = target_nodes(i);
        [row_idx, col_idx] = find(A_normal(node, :));
        for j = 1:length(col_idx)
            if col_idx(j) ~= node  % 排除自连接
                target_edges(end+1, :) = [node, col_idx(j)];
            end
        end
    end
    
    % 断开目标节点的连接
    removed_count = 0;
    for i = 1:size(target_edges, 1)
        if removed_count >= edges_to_remove
            break;
        end
        row = target_edges(i, 1);
        col = target_edges(i, 2);
        if A_attacked(row, col) == 1
            A_attacked(row, col) = 0;
            A_attacked(col, row) = 0;  % 保持对称性
            removed_count = removed_count + 1;
        end
    end
    
    % 如果还需要断开更多连接，随机断开其他连接
    if removed_count < edges_to_remove
        remaining_edges = edges_to_remove - removed_count;
        A_attacked = apply_random_link_failure(A_attacked, remaining_edges / sum(A_attacked(:)));
    end
end

function A_attacked = apply_edge_based_attack(A_normal, reduction)
    % 应用基于边的攻击
    %
    % 输入:
    %   A_normal: 正常邻接矩阵
    %   reduction: 攻击强度
    %
    % 输出:
    %   A_attacked: 受攻击后的邻接矩阵
    
    A_attacked = A_normal;
    N = size(A_normal, 1);
    
    % 计算每个节点的度
    degrees = sum(A_normal, 2);
    
    % 优先攻击高度节点的连接
    [~, sorted_indices] = sort(degrees, 'descend');
    
    % 计算要断开的连接数
    total_edges = sum(A_normal(:));
    edges_to_remove = round(total_edges * reduction);
    
    removed_count = 0;
    for i = 1:length(sorted_indices)
        if removed_count >= edges_to_remove
            break;
        end
        
        node = sorted_indices(i);
        [~, neighbors] = find(A_normal(node, :));
        
        % 随机断开该节点的一些连接
        num_neighbors = length(neighbors);
        edges_to_remove_from_node = min(round(num_neighbors * reduction), edges_to_remove - removed_count);
        
        if edges_to_remove_from_node > 0
            remove_indices = randperm(num_neighbors, min(edges_to_remove_from_node, num_neighbors));
            for j = 1:length(remove_indices)
                neighbor = neighbors(remove_indices(j));
                if A_attacked(node, neighbor) == 1
                    A_attacked(node, neighbor) = 0;
                    A_attacked(neighbor, node) = 0;
                    removed_count = removed_count + 1;
                end
            end
        end
    end
end

function A_attacked = apply_random_link_failure(A_normal, reduction)
    % 应用随机链路失效攻击
    A_attacked = A_normal;
    num_edges = sum(A_normal(:));
    edges_to_remove = round(num_edges * reduction);
    
    if edges_to_remove > 0
        [row_idx, col_idx] = find(A_normal);
        remove_indices = randperm(length(row_idx), min(edges_to_remove, length(row_idx)));
        
        for k = 1:length(remove_indices)
            i = row_idx(remove_indices(k));
            j = col_idx(remove_indices(k));
            A_attacked(i, j) = 0;
        end
    end
end

function A_attacked = apply_network_partition_attack(A_normal, reduction)
    % 应用网络分割攻击
    N = size(A_normal, 1);
    A_attacked = A_normal;
    
    % 将网络分割为两个子网
    partition_point = round(N / 2);
    
    % 断开两个子网之间的连接
    for i = 1:partition_point
        for j = (partition_point+1):N
            if rand() < reduction
                A_attacked(i, j) = 0;
                A_attacked(j, i) = 0;
            end
        end
    end
end

function connectivity_score = calculate_connectivity_score(attack_mode, attack_info)
    % 计算网络连通性分数
    %
    % 输入:
    %   attack_mode: 攻击模式 (0=正常, 1=Y1攻击, 2=Y2攻击)
    %   attack_info: 攻击信息结构体
    %
    % 输出:
    %   connectivity_score: 连通性分数 (0-1, 1表示完全连通)
    
    switch attack_mode
        case 0
            % 正常状态，完全连通
            connectivity_score = 1.0;
            
        case 1
            % Y1攻击 - 边断联攻击
            % 根据断开的边数计算连通性损失
            if isfield(attack_info, 'target_edges') && ~isempty(attack_info.target_edges)
                num_disconnected_edges = size(attack_info.target_edges, 1);
                % 假设最大可能的边数为6*5/2=15（6个节点的完全图）
                max_possible_edges = 15;
                connectivity_score = max(0.1, 1.0 - (num_disconnected_edges / max_possible_edges));
            else
                connectivity_score = 0.7; % 默认中等连通性损失
            end
            
        case 2
            % Y2攻击 - 混合孤立和边攻击
            % 根据孤立的节点数和断开的边数计算连通性损失
            connectivity_loss = 0;
            
            % 节点孤立造成的连通性损失
            if isfield(attack_info, 'isolated_nodes') && ~isempty(attack_info.isolated_nodes)
                num_isolated_nodes = length(attack_info.isolated_nodes);
                total_nodes = 6;
                connectivity_loss = connectivity_loss + (num_isolated_nodes / total_nodes) * 0.6;  % 节点孤立权重较高
            end
            
            % 边断联造成的额外连通性损失
            if isfield(attack_info, 'target_edges') && ~isempty(attack_info.target_edges)
                num_disconnected_edges = size(attack_info.target_edges, 1);
                max_possible_edges = 15;
                connectivity_loss = connectivity_loss + (num_disconnected_edges / max_possible_edges) * 0.3;  % 边断联权重较低
            end
            
            connectivity_score = max(0.1, 1.0 - connectivity_loss);
            
        case 3
            % 攻击后恢复模式 - 根据恢复进度计算连通性
            recovery_progress = attack_info.recovery_progress;
            
            % 计算恢复前的连通性损失（与Y2攻击相同）
            connectivity_loss = 0;
            
            % 节点孤立造成的连通性损失
            if isfield(attack_info, 'isolated_nodes') && ~isempty(attack_info.isolated_nodes)
                num_isolated_nodes = length(attack_info.isolated_nodes);
                total_nodes = 6;
                connectivity_loss = connectivity_loss + (num_isolated_nodes / total_nodes) * 0.6;
            end
            
            % 边断联造成的额外连通性损失
            if isfield(attack_info, 'target_edges') && ~isempty(attack_info.target_edges)
                num_disconnected_edges = size(attack_info.target_edges, 1);
                max_possible_edges = 15;
                connectivity_loss = connectivity_loss + (num_disconnected_edges / max_possible_edges) * 0.3;
            end
            
            % 根据恢复进度调整连通性损失
            connectivity_loss = connectivity_loss * (1 - recovery_progress);
            connectivity_score = max(0.1, 1.0 - connectivity_loss);
            
        otherwise
            % 未知攻击模式
            connectivity_score = 0.5;
    end
    
    % 确保分数在合理范围内
    connectivity_score = max(0.0, min(1.0, connectivity_score));
end
