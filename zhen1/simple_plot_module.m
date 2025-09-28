%% 简化的分析和可视化模块
% 只生成4个独立图：观测器误差（角度）、观测器误差（角速度）、跟踪误差（角度）、跟踪误差（角速度）

function simple_plot_module(t, x_leader, x_followers, xhat_leader, observer_errors, ...
                            attack_modes, N)
    % 简化的可视化函数 - 4个独立图片
    %
    % 输入:
    %   t: 时间向量
    %   x_leader: 领导者状态 [2 x length(t)]
    %   x_followers: 追随者状态 [2 x N x length(t)]
    %   xhat_leader: 观测器状态 [2 x N x length(t)]
    %   observer_errors: 观测器误差 [2*N x length(t)]
    %   attack_modes: 攻击模式向量
    %   N: 追随者数量

    % 定义颜色方案
    colors = generate_color_scheme(N);
    
    % 计算跟踪误差（追随者相对于领导者的误差）
    tracking_errors = compute_tracking_errors(x_leader, x_followers, t, N);
    
    % 创建攻击区域信息
    attack_regions = get_attack_regions(t, attack_modes);
    
    fprintf('Generating 2 combined charts...\n');
    
    %% 图1：观测器误差合并图（角度和角速度）
    figure('Name', '观测器误差', 'Position', [100, 100, 1800, 600]);
    
    % 子图1：观测器误差 - 状态1（角度）
    subplot(2, 1, 1);
    plot_observer_errors_state(t, observer_errors, 1, N, colors, attack_regions);
    grid on;
    grid minor;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);  % 设置网格透明度，使网格更稀疏
    set(gca, 'Position', [0.02, 0.6, 0.96, 0.38]);   % 减少左右边距，减少上边距
    
    % 子图2：观测器误差 - 状态2（角速度）
    subplot(2, 1, 2);
    plot_handles2 = plot_observer_errors_state(t, observer_errors, 2, N, colors, attack_regions);
    xlabel('Time (s)', 'FontSize', 12);
    grid on;
    grid minor;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);  % 设置网格透明度，使网格更稀疏
    set(gca, 'Position', [0.02, 0.02, 0.96, 0.48]);   % 减少左右边距，减少下边距
    
    % 添加统一图例（在速度图下方）- 使用速度图的绘图句柄
    legend_labels = arrayfun(@(i) sprintf('e%d', i), 1:N, 'UniformOutput', false);
    legend(plot_handles2, legend_labels, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 10);
    
    saveas(gcf, 'observer_errors_combined.png');
    fprintf('Observer error combined chart saved as observer_errors_combined.png\n');
    
    %% 图2：跟踪误差合并图（角度和角速度）
    figure('Name', '跟踪误差', 'Position', [200, 200, 1800, 600]);
    
    % 子图1：跟踪误差 - 状态1（角度）
    subplot(2, 1, 1);
    plot_tracking_errors_state(t, tracking_errors, 1, N, colors, attack_regions);
    grid on;
    grid minor;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);  % 设置网格透明度，使网格更稀疏
    set(gca, 'Position', [0.02, 0.6, 0.96, 0.38]);   % 减少左右边距，减少上边距
    
    % 子图2：跟踪误差 - 状态2（角速度）
    subplot(2, 1, 2);
    plot_handles2 = plot_tracking_errors_state(t, tracking_errors, 2, N, colors, attack_regions);
    xlabel('Time (s)', 'FontSize', 12);
    grid on;
    grid minor;
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);  % 设置网格透明度，使网格更稀疏
    set(gca, 'Position', [0.02, 0.02, 0.96, 0.48]);   % 减少左右边距，减少下边距
    
    % 添加统一图例（在速度图下方）- 使用速度图的绘图句柄
    legend_labels = arrayfun(@(i) sprintf('e%d', i), 1:N, 'UniformOutput', false);
    legend(plot_handles2, legend_labels, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 10);
    
    saveas(gcf, 'tracking_errors_combined.png');
    fprintf('Tracking error combined chart saved as tracking_errors_combined.png\n');
    
    fprintf('2 combined charts generation completed!\n');
end

function colors = generate_color_scheme(N)
    % 生成颜色方案
    if N <= 6
        colors = [
            0.0000, 0.4470, 0.7410;  % 蓝色
            0.8500, 0.3250, 0.0980;  % 橙色
            0.9290, 0.6940, 0.1250;  % 黄色
            0.4940, 0.1840, 0.5560;  % 紫色
            0.4660, 0.6740, 0.1880;  % 绿色
            0.3010, 0.7450, 0.9330;  % 青色
        ];
    else
        % 为更多追随者生成颜色
        colors = hsv(N);
    end
    colors = colors(1:N, :);
end

function attack_regions = get_attack_regions(t, attack_modes)
    % 提取攻击区域信息
    attack_regions = struct();
    
    % 找到Y1攻击区域
    Y1_indices = find(attack_modes == 1);
    if ~isempty(Y1_indices)
        Y1_starts = [];
        Y1_ends = [];
        
        % 找到连续的Y1攻击区间
        diff_indices = diff(Y1_indices);
        break_points = find(diff_indices > 1);
        
        start_idx = 1;
        for i = 1:length(break_points)
            end_idx = break_points(i);
            Y1_starts(end+1) = t(Y1_indices(start_idx));
            Y1_ends(end+1) = t(Y1_indices(end_idx));
            start_idx = end_idx + 1;
        end
        % 最后一个区间
        Y1_starts(end+1) = t(Y1_indices(start_idx));
        Y1_ends(end+1) = t(Y1_indices(end));
        
        attack_regions.Y1_regions = [Y1_starts' Y1_ends'];
    else
        attack_regions.Y1_regions = [];
    end
    
    % 找到Y2攻击区域
    Y2_indices = find(attack_modes == 2);
    if ~isempty(Y2_indices)
        Y2_starts = [];
        Y2_ends = [];
        
        % 找到连续的Y2攻击区间
        diff_indices = diff(Y2_indices);
        break_points = find(diff_indices > 1);
        
        start_idx = 1;
        for i = 1:length(break_points)
            end_idx = break_points(i);
            Y2_starts(end+1) = t(Y2_indices(start_idx));
            Y2_ends(end+1) = t(Y2_indices(end_idx));
            start_idx = end_idx + 1;
        end
        % 最后一个区间
        Y2_starts(end+1) = t(Y2_indices(start_idx));
        Y2_ends(end+1) = t(Y2_indices(end));
        
        attack_regions.Y2_regions = [Y2_starts' Y2_ends'];
    else
        attack_regions.Y2_regions = [];
    end
end

function add_attack_region_shading(attack_regions, y_lim)
    % 添加攻击区域阴影
    attack_colors = struct('Y1', [1 1 0], 'Y2', [1 0 0]);
    
    % Y1攻击区域
    if ~isempty(attack_regions.Y1_regions)
        for i = 1:size(attack_regions.Y1_regions, 1)
            x_region = [attack_regions.Y1_regions(i, 1), attack_regions.Y1_regions(i, 2), ...
                       attack_regions.Y1_regions(i, 2), attack_regions.Y1_regions(i, 1)];
            y_region = [y_lim(1), y_lim(1), y_lim(2), y_lim(2)];
            fill(x_region, y_region, attack_colors.Y1, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        end
    end
    
    % Y2攻击区域
    if ~isempty(attack_regions.Y2_regions)
        for i = 1:size(attack_regions.Y2_regions, 1)
            x_region = [attack_regions.Y2_regions(i, 1), attack_regions.Y2_regions(i, 2), ...
                       attack_regions.Y2_regions(i, 2), attack_regions.Y2_regions(i, 1)];
            y_region = [y_lim(1), y_lim(1), y_lim(2), y_lim(2)];
            fill(x_region, y_region, attack_colors.Y2, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
        end
    end
end

function plot_handles = plot_observer_errors_state(t, observer_errors, state_idx, N, colors, attack_regions)
    % 绘制特定状态的观测器误差
    hold on;
    
    % 设置y轴范围
    state_errors = observer_errors((state_idx-1)*N+1:state_idx*N, :);
    y_min = min(state_errors(:));
    y_max = max(state_errors(:));
    y_range = y_max - y_min;
    y_lim = [y_min - 0.1*y_range, y_max + 0.1*y_range];
    
    % 首先添加攻击区域阴影
    add_attack_region_shading(attack_regions, y_lim);
    
    % 绘制每个追随者的观测误差
    markers = {'s', 'o', '^', 'd', 'v', 'p', 'h', '*', '+', 'x'};  % 扩展标记符号
    % 计算每秒的标记索引，确保每1秒一个标记
    marker_indices = [];
    for sec = 0:20  % 从0秒到20秒
        [~, idx] = min(abs(t - sec));  % 找到最接近该秒的数据点索引
        marker_indices = [marker_indices, idx];
    end
    marker_indices = unique(marker_indices);  % 去除重复索引
    
    plot_handles = [];
    for i = 1:N
        error_data = state_errors(i, :);
        marker_idx = mod(i-1, length(markers)) + 1;  % 循环使用标记符号
        h = plot(t, error_data, 'Color', colors(i, :), 'LineWidth', 1, ...
             'Marker', markers{marker_idx}, 'MarkerSize', 4, 'MarkerIndices', marker_indices, ...
             'DisplayName', sprintf('e%d', i));
        plot_handles(end+1) = h;
    end
    
    ylim(y_lim);
    xlim([t(1), t(end)]);
    
    % 设置时间刻度（更稀疏的网格）
    time_ticks = 0:2:20;  % 每2秒一个刻度，使网格更稀疏
    xticks(time_ticks);
    xticklabels(arrayfun(@(x) sprintf('%.0f', x), time_ticks, 'UniformOutput', false));
    
    hold off;
end

function plot_handles = plot_tracking_errors_state(t, tracking_errors, state_idx, N, colors, attack_regions)
    % 绘制特定状态的跟踪误差
    hold on;
    
    % 提取特定状态的跟踪误差
    state_errors = squeeze(tracking_errors(state_idx, :, :));
    
    % 计算y轴范围
    y_min = min(state_errors(:));
    y_max = max(state_errors(:));
    y_range = y_max - y_min;
    y_lim = [y_min - 0.1*y_range, y_max + 0.1*y_range];
    
    % 首先添加攻击区域阴影
    add_attack_region_shading(attack_regions, y_lim);
    
    % 绘制每个追随者的跟踪误差
    markers = {'s', 'o', '^', 'd', 'v', 'p', 'h', '*', '+', 'x'};  % 扩展标记符号
    % 计算每秒的标记索引，确保每1秒一个标记
    marker_indices = [];
    for sec = 0:20  % 从0秒到20秒
        [~, idx] = min(abs(t - sec));  % 找到最接近该秒的数据点索引
        marker_indices = [marker_indices, idx];
    end
    marker_indices = unique(marker_indices);  % 去除重复索引
    
    plot_handles = [];
    for i = 1:N
        error_data = state_errors(i, :);
        marker_idx = mod(i-1, length(markers)) + 1;  % 循环使用标记符号
        h = plot(t, error_data, 'Color', colors(i, :), 'LineWidth', 1, ...
             'Marker', markers{marker_idx}, 'MarkerSize', 4, 'MarkerIndices', marker_indices, ...
             'DisplayName', sprintf('e%d', i));
        plot_handles(end+1) = h;
    end
    
    ylim(y_lim);
    xlim([t(1), t(end)]);
    
    % 设置时间刻度（更稀疏的网格）
    time_ticks = 0:2:20;  % 每2秒一个刻度，使网格更稀疏
    xticks(time_ticks);
    xticklabels(arrayfun(@(x) sprintf('%.0f', x), time_ticks, 'UniformOutput', false));
    
    hold off;
end

function tracking_errors = compute_tracking_errors(x_leader, x_followers, t, N)
    % 计算跟踪误差（不使用绝对值）
    tracking_errors = zeros(2, N, length(t));
    
    for k = 1:length(t)
        for i = 1:N
            for state = 1:2
                tracking_errors(state, i, k) = x_followers(state, i, k) - x_leader(state, k);
            end
        end
    end
end
