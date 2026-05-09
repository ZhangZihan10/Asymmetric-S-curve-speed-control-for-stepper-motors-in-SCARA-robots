function path_even = Smooth_and_Even_Sample2(path, num_points, smooth_window, robot, obstacles)
    % =====================================================================
    % 机器人轨迹平滑与等距重采样 (带正运动学笛卡尔空间避障可视化)
    % 适配: PointCloud 点云障碍物 & Sphere/Cylinder 几何体障碍物
    % 新增: 计算原始轨迹与优化轨迹的步长方差
    % =====================================================================
    
    if nargin < 5, obstacles = []; end 
    if nargin < 4, robot = []; end     
    if nargin < 3, smooth_window = 10; end
    if nargin < 2, num_points = 30; end
    
    N_orig = size(path, 1);
    
    % --- 1. 数据预处理：消除连续重复的冗余点 ---
    diff_path = [1; sum(abs(diff(path)), 2)];
    path = path(diff_path > 1e-6, :); 
    
    % --- 2. 计算原始路径在加权关节空间内的累积弦长 ---
    weights = [10, 1, 1, 1]; 
    s_orig = zeros(size(path,1), 1);
    for i = 2:size(path,1)
        d = (path(i,:) - path(i-1,:)) .* weights;
        s_orig(i) = s_orig(i-1) + norm(d);
    end
    
    % --- 3. 密集插值与高斯平滑 ---
    s_dense = linspace(0, s_orig(end), 1000)';
    path_dense = zeros(1000, 4);
    for col = 1:4
        path_dense(:, col) = interp1(s_orig, path(:, col), s_dense, 'pchip');
    end
    
    path_smooth = smoothdata(path_dense, 1, 'gaussian', smooth_window);
    
    path_smooth(1,:) = path(1,:);
    path_smooth(end,:) = path(end,:);
    
    % --- 4. 计算平滑后曲线的真实等距累积弦长 ---
    s_smooth = zeros(1000, 1);
    for i = 2:1000
        d = (path_smooth(i,:) - path_smooth(i-1,:)) .* weights;
        s_smooth(i) = s_smooth(i-1) + norm(d);
    end
    
    % --- 5. 绝对等距重采样 ---
    s_even = linspace(0, s_smooth(end), num_points)';
    path_even = zeros(num_points, 4);
    for col = 1:4
        path_even(:, col) = interp1(s_smooth, path_smooth(:, col), s_even, 'linear');
    end

    % --- 6. 计算两种轨迹的步长及步长方差 ---
    dist_orig = vecnorm(diff(path) .* weights, 2, 2);
    dist_even = vecnorm(diff(path_even) .* weights, 2, 2);

    var_orig = var(dist_orig, 1);   % 总体方差
    var_even = var(dist_even, 1);   % 总体方差

    fprintf('Original path step-size variance  : %.8f\n', var_orig);
    fprintf('Optimized path step-size variance : %.8f\n', var_even);
    
    % =====================================================================
    % 绘图对比反馈 (符合 SCI 期刊排版要求 + 混合障碍物渲染)
    % =====================================================================
    figure('Name', 'Gaussian Smoothing & Resampling with Obstacles', ...
           'Position', [200, 200, 1100, 450], 'Color', 'w');
    
    % -- 子图 (a)：3D 物理任务空间 --
    ax1 = subplot(1,2,1);
    hold(ax1, 'on');
    grid(ax1, 'on'); 
    box(ax1, 'on');
    axis(ax1, 'equal'); 
    view(ax1, [-45, 30]); 
    set(ax1, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k'); 
    
    % ★ 调用全新适配的障碍物绘制函数
    if ~isempty(obstacles)
        drawObstaclesAdvanced(ax1, obstacles); 
    end

    if ~isempty(robot)
        % 提取原始冗余路径末端位置
        orig_ee = zeros(size(path, 1), 3);
        for i = 1:size(path, 1)
            T = robot.fkine([path(i,:) 0 0]);
            orig_ee(i,:) = T.t';
        end

        % 提取平滑重采样后末端位置
        even_ee = zeros(size(path_even, 1), 3);
        for i = 1:size(path_even, 1)
            T = robot.fkine([path_even(i,:) 0 0]);
            even_ee(i,:) = T.t';
        end
        
        % 绘制轨迹
        plot3(ax1, orig_ee(:,1), orig_ee(:,2), orig_ee(:,3), ...
            'r-o', 'LineWidth', 1.5, 'MarkerSize', 4);
        plot3(ax1, even_ee(:,1), even_ee(:,2), even_ee(:,3), ...
            'b-*', 'LineWidth', 2.0);
        
        % 标记起点和终点
        scatter3(ax1, orig_ee(1,1), orig_ee(1,2), orig_ee(1,3), 60, 'ro', 'filled');
        scatter3(ax1, orig_ee(end,1), orig_ee(end,2), orig_ee(end,3), 60, 'go', 'filled');

        xlabel(ax1, 'Workspace X (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel(ax1, 'Workspace Y (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
        zlabel(ax1, 'Workspace Z (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
        title(ax1, '(a) Cartesian Trajectory Smoothing Comparison', 'FontName', 'Times New Roman', 'FontSize', 20);
    else
        plot3(ax1, path(:,3), path(:,2), path(:,1)*10, 'r-o', 'LineWidth', 1.5);
        plot3(ax1, path_even(:,3), path_even(:,2), path_even(:,1)*10, 'b-*', 'LineWidth', 1.5);
        xlabel(ax1, 'Joint X (rad)');
        ylabel(ax1, 'Joint Y (rad)');
        zlabel(ax1, 'Joint Z (scaled)');
        title(ax1, '(a) Joint Space Trajectory Smoothing Comparison');
    end
    
    % 图例设置
    if ~isempty(obstacles)
        leg = legend(ax1, 'Obstacles','Original Path', 'Optimized path',  'Location', 'best');
    else
        leg = legend(ax1, 'Original Path', 'Optimized path', 'Location', 'best');
    end
    set(leg, 'FontName', 'Times New Roman', 'FontSize', 11, 'EdgeColor', [0.8 0.8 0.8]);
    set(ax1, 'FontSize', 11, 'FontName', 'Times New Roman');
    hold(ax1, 'off');
    
    % -- 子图 (b)：相邻节点步长方差分析 --
    ax2 = subplot(1,2,2);
    
    plot(ax2, dist_orig, 'r-o'); hold(ax2, 'on');
    plot(ax2, dist_even, 'b-*', 'LineWidth', 2);
    
    grid(ax2, 'on'); 
    box(ax2, 'on');
    title(ax2, '(b) Step Size Variance Analysis', 'FontName', 'Times New Roman', 'FontSize', 20);
    
    legend_text_1 = sprintf('Original Step Size (Var = %.6f)', var_orig);
    legend_text_2 = sprintf('Optimized Step Size (Var = %.6f)', var_even);
    legend(ax2, legend_text_1, legend_text_2, 'Location', 'best');
    
    xlabel(ax2, 'Segment Index', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel(ax2, 'Weighted Spatial Distance', 'FontName', 'Times New Roman', 'FontSize', 12);
    set(ax2, 'FontSize', 11, 'FontName', 'Times New Roman');

    % 图中额外显示方差值
    txt = {
        sprintf('Original variance  = %.6f', var_orig), ...
        sprintf('Optimized variance = %.6f', var_even)
    };
    text(ax2, 0.03, 0.95, txt, ...
        'Units', 'normalized', ...
        'VerticalAlignment', 'top', ...
        'FontName', 'Times New Roman', ...
        'FontSize', 11, ...
        'BackgroundColor', 'w', ...
        'EdgeColor', [0.8 0.8 0.8]);
    
    hold(ax2, 'off');
end

% =====================================================================
% 辅助绘图函数：混合几何体与点云障碍物渲染 (SCI 白底适配版)
% =====================================================================
function drawObstaclesAdvanced(ax, obstacles)
    drawn_count = 0;
    
    % --- 处理点云类障碍物 ---
    if isa(obstacles, 'pointCloud')
        locs = obstacles.Location;
        scatter3(ax, locs(:,1), locs(:,2), locs(:,3), 12, [0.9 0.8 0], 'filled', 'HandleVisibility', 'off');
        drawn_count = 1;
        
    % --- 处理混合几何体类障碍物 ---
    elseif iscell(obstacles)
        for i = 1:length(obstacles)
            obs = obstacles{i};
            if ~iscell(obs) || isempty(obs)
                continue; 
            end
            
            switch obs{1}
                case 'sphere'
                    [x, y, z] = sphere(20);
                    surf(ax, x*obs{3}+obs{2}(1), y*obs{3}+obs{2}(2), z*obs{3}+obs{2}(3), ...
                        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'r', 'HandleVisibility', 'off');
                    drawn_count = 1;
                    
                case 'cylinder'
                    [x, y, z] = cylinder(obs{3}, 20);
                    z = z * obs{4} - obs{4}/2 + obs{2}(3);
                    surf(ax, x+obs{2}(1), y+obs{2}(2), z, ...
                        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'b', 'HandleVisibility', 'off');
                    drawn_count = 1;
            end
        end
    end
    
    % 为图例注入一个伪数据项
    if drawn_count > 0
        if isa(obstacles, 'pointCloud')
            scatter3(ax, NaN, NaN, NaN, 50, [0.9 0.8 0], 'filled', 'DisplayName', 'Obstacles');
        else
            fill(ax, [NaN, NaN, NaN, NaN], [NaN, NaN, NaN, NaN], 'r', ...
                'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', 'Obstacles');
        end
    end
end