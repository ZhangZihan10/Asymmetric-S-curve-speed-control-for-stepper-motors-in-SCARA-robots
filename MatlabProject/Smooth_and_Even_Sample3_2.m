function path_even = Smooth_and_Even_Sample3_2(path, num_points, smooth_window, robot, obstacles)
    % =====================================================================
    % 机器人轨迹平滑与等距重采样 (基于纯笛卡尔空间物理距离 m)
    % 逻辑修正：全面引入正运动学，确保重采样基准与最终测算严格统一为物理米(m)
    % =====================================================================
    
    if nargin < 5, obstacles = []; end 
    if nargin < 4, robot = []; end     
    if nargin < 3, smooth_window = 10; end
    if nargin < 2, num_points = 30; end
    
    N_orig = size(path, 1);
    
    % --- 1. 数据预处理：消除连续重复的冗余点 ---
    diff_path = [1; sum(abs(diff(path)), 2)];
    path = path(diff_path > 1e-6, :); 
    
    % =====================================================================
    % --- 2. ★ 核心修正：利用正运动学，计算真实的物理空间累积弧长 (m) ---
    % =====================================================================
    if isempty(robot)
        error('必须传入 robot 对象以进行纯物理空间(m)的量纲转换！');
    end
    
    s_orig = zeros(size(path,1), 1);
    ee_orig_temp = zeros(size(path,1), 3); % 缓存末端坐标
    for i = 1:size(path,1)
        T = robot.fkine([path(i,:) 0 0]);
        ee_orig_temp(i,:) = T.t';
        if i > 1
            % 计算真实的物理直线步距 (m)
            s_orig(i) = s_orig(i-1) + norm(ee_orig_temp(i,:) - ee_orig_temp(i-1,:));
        end
    end
    
    % --- 3. 密集插值与高斯平滑 (以真实物理距离 s_orig 为 X 轴基准) ---
    s_dense = linspace(0, s_orig(end), 1000)';
    path_dense = zeros(1000, 4);
    for col = 1:4
        path_dense(:, col) = interp1(s_orig, path(:, col), s_dense, 'pchip');
    end
    
    path_smooth = smoothdata(path_dense, 1, 'gaussian', smooth_window);
    path_smooth(1,:) = path(1,:);
    path_smooth(end,:) = path(end,:);
    
    % --- 4. ★ 计算平滑后曲线的真实物理累积弦长 (m) ---
    s_smooth = zeros(1000, 1);
    ee_smooth_temp = zeros(1000, 3);
    for i = 1:1000
        T = robot.fkine([path_smooth(i,:) 0 0]);
        ee_smooth_temp(i,:) = T.t';
        if i > 1
            s_smooth(i) = s_smooth(i-1) + norm(ee_smooth_temp(i,:) - ee_smooth_temp(i-1,:));
        end
    end
    
    % --- 5. 绝对等距重采样 (切分真实的物理长度) ---
    s_even = linspace(0, s_smooth(end), num_points)';
    path_even = zeros(num_points, 4);
    for col = 1:4
        path_even(:, col) = interp1(s_smooth, path_smooth(:, col), s_even, 'linear');
    end
    
    % --- 6. 计算最终的物理总路程及步长标准差 (验证切分效果) ---
    ee_orig = ee_orig_temp; % 复用上面的计算结果
    ee_even = zeros(num_points, 3);
    for i = 1:num_points
        T = robot.fkine([path_even(i,:) 0 0]);
        ee_even(i,:) = T.t';
    end
    
    dist_orig = vecnorm(diff(ee_orig), 2, 2);
    dist_even = vecnorm(diff(ee_even), 2, 2);
    
    len_orig  = sum(dist_orig);      
    len_even  = sum(dist_even);      
    
    std_orig  = std(dist_orig, 1);   
    std_even  = std(dist_even, 1);   
    
    % 控制台反馈
    fprintf('=== Cartesian Physical Metrics (Unit: m) ===\n');
    fprintf('Original path total length        : %.6f m\n', len_orig);
    fprintf('Optimized path total length       : %.6f m\n', len_even);
    fprintf('Original path step-size std dev   : %.8e m\n', std_orig);
    fprintf('Optimized path step-size std dev  : %.8e m\n', std_even);
    
    % =====================================================================
    % 绘图对比反馈 (符合 SCI 期刊排版要求 + 混合障碍物渲染)
    % =====================================================================
    figure('Name', 'Cartesian Trajectory Smoothing & Evaluation', ...
           'Position', [200, 200, 1100, 450], 'Color', 'w');
    
    % -- 子图 (a)：3D 任务空间 --
    ax1 = subplot(1,2,1);
    hold(ax1, 'on');
    grid(ax1, 'on'); box(ax1, 'on'); axis(ax1, 'equal'); view(ax1, [-45, 30]); 
    set(ax1, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k'); 
    
    if ~isempty(obstacles)
        drawObstaclesAdvanced(ax1, obstacles); 
    end
    
    plot3(ax1, ee_orig(:,1), ee_orig(:,2), ee_orig(:,3), ...
          'r-o', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', 'Original Path');
    plot3(ax1, ee_even(:,1), ee_even(:,2), ee_even(:,3), ...
          'b-*', 'LineWidth', 2.0, 'DisplayName', 'Optimized path');
    
    scatter3(ax1, ee_orig(1,1), ee_orig(1,2), ee_orig(1,3), 60, 'ro', 'filled', 'HandleVisibility', 'off');
    scatter3(ax1, ee_orig(end,1), ee_orig(end,2), ee_orig(end,3), 60, 'go', 'filled', 'HandleVisibility', 'off');
    
    xlabel(ax1, 'Workspace X (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax1, 'Workspace Y (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel(ax1, 'Workspace Z (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    title(ax1, '(a) Cartesian Trajectory Smoothing Comparison', 'FontName', 'Times New Roman', 'FontSize', 18);
    
    if ~isempty(obstacles)
        leg = legend(ax1, 'Obstacles', 'Original Path', 'Optimized path', 'Location', 'best');
    else
        leg = legend(ax1, 'Original Path', 'Optimized path', 'Location', 'best');
    end
    set(leg, 'FontName', 'Times New Roman', 'FontSize', 11, 'EdgeColor', [0.8 0.8 0.8]);
    set(ax1, 'FontSize', 11, 'FontName', 'Times New Roman');
    hold(ax1, 'off');
    
    % -- 子图 (b)：相邻节点步长分布分析 (真实的物理米数) --
    ax2 = subplot(1,2,2);
    
    plot(ax2, dist_orig, 'r-o'); hold(ax2, 'on');
    plot(ax2, dist_even, 'b-*', 'LineWidth', 2);
    
    grid(ax2, 'on'); box(ax2, 'on');
    title(ax2, '(b) Step Size Standard Deviation Analysis', 'FontName', 'Times New Roman', 'FontSize', 18);
    
    legend_text_1 = sprintf('Original Step Size (Std = %.2e)', std_orig);
    legend_text_2 = sprintf('Optimized Step Size (Std = %.2e)', std_even);
    legend(ax2, legend_text_1, legend_text_2, 'Location', 'best');
    
    xlabel(ax2, 'Segment Index', 'FontName', 'Times New Roman', 'FontSize', 12);
    ylabel(ax2, 'Cartesian Step Distance (m)', 'FontName', 'Times New Roman', 'FontSize', 12);
    set(ax2, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    txt = {
        sprintf('Original length (m)   = %.4f', len_orig), ...
        sprintf('Optimized length (m)  = %.4f', len_even), ...
        sprintf('Original std dev      = %.2e', std_orig), ...
        sprintf('Optimized std dev     = %.2e', std_even)
    };
    text(ax2, 0.03, 0.95, txt, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontName', 'Times New Roman', ...
         'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', [0.8 0.8 0.8]);
    
    hold(ax2, 'off');
end

% =====================================================================
% 辅助绘图函数：混合几何体与点云障碍物渲染
% =====================================================================
function drawObstaclesAdvanced(ax, obstacles)
    drawn_count = 0;
    if isa(obstacles, 'pointCloud')
        locs = obstacles.Location;
        scatter3(ax, locs(:,1), locs(:,2), locs(:,3), 12, [0.9 0.8 0], 'filled', 'HandleVisibility', 'off');
        drawn_count = 1;
    elseif iscell(obstacles)
        for i = 1:length(obstacles)
            obs = obstacles{i};
            if ~iscell(obs) || isempty(obs), continue; end
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
    if drawn_count > 0
        if isa(obstacles, 'pointCloud')
            scatter3(ax, NaN, NaN, NaN, 50, [0.9 0.8 0], 'filled', 'DisplayName', 'Obstacles');
        else
            fill(ax, [NaN, NaN, NaN, NaN], [NaN, NaN, NaN, NaN], 'r', ...
                'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', 'Obstacles');
        end
    end
end