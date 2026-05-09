function sparse_path = Sparsify_Path(path, min_dist, min_angle_deg)
    % =====================================================================
    % 工业级机器人路径抽稀算法 (Spatial Threshold & Corner Preserving)
    % 目标：消除 RRT 产生的密集微小线段，释放电机的高速巡航性能。
    %
    % Inputs:
    %   path: 原始避障路径，N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
    %   min_dist: 最小允许距离。两点间距离小于此值将被剔除 (建议值: 0.05 ~ 0.1)
    %   min_angle_deg: 拐角保护阈值(度)。即使距离很短，只要转折角度大于此值，
    %                  就视为避障关键拐角，强制保留 (建议值: 15)
    % Output:
    %   sparse_path: 抽稀后的干净路径
    % =====================================================================
    
    if nargin < 3, min_angle_deg = 15; end
    if nargin < 2, min_dist = 0.05; end
    
    num_points = size(path, 1);
    if num_points <= 2
        sparse_path = path;
        return;
    end
    
    % 为了公平计算距离，对 Z 轴(m) 和 X/Y 轴(rad) 进行权重统一映射
    % 假设 1 rad 约等于 0.2 m 的臂展位移，给 Z 放大 5 倍
    weights = [5, 1, 1]; 
    
    keep_flags = false(num_points, 1);
    keep_flags(1) = true;   % 永远保留起点
    keep_flags(end) = true; % 永远保留终点
    
    last_kept_idx = 1;
    
    for i = 2:(num_points - 1)
        % 1. 计算当前点到上一个保留点的加权空间距离
        p_last = path(last_kept_idx, 1:3) .* weights;
        p_curr = path(i, 1:3) .* weights;
        dist = norm(p_curr - p_last);
        
        % 2. 预判拐角：计算上一个向量和下一个向量的夹角
        v_in = path(i, 1:3) - path(last_kept_idx, 1:3);
        v_out = path(i+1, 1:3) - path(i, 1:3);
        
        n_in = norm(v_in); n_out = norm(v_out);
        angle_deg = 0;
        if n_in > 1e-5 && n_out > 1e-5
            costheta = dot(v_in, v_out) / (n_in * n_out);
            % 防止精度溢出
            if costheta > 1, costheta = 1; elseif costheta < -1, costheta = -1; end
            angle_deg = acos(costheta) * 180 / pi;
        end
        
        % 3. 核心裁决逻辑：
        % 如果该点是一个急转弯（角度 > min_angle_deg），强制保留！(为了安全避障)
        % 否则，如果距离已经积累得足够长（> min_dist），保留！
        if angle_deg > min_angle_deg || dist > min_dist
            keep_flags(i) = true;
            last_kept_idx = i;
        end
    end
    
    % 提取保留的路径点
    sparse_path = path(keep_flags, :);
    
    % 控制台反馈
    fprintf('【路径抽稀优化】原始节点: %d -> 抽稀后节点: %d (剔除冗余点 %d 个)\n', ...
            num_points, size(sparse_path, 1), num_points - size(sparse_path, 1));
            
    % =====================================================================
    % SCI 期刊级学术绘图部分 (3D 关节空间轨迹对比)
    % =====================================================================
    % 提取绘图坐标 (注意矩阵列的对应关系: X对应第3列, Y对应第2列, Z对应第1列)
    orig_X = path(:, 3); orig_Y = path(:, 2); orig_Z = path(:, 1);
    sp_X = sparse_path(:, 3); sp_Y = sparse_path(:, 2); sp_Z = sparse_path(:, 1);
    
    % 创建高分辨率白色背景画布
    figure('Name', 'Trajectory Sparsification Result', 'Position', [150, 150, 750, 600], 'Color', 'w');
    
    % 1. 绘制原始 RRT 路径 (低调背景色，突出原始的锯齿和密集感)
    plot3(orig_X, orig_Y, orig_Z, 'Color', [0.7 0.7 0.7], 'LineStyle', '-', ...
          'LineWidth', 1.2, 'Marker', '.', 'MarkerSize', 8);
    hold on;
    
    % 2. 绘制抽稀后的骨架路径 (高亮红色，空心圆圈标记保留的节点)
    plot3(sp_X, sp_Y, sp_Z, 'Color', '#D95319', 'LineStyle', '-', ...
          'LineWidth', 2.0, 'Marker', 'o', 'MarkerSize', 6, ...
          'MarkerFaceColor', 'w', 'MarkerEdgeColor', '#D95319');
    
    % 3. 标记起点和终点 (使用明显的绿色和蓝色方块)
    plot3(orig_X(1), orig_Y(1), orig_Z(1), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', '#77AC30'); % 起点
    plot3(orig_X(end), orig_Y(end), orig_Z(end), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', '#0072BD'); % 终点
    
    % 4. 设置视角、网格与坐标轴
    view([-45, 30]); % 设置一个易于观察三维空间特征的默认视角
    grid on;
    box on;
    
    % 5. 学术级字体与标签设置 (严格限制为 Times New Roman)
    set(gca, 'FontName', 'Times New Roman', 'FontSize', 11, 'GridLineStyle', '--', 'GridAlpha', 0.4);
    
    xlabel('Joint X (rad)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Joint Y (rad)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Joint Z (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    title('Spatial Trajectory Sparsification via RRT Smoothing', 'FontName', 'Times New Roman', 'FontSize', 14);
    
    % 6. 图例设置
    leg = legend('Original Path (Raw RRT)', 'Sparsified Path (Key Nodes)', 'Start Point', 'Target Point', ...
                 'Location', 'best');
    set(leg, 'FontName', 'Times New Roman', 'FontSize', 11, 'EdgeColor', [0.8 0.8 0.8]);
    
    hold off;
end