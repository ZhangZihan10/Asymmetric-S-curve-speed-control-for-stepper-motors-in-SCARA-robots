function sparse_path = Sparsify_Path2(path, min_dist, min_angle_deg, robot)
    % =====================================================================
    % 工业级 SCARA 机器人路径抽稀算法 (带正运动学任务空间映射)
    % 目标：消除 RRT 产生的密集冗余节点，释放底层电机巡航性能。
    %
    % Inputs:
    %   path: 原始避障路径，N x 4 矩阵 [Z, Y, X, T]
    %   min_dist: 最小允许距离 (建议值: 0.05 ~ 0.1)
    %   min_angle_deg: 拐角保护阈值(度) (建议值: 15)
    %   robot: 机器人对象 (用于计算末端执行器的正运动学以绘制真实空间轨迹)
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
    
    % --- 1. 关节空间下的几何抽稀逻辑 ---
    weights = [5, 1, 1]; % 关节权重统一映射
    keep_flags = false(num_points, 1);
    keep_flags(1) = true;   
    keep_flags(end) = true; 
    
    last_kept_idx = 1;
    for i = 2:(num_points - 1)
        p_last = path(last_kept_idx, 1:3) .* weights;
        p_curr = path(i, 1:3) .* weights;
        dist = norm(p_curr - p_last);
        
        v_in = path(i, 1:3) - path(last_kept_idx, 1:3);
        v_out = path(i+1, 1:3) - path(i, 1:3);
        
        n_in = norm(v_in); n_out = norm(v_out);
        angle_deg = 0;
        if n_in > 1e-5 && n_out > 1e-5
            costheta = dot(v_in, v_out) / (n_in * n_out);
            if costheta > 1, costheta = 1; elseif costheta < -1, costheta = -1; end
            angle_deg = acos(costheta) * 180 / pi;
        end
        
        if angle_deg > min_angle_deg || dist > min_dist
            keep_flags(i) = true;
            last_kept_idx = i;
        end
    end
    sparse_path = path(keep_flags, :);
    fprintf('【路径抽稀优化】原始节点: %d -> 抽稀后节点: %d\n', num_points, size(sparse_path, 1));
            
    % =====================================================================
    % SCI 期刊级学术绘图 (基于正运动学的真实三维物理空间)
    % =====================================================================
    if nargin >= 4 && ~isempty(robot)
        % 1. 计算原始 RRT 路径的末端空间坐标 (Task Space)
        orig_ee = zeros(size(path, 1), 3);
        for i = 1:size(path, 1)
            q = [path(i, :) 0 0]; % 补齐 6 轴格式匹配 fkine
            T = robot.fkine(q);
            orig_ee(i, :) = T.t';
        end
        
        % 2. 计算抽稀后路径的末端空间坐标
        sp_ee = zeros(size(sparse_path, 1), 3);
        for i = 1:size(sparse_path, 1)
            q = [sparse_path(i, :) 0 0];
            T = robot.fkine(q);
            sp_ee(i, :) = T.t';
        end
        
        % 3. 创建高分辨率白色背景画布
        figure('Name', 'End-Effector Cartesian Trajectory', 'Position', [150, 150, 800, 650], 'Color', 'w');
        hold on;
        
        % 绘制原始 RRT 末端轨迹 (浅灰色背景线)
        plot3(orig_ee(:,1), orig_ee(:,2), orig_ee(:,3), 'Color', [0.7 0.7 0.7], 'LineStyle', '-', ...
              'LineWidth', 1.2, 'Marker', '.', 'MarkerSize', 10, 'DisplayName', 'Original RRT Path');
        
        % 绘制抽稀后末端轨迹 (高亮红色，带关键节点空心圆标记)
        plot3(sp_ee(:,1), sp_ee(:,2), sp_ee(:,3), 'Color', '#D95319', 'LineStyle', '-', ...
              'LineWidth', 1.2, 'Marker', 'o', 'MarkerSize', 6, ...
              'MarkerFaceColor', 'w', 'MarkerEdgeColor', '#D95319', 'DisplayName', 'Sparsified Key Nodes');
        
        % 标记绝对物理起点和终点
        plot3(orig_ee(1,1), orig_ee(1,2), orig_ee(1,3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', '#77AC30', 'DisplayName', 'Start Position'); 
        plot3(orig_ee(end,1), orig_ee(end,2), orig_ee(end,3), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', '#0072BD', 'DisplayName', 'Target Position'); 
        
        % 4. 坐标轴、网格与视角优化
        view([-45, 30]); 
        grid on; box on;
        set(gca, 'FontName', 'Times New Roman', 'FontSize', 11, 'GridLineStyle', '--', 'GridAlpha', 0.4);
        
        % 标签已更新为实际的三维物理坐标 (米)
        xlabel('Workspace X (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Workspace Y (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
        zlabel('Workspace Z (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
        title('End-Effector Trajectory in Cartesian Space', 'FontName', 'Times New Roman', 'FontSize', 14);
        
        leg = legend('Location', 'best');
        set(leg, 'FontName', 'Times New Roman', 'FontSize', 11, 'EdgeColor', [0.8 0.8 0.8]);
        
        % 让比例尺相等，真实反映机器人在真实空间中的移动轨迹形态
        axis equal; 
        hold off;
    else
        warning('未传入 robot 对象，跳过末端执行器物理空间轨迹绘制。');
    end
end