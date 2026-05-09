function Plot_Trajectory_Comparison(EE_traj, EE_traj2)
    % =====================================================================
    % 机械臂末端 3D 物理空间轨迹对比 (SCI 期刊/国际会议展示专用)
    % Inputs:
    %   EE_traj  : 改进算法 (S曲线+时间同步) 的末端轨迹 [N x 3]
    %   EE_traj2 : 基线算法 (传统梯形+独立未同步) 的末端轨迹 [M x 3]
    % =====================================================================
    
    if nargin < 2
        error('需要输入两条轨迹数据：EE_traj 和 EE_traj2');
    end

    % 提取起点和终点 (假设两条轨迹的物理起终点一致)
    start_pt = EE_traj(1, :);
    end_pt = EE_traj(end, :);

    % 创建高分辨率宽屏画布
    figure('Name', '3D Cartesian Trajectory Comparison', 'Position', [100, 150, 1200, 550], 'Color', 'w');

    % ==========================================================
    % 子图 (a)：3D 空间全景对比
    % ==========================================================
    ax1 = subplot(1, 2, 1);
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on'); axis(ax1, 'equal');
    
    % 绘制基线算法轨迹 (红色虚线，突出其偏离和粗糙)
    plot3(ax1, EE_traj2(:,1), EE_traj2(:,2), EE_traj2(:,3), 'r-', 'LineWidth', 2, ...
          'DisplayName', 'Baseline: Unsynchronized Trapezoidal');
      
    % 绘制改进算法轨迹 (蓝色实线，突出其平滑和精准)
    plot3(ax1, EE_traj(:,1), EE_traj(:,2), EE_traj(:,3), 'b-', 'LineWidth', 2.5, ...
          'DisplayName', 'Proposed: Time-Synchronized S-Curve');
      
    % 标记起点和终点
    scatter3(ax1, start_pt(1), start_pt(2), start_pt(3), 80, 'ko', 'filled', 'DisplayName', 'Start Point');
    scatter3(ax1, end_pt(1), end_pt(2), end_pt(3), 80, 'k^', 'filled', 'DisplayName', 'Target Point');

    % 视角与标签设置
    view(ax1, [-45, 30]); % 经典等轴测视角
    title(ax1, '(a) 3D Spatial Trajectory Comparison', 'FontName', 'Times New Roman', 'FontSize', 16);
    xlabel(ax1, 'Workspace X (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax1, 'Workspace Y (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel(ax1, 'Workspace Z (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    leg1 = legend(ax1, 'Location', 'best');
    set(leg1, 'FontName', 'Times New Roman', 'FontSize', 11, 'EdgeColor', [0.8 0.8 0.8]);
    set(ax1, 'FontSize', 11, 'FontName', 'Times New Roman');

    % ==========================================================
    % 子图 (b)：X-Y 平面俯视对比 (凸显同步性带来的路径偏差)
    % ==========================================================
    ax2 = subplot(1, 2, 2);
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on'); axis(ax2, 'equal');
    
    % 绘制基线轨迹投影
    plot(ax2, EE_traj2(:,1), EE_traj2(:,2), 'r--', 'LineWidth', 1.8, ...
         'DisplayName', 'Baseline Path');
     
    % 绘制改进轨迹投影
    plot(ax2, EE_traj(:,1), EE_traj(:,2), 'b-', 'LineWidth', 2.5, ...
         'DisplayName', 'Proposed Path');
     
    % 标记起终点投影
    scatter(ax2, start_pt(1), start_pt(2), 80, 'ko', 'filled', 'HandleVisibility', 'off');
    scatter(ax2, end_pt(1), end_pt(2), 80, 'k^', 'filled', 'HandleVisibility', 'off');

    % 标签设置
    title(ax2, '(b) X-Y Plane Projection (Top View)', 'FontName', 'Times New Roman', 'FontSize', 16);
    xlabel(ax2, 'Workspace X (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel(ax2, 'Workspace Y (m)', 'FontName', 'Times New Roman', 'FontSize', 12, 'FontWeight', 'bold');
    
    leg2 = legend(ax2, 'Location', 'best');
    set(leg2, 'FontName', 'Times New Roman', 'FontSize', 11, 'EdgeColor', [0.8 0.8 0.8]);
    set(ax2, 'FontSize', 11, 'FontName', 'Times New Roman');

    % 自动调整子图间距
    sgtitle('End-Effector Spatial Tracking Performance Evaluation', 'FontName', 'Times New Roman', 'FontSize', 18, 'FontWeight', 'bold');
    
    disp('轨迹对比图表绘制完成！');
end