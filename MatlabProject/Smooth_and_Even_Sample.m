function path_even = Smooth_and_Even_Sample(path, num_points, smooth_window)
    % =====================================================================
    % 机器人关节空间路径平滑与等距重采样 (为 S 曲线时间同步做完美准备)
    % Inputs:
    %   path: 原始 RRT 避障路径，N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
    %   num_points: 期望输出的均匀点数 (建议 50~60 之间)
    %   smooth_window: 高斯平滑窗口大小 (建议 20~50，越大越圆滑)
    % Output:
    %   path_even: 优化后的路径，没有任何锐角，且相邻点空间距离绝对相等
    % =====================================================================
    
    if nargin < 3, smooth_window = 30; end
    if nargin < 2, num_points = 50; end
    
    N_orig = size(path, 1);
    
    % 1. 消除连续重复的冗余点 (防止插值报错)
    diff_path = [1; sum(abs(diff(path)), 2)];
    path = path(diff_path > 1e-6, :); 
    
    % 2. 计算原始路径的累积弦长 (参数化变量 s)
    % 【关键】：Z轴单位是m，Y/X是rad。为了公平计算距离，给Z乘以10进行权重缩放
    weights = [10, 1, 1, 1]; 
    s_orig = zeros(size(path,1), 1);
    for i = 2:size(path,1)
        d = (path(i,:) - path(i-1,:)) .* weights;
        s_orig(i) = s_orig(i-1) + norm(d);
    end
    
    % 3. 密集插值与高斯平滑 (消除 RRT 的锯齿与锐角)
    % 先用 pchip (保形插值) 扩充到 1000 个密集点，防止超调撞到障碍物
    s_dense = linspace(0, s_orig(end), 1000)';
    path_dense = zeros(1000, 4);
    for col = 1:4
        path_dense(:, col) = interp1(s_orig, path(:, col), s_dense, 'pchip');
    end
    
    % 应用零相位高斯移动平均滤波，把锐角“盘圆润”
    path_smooth = smoothdata(path_dense, 1, 'gaussian', smooth_window);
    
    % 强制首尾点不漂移 (非常重要，否则机器人抓不到目标)
    path_smooth(1,:) = path(1,:);
    path_smooth(end,:) = path(end,:);
    
    % 4. 计算平滑后曲线的真实等距累积弦长
    s_smooth = zeros(1000, 1);
    for i = 2:1000
        d = (path_smooth(i,:) - path_smooth(i-1,:)) .* weights;
        s_smooth(i) = s_smooth(i-1) + norm(d);
    end
    
    % 5. 绝对等距重采样 (Equidistant Resampling)
    % 把总弧长完美均分为 num_points 份
    s_even = linspace(0, s_smooth(end), num_points)';
    path_even = zeros(num_points, 4);
    
    for col = 1:4
        % 采用线性插值提取均匀点位
        path_even(:, col) = interp1(s_smooth, path_smooth(:, col), s_even, 'linear');
    end
    
    % 绘图对比反馈 (供论文截图使用)
    figure('Name', 'Path Optimization', 'Position', [200, 200, 1000, 400], 'Color', 'w');
    
    % 子图 (a)：3D 空间轨迹对比
    subplot(1,2,1);
    plot3(path(:,3), path(:,2), path(:,1)*10, 'r-o', 'LineWidth', 1.5); hold on;
    plot3(path_even(:,3), path_even(:,2), path_even(:,1)*10, 'b-*', 'LineWidth', 1.5);
    grid on; 
    title('(a) 3D Spatial Trajectory Smoothing Comparison');
    legend('Original RRT Path', 'Gaussian Smoothing & Equidistant Resampling', 'Location', 'best');
    xlabel('X (rad)'); ylabel('Y (rad)'); zlabel('Z (scaled)');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % 子图 (b)：步长方差分析
    subplot(1,2,2);
    dist_orig = vecnorm(diff(path) .* weights, 2, 2);
    dist_even = vecnorm(diff(path_even) .* weights, 2, 2);
    plot(dist_orig, 'r-o'); hold on;
    plot(dist_even, 'b-*', 'LineWidth', 2);
    grid on; 
    title('(b) Step Size Variance Analysis of Adjacent Nodes');
    legend('Original Step Size (Uneven)', sprintf('Resampled Step Size (Uniform: %.4f)', dist_even(1)), 'Location', 'best');
    xlabel('Segment Index'); ylabel('Spatial Distance');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
end