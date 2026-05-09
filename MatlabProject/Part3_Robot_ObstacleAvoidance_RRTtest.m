%% AGHP-RRT星 路径规划 成功 结合鱼眼相机重构系统结果(真实环境）改变逆运动学解  结合自动生成点云
%% 使用arduino控制程序
clc; clear;

%name = "Matlab";
%Client = TCPInit('127.0.0.1',55016,name);
%arduino=serialport("COM3",115200); %只需要运行1次，连接端口,现在用

% 机械臂参数
gripping_point = 0.056;
L(1) = Link([0 0 0.067 0 1], 'standard'); 
L(2) = Link([0 -0.017 0.092 0 0], 'standard');
L(3) = Link([0 -0.01 0.095 0 0], 'standard');
L(4) = Link([0 -0.04 0 0 0], 'standard');
L(5) = Link([0 0 0 0 0], 'standard');
L(6) = Link([0 0 0 0 0], 'standard');
L(1).qlim = [0.01 0.16];  
L(2).qlim = [-160 160]/180*pi;
L(3).qlim = [-160 160]/180*pi;
L(4).qlim = [0 180]/180*pi;
L(5).qlim = [0 0];  
L(6).qlim = [0 0];  

robot = SerialLink(L, 'name', 'MyRobot');

%% === 输入起点与终点的空间坐标 ===
start_pos = [0.15, -0.06, 0.02]; % 3D坐标
goal_pos  = [0.17, 0.07, 0.04]; % 3D坐标
%%
% === 通过逆解求关节角 ===
q_start = inverse_kinematics(robot, start_pos);
q_goal  = inverse_kinematics(robot, goal_pos);

 
% 如果逆解失败则退出
if isempty(q_start) || isempty(q_goal)
    error('起点或终点逆解失败，请检查坐标是否超出机械臂工作空间。');
end

% 定义障碍物
%obstacles = {
%    {'sphere', [0.2; 0; 0.1], 0.04} ,{'sphere', [0.2; 0; 0], 0.04} , {'sphere', [0.2; 0.1; 0.07], 0.04}
%};

% === 替换原来的 obstacles 定义 ===
% 假设你已经运行了之前的重构代码，得到了 ptCloud_Recon
%% 3. 自动导入、识别并生成【3D 密集云图】
ptCloud_Recon = generateObstaclePointCloud();
obstacles = ptCloud_Recon;

%% 1. 基础合法性检查（逆解是否存在）
if isempty(q_start) || isempty(q_goal)
    error('起点或终点逆解失败，请检查坐标是否超出机械臂工作空间。');
end

% 2. 核心修正：初始碰撞检测
% 检测起点是否碰撞
if checkCollision2(robot, q_start, q_start, obstacles)
    fprintf('初始起点构型碰撞，尝试切换构型...\n');
    found_safe = false;
    
    q_candidates = ikine_scara_analytic(robot, start_pos);

    if isempty(q_candidates)
        error('起点超出机械臂物理工作空间！');
    end
    
    found_safe = false;
    for r = 1:size(q_candidates, 1)
        q_try = q_candidates(r, :);
        % 检查当前构型（整根连杆）是否碰撞
        if ~checkCollision2(robot, q_try, q_try, obstacles)
            q_start = q_try;
            found_safe = true;
            fprintf('成功找到安全起点构型：解编号 %d\n', r);
            break;
        end
    end
    
    if ~found_safe
        error('起点位置正确，但“左手系”和“右手系”两种姿态均会碰撞障碍物。');
    end
   
end

% 检测目标点是否碰撞
% 2. 核心修正：初始碰撞检测
% 检测起点是否碰撞
if checkCollision2(robot, q_goal, q_goal, obstacles)
    fprintf('目标点构型碰撞，尝试切换构型...\n');
    found_safe = false;
    
    q_candidates = ikine_scara_analytic(robot, goal_pos);

    if isempty(q_candidates)
        error('目标超出机械臂物理工作空间！');
    end
    
    found_safe = false;
    for r = 1:size(q_candidates, 1)
        q_try = q_candidates(r, :);
        % 检查当前构型（整根连杆）是否碰撞
        if ~checkCollision2(robot, q_try, q_try, obstacles)
            q_goal = q_try;
            found_safe = true;
            fprintf('成功找到安全目标点构型：解编号 %d\n', r);
            break;
        end
    end
    
    if ~found_safe
        error('起点位置正确，但“左手系”和“右手系”两种姿态均会碰撞障碍物。');
    end
   
end

%numberTran3(arduino,q_start(1,1),q_start(1,2),q_start(1,3),q_start(1,4));%将计算结果转换为电机参数,并传入arduino

%% PH-RRT参数设置
t = 0.05;         % 均匀概率阈值 (论文推荐0.15~0.3)
rho2_ratio = 0.01; % 目标重力步长比例
rho3_ratio = 0.02 ; % 随机点步长比例 (ρ2/ρ3=1/2 符合论文)

% 开始计时
fprintf('开始PH-RRT路径规划...\n');
start_time = tic;

% 路径规划
[path, tree] = aghp_rrt_star_planning(robot, q_start, q_goal, obstacles, t, rho2_ratio, rho3_ratio);

% 结束计时
elapsed_time = toc(start_time);
fprintf('路径规划完成，耗时: %.4f 秒\n', elapsed_time);


if ~isempty(path)
    fprintf('路径规划成功，共 %d 个节点\n', size(path,1));
    % 计算关节空间路径长度
    joint_path_length = compute_joint_path_length(path);
    fprintf('关节空间路径长度: %.4f rad\n', joint_path_length);
    % 计算任务空间末端轨迹长度
    cartesian_path_length = compute_cartesian_path_length(robot, path);
    fprintf('任务空间末端轨迹长度: %.4f m\n', cartesian_path_length);

    plot3_path(robot, path, obstacles);
    plot3_rrt_tree(robot, tree, path,false);
    plot3_rrt_robot_poses(robot, path, obstacles);  % 所有节点下机械臂姿态
    
    %b = 1;
    %for a = 1 : length(path)
    %numberTran3(arduino,path(61,1),path(61,2),path(61,3),path(61,4));
    %    b=b+1;     
    %end

else
    disp('路径规划失败');
end


%% 路径传给机器人
%★ 插入平滑与等距重采样 ★
% 将折线提纯为 50 个绝对均匀的平滑点，高斯平滑窗口设为 30
% 2. ★ 插入路径抽稀模块 ★
% 参数 0.05 代表综合距离阈值，15 代表 15 度的拐弯保护
path_sparse = Sparsify_Path2(path, 0.05, 30,robot);

% (可选) 3. 如果你还需要节点分布绝对均匀，可以再过一遍之前的等距重采样
path_optimized = Smooth_and_Even_Sample2_2(path_sparse, 27, 60,robot, obstacles);
plot3_rrt_robot_poses(robot, path_optimized, obstacles);  % 所有节点下机械臂姿态


%numberTran4(arduino,q_start(1,1),q_start(1,2),q_start(1,3),q_start(1,4));
%numberTran4(arduino,path(:, 1),path(:, 2),path(:, 3),path(:, 4));
%numberTran5(arduino,[q_start(1,1),q_start(1,2),q_start(1,3),q_start(1,4)]);
%numberTran5(arduino,path_optimized);
%numberTran5，numberTran6 改进算法，先将路径数据传入单片机，再进行运行控制。6带有绘图功能，但都会顿挫
%numberTran7 (失败)S曲线(变加速) 动力学控制与完美绘图版 (适配 Arduino 动态速度融合引擎)。J1运行正常，其它关节顿挫
%numberTran8  （成功）终极时间同步 S 曲线控制 (适配 Arduino 时间驱动引擎)
%numberTran9    （成功）运行更顺畅
%numberTran9_2   （成功）各关节分布控制

%arduino=serialport("COM3",115200); %只需要运行1次，连接端口,现在用


%numberTran5(arduino, path_optimized);
      %numberTran8(arduino, path_to_start, true);
      %numberTran8(arduino,[path_optimized(1,1),path_optimized(1,2),path_optimized(1,3),path_optimized(1,4)]);

%numberTran9(arduino,path_optimized);  %numberTran9_2_Plot(path_optimized); %画图部分
%numberTran9_2(arduino,path_optimized);   %EE_traj =numberTran9_2_Plot2(path_optimized); %改进方法画图部分

%EE_traj2 =Baseline_Trapezoidal_Plot3(path_optimized);   %原始方法下画图
%Plot_Trajectory_Comparison(EE_traj, EE_traj2);  %两种方法下空间轨迹对比



function [path, tree] = aghp_rrt_star_planning(robot, q_start, q_goal, obstacles, t, rho2_ratio, rho3_ratio, max_iter)
    if nargin < 8, max_iter = 2000; end
    threshold = 0.08;   % 到达目标的阈值
    step_size = 0.04;   % 减小步长以适应 Z 轴精细避障
    grid_resolution = 0.04; 
    W = [2.0, 1.0, 1.0, 1.0]; % 强化 Z 轴在代价和搜索中的权重
    
    grid_map_xyz = containers.Map('KeyType','char','ValueType','any');
    tree = struct('q', q_start, 'parent', 0, 'cost', 0);
    
    % 初始化
    xyz_start = get_xyz(robot, q_start);
    grid_map_xyz(get_grid_key_xyz(xyz_start, grid_resolution)) = [1];
    open_list = [1];
    found = false;

    for i = 1:max_iter
        % --- 1. 动态自适应采样 ---
        %p = t + (1 - t) * exp(-length(open_list) / 250);
        p = t + (1 - t) * (1 - exp(-length(open_list) / 250));
        k = rand();
        
        if k < p
            q_target = q_goal;
        elseif k < p + 0.15 % 15% 概率强制向上采样（解决 Z 轴探索不足）
            q_target = sample_random_q(robot);
            q_target(1) = robot.links(1).qlim(2); % 强制拉到最大高度
        else
            % 任务空间随机采样
            x_rnd = rand_range(-0.3, 0.3);
            y_rnd = rand_range(-0.3, 0.3);
            z_rnd = rand_range(0.08, 0.25); % 适配 q1 限位 [0.01, 0.2]
            qs = ikine_scara_analytic(robot, [x_rnd, y_rnd, z_rnd]);
            if isempty(qs), continue; end
            q_target = qs(randi(size(qs,1)), :);
        end

        % --- 2. 寻找最近节点 (使用加权距离) ---
        [q_near, idx_near] = find_nearest_grid(tree, grid_map_xyz, robot, q_target, grid_resolution);
        
        % --- 3. 启发式步进 (Heuristic Step) ---
        direction = q_target - q_near;
        q_new = q_near + step_size * normalize_vector(direction);
        q_new = clamp_to_limits(robot, q_new);

        % --- 4. 碰撞检测与主动避障重试 ---
        if checkCollision2(robot, q_near, q_new, obstacles)
            % 如果直接向目标走碰撞了，尝试“向上垂直抬升”避障
            q_up = q_near;
            q_up(1) = q_near(1) + step_size; % 尝试在 Z 轴向上迈一步
            q_up = clamp_to_limits(robot, q_up);
            
            if ~checkCollision2(robot, q_near, q_up, obstacles)
                q_new = q_up; % 避障策略切换：优先向上走
            else
                continue; % 向上也走不通，放弃该节点
            end
        end

        % --- 5. RRT* Rewiring (加权距离版) ---
        [neighbors, neighbor_idxs] = get_neighbors(tree, q_new, grid_map_xyz, robot, grid_resolution, 0.15);
        min_dist_new = norm((q_new - q_near) .* W);
        min_cost = tree(idx_near).cost + min_dist_new;
        best_parent = idx_near;

        for j = 1:length(neighbors)
            cost_j = tree(neighbor_idxs(j)).cost + norm((q_new - neighbors{j}) .* W);
            if cost_j < min_cost && ~checkCollision2(robot, neighbors{j}, q_new, obstacles)
                min_cost = cost_j;
                best_parent = neighbor_idxs(j);
            end
        end

        % 插入树
        tree(end+1) = struct('q', q_new, 'parent', best_parent, 'cost', min_cost);
        new_idx = length(tree);
        
        % 更新索引和 OpenList
        xyz_new = get_xyz(robot, q_new);
        key = get_grid_key_xyz(xyz_new, grid_resolution);
        if isKey(grid_map_xyz, key), grid_map_xyz(key) = [grid_map_xyz(key), new_idx];
        else, grid_map_xyz(key) = [new_idx]; end
        open_list(end+1) = new_idx;

        % Rewire 邻居
        for j = 1:length(neighbors)
            idx = neighbor_idxs(j);
            cost_through_new = min_cost + norm((neighbors{j} - q_new) .* W);
            if cost_through_new < tree(idx).cost && ~checkCollision2(robot, q_new, neighbors{j}, obstacles)
                tree(idx).parent = new_idx;
                tree(idx).cost = cost_through_new;
            end
        end

        % --- 6. 检查是否到达目标 ---
        if norm((q_new - q_goal) .* [1, 0.1, 0.1, 0]) < threshold && ~checkCollision2(robot, q_new, q_goal, obstacles)
            tree(end+1) = struct('q', q_goal, 'parent', new_idx, 'cost', min_cost + norm((q_goal - q_new) .* W));
            found = true;
            break;
        end
    end
    
    % 生成路径 (逻辑保持不变)
    if found, path = trace_path(tree); else, path = []; end
end


function v = normalize_vector(v)
    if norm(v) > 0
        v = v / norm(v);
    end
end



function [q_near, idx_near] = find_nearest_grid(tree, grid_map_xyz, robot, q_rand, res)
    xyz = get_xyz(robot, q_rand);
    neighbor_keys = generate_neighbor_keys_xyz(xyz, res, 1);
    min_dist = inf;
    idx_near = -1;

    W = [2.0, 1.0, 1.0, 1.0];

    for i = 1:length(neighbor_keys)
        key = neighbor_keys{i};
        if isKey(grid_map_xyz, key)
            idx_list = grid_map_xyz(key);
            for j = 1:length(idx_list)
                q = tree(idx_list(j)).q;
                d = norm((q_rand - q) .* W);
                if d < min_dist
                    min_dist = d;
                    idx_near = idx_list(j);
                end
            end
        end
    end

    % 如果局部格子里没找到，回退到全局搜索
    if idx_near == -1
        for k = 1:length(tree)
            q = tree(k).q;
            d = norm((q_rand - q) .* W);
            if d < min_dist
                min_dist = d;
                idx_near = k;
            end
        end
    end

    q_near = tree(idx_near).q;
end

function [neighbors, idxs] = get_neighbors(tree, q_new, grid_map_xyz, robot, res, radius)
    xyz = get_xyz(robot, q_new);
    neighbor_keys = generate_neighbor_keys_xyz(xyz, res, ceil(radius/res));
    neighbors = {};
    idxs = [];

    % --- 权重向量需与 find_nearest_grid 保持一致 ---
    W = [2.0, 1.0, 1.0, 1.0];

    for i = 1:length(neighbor_keys)
        key = neighbor_keys{i};
        if isKey(grid_map_xyz, key)
            idx_list = grid_map_xyz(key);
            for j = 1:length(idx_list)
                q = tree(idx_list(j)).q;
                %if norm(q - q_new) < radius
                % --- 修改此处：使用加权距离判断半径 ---
                if norm((q - q_new) .* W) < radius
                    neighbors{end+1} = q;
                    idxs(end+1) = idx_list(j);
                end
            end
        end
    end
end

function xyz = get_xyz(robot, q)
    T = robot.fkine([q 0 0]);
    xyz = T.t';
end

function key = get_grid_key_xyz(xyz, res)
    idx = floor(xyz / res);
    key = sprintf('%d_%d_%d', idx(1), idx(2), idx(3));
end

function keys = generate_neighbor_keys_xyz(xyz, res, range)
    center = floor(xyz / res);
    keys = {};
    for dx = -range:range
        for dy = -range:range
            for dz = -range:range
                idx = center + [dx dy dz];
                key = sprintf('%d_%d_%d', idx(1), idx(2), idx(3));
                keys{end+1} = key;
            end
        end
    end
end

%% ====== 辅助函数 ======
function q = clamp_to_limits(robot, q)
    % 确保关节角度在限位范围内
    for j = 1:4
        qlim = robot.links(j).qlim;
        if q(j) < qlim(1)
            q(j) = qlim(1);
        elseif q(j) > qlim(2)
            q(j) = qlim(2);
        end
    end
end

%% ====== 以下函数保持不变 ======
% 逆解函数（返回4维角度向量）
function q = inverse_kinematics(robot, pos)
    T = transl(pos);       % 目标位姿（无姿态要求）
    % 随机生成一个符合关节限位的初始猜想值 q0
    q0 = sample_random_q(robot);
    try
        %q_full = robot.ikine(T, 'mask', [1 1 1 1 0 0]); % 只考虑位置
        %q = q_full(1:4);    % 只保留前4个关节
        % 传入 q0，让每次搜索的起点都不一样
        q_full = robot.ikine(T, q0, 'mask', [1 1 1 1 0 0]); 
        q = q_full(1:4);
    catch
        q = [];
    end
end

function q = sample_random_q(robot)
    q = zeros(1,4);
    for i = 1:4
        qlim = robot.links(i).qlim;
        q(i) = qlim(1) + rand * (qlim(2) - qlim(1));
    end
end


function collision = checkCollision(robot, q1, q2, obstacles)
    steps = 10;
    for i = 0:steps
        q = q1 + (q2 - q1) * i / steps;
        q_full = [q 0 0];

        % 获取每段连杆的起点与终点位置
        points = zeros(3, 5); % 4连杆 + base
        points(:,1) = [0;0;0];  % 基座点

        for j = 1:4
            Tj = robot.A(1:j, q_full);
            points(:,j+1) = Tj.t;
        end

        % 对每一段连杆（线段）进行检测
        for k = 1:4
            p1 = points(:,k);
            p2 = points(:,k+1);

            for m = 1:length(obstacles)
                obs = obstacles{m};
                if strcmp(obs{1}, 'sphere')
                    center = obs{2};
                    radius = obs{3};

                    % 点到线段距离小于半径 -> 碰撞
                    d = point_to_segment_distance(center, p1, p2);
                    if d < radius
                        collision = true;
                        return;
                    end
                end
            end
        end
    end
    collision = false;
end

function collision = checkCollision2(robot, q1, q2, ptCloud_Obs)
    steps = 15; % 插值步长（关节空间移动的平滑度）
    link_samples = 8; % 关键改进：在每根连杆上均匀取 8 个点进行检测
    
    % 根据你机械臂的实际粗细设定（单位：米）
    % 建议设为：连杆物理半径 + 视觉误差裕量(约0.02)
    link_radius = 0.03; 

    for i = 0:steps
        q = q1 + (q2 - q1) * i / steps;
        q_full = [q 0 0];
        
        % 获取各关节点坐标
        points = zeros(3, 5); 
        points(:,1) = [0;0;0];  
        for j = 1:4
            Tj = robot.A(1:j, q_full);
            points(:,j+1) = Tj.t;
        end
        
        % 遍历 4 段连杆
        for k = 1:4
            p_start = points(:,k);
            p_end = points(:,k+1);
            
            % --- 改进点：对整根连杆进行多点采样 ---
            for s = 0:(1/link_samples):1
                test_point = p_start + s * (p_end - p_start);
                
                % 搜索点云中距离当前采样点最近的点
                [~, dists] = knnsearch(ptCloud_Obs.Location, test_point');
                
                % 如果距离小于连杆半径，判定碰撞
                if min(dists) < link_radius
                    collision = true;
                    return;
                end
            end
        end
    end
    collision = false;
end

function d = point_to_segment_distance(pt, a, b)
    ab = b - a;
    t = dot(pt - a, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(pt - proj);
end

function plot3_path(robot, path, obstacles)
    figure('Color', 'w');
    hold on;
    drawObstacles2(obstacles);
    
    view(3);
    grid on;
    axis([-0.3 0.3 -0.3 0.3 0 0.3]); % 按照机械臂尺度设定
    axis equal; % 保持比例，防止变形
    % 用于记录末端执行器轨迹
    trajectory = [];

    for i = 1:size(path,1)
        q = [path(i,:) 0 0];  % 填补无效关节
        robot.plot(q, 'workspace', [-0.3 0.3 -0.3 0.3 -0.3 0.3], 'delay', 0.05);
        T = robot.fkine(q);
        trajectory = [trajectory; T.t'];  % 末端位置加入轨迹
        drawnow;
    end

    % 绘制末端轨迹
    plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'b-', 'LineWidth', 2);
    scatter3(trajectory(end,1), trajectory(end,2), trajectory(end,3), 50, 'g', 'filled'); % 终点
    scatter3(trajectory(1,1), trajectory(1,2), trajectory(1,3), 50, 'r', 'filled');       % 起点

    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
    title('机械臂运动路径与末端轨迹');
    grid on;
end

function drawObstacles(obstacles)
    for i = 1:length(obstacles)
        obs = obstacles{i};
        switch obs{1}
            case 'sphere'
                center = obs{2};
                radius = obs{3};
                [x, y, z] = sphere(20);
                x = x * radius + center(1);
                y = y * radius + center(2);
                z = z * radius + center(3);
                surf(x, y, z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'r');

            case 'cylinder'
                center = obs{2};
                radius = obs{3};
                height = obs{4};
                [x, y, z] = cylinder(radius, 20);
                z = z * height; 
                z = z - height/2 + center(3);
                surf(x + center(1), y + center(2), z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'b');
        end
    end
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(3);
end

function drawObstacles2(obstacles)
    % 获取当前坐标轴，并强制清理环境
    ax = gca;
    hold(ax, 'on'); 
    
    % --- 设置全局视觉环境：白色背景 ---
    set(gcf, 'Color', 'w');      % 窗口背景设为白色
    set(ax, 'Color', 'w');       % 坐标轴背景设为白色
    set(ax, 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k'); % 刻度设为黑色
    grid(ax, 'on');
    
    if isa(obstacles, 'pointCloud')
        % --- 关键更改：将点云显示颜色设为黄色 [1 1 0] ---
        % 创建一个与点数一致的 RGB 矩阵，每一行都是 [1, 1, 0]
        num_pts = obstacles.Count;
        yellow_color = repmat([1 1 0], num_pts, 1); 
        
        % 绘制黄色点云，并增大 MarkerSize 以便在白背景下看得更清
        pcshow(obstacles.Location, yellow_color, 'MarkerSize', 12, 'Parent', ax);
        
    else
        % --- 几何体障碍物绘制逻辑 (保持不变) ---
        for i = 1:length(obstacles)
            obs = obstacles{i};
            switch obs{1}
                case 'sphere'
                    [x, y, z] = sphere(20);
                    surf(ax, x*obs{3}+obs{2}(1), y*obs{3}+obs{2}(2), z*obs{3}+obs{2}(3), ...
                        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'r');
                case 'cylinder'
                    [x, y, z] = cylinder(obs{3}, 20);
                    z = z * obs{4} - obs{4}/2 + obs{2}(3);
                    surf(ax, x+obs{2}(1), y+obs{2}(2), z, ...
                        'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'b');
            end
        end
    end
    
    % 视角与缩放优化
    axis(ax, 'equal');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    view(ax, 3);
end

%plot3_rrt_tree(robot, tree, path,false);
function plot3_rrt_tree(robot, tree, path, show_cube)
    if nargin < 4
        show_cube = true;  % 默认显示立方体栅格
    end

    figure;
    hold on;
    view(3);
    grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('RRT 采样树及任务空间格栅');

    grid_resolution = 0.1;  % 与路径规划中的保持一致
    grid_set = containers.Map();

   

    % ==== 绘制 RRT 树结构 ====
    for i = 2:length(tree)
        q1 = [tree(i).q 0 0];
        q2 = [tree(tree(i).parent).q 0 0];
        T1 = robot.fkine(q1);
        T2 = robot.fkine(q2);
        p1 = T1.t;
        p2 = T2.t;
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'k-', 'LineWidth', 1, 'DisplayName', 'RRT树连接');
    end

    % ==== 绘制最终路径 ====
    if ~isempty(path)
        ee_path = zeros(size(path,1), 3);
        for i = 1:size(path,1)
            q = [path(i,:) 0 0];
            T = robot.fkine(q);
            ee_path(i,:) = T.t';
        end
        plot3(ee_path(:,1), ee_path(:,2), ee_path(:,3), 'r-', 'LineWidth', 3, 'DisplayName', '最佳路径轨迹');
    end

    % ==== 起点终点标记 ====
    T_start = robot.fkine([tree(1).q 0 0]);
    scatter3(T_start.t(1), T_start.t(2), T_start.t(3), 100, 'g', 'filled', 'DisplayName', '起点');
    T_end = robot.fkine([tree(end).q 0 0]);
    scatter3(T_end.t(1), T_end.t(2), T_end.t(3), 100, 'r', 'filled', 'DisplayName', '终点');

    legend('Location','bestoutside');
end


function plot3_rrt_robot_poses(robot, path, obstacles)
    figure;
    hold on;
    grid on;
    view(3);
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('机械臂最终路径姿态及运动轨迹');

    % 绘制障碍物
    drawObstacles2(obstacles);

    % 绘制路径中机械臂各节点姿态（用线条连杆）
    n = size(path,1);
    colors = jet(n); % 用渐变色显示轨迹顺序

    for i = 1:n
         q = [path(i,:) 0 0]; % 补两个无效关节
        % 计算各关节坐标，用forward kinematics分段求各关节位置
        joint_positions = zeros(3, robot.n+1); % 每列一个关节位置 (x,y,z)
        T = eye(4);
        joint_positions(:,1) = T(1:3,4); % 基座坐标原点
        for j = 1:robot.n
            T = T * robot.A(j, q).T;  % 关键修复点
            joint_positions(:, j+1) = T(1:3,4);
        end

        % 绘制连杆
        for j = 1:robot.n
            plot3(joint_positions(1, j:j+1), joint_positions(2, j:j+1), joint_positions(3, j:j+1), ...
                'Color', colors(i,:), 'LineWidth', 2);
        end
    end
    % 计算起点末端执行器位置
    q_start = [path(1,:) 0 0];
    T_start = robot.fkine(q_start);
    pos_start = T_start.t;
    scatter3(pos_start(1), pos_start(2), pos_start(3), 100, 'r', 'filled');
    text(pos_start(1), pos_start(2), pos_start(3), '  Start EE', 'Color', 'g');

    % 计算终点末端执行器位置
    q_end = [path(end,:) 0 0];
    T_end = robot.fkine(q_end);
    pos_end = T_end.t;
    scatter3(pos_end(1), pos_end(2), pos_end(3), 100, 'g', 'filled');
    text(pos_end(1), pos_end(2), pos_end(3), '  Goal EE', 'Color', 'r');

    colorbar;
    colormap(jet);
    caxis([1 n]);
    hold off;
end


function length = compute_joint_path_length(path)
    % 计算关节空间路径长度（各关节角度变化总和）
    length = 0;
    for i = 2:size(path, 1)
        delta = path(i, :) - path(i-1, :);
        length = length + norm(delta);
    end
end


function length = compute_cartesian_path_length(robot, path)
    % 计算任务空间末端轨迹长度
    length = 0;
    prev_pos = [];
    
    for i = 1:size(path, 1)
        q_full = [path(i, :) 0 0];
        T = robot.fkine(q_full);
        current_pos = T.t';
        
        if ~isempty(prev_pos)
            delta = current_pos - prev_pos;
            length = length + norm(delta);
        end
        
        prev_pos = current_pos;
    end
end

function q_all = ikine_scara_analytic(robot, pos)
    % 初始化返回值，防止“变量未赋值”错误
    q_all = []; 
    
    x = pos(1)-0.07; y = pos(2); z = pos(3);
    a1 = robot.links(2).a; 
    a2 = robot.links(3).a;
    d1_offset = 0.067;
    
    % 1. 计算关节 1 (Z 轴)
    % 注意：请确认是 z - 0.067 还是 z + 0.067，通常向上升起是减去基座高
    q1 = z + d1_offset; 
    
    % 2. 范围检查：如果 q1 不在限位内，直接返回空的 q_all
    if q1 < 0.01 || q1 > 0.156
        return; % 此时 q_all 已经是 []
    end
    
    % 3. 计算关节 3 几何关系
    D = (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2);
    
    % 如果超出臂展物理极限，直接返回
    if abs(D) > 1
        return; 
    end
    
    % 4. 计算解析解（左手与右手构型）
    q3_1 = acos(D);
    q2_1 = atan2(y, x) - atan2(a2*sin(q3_1), a1 + a2*cos(q3_1));
    
    q3_2 = -acos(D);
    q2_2 = atan2(y, x) - atan2(a2*sin(q3_2), a1 + a2*cos(q3_2));
    
    % 5. 组装并再次检查关节旋转限位 (L2, L3)
    q_candidates = [q1, q2_1, q3_1, 0;
                    q1, q2_2, q3_2, 0];
    
    q2_lim = robot.links(2).qlim;
    q3_lim = robot.links(3).qlim;
    
    for i = 1:2
        q_test = q_candidates(i,:);
        if q_test(2) >= q2_lim(1) && q_test(2) <= q2_lim(2) && ...
           q_test(3) >= q3_lim(1) && q_test(3) <= q3_lim(2)
            q_all = [q_all; q_test];
        end
    end
end


% 任务空间范围随机数生成
function val = rand_range(min_val, max_val)
    val = min_val + (max_val - min_val) * rand();
end

% 路径回溯
function path = trace_path(tree)
    path = [];
    idx = length(tree);
    while idx ~= 0
        path = [tree(idx).q; path];
        idx = tree(idx).parent;
    end
end
