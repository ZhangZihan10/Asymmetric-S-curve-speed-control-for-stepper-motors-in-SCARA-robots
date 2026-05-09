%% AGHP-RRT星 路径规划 成功 结合鱼眼相机重构系统结果
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
L(1).qlim = [0.01 0.13];  
L(2).qlim = [-160 160]/180*pi;
L(3).qlim = [-160 160]/180*pi;
L(4).qlim = [0 180]/180*pi;
L(5).qlim = [0 0];  
L(6).qlim = [0 0];  

robot = SerialLink(L, 'name', 'MyRobot');

% === 输入起点与终点的空间坐标 ===
start_pos = [0.08, 0.1, 0.07]; % 3D坐标
goal_pos  = [0.07, -0.15, 0.08]; % 3D坐标

% === 通过逆解求关节角 ===
q_start = inverse_kinematics(robot, start_pos);
q_goal  = inverse_kinematics(robot, goal_pos);

% 如果逆解失败则退出
if isempty(q_start) || isempty(q_goal)
    error('起点或终点逆解失败，请检查坐标是否超出机械臂工作空间。');
end

% 定义障碍物
obstacles = {
    {'sphere', [0.2; -0.1; 0.1], 0.05} ,{'sphere', [0.2; 0; 0], 0.04} , {'sphere', [0.2; 0.1; 0.07], 0.04}
};

%obstacles = {'cylinder', [0.18; 0.05; 0.1], 0.03, 0.15};

% PH-RRT参数设置
t = 0.2;         % 均匀概率阈值 (论文推荐0.15~0.3)
rho2_ratio = 0.4; % 目标重力步长比例
rho3_ratio = 0.8; % 随机点步长比例 (ρ2/ρ3=1/2 符合论文)

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
path_sparse = Sparsify_Path2(path, 0.05, 30,robot);
path_optimized = Smooth_and_Even_Sample3_2(path_sparse, 27, 60,robot, obstacles);
plot3_path(robot, path_optimized, obstacles);
plot3_rrt_robot_poses(robot, path_optimized, obstacles);  % 所有节点下机械臂姿态

%numberTran4原始传输算法，实时传输路径，速度较慢
%numberTran5，numberTran6 改进算法，先将路径数据传入单片机，再进行运行控制。6带有绘图功能
%numberTran7(失败）全局 S 曲线速度包络映射，先将路径数据传入单片机，再进行运行控制。6带有绘图功能
%numberTran9_2(arduino,path_optimized);  最终版本

%numberTran4(arduino,q_start(1,1),q_start(1,2),q_start(1,3),q_start(1,4));
%numberTran4(arduino,path(:, 1),path(:, 2),path(:, 3),path(:, 4));
%numberTran4(arduino,path_optimized(:, 1),path_optimized(:, 2),path_optimized(:, 3),path_optimized(:, 4));


%arduino=serialport("COM3",115200); %只需要运行1次，连接端口,现在用

%q_zero = [0.05676, 2.01058, -2.35024, 0];
%path_to_start = [
%    q_zero;
%    q_start(1,1), q_start(1,2), q_start(1,3), q_start(1,4)
%];


%numberTran6(arduino, path_to_start);
      %numberTran8(arduino, path_to_start, true);
      %numberTran6(arduino,[q_start(1,1),q_start(1,2),q_start(1,3),q_start(1,4)]);
      %numberTran6(arduino,path_optimized);
%numberTran9_2(arduino,path_optimized);   %EE_traj =numberTran9_2_Plot2(path_optimized); %改进方法画图部分
%EE_traj2 =Baseline_Trapezoidal_Plot3(path_optimized)   %原始方法下画图
%Plot_Trajectory_Comparison(EE_traj, EE_traj2);  %两种方法下空间轨迹对比
%% ====== AGHP-RRT 核心函数 ======
function [path, tree] = aghp_rrt_star_planning(robot, q_start, q_goal, obstacles, t, rho2_ratio, rho3_ratio, max_iter)
    if nargin < 8
        max_iter = 50000;
    end

    threshold = 0.05;   % 到达目标的阈值
    step_size = 0.05;   % 每次扩展步长
   % dim = 4;            % 有效维度
    grid_resolution = 0.1;  % 任务空间栅格分辨率
    grid_map_xyz = containers.Map('KeyType','char','ValueType','any');

    tree = struct('q', q_start, 'parent', 0, 'cost', 0);
    xyz_start = get_xyz(robot, q_start);
    key_start = get_grid_key_xyz(xyz_start, grid_resolution);
    grid_map_xyz(key_start) = [1];
    open_list = [1];
    found = false;

    for i = 1:max_iter
        open_node_count = length(open_list);
        open_max = 200;
        p = t + (1 - t) * exp(-open_node_count / open_max);
        k = rand();

        if k < p
            q_rand = q_goal;
        else
            q_rand = sample_random_q(robot);
        end

        [q_near, idx_near] = find_nearest_grid(tree, grid_map_xyz, robot, q_rand, grid_resolution);

        if k < p
            direction = normalize_vector(q_goal - q_near);
            q_new = q_near + step_size * direction;
        else
            direction_goal = normalize_vector(q_goal - q_near);
            direction_rand = normalize_vector(q_rand - q_near);
            X1 = rho2_ratio * step_size * direction_goal;
            X2 = rho3_ratio * step_size * direction_rand;
            q_new = q_near + X1 + X2;
        end

        q_new = clamp_to_limits(robot, q_new);

        if ~checkCollision(robot, q_near, q_new, obstacles)
            [neighbors, neighbor_idxs] = get_neighbors(tree, q_new, grid_map_xyz, robot, grid_resolution, 0.1);
            min_cost = tree(idx_near).cost + norm(q_new - q_near);
            best_parent = idx_near;

            for j = 1:length(neighbors)
                cost_j = tree(neighbor_idxs(j)).cost + norm(q_new - neighbors{j});
                if cost_j < min_cost && ~checkCollision(robot, neighbors{j}, q_new, obstacles)
                    min_cost = cost_j;
                    best_parent = neighbor_idxs(j);
                end
            end

            tree(end+1) = struct('q', q_new, 'parent', best_parent, 'cost', min_cost);
            new_idx = length(tree);

            xyz_new = get_xyz(robot, q_new);
            key = get_grid_key_xyz(xyz_new, grid_resolution);
            if isKey(grid_map_xyz, key)
                grid_map_xyz(key) = [grid_map_xyz(key), new_idx];
            else
                grid_map_xyz(key) = [new_idx];
            end
            open_list(end+1) = new_idx;

            for j = 1:length(neighbors)
                neighbor = neighbors{j};
                idx = neighbor_idxs(j);
                cost_through_new = min_cost + norm(neighbor - q_new);
                if cost_through_new < tree(idx).cost && ~checkCollision(robot, q_new, neighbor, obstacles)
                    tree(idx).parent = new_idx;
                    tree(idx).cost = cost_through_new;
                end
            end

            if norm(q_new - q_goal) < threshold && ~checkCollision(robot, q_new, q_goal, obstacles)
                tree(end+1) = struct('q', q_goal, 'parent', new_idx, 'cost', min_cost + norm(q_new - q_goal));
                found = true;
                break;
            end
        end
    end

    if found
        path = [];
        idx = length(tree);
        while idx ~= 0
            path = [tree(idx).q; path];
            idx = tree(idx).parent;
        end
    else
        path = [];
        warning('AGHP-RRT*未能找到路径');
    end
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
    idx_near = 1;

    for i = 1:length(neighbor_keys)
        key = neighbor_keys{i};
        if isKey(grid_map_xyz, key)
            idx_list = grid_map_xyz(key);
            for j = 1:length(idx_list)
                q = tree(idx_list(j)).q;
                d = norm(q_rand - q);
                if d < min_dist
                    min_dist = d;
                    idx_near = idx_list(j);
                end
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
    for i = 1:length(neighbor_keys)
        key = neighbor_keys{i};
        if isKey(grid_map_xyz, key)
            idx_list = grid_map_xyz(key);
            for j = 1:length(idx_list)
                q = tree(idx_list(j)).q;
                if norm(q - q_new) < radius
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
    try
        q_full = robot.ikine(T, 'mask', [1 1 1 1 0 0]); % 只考虑位置
        q = q_full(1:4);    % 只保留前4个关节
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

function d = point_to_segment_distance(pt, a, b)
    ab = b - a;
    t = dot(pt - a, ab) / dot(ab, ab);
    t = max(0, min(1, t));
    proj = a + t * ab;
    d = norm(pt - proj);
end

function plot3_path(robot, path, obstacles)
    figure;
    hold on;
    drawObstacles(obstacles);

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
    drawObstacles(obstacles);

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



