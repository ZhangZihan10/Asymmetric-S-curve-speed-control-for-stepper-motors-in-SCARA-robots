function EE_traj = Baseline_Trapezoidal_Plot3(path)
    % =====================================================================
    % [对比基线算法] 传统独立梯形加减速模拟 (含方向速度及独立物理位移反推图)
    % 纯本地仿真版：物理运动学反向解算 & 输出三维空间正向运动学轨迹
    % Inputs: 
    %   path: 避障算法算出的 N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
    % Outputs:
    %   EE_traj: 机械臂末端在三维物理空间中的轨迹矩阵 [N_traj x 3] (单位: m)
    % =====================================================================
    
    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少！');
    end
    
    % 1. 严格使用旧版 Arduino 代码中的硬件极限参数 (脉冲域)
    Vmax_Z = 6000; Amax_Z = 2000;
    Vmax_Y = 5000; Amax_Y = 1500;
    Vmax_X = 5000; Amax_X = 1500;
    
    % 2. 物理空间 -> 脉冲空间映射
    motor_pos = zeros(num_points, 4);
    for i = 1:num_points
        nz = path(i, 1); ny = path(i, 2); nx = path(i, 3); nt = path(i, 4);
        z_pulse = nz * 205560 - 11667;
        y_pulse = ny * 945 - 1900;
        x_pulse = 0.8 * y_pulse + 3419.6 + nx * 1455; 
        t_angle = nt * 180 / pi;
        motor_pos(i, :) = round([z_pulse, y_pulse, x_pulse, t_angle], 2);
    end
    
    % 3. 模拟旧版 AccelStepper 的独立运行与"死等"逻辑
    t_global = [];
    v_z_global = []; a_z_global = []; p_z_global = [];
    v_y_global = []; a_y_global = []; p_y_global = [];
    v_x_global = []; a_x_global = []; p_x_global = [];
    
    current_time = 0;
    
    for k = 1:(num_points - 1)
        % 提取带方向的脉冲差值
        diff_Z = motor_pos(k+1, 1) - motor_pos(k, 1);
        diff_Y = motor_pos(k+1, 2) - motor_pos(k, 2);
        diff_X = motor_pos(k+1, 3) - motor_pos(k, 3);
        
        % 分离绝对距离与方向符号
        dist_Z = abs(diff_Z); sign_Z = sign(diff_Z); if sign_Z == 0, sign_Z = 1; end
        dist_Y = abs(diff_Y); sign_Y = sign(diff_Y); if sign_Y == 0, sign_Y = 1; end
        dist_X = abs(diff_X); sign_X = sign(diff_X); if sign_X == 0, sign_X = 1; end
        
        % 分别计算每个轴独立跑完所需的时间
        T_Z = calc_independent_time(dist_Z, Vmax_Z, Amax_Z);
        T_Y = calc_independent_time(dist_Y, Vmax_Y, Amax_Y);
        T_X = calc_independent_time(dist_X, Vmax_X, Amax_X);
        
        % 旧版逻辑：当前段总耗时由“最慢的那个轴”决定
        T_seg = max([T_Z, T_Y, T_X, 0.001]);
        
        % 生成当前段的稠密时间序列 (用于高精度绘图)
        t_array = linspace(0, T_seg, 150);
        
        % 计算各自的梯形绝对速度、加速度与【绝对位移增量】
        [v_z_abs, a_z_abs, p_z_inc] = get_trapz_state(t_array, dist_Z, Vmax_Z, Amax_Z);
        [v_y_abs, a_y_abs, p_y_inc] = get_trapz_state(t_array, dist_Y, Vmax_Y, Amax_Y);
        [v_x_abs, a_x_abs, p_x_inc] = get_trapz_state(t_array, dist_X, Vmax_X, Amax_X);
        
        % 将方向符号乘回去，恢复真实的物理正反转速度与加速度
        v_z = v_z_abs * sign_Z; a_z = a_z_abs * sign_Z;
        v_y = v_y_abs * sign_Y; a_y = a_y_abs * sign_Y;
        v_x = v_x_abs * sign_X; a_x = a_x_abs * sign_X;
        
        % 叠加起始位置，计算当前时间的真实绝对脉冲位置
        p_z = motor_pos(k, 1) + p_z_inc * sign_Z;
        p_y = motor_pos(k, 2) + p_y_inc * sign_Y;
        p_x = motor_pos(k, 3) + p_x_inc * sign_X;
        
        % 拼接到全局时间轴
        t_global = [t_global, t_array + current_time];
        
        v_z_global = [v_z_global, v_z]; a_z_global = [a_z_global, a_z]; p_z_global = [p_z_global, p_z];
        v_y_global = [v_y_global, v_y]; a_y_global = [a_y_global, a_y]; p_y_global = [p_y_global, p_y];
        v_x_global = [v_x_global, v_x]; a_x_global = [a_x_global, a_x]; p_x_global = [p_x_global, p_x];
        
        current_time = current_time + T_seg;
    end
    
    % ==========================================================
    % 4. 逆向物理映射 (Pulse Domain -> Physical Domain)
    % ==========================================================
    % 根据原正向公式反解出物理空间的 Z(m), Y(rad), X(rad)
    Z_physical = (p_z_global + 11667) / 205560;
    Y_physical = (p_y_global + 1900) / 945;
    X_physical = (p_x_global - 0.8 * p_y_global - 3419.6) / 1455;
    
    % ==========================================================
    % 5. ★ 构建机械臂模型与正运动学解算 (Forward Kinematics)
    % ==========================================================
    disp('正在加载 Robotics Toolbox 机械臂参数进行正运动学解算 (Baseline)...');
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
    
    % 构建 Nx6 关节空间矩阵
    N_traj = length(Z_physical);
    q_traj = zeros(N_traj, 6);
    q_traj(:, 1) = Z_physical;
    q_traj(:, 2) = Y_physical;
    q_traj(:, 3) = X_physical;
    
    % 解算末端空间轨迹
    EE_traj = zeros(N_traj, 3);
    for idx = 1:N_traj
        T_matrix = robot.fkine(q_traj(idx, :));
        
        % 兼容 Robotics Toolbox v9(返回矩阵) 与 v10(返回 SE3 对象)
        if isa(T_matrix, 'SE3')
            t_vec = T_matrix.t;
        else
            t_vec = T_matrix(1:3, 4);
        end
        
        EE_traj(idx, 1) = t_vec(1);
        EE_traj(idx, 2) = t_vec(2);
        EE_traj(idx, 3) = t_vec(3) - gripping_point; % 考虑夹爪末端点偏移
    end
    
    % ==========================================================
    % 6. 绘制对比图表 (SCI 格式)
    % ==========================================================
    
    % ----------------------------------------------------------
    % Figure 1: 控制器输出指令级 (脉冲域的速度与加速度)
    % ----------------------------------------------------------
    figure('Name', 'Controller Output (Pulse Domain)', 'Position', [100, 150, 1000, 600], 'Color', 'w');
    
    % 子图 1(a)：未同步的独立速度曲线 (脉冲域)
    subplot(2,1,1);
    plot(t_global, v_z_global, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_global, v_y_global, 'g-', 'LineWidth', 1.5);
    plot(t_global, v_x_global, 'b-', 'LineWidth', 1.5);
    grid on;
    title('(a) Independent Trapezoidal Velocity Profile (Unsynchronized Baseline)');
    xlabel('Time (s)');
    ylabel('Velocity (Pulse/s)');
    legend('J1 Velocity', 'J2 Velocity', 'J3 Velocity', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % 子图 1(b)：未同步的独立加速度曲线 (脉冲域)
    subplot(2,1,2);
    plot(t_global, a_z_global, 'r-', 'LineWidth', 1.2); hold on;
    plot(t_global, a_y_global, 'g-', 'LineWidth', 1.2);
    plot(t_global, a_x_global, 'b-', 'LineWidth', 1.2);
    grid on;
    title('(b) Independent Acceleration Profile (Unsynchronized Baseline)');
    xlabel('Time (s)');
    ylabel('Acceleration (Pulse/s^2)');
    legend('J1 Accel', 'J2 Accel', 'J3 Accel', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % ----------------------------------------------------------
    % Figure 2: 真实的物理运动学响应 (反解位移与角度)
    % ----------------------------------------------------------
    figure('Name', 'Physical Kinematics (Inverse Mapped)', 'Position', [600, 100, 1000, 800], 'Color', 'w');
    
    % 子图 2(a)：Z轴 物理位移 (米)
    subplot(3,1,1);
    plot(t_global, Z_physical, 'r-', 'LineWidth', 1.5);
    grid on;
    title('(a) J1: Z-Axis Physical Displacement');
    xlabel('Time (s)');
    ylabel('Displacement (m)');
    legend('Z Axis Trajectory', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % 子图 2(b)：Y轴 物理角度 (弧度)
    subplot(3,1,2);
    plot(t_global, Y_physical, 'g-', 'LineWidth', 1.5);
    grid on;
    title('(b) J2: Y-Axis Physical Angle');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('Y Axis Trajectory', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % 子图 2(c)：X轴 物理角度 (弧度)
    subplot(3,1,3);
    plot(t_global, X_physical, 'b-', 'LineWidth', 1.5);
    grid on;
    title('(c) J3: X-Axis Physical Angle');
    xlabel('Time (s)');
    ylabel('Angle (rad)');
    legend('X Axis Trajectory', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    disp('基线算法解算完成！末端轨迹数据已输出。');
end

% --- 内部数学计算函数：计算独立运行时间 ---
function T_total = calc_independent_time(S, Vmax, Amax)
    if S < 1e-5
        T_total = 0; return;
    end
    if S >= (Vmax^2 / Amax) % 达到最大速度
        t_acc = Vmax / Amax;
        T_total = 2 * t_acc + (S - Vmax^2 / Amax) / Vmax;
    else % 三角形速度曲线
        t_acc = sqrt(S / Amax);
        T_total = 2 * t_acc;
    end
end

% --- 内部数学计算函数：生成梯形曲线数组 (含位移积分) ---
function [v, a, p] = get_trapz_state(t_array, S, Vmax, Amax)
    v = zeros(size(t_array));
    a = zeros(size(t_array));
    p = zeros(size(t_array)); % 记录位移累积量
    
    if S < 1e-5
        return;
    end
    
    if S >= (Vmax^2 / Amax)
        t_acc = Vmax / Amax;
        t_cruise = (S - Vmax^2 / Amax) / Vmax;
        Vpeak = Vmax;
    else
        Vpeak = sqrt(S * Amax);
        t_acc = Vpeak / Amax;
        t_cruise = 0;
    end
    
    % 记录各个阶段结束时的位移基准，用于后续阶段的累加
    p_acc_end = 0.5 * Amax * t_acc^2;
    p_cruise_end = p_acc_end + Vpeak * t_cruise;
    
    for i = 1:length(t_array)
        t = t_array(i);
        if t <= t_acc
            v(i) = Amax * t;
            a(i) = Amax;
            p(i) = 0.5 * Amax * t^2; % 匀加速位移公式
        elseif t <= (t_acc + t_cruise)
            v(i) = Vpeak;
            a(i) = 0;
            p(i) = p_acc_end + Vpeak * (t - t_acc); % 匀速位移公式
        elseif t <= (2 * t_acc + t_cruise)
            dt_dec = t - (t_acc + t_cruise);
            v(i) = Vpeak - Amax * dt_dec;
            a(i) = -Amax;
            p(i) = p_cruise_end + Vpeak * dt_dec - 0.5 * Amax * dt_dec^2; % 匀减速位移公式
        else
            % 提前到达终点，进入“死等”队友状态
            v(i) = 0;
            a(i) = 0;
            p(i) = S;
        end
    end
end