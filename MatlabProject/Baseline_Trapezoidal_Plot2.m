function Baseline_Trapezoidal_Plot2(path)
    % =====================================================================
    % [对比基线算法] 传统独立梯形加减速模拟 (带正负方向真实速度)
    % Inputs: 
    %   path: 避障算法算出的 N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
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
    v_z_global = []; a_z_global = [];
    v_y_global = []; a_y_global = [];
    v_x_global = []; a_x_global = [];
    
    current_time = 0;
    
    for k = 1:(num_points - 1)
        % ★ 提取带方向的脉冲差值
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
        
        % 计算各自的梯形绝对速度与加速度曲线
        [v_z_abs, a_z_abs] = get_trapz_state(t_array, dist_Z, Vmax_Z, Amax_Z);
        [v_y_abs, a_y_abs] = get_trapz_state(t_array, dist_Y, Vmax_Y, Amax_Y);
        [v_x_abs, a_x_abs] = get_trapz_state(t_array, dist_X, Vmax_X, Amax_X);
        
        % ★ 将方向符号乘回去，恢复真实的物理正反转速度与加速度
        v_z = v_z_abs * sign_Z; a_z = a_z_abs * sign_Z;
        v_y = v_y_abs * sign_Y; a_y = a_y_abs * sign_Y;
        v_x = v_x_abs * sign_X; a_x = a_x_abs * sign_X;
        
        % 拼接到全局时间轴
        t_global = [t_global, t_array + current_time];
        v_z_global = [v_z_global, v_z]; a_z_global = [a_z_global, a_z];
        v_y_global = [v_y_global, v_y]; a_y_global = [a_y_global, a_y];
        v_x_global = [v_x_global, v_x]; a_x_global = [a_x_global, a_x];
        
        current_time = current_time + T_seg;
    end
    
    % ==========================================================
    % 4. 绘制对比图表 (SCI 格式)
    % ==========================================================
    figure('Name', 'Baseline Unsynchronized Trajectory', 'Position', [150, 150, 1000, 700], 'Color', 'w');
    
    % 子图 (a)：未同步的独立速度曲线
    subplot(2,1,1);
    plot(t_global, v_z_global, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_global, v_y_global, 'g-', 'LineWidth', 1.5);
    plot(t_global, v_x_global, 'b-', 'LineWidth', 1.5);
    grid on;
    title('(a) Independent Trapezoidal Velocity Profile (Unsynchronized Baseline)');
    xlabel('Time (s)');
    ylabel('Velocity (Hz or steps/s)');
    legend('J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % 子图 (b)：未同步的独立加速度曲线
    subplot(2,1,2);
    plot(t_global, a_z_global, 'r-', 'LineWidth', 1.2); hold on;
    plot(t_global, a_y_global, 'g-', 'LineWidth', 1.2);
    plot(t_global, a_x_global, 'b-', 'LineWidth', 1.2);
    grid on;
    title('(b) Independent Acceleration Profile (Unsynchronized Baseline)');
    xlabel('Time (s)');
    ylabel('Acceleration (Hz/s)');
    legend('J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    disp('基线对比图表绘制完成！');
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

% --- 内部数学计算函数：生成梯形曲线数组 ---
function [v, a] = get_trapz_state(t_array, S, Vmax, Amax)
    v = zeros(size(t_array));
    a = zeros(size(t_array));
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
    
    for i = 1:length(t_array)
        t = t_array(i);
        if t <= t_acc
            v(i) = Amax * t;
            a(i) = Amax;
        elseif t <= (t_acc + t_cruise)
            v(i) = Vpeak;
            a(i) = 0;
        elseif t <= (2 * t_acc + t_cruise)
            v(i) = Vpeak - Amax * (t - (t_acc + t_cruise));
            a(i) = -Amax;
        else
            % 提前到达终点，速度和加速度归零，进入“死等”队友状态
            v(i) = 0;
            a(i) = 0;
        end
    end
end