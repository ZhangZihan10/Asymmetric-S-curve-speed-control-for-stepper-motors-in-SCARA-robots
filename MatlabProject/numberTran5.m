function numberTran5(arduino, path)
    % path: 避障算法算出的 N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
    
    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少，无法执行运动规划！');
    end
    
    % ==========================================================
    % 1. 物理参数设置 (严格匹配 Arduino 5000/6000 脉冲物理极限)
    % ==========================================================
    % Z轴绝对极限 (6000 / 205560)
    V_max_Z = 0.04; A_max_Z = 0.09; D_max_Z = 0.08; 
    
    % Y轴绝对极限 (5000 / 945)
    V_max_Y = 5.2;   A_max_Y = 20;   D_max_Y = 16;  
    
    % X轴绝对极限 (5000 / 1455)
    V_max_X = 3.4;   A_max_X = 20;   D_max_X = 16;
    
    V_limits = [V_max_Z, V_max_Y, V_max_X];
    A_limits = [A_max_Z, A_max_Y, A_max_X];
    D_limits = [D_max_Z, D_max_Y, D_max_X];
    
    % ==========================================================
    % 2. 物理空间 -> 脉冲空间映射
    % ==========================================================
    motor_pos = zeros(num_points, 4);
    for i = 1:num_points
        nz = path(i, 1); ny = path(i, 2); nx = path(i, 3); nt = path(i, 4);
        
        % 机械传动比映射
        z_pulse = nz * 205560 - 11667;
        y_pulse = ny * 945 - 1900;
        x_pulse = 0.8 * y_pulse + 3419.6 + nx * 1455; 
        t_angle = nt * 180 / pi;
        
        motor_pos(i, :) = round([z_pulse, y_pulse, x_pulse, t_angle], 2);
    end
    
    % ==========================================================
    % 3. 多轴时间同步优化算法 (极速拉扯)
    % ==========================================================
    V_MAX_LIMITS = [6000, 5000, 5000]; % 保护性钳位极限 (与底层硬件上限一致)
    V_sync = zeros(num_points, 3);
    
    for k = 1:(num_points - 1)
        P_start = path(k, 1:3);
        P_end   = path(k+1, 1:3);
        
        T_independent = zeros(1, 3); 
        
        % 试算独立时间
        for j = 1:3
            S = abs(P_end(j) - P_start(j));
            if S > 1e-5
                [~, ~, ~, ~, ~, ~, T_total] = calc_asym_logic_cp(...
                    0, S, V_limits(j), A_limits(j), D_limits(j), 0, 0);
                T_independent(j) = T_total;
            end
        end
        
        % 寻找最短板时间
        T_sync = max(T_independent);
        if T_sync < 0.001, T_sync = 0.001; end 
        
        % 计算脉冲步数差
        delta_pulses = abs(motor_pos(k+1, 1:3) - motor_pos(k, 1:3));
        
        % 动态降速计算同步脉冲频率
        V_sync(k, :) = delta_pulses / T_sync;
        
        % 频率钳位安全保护
        V_sync(k, 1) = min(V_sync(k, 1), V_MAX_LIMITS(1));
        V_sync(k, 2) = min(V_sync(k, 2), V_MAX_LIMITS(2));
        V_sync(k, 3) = min(V_sync(k, 3), V_MAX_LIMITS(3));
        
        % 保底速度防止驱动器死锁
        V_sync(k, V_sync(k,:) < 10 & delta_pulses > 0) = 10;
    end
    % 最后一个点的速度设为0，确保最终停稳
    V_sync(num_points, :) = [0, 0, 0];
    
    % ==========================================================
    % ★ 新增：在控制台打印生成的同步速度表，用于调试监控 ★
    % ==========================================================
    disp('================================================================');
    disp('                   多轴时间同步规划速度表 (Hz)                  ');
    disp('================================================================');
    disp('  段序号  |   Z轴脉冲频率(Hz)  |   Y轴脉冲频率(Hz)  |   X轴脉冲频率(Hz)');
    disp('----------------------------------------------------------------');
    for i = 1:(num_points-1)
        fprintf(' %2d -> %2d | %16.2f | %16.2f | %16.2f\n', ...
                i, i+1, V_sync(i, 1), V_sync(i, 2), V_sync(i, 3));
    end
    disp('================================================================');
    
    % ==========================================================
    % 4. 串口批处理打包下发
    % ==========================================================
    if arduino.Status == "closed", fopen(arduino); end
    
    writeline(arduino, sprintf('BEGIN,%d', num_points));
    pause(0.5); 
    
    fprintf('正在下发 %d 个同步轨迹点...\n', num_points);
    for i = 1:num_points
        str_data = sprintf('q,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f', ...
                           motor_pos(i, 1), motor_pos(i, 2), motor_pos(i, 3), motor_pos(i, 4), ...
                           V_sync(i, 1), V_sync(i, 2), V_sync(i, 3));
                       
        writeline(arduino, str_data);
        pause(0.015); 
    end
    
    writeline(arduino, 'RUN');
    disp('轨迹下发完毕，机械臂启动执行！');
end

% --- 辅助函数保持不变 ---
function [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(p1, p2, vmax, aa, ad, vs, ve)
    S = abs(p2 - p1);
    if S < 1e-5, S = 1e-5; end
    S_acc = 0.75 * (vmax^2 - vs^2) / aa; if S_acc < 0, S_acc = 0; end
    S_dec = 0.75 * (vmax^2 - ve^2) / ad; if S_dec < 0, S_dec = 0; end
    S_limit = S_acc + S_dec;
    if S >= S_limit
        vp = vmax; Tc = (S - S_limit) / vmax;
    else
        vp_raw = ((4.0/3.0)*S*aa*ad + vs^2*ad + ve^2*aa) / (aa+ad);
        if vp_raw < 0, vp_raw = 0; end
        vp = sqrt(vp_raw); Tc = 0;
    end
    T_ja = (vp - vs) / (aa * 2.0); T_jd = (vp - ve) / (ad * 2.0);
    if T_ja < 0, T_ja = 0; end; if T_jd < 0, T_jd = 0; end
    Ta = 3.0 * T_ja; Td = 3.0 * T_jd; Ttot = Ta + Tc + Td;
end