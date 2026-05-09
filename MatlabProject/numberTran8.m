function numberTran8(arduino, path)
    % ==========================================================
    % 终极时间同步 S 曲线控制 (适配 Arduino 时间驱动引擎)
    % 特性：9参数下发、前向后向包络扫描、精确时间控制、完美图表绘制
    % ==========================================================
    
    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少！');
    end
    
    % 1. 物理极限设置 (严格锁定 Z<2000Hz, Y/X<1000Hz)
    V_max_Z = 0.0097; A_max_Z = 0.03; D_max_Z = 0.03; 
    V_max_Y = 1.05;   A_max_Y = 5.0;  D_max_Y = 4.0;  
    V_max_X = 0.68;   A_max_X = 5.0;  D_max_X = 4.0;
    
    V_limits = [V_max_Z * 205560, V_max_Y * 945, V_max_X * 1455];
    A_limits = [A_max_Z * 205560, A_max_Y * 945, A_max_X * 1455];
    D_limits = [D_max_Z * 205560, D_max_Y * 945, D_max_X * 1455];
    
    V_MAX_LIMITS = [2000, 800, 800]; 
    V_limits = min(V_limits, V_MAX_LIMITS);
    
    % 2. 映射到脉冲空间
    motor_pos = zeros(num_points, 4);
    for i = 1:num_points
        z_pulse = path(i, 1) * 205560 - 11667;
        y_pulse = path(i, 2) * 945 - 1900;
        x_pulse = 0.8 * y_pulse + 3419.6 + path(i, 3) * 1455; 
        motor_pos(i, :) = round([z_pulse, y_pulse, x_pulse, path(i, 4)*180/pi], 2);
    end
    
    delta_pulses = diff(motor_pos(:, 1:3)); 
    abs_delta = abs(delta_pulses);
    num_segments = num_points - 1;
    [dS_dom, dom_idx_arr] = max(abs_delta, [], 2); 
    
    % 3. 前瞻与包络扫描 (确定节点安全速度)
    V_max_seg = zeros(num_segments, 1);
    for k = 1:num_segments
        r = abs_delta(k,:) / max(dS_dom(k), 1e-5);
        V_max_seg(k) = min([V_limits(1)/max(r(1),1e-5), V_limits(2)/max(r(2),1e-5), V_limits(3)/max(r(3),1e-5), V_limits(dom_idx_arr(k))]);
    end
    
    V_node = zeros(num_points, 1); 
    for k = 2:num_segments
        if isequal(sign(delta_pulses(k-1,:)), sign(delta_pulses(k,:))) && (dom_idx_arr(k-1)==dom_idx_arr(k))
            V_node(k) = min(V_max_seg(k-1), V_max_seg(k)) * 0.9;
        else
            V_node(k) = 0;
        end
    end
    
    % 双向扫描
    for k = 2:num_points
        V_node(k) = min(V_node(k), sqrt(V_node(k-1)^2 + 2*A_limits(dom_idx_arr(k-1))*dS_dom(k-1)));
    end
    for k = num_points-1:-1:1
        V_node(k) = min(V_node(k), sqrt(V_node(k+1)^2 + 2*D_limits(dom_idx_arr(k))*dS_dom(k)));
    end
    
    % ==========================================================
    % 4. 提取下发参数与 S 曲线动力学阵列生成
    % ==========================================================
    V_out = zeros(num_points, 3);
    T_out = zeros(num_points, 1);
    
    t_plot_all = []; Vz_plot_all = []; Vy_plot_all = []; Vx_plot_all = [];
    Az_plot_all = []; Ay_plot_all = []; Ax_plot_all = [];
    
    t_node = zeros(num_points, 1);
    Vz_node = zeros(num_points, 1); Vy_node = zeros(num_points, 1); Vx_node = zeros(num_points, 1);
    Az_node = zeros(num_points, 1); Ay_node = zeros(num_points, 1); Ax_node = zeros(num_points, 1);
    
    current_time = 0;
    
    for k = 1:num_segments
        S_seg = dS_dom(k);
        dom_idx = dom_idx_arr(k);
        
        if S_seg < 1e-5
            V_out(k, :) = [0, 0, 0];
            T_out(k) = 0.001;
            t_node(k) = current_time;
            continue;
        end
        
        vs = V_node(k); 
        ve = V_node(k+1);
        vmax = V_max_seg(k);
        aa = A_limits(dom_idx);
        ad = D_limits(dom_idx);
        
        [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(0, S_seg, vmax, aa, ad, vs, ve);
        
        T_out(k) = max(Ttot, 0.001);
        r = abs_delta(k,:) / S_seg;
        V_out(k,:) = ve * r;
        
        % 记录节点瞬时状态 (用于绘图)
        t_node(k) = current_time;
        Vz_node(k) = vs * r(1); Vy_node(k) = vs * r(2); Vx_node(k) = vs * r(3);
        Az_node(k) = 0; Ay_node(k) = 0; Ax_node(k) = 0; 
        
        % 绘图采样阵列 (100个采样点保障曲线平滑度)
        dt_arr = linspace(0, T_out(k), 100);
        v_dom_array = zeros(1, 100); 
        a_dom_array = zeros(1, 100);
        
        for i=1:length(dt_arr)
            dt = dt_arr(i);
            if dt < T_ja
                j = aa / max(T_ja, 1e-6); a_dom_array(i) = j * dt; v_dom_array(i) = vs + 0.5 * j * dt^2; 
            elseif dt < 2.0 * T_ja
                a_dom_array(i) = aa; v_dom_array(i) = vs + 0.5 * aa * T_ja + aa * (dt - T_ja);
            elseif dt < Ta
                d_dt_acc = Ta - dt; j = -aa / max(T_ja, 1e-6); a_dom_array(i) = j * d_dt_acc; v_dom_array(i) = vp - 0.5 * abs(j) * d_dt_acc^2;
            elseif dt < (Ta + Tc)
                v_dom_array(i) = vp; a_dom_array(i) = 0;
            elseif dt < (Ta + Tc + T_jd)
                d_dt_dec = dt - (Ta + Tc); j = -ad / max(T_jd, 1e-6); a_dom_array(i) = j * d_dt_dec; v_dom_array(i) = vp + 0.5 * j * d_dt_dec^2;
            elseif dt < (Ta + Tc + 2.0 * T_jd)
                a_dom_array(i) = -ad; d_dt_dec = dt - (Ta + Tc + T_jd); v_dom_array(i) = (vp - 0.5 * ad * T_jd) - ad * d_dt_dec;
            elseif dt <= Ttot
                d_dt_end = Ttot - dt; if d_dt_end < 0, d_dt_end = 0; end 
                j = ad / max(T_jd, 1e-6); a_dom_array(i) = -j * d_dt_end; v_dom_array(i) = ve + 0.5 * j * d_dt_end^2; 
            else
                v_dom_array(i) = ve; a_dom_array(i) = 0;
            end
        end
        
        t_plot_all = [t_plot_all, dt_arr + current_time];
        Vz_plot_all = [Vz_plot_all, v_dom_array * r(1)]; Vy_plot_all = [Vy_plot_all, v_dom_array * r(2)]; Vx_plot_all = [Vx_plot_all, v_dom_array * r(3)];
        Az_plot_all = [Az_plot_all, a_dom_array * r(1)]; Ay_plot_all = [Ay_plot_all, a_dom_array * r(2)]; Ax_plot_all = [Ax_plot_all, a_dom_array * r(3)];
        
        current_time = current_time + T_out(k);
    end
    
    % 终点节点数据
    t_node(num_points) = current_time;
    Vz_node(num_points) = 0; Vy_node(num_points) = 0; Vx_node(num_points) = 0;
    Az_node(num_points) = 0; Ay_node(num_points) = 0; Ax_node(num_points) = 0;
    
    % ==========================================================
    % 5. 绘制 SCI 学术级别 S 曲线动力学图表
    % ==========================================================
    figure('Name', 'True Kinematics Sweep & Dynamic Blending', 'Position', [100, 100, 1000, 700], 'Color', 'w');
    
    subplot(2,1,1);
    h1 = plot(t_plot_all, Vz_plot_all, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all, Vy_plot_all, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all, Vx_plot_all, 'b-', 'LineWidth', 1.5);
    
    % 绘制离散计算节点
    plot(t_node, Vz_node, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Vy_node, 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Vx_node, 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    
    grid on; title('(a) S-Curve Velocity Profile (Time-Synchronized Blending)');
    xlabel('Time (s)'); ylabel('Velocity (Hz)'); legend([h1, h2, h3], 'Z-Axis', 'Y-Axis', 'X-Axis', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    subplot(2,1,2);
    plot(t_plot_all, Az_plot_all, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_plot_all, Ay_plot_all, 'g-', 'LineWidth', 1.5);
    plot(t_plot_all, Ax_plot_all, 'b-', 'LineWidth', 1.5);
    
    % 绘制离散加速度节点 (完美落在零基准线上)
    plot(t_node, Az_node, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Ay_node, 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Ax_node, 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    
    grid on; title('(b) Safe Acceleration Profile');
    xlabel('Time (s)'); ylabel('Acceleration (Hz/s)'); legend([h1, h2, h3], 'Z-Axis', 'Y-Axis', 'X-Axis', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % ==========================================================
    % 6. 自动化两阶段双模下发 (1. 归位就绪 -> 2. 全速避障)
    % ==========================================================
    if arduino.Status == "closed", fopen(arduino); end
    flush(arduino); 
    pause(0.5); 
    
    % --- 阶段一：下发唯一起点，触发安全归位模式 ---
    fprintf('▶ 阶段一：正在调度机械臂平滑归位至规划起点...\n');
    writeline(arduino, 'BEGIN,1');
    pause(0.1); 
    
    % 发送第 1 个点的参数 (速度和时间参数此时用0填充，单点模式下硬件会自动使用原生加减速)
    str_start = sprintf('q,%.2f,%.2f,%.2f,%.3f,0,0,0,0', ...
                        motor_pos(1, 1), motor_pos(1, 2), motor_pos(1, 3), motor_pos(1, 4));
    writeline(arduino, str_start);
    pause(0.1); 
    writeline(arduino, 'RUN');
    
    % 阻塞等待 Arduino 返回到达信号
    is_ready = false;
    while ~is_ready
        if arduino.NumBytesAvailable > 0
            response = strtrim(readline(arduino));
            if strcmp(response, "FINISHED_ALL")
                is_ready = true;
            end
        end
        pause(0.01);
    end
    fprintf('  ✓ 已安全到达起点位置，机械臂就绪。\n');
    pause(0.5); % 给真机留半秒钟的视觉停顿
    
    
    % --- 阶段二：下发完整轨迹，触发时间同步高阶插补 ---
    fprintf('▶ 阶段二：正在下发 %d 个避障轨迹节点 (启动高阶动态插补)...\n', num_points);
    writeline(arduino, sprintf('BEGIN,%d', num_points));
    pause(0.2); 
    
    for i = 1:num_points
        % 发送完整的 9 参数协议
        str_data = sprintf('q,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f,%.4f', ...
                           motor_pos(i, 1), motor_pos(i, 2), motor_pos(i, 3), motor_pos(i, 4), ...
                           V_out(i, 1), V_out(i, 2), V_out(i, 3), T_out(i));
        writeline(arduino, str_data);
        pause(0.01); 
    end
    
    writeline(arduino, 'RUN');
    disp('  ✓ 避障轨迹下发完毕！');
end

function [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(p1, p2, vmax, aa, ad, vs, ve)
    S = abs(p2-p1); if S<1e-4, S=1e-4; end
    S_l = (0.75*(vmax^2-vs^2)/aa) + (0.75*(vmax^2-ve^2)/ad);
    if S>=S_l, vp=vmax; Tc=(S-S_l)/vmax; else
    vp=sqrt(((4/3)*S*aa*ad + vs^2*ad + ve^2*aa)/(aa+ad)); Tc=0; end
    T_ja=(vp-vs)/(2*aa); T_jd=(vp-ve)/(2*ad);
    Ta=3*T_ja; Td=3*T_jd; Ttot=Ta+Tc+Td;
end