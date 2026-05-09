function numberTran7(arduino, path)
    
%% 终极 7段 S曲线(变加速) 动力学控制与完美绘图版 (适配 Arduino 动态速度融合引擎)
% path: 避障算法算出的 N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
    
    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少，无法执行运动规划！');
    end
    
    % ==========================================================
    % 1. 物理参数设置 (严格限制 Z轴 < 2000 Hz, Y/X < 1000 Hz)
    % ==========================================================
    % Z 轴限制在 2000 Hz 左右 (极其稳重，防丢步)
    V_max_Z = 0.0097; A_max_Z = 0.03; D_max_Z = 0.03; 
    
    % Y 轴限制在 1000 Hz 左右
    V_max_Y = 1.05;   A_max_Y = 5.0;  D_max_Y = 4.0;  
    
    % X 轴限制在 1000 Hz 左右
    V_max_X = 0.68;   A_max_X = 5.0;  D_max_X = 4.0;
    
    V_limits = [V_max_Z * 205560, V_max_Y * 945, V_max_X * 1455];
    A_limits = [A_max_Z * 205560, A_max_Y * 945, A_max_X * 1455];
    D_limits = [D_max_Z * 205560, D_max_Y * 945, D_max_X * 1455];
    
    % 强制硬件钳位保护 (双重保险)
    V_MAX_LIMITS = [2000, 1000, 1000]; 
    V_limits = min(V_limits, V_MAX_LIMITS);
    
    % ==========================================================
    % 2. 物理空间 -> 脉冲空间映射
    % ==========================================================
    motor_pos = zeros(num_points, 4);
    for i = 1:num_points
        nz = path(i, 1); ny = path(i, 2); nx = path(i, 3); nt = path(i, 4);
        z_pulse = nz * 205560 - 11667;
        y_pulse = ny * 945 - 1900;
        x_pulse = 0.8 * y_pulse + 3419.6 + nx * 1455; 
        t_angle = nt * 180 / pi;
        motor_pos(i, :) = round([z_pulse, y_pulse, x_pulse, t_angle], 2);
    end
    
    delta_pulses = diff(motor_pos(:, 1:3)); 
    abs_delta = abs(delta_pulses);
    num_segments = num_points - 1;
    
    % ==========================================================
    % 3. 核心修复：动力学包络扫描与安全前瞻 (Kinematic Sweep)
    % ==========================================================
    [dS_dom, dom_idx_arr] = max(abs_delta, [], 2); 
    
    V_max_dom = zeros(num_segments, 1);
    for k = 1:num_segments
        if dS_dom(k) < 1e-5
            V_max_dom(k) = 10; continue;
        end
        r_z = abs_delta(k, 1) / dS_dom(k);
        r_y = abs_delta(k, 2) / dS_dom(k);
        r_x = abs_delta(k, 3) / dS_dom(k);
        
        limit_z = V_limits(1) / max(r_z, 1e-5);
        limit_y = V_limits(2) / max(r_y, 1e-5);
        limit_x = V_limits(3) / max(r_x, 1e-5);
        
        V_max_dom(k) = min([limit_z, limit_y, limit_x]);
        V_max_dom(k) = min(V_max_dom(k), V_limits(dom_idx_arr(k))); 
    end
    
    V_node_dom = zeros(num_points, 1); 
    for k = 2:num_segments
        dir_in = sign(delta_pulses(k-1, :));
        dir_out = sign(delta_pulses(k, :));
        
        % 只有方向一致且主导轴没变，才允许带速飞越 (留 10% 安全余量)
        if isequal(dir_in, dir_out) && (dom_idx_arr(k-1) == dom_idx_arr(k))
            V_node_dom(k) = min(V_max_dom(k-1), V_max_dom(k)) * 0.9; 
        else
            V_node_dom(k) = 0; 
        end
    end
    V_node_dom(1) = 0; V_node_dom(end) = 0;
    
    % 前向扫描 (敬畏加速距离)
    for k = 2:num_points
        a_max = A_limits(dom_idx_arr(k-1));
        max_v_fwd = sqrt(V_node_dom(k-1)^2 + 2 * a_max * dS_dom(k-1));
        V_node_dom(k) = min(V_node_dom(k), max_v_fwd);
    end
    
    % 后向扫描 (敬畏刹车距离)
    for k = num_points-1:-1:1
        d_max = D_limits(dom_idx_arr(k));
        max_v_bwd = sqrt(V_node_dom(k+1)^2 + 2 * d_max * dS_dom(k));
        V_node_dom(k) = min(V_node_dom(k), max_v_bwd);
    end
    
    % ==========================================================
    % 4. S 曲线解析生成与【目标节点速度】提取
    % ==========================================================
    % ★ 核心：用于存储下发给 Arduino 的目标节点过弯速度
    V_out = zeros(num_points, 3);
    
    t_plot_all = []; Vz_plot_all = []; Vy_plot_all = []; Vx_plot_all = [];
    Az_plot_all = []; Ay_plot_all = []; Ax_plot_all = [];
    
    t_node = zeros(num_points, 1);
    Vz_node = zeros(num_points, 1); Vy_node = zeros(num_points, 1); Vx_node = zeros(num_points, 1);
    Az_node = zeros(num_points, 1); Ay_node = zeros(num_points, 1); Ax_node = zeros(num_points, 1);
    
    current_time = 0;
    
    for k = 1:num_segments
        dom_idx = dom_idx_arr(k);
        S_seg = dS_dom(k);
        
        if S_seg < 1e-5
            V_out(k, :) = [10, 10, 10]; 
            t_node(k) = current_time;
            continue;
        end
        
        vs = V_node_dom(k);
        ve = V_node_dom(k+1);
        vmax = V_max_dom(k); 
        aa = A_limits(dom_idx);
        ad = D_limits(dom_idx);
        
        t_node(k) = current_time;
        ratio_z_node = abs_delta(k, 1) / S_seg;
        ratio_y_node = abs_delta(k, 2) / S_seg;
        ratio_x_node = abs_delta(k, 3) / S_seg;
        
        Vz_node(k) = vs * ratio_z_node; Vy_node(k) = vs * ratio_y_node; Vx_node(k) = vs * ratio_x_node;
        Az_node(k) = 0; Ay_node(k) = 0; Ax_node(k) = 0; 
        
        % -----------------------------------------------------
        % ★ 提取下发参数：按比例分配到达本段终点(k+1)时的目标速度 ve
        % -----------------------------------------------------
        ve_z = ve * ratio_z_node;
        ve_y = ve * ratio_y_node;
        ve_x = ve * ratio_x_node;
        
        % 保存给下位机，下位机将以此为目标进行距离融合插值
        V_out(k, :) = [max(ve_z, 0), max(ve_y, 0), max(ve_x, 0)];
        
        % --- 计算 S 曲线详细时间参数用于绘图 ---
        [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(0, S_seg, vmax, aa, ad, vs, ve);
        if Ttot < 0.001, Ttot = 0.001; end
        
        dt_array = linspace(0, Ttot, 100); 
        v_dom_array = zeros(1, 100); a_dom_array = zeros(1, 100);
        
        for i = 1:length(dt_array)
            dt = dt_array(i);
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
        
        t_plot_all = [t_plot_all, dt_array + current_time];
        Vz_plot_all = [Vz_plot_all, v_dom_array * ratio_z_node]; Vy_plot_all = [Vy_plot_all, v_dom_array * ratio_y_node]; Vx_plot_all = [Vx_plot_all, v_dom_array * ratio_x_node];
        Az_plot_all = [Az_plot_all, a_dom_array * ratio_z_node]; Ay_plot_all = [Ay_plot_all, a_dom_array * ratio_y_node]; Ax_plot_all = [Ax_plot_all, a_dom_array * ratio_x_node];
        
        current_time = current_time + Ttot;
    end
    
    t_node(num_points) = current_time;
    Vz_node(num_points) = 0; Vy_node(num_points) = 0; Vx_node(num_points) = 0;
    Az_node(num_points) = 0; Ay_node(num_points) = 0; Ax_node(num_points) = 0;
    V_out(num_points, :) = [0, 0, 0]; % 终点速度强制为 0
    
    % ==========================================================
    % 5. 绘制 SCI 学术级别 S 曲线动力学图表
    % ==========================================================
    figure('Name', 'True Kinematics Sweep & Dynamic Blending', 'Position', [100, 100, 1000, 700], 'Color', 'w');
    
    subplot(2,1,1);
    h1 = plot(t_plot_all, Vz_plot_all, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all, Vy_plot_all, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all, Vx_plot_all, 'b-', 'LineWidth', 1.5);
    
    plot(t_node, Vz_node, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Vy_node, 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Vx_node, 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    
    grid on; title('(a) S-Curve Velocity Profile (Dynamic Blending Target Speeds)');
    xlabel('Time (s)'); ylabel('Velocity (Hz)'); legend([h1, h2, h3], 'Z-Axis', 'Y-Axis', 'X-Axis', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    subplot(2,1,2);
    plot(t_plot_all, Az_plot_all, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_plot_all, Ay_plot_all, 'g-', 'LineWidth', 1.5);
    plot(t_plot_all, Ax_plot_all, 'b-', 'LineWidth', 1.5);
    
    plot(t_node, Az_node, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Ay_node, 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Ax_node, 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    
    grid on; title('(b) Safe Acceleration Profile');
    xlabel('Time (s)'); ylabel('Acceleration (Hz/s)'); legend([h1, h2, h3], 'Z-Axis', 'Y-Axis', 'X-Axis', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    % ==========================================================
    % 6. 串口批处理打包下发 (匹配 Arduino 的 8 参数动态融合引擎)
    % ==========================================================
    if arduino.Status == "closed", fopen(arduino); end
    writeline(arduino, sprintf('BEGIN,%d', num_points)); pause(0.5); 
    
    fprintf('正在下发 %d 个同步轨迹点 (包含位移与目标飞越速度)...\n', num_points);
    for i = 1:num_points
        % 恢复精简且高效的 8 参数协议：q, Pz, Py, Px, Claw, Vz, Vy, Vx
        str_data = sprintf('q,%.2f,%.2f,%.2f,%.3f,%.2f,%.2f,%.2f', ...
                           motor_pos(i, 1), motor_pos(i, 2), motor_pos(i, 3), motor_pos(i, 4), ...
                           V_out(i, 1), V_out(i, 2), V_out(i, 3));
        writeline(arduino, str_data); pause(0.015); 
    end
    writeline(arduino, 'RUN'); disp('动态融合版轨迹下发完毕！');
end

% === S 曲线辅助函数保持原样 ===
function [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(p1, p2, vmax, aa, ad, vs, ve)
    S = abs(p2 - p1); if S < 1e-4, S = 1e-4; end
    S_acc = 0.75 * (vmax^2 - vs^2) / aa; if S_acc < 0, S_acc = 0; end
    S_dec = 0.75 * (vmax^2 - ve^2) / ad; if S_dec < 0, S_dec = 0; end
    S_limit = S_acc + S_dec;
    if S >= S_limit
        vp = vmax; Tc = (S - S_limit) / vmax;
    else
        vp_raw = ((4.0/3.0)*S*aa*ad + vs^2*ad + ve^2*aa) / (aa+ad);
        if vp_raw < 0, vp_raw = 0; end; vp = sqrt(vp_raw); Tc = 0;
    end
    T_ja = (vp - vs) / (aa * 2.0); T_jd = (vp - ve) / (ad * 2.0);
    if T_ja < 0, T_ja = 0; end; if T_jd < 0, T_jd = 0; end
    Ta = 3.0 * T_ja; Td = 3.0 * T_jd; Ttot = Ta + Tc + Td;
end