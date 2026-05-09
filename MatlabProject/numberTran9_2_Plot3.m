function EE_traj = numberTran9_2_Plot3(path)
    % ==========================================================
    % 终极改进版：各轴独立 S 曲线 + 全局时间同步 + 迭代式节点平滑
    % 算法亮点：消除时间放缩导致的速度跳变，保证 C1 连续性
    % Inputs: 
    %   path: 避障算法算出的 N x 4 矩阵 [Z(m), Y(rad), X(rad), T(rad)]
    % Outputs:
    %   EE_traj: 机械臂末端在三维物理空间中的轨迹矩阵 [N_traj x 3] (m)
    % ==========================================================
    
    %% 0. 基本检查
    if nargin < 1
        error('需要输入参数：path');
    end
    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少，至少需要 2 个点。');
    end
    
    %% 1. 物理极限设置
    V_max_Z = 0.0097; A_max_Z = 0.03; D_max_Z = 0.03;
    V_max_Y = 1.05;   A_max_Y = 5.0;  D_max_Y = 4.0;
    V_max_X = 0.68;   A_max_X = 5.0;  D_max_X = 4.0;
    
    SCALE_Z = 205560; OFFSET_Z = -11667;
    SCALE_Y = 945;    OFFSET_Y = -1900;
    SCALE_X = 1455;   OFFSET_X_BIAS = 3419.6;
    
    V_limits = [V_max_Z * SCALE_Z, V_max_Y * SCALE_Y, V_max_X * SCALE_X];
    A_limits = [A_max_Z * SCALE_Z, A_max_Y * SCALE_Y, A_max_X * SCALE_X];
    D_limits = [D_max_Z * SCALE_Z, D_max_Y * SCALE_Y, D_max_X * SCALE_X];
    V_MAX_LIMITS = [2000, 600, 600];
    V_limits = min(V_limits, V_MAX_LIMITS);
    
    %% 2. 路径点映射到电机脉冲空间
    motor_pos = zeros(num_points, 4);
    for i = 1:num_points
        z_pulse = path(i, 1) * SCALE_Z + OFFSET_Z;
        y_pulse = path(i, 2) * SCALE_Y + OFFSET_Y;
        x_pulse = 0.8 * y_pulse + OFFSET_X_BIAS + path(i, 3) * SCALE_X;
        servo_deg = path(i, 4) * 180 / pi;
        motor_pos(i, 1:3) = round([z_pulse, y_pulse, x_pulse]);
        motor_pos(i, 4)   = round(servo_deg, 2);
    end
    
    %% 3. 逐段位移分析
    delta_pulses = diff(motor_pos(:, 1:3));
    abs_delta = abs(delta_pulses);
    num_segments = num_points - 1;
    
    %% 4. 各轴独立节点速度求解 (Initial Guess)
    V_node_init_axis = zeros(num_points, 3);
    V_node_fwd_axis  = zeros(num_points, 3);
    V_node_axis      = zeros(num_points, 3);
    for j = 1:3
        % (1) 初始前瞻
        for k = 2:num_segments
            d_prev = delta_pulses(k-1, j);
            d_curr = delta_pulses(k,   j);
            if abs(d_prev) < 1e-9 || abs(d_curr) < 1e-9
                V_node_init_axis(k, j) = 0; continue;
            end
            if (sign(d_prev) * sign(d_curr) < 0)
                V_node_init_axis(k, j) = 0;
            else
                V_node_init_axis(k, j) = 0.9 * V_limits(j);
            end
        end
        
        % (2) 前向扫描
        V_node_fwd_axis(:, j) = V_node_init_axis(:, j);
        for k = 2:num_points
            S_prev = abs_delta(k - 1, j);
            if S_prev < 1e-9, continue; end
            V_node_fwd_axis(k, j) = min(V_node_fwd_axis(k, j), ...
                sqrt(max(V_node_fwd_axis(k-1, j)^2 + 2 * A_limits(j) * S_prev, 0)));
        end
        
        % (3) 后向扫描
        V_node_axis(:, j) = V_node_fwd_axis(:, j);
        for k = num_points-1:-1:1
            S_k = abs_delta(k, j);
            if S_k < 1e-9, continue; end
            V_node_axis(k, j) = min(V_node_axis(k, j), ...
                sqrt(max(V_node_axis(k+1, j)^2 + 2 * D_limits(j) * S_k, 0)));
        end
        V_node_axis(1, j) = 0; V_node_axis(end, j) = 0;
    end
    
    %% 5. ★ 核心升级：迭代式全局节点速度均衡 (消除跳变)
    MAX_ITER = 4; % 迭代 4 次即可完美收敛
    
    vs_raw_matrix = zeros(num_segments, 3);
    ve_raw_matrix = zeros(num_segments, 3);
    for k = 1:num_segments
        for j = 1:3
            vs_raw_matrix(k,j) = V_node_axis(k, j);
            ve_raw_matrix(k,j) = V_node_axis(k+1, j);
        end
    end
    
    for iter = 1:MAX_ITER
        T_raw_matrix = zeros(num_segments, 3);
        T_sync_arr = zeros(num_segments, 1);
        
        % 5.1 试算所有 Raw S曲线的总耗时
        for k = 1:num_segments
            for j = 1:3
                S_seg = abs_delta(k, j);
                if S_seg < 1e-9
                    T_raw_matrix(k, j) = 0; continue;
                end
                [~, ~, ~, ~, ~, ~, Ttot] = calc_asym_logic_cp_axis(...
                    S_seg, V_limits(j), A_limits(j), D_limits(j), ...
                    vs_raw_matrix(k,j), ve_raw_matrix(k,j));
                T_raw_matrix(k, j) = Ttot;
            end
            T_sync_arr(k) = max(max(T_raw_matrix(k, :)), 0.001);
        end
        
        % 5.2 计算放缩系数与暂态同步速度
        gamma_matrix = zeros(num_segments, 3);
        ve_sync_tmp = zeros(num_segments, 3);
        vs_sync_tmp = zeros(num_segments, 3);
        for k = 1:num_segments
            for j = 1:3
                if T_raw_matrix(k, j) < 1e-9
                    gamma_matrix(k,j) = 1;
                else
                    gamma_matrix(k,j) = T_sync_arr(k) / T_raw_matrix(k, j);
                end
                vs_sync_tmp(k,j) = vs_raw_matrix(k,j) / gamma_matrix(k,j);
                ve_sync_tmp(k,j) = ve_raw_matrix(k,j) / gamma_matrix(k,j);
            end
        end
        
        % 5.3 缝合断层：在节点处强制同步速度相等
        for k = 1:num_segments-1
            for j = 1:3
                v_out = ve_sync_tmp(k, j);
                v_in  = vs_sync_tmp(k+1, j);
                v_target = min(v_out, v_in); % 削峰填谷，取最安全的小值
                
                % 反向补偿 Raw 速度限界
                ve_raw_matrix(k, j)   = v_target * gamma_matrix(k, j);
                vs_raw_matrix(k+1, j) = v_target * gamma_matrix(k+1, j);
            end
        end
    end
    
    %% 6. 生成最终平滑的 segData
    segData = repmat(struct( ...
        'z0', 0, 'y0', 0, 'x0', 0, ...
        'signs', zeros(1,3), 'S', zeros(1,3), ...
        'vs_raw', zeros(1,3), 've_raw', zeros(1,3), 'vp_raw', zeros(1,3), ...
        'aa_raw', zeros(1,3), 'ad_raw', zeros(1,3), ...
        'Tja_raw', zeros(1,3), 'Tjd_raw', zeros(1,3), ...
        'Ta_raw', zeros(1,3), 'Tc_raw', zeros(1,3), 'Td_raw', zeros(1,3), ...
        'Ttot_raw', zeros(1,3), ...
        'sync_scale', ones(1,3), ...
        'vs_sync', zeros(1,3), 've_sync', zeros(1,3), 'vp_sync', zeros(1,3), ...
        'aa_sync', zeros(1,3), 'ad_sync', zeros(1,3), ...
        'Tja_sync', zeros(1,3), 'Tjd_sync', zeros(1,3), ...
        'Ta_sync', zeros(1,3), 'Tc_sync', zeros(1,3), 'Td_sync', zeros(1,3), ...
        'Ttot_sync', 0), num_segments, 1);
        
    for k = 1:num_segments
        Ttot_axis = zeros(1,3);
        segData(k).z0 = motor_pos(k, 1); segData(k).y0 = motor_pos(k, 2); segData(k).x0 = motor_pos(k, 3);
        
        for j = 1:3
            S_seg = abs_delta(k, j);
            sgn_j = sign(delta_pulses(k, j)); if sgn_j == 0, sgn_j = 1; end
            
            % 取用迭代稳定后的极值
            vs = vs_raw_matrix(k, j); 
            ve = ve_raw_matrix(k, j);
            vmax = V_limits(j); aa = A_limits(j); ad = D_limits(j);
            
            segData(k).signs(j) = sgn_j; segData(k).S(j) = S_seg;
            segData(k).vs_raw(j) = vs; segData(k).ve_raw(j) = ve;
            segData(k).aa_raw(j) = aa; segData(k).ad_raw(j) = ad;
            
            if S_seg < 1e-9
                segData(k).vp_raw(j) = 0; segData(k).Ttot_raw(j) = 0; continue;
            end
            
            [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp_axis(S_seg, vmax, aa, ad, vs, ve);
            segData(k).vp_raw(j) = vp; segData(k).Tja_raw(j) = Tja; segData(k).Tjd_raw(j) = Tjd;
            segData(k).Ta_raw(j) = Ta; segData(k).Tc_raw(j) = Tc; segData(k).Td_raw(j) = Td;
            segData(k).Ttot_raw(j) = Ttot;
            Ttot_axis(j) = Ttot;
        end
        
        T_sync = max(Ttot_axis);
        if T_sync < 1e-6, T_sync = 0.001; end
        segData(k).Ttot_sync = T_sync;
        
        for j = 1:3
            Traw = segData(k).Ttot_raw(j);
            if segData(k).S(j) < 1e-9 || Traw < 1e-9
                segData(k).sync_scale(j) = 1; segData(k).vs_sync(j) = 0; segData(k).ve_sync(j) = 0;
                segData(k).vp_sync(j) = 0; segData(k).aa_sync(j) = 0; segData(k).ad_sync(j) = 0;
                segData(k).Tja_sync(j) = 0; segData(k).Tjd_sync(j) = 0; segData(k).Ta_sync(j) = 0;
                segData(k).Tc_sync(j) = 0; segData(k).Td_sync(j) = 0; continue;
            end
            
            gamma = T_sync / Traw;
            segData(k).sync_scale(j) = gamma;
            segData(k).vs_sync(j) = segData(k).vs_raw(j) / gamma;
            segData(k).ve_sync(j) = segData(k).ve_raw(j) / gamma;
            segData(k).vp_sync(j) = segData(k).vp_raw(j) / gamma;
            segData(k).aa_sync(j) = segData(k).aa_raw(j) / gamma^2;
            segData(k).ad_sync(j) = segData(k).ad_raw(j) / gamma^2;
            segData(k).Tja_sync(j) = segData(k).Tja_raw(j) * gamma;
            segData(k).Tjd_sync(j) = segData(k).Tjd_raw(j) * gamma;
            segData(k).Ta_sync(j)  = segData(k).Ta_raw(j)  * gamma;
            segData(k).Tc_sync(j)  = segData(k).Tc_raw(j)  * gamma;
            segData(k).Td_sync(j)  = segData(k).Td_raw(j)  * gamma;
        end
    end
    
    %% 7. 绘图数据生成与积分位移
    t_plot_all = [];
    V_plot_all = cell(1,3); A_plot_all = cell(1,3); P_plot_all = cell(1,3);
    for j = 1:3
        V_plot_all{j} = []; A_plot_all{j} = []; P_plot_all{j} = [];
    end
    t_node = zeros(num_points,1);
    V_node_sync = zeros(num_points,3);
    t_cur = 0;
    
    for k = 1:num_segments
        Tseg = segData(k).Ttot_sync;
        if Tseg < 1e-9, Tseg = 0.001; end
        dt_arr = linspace(0, Tseg, 200); 
        if isempty(t_plot_all), t_plot_all = dt_arr + t_cur; else, t_plot_all = [t_plot_all, dt_arr + t_cur]; end
        t_node(k) = t_cur;
        
        for j = 1:3
            v_seg = zeros(size(dt_arr)); a_seg = zeros(size(dt_arr));
            if segData(k).S(j) >= 1e-9
                for ii = 1:length(dt_arr)
                    [v_tmp, a_tmp] = eval_scurve_at_time( ...
                        dt_arr(ii), segData(k).vs_sync(j), segData(k).ve_sync(j), segData(k).vp_sync(j), ...
                        segData(k).aa_sync(j), segData(k).ad_sync(j), segData(k).Tja_sync(j), ...
                        segData(k).Tjd_sync(j), segData(k).Ta_sync(j), segData(k).Tc_sync(j), ...
                        segData(k).Td_sync(j), segData(k).Ttot_sync);
                    v_seg(ii) = segData(k).signs(j) * v_tmp;
                    a_seg(ii) = segData(k).signs(j) * a_tmp;
                end
            end
            p_seg = cumtrapz(dt_arr, v_seg);
            base_pos = motor_pos(k, j);
            p_seg_actual = base_pos + p_seg;
            
            V_plot_all{j} = [V_plot_all{j}, v_seg];
            A_plot_all{j} = [A_plot_all{j}, a_seg];
            P_plot_all{j} = [P_plot_all{j}, p_seg_actual];
            V_node_sync(k, j) = segData(k).signs(j) * segData(k).vs_sync(j);
        end
        t_cur = t_cur + Tseg;
    end
    t_node(end) = t_cur; V_node_sync(end,:) = 0;
    
    %% 8. 逆向物理映射 (Pulse Domain -> Physical Domain)
    Z_physical = (P_plot_all{1} - OFFSET_Z) / SCALE_Z;
    Y_physical = (P_plot_all{2} - OFFSET_Y) / SCALE_Y;
    X_physical = (P_plot_all{3} - 0.8 * P_plot_all{2} - OFFSET_X_BIAS) / SCALE_X;
    
    %% 9. 构建机械臂模型与正运动学解算 (Forward Kinematics)
    disp('正在加载 Robotics Toolbox 机械臂参数进行正运动学解算 (Optimized)...');
    gripping_point = 0.056;
    L(1) = Link([0 0 0.067 0 1], 'standard'); 
    L(2) = Link([0 -0.017 0.092 0 0], 'standard');
    L(3) = Link([0 -0.01 0.095 0 0], 'standard');
    L(4) = Link([0 -0.04 0 0 0], 'standard');
    L(5) = Link([0 0 0 0 0], 'standard');
    L(6) = Link([0 0 0 0 0], 'standard');
    robot = SerialLink(L, 'name', 'MyRobot');
    
    N_traj = length(Z_physical);
    q_traj = zeros(N_traj, 6);
    q_traj(:, 1) = Z_physical; q_traj(:, 2) = Y_physical; q_traj(:, 3) = X_physical;
    
    EE_traj = zeros(N_traj, 3);
    for idx = 1:N_traj
        T_matrix = robot.fkine(q_traj(idx, :));
        if isa(T_matrix, 'SE3'), t_vec = T_matrix.t; else, t_vec = T_matrix(1:3, 4); end
        EE_traj(idx, 1) = t_vec(1); EE_traj(idx, 2) = t_vec(2); EE_traj(idx, 3) = t_vec(3) - gripping_point;
    end
    
    %% 10. 绘制图表 (SCI 格式)
    figure('Name', 'Independent Node Velocities', 'Position', [50, 80, 800, 780], 'Color', 'w');
    axis_names = {'J1','J2','J3'}; node_idx = 1:num_points;
    for j = 1:3
        subplot(3,1,j);
        plot(node_idx, V_node_init_axis(:,j), 'bo-', 'LineWidth', 1.2, 'MarkerSize', 5); hold on;
        plot(node_idx, V_node_fwd_axis(:,j),  'ms-', 'LineWidth', 1.2, 'MarkerSize', 5);
        plot(node_idx, V_node_axis(:,j),      'r^-', 'LineWidth', 1.8, 'MarkerSize', 6);
        grid on; box on; xlabel('Node Index'); ylabel('Velocity (Hz)');
        title(['Node Velocity of ', axis_names{j}]);
        legend('Initial Look-ahead', 'After Forward Pass', 'Final Node Velocity', 'Location', 'best');
        set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    end
    
    figure('Name', 'Synchronized S-Curve Pulse Profile', 'Position', [900, 80, 800, 760], 'Color', 'w');
    subplot(2,1,1);
    h1 = plot(t_plot_all, V_plot_all{1}, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all, V_plot_all{2}, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all, V_plot_all{3}, 'b-', 'LineWidth', 1.5);
    plot(t_node, V_node_sync(:,1), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    plot(t_node, V_node_sync(:,2), 'gs', 'MarkerSize', 6, 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    plot(t_node, V_node_sync(:,3), 'b^', 'MarkerSize', 6, 'MarkerFaceColor', 'w', 'LineWidth', 1.2);
    grid on; title('(a) Time-Synchronized S-Curve Velocity Profile (Iteration Smoothed)');
    xlabel('Time (s)'); ylabel('Velocity (Hz)'); legend([h1, h2, h3], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    subplot(2,1,2);
    plot(t_plot_all, A_plot_all{1}, 'r-', 'LineWidth', 1.5); hold on;
    plot(t_plot_all, A_plot_all{2}, 'g-', 'LineWidth', 1.5);
    plot(t_plot_all, A_plot_all{3}, 'b-', 'LineWidth', 1.5);
    grid on; title('(b) Time-Synchronized S-Curve Acceleration Profile');
    xlabel('Time (s)'); ylabel('Acceleration (Hz/s)'); legend('J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    figure('Name', 'Physical Kinematics (Inverse Mapped)', 'Position', [150, 100, 900, 850], 'Color', 'w');
    subplot(3,1,1); plot(t_plot_all, Z_physical, 'r-', 'LineWidth', 1.8); grid on;
    title('(a) J1: Z-Axis Physical Displacement'); xlabel('Time (s)'); ylabel('Displacement (m)');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    subplot(3,1,2); plot(t_plot_all, Y_physical, 'g-', 'LineWidth', 1.8); grid on;
    title('(b) J2: Y-Axis Physical Angle'); xlabel('Time (s)'); ylabel('Angle (rad)');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    subplot(3,1,3); plot(t_plot_all, X_physical, 'b-', 'LineWidth', 1.8); grid on;
    title('(c) J3: X-Axis Physical Angle'); xlabel('Time (s)'); ylabel('Angle (rad)');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    
    disp('优化算法解算完成！已消除节点速度撕裂。末端轨迹数据已输出。');
end

%% ==========================================================
% 内部计算函数
%% ==========================================================
function [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp_axis(S, vmax, aa, ad, vs, ve)
    if S < 1e-6, S = 1e-6; end
    S_l = (0.75 * (vmax^2 - vs^2) / aa) + (0.75 * (vmax^2 - ve^2) / ad);
    if S >= S_l
        vp = vmax; Tc = (S - S_l) / max(vp, 1e-9);
    else
        vp = sqrt(max(((4/3) * S * aa * ad + vs^2 * ad + ve^2 * aa) / (aa + ad), 0)); Tc = 0;
    end
    Tja = max((vp - vs) / (2 * aa), 0); Tjd = max((vp - ve) / (2 * ad), 0);
    Ta = 3 * Tja; Td = 3 * Tjd; Ttot = Ta + Tc + Td;
end

function [v, a] = eval_scurve_at_time(t, vs, ve, vp, aa, ad, Tja, Tjd, Ta, Tc, Td, Ttot)
    if t <= 0, v = vs; a = 0; return; end
    if t >= Ttot, v = ve; a = 0; return; end
    if t < Tja
        j = aa / max(Tja, 1e-9); a = j * t; v = vs + 0.5 * j * t^2;
    elseif t < 2 * Tja
        dt = t - Tja; a = aa; v = vs + 0.5 * aa * Tja + aa * dt;
    elseif t < Ta
        d_dt_acc = Ta - t; j = -aa / max(Tja, 1e-9); a = j * d_dt_acc; v = vp - 0.5 * abs(j) * d_dt_acc^2;
    elseif t < (Ta + Tc)
        a = 0; v = vp;
    elseif t < (Ta + Tc + Tjd)
        d_dt_dec = t - (Ta + Tc); j = -ad / max(Tjd, 1e-9); a = j * d_dt_dec; v = vp + 0.5 * j * d_dt_dec^2;
    elseif t < (Ta + Tc + 2 * Tjd)
        d_dt_dec = t - (Ta + Tc + Tjd); a = -ad; v = (vp - 0.5 * ad * Tjd) - ad * d_dt_dec;
    else
        d_dt_end = Ttot - t; if d_dt_end < 0, d_dt_end = 0; end
        j = ad / max(Tjd, 1e-9); a = -j * d_dt_end; v = ve + 0.5 * j * d_dt_end^2;
    end
end