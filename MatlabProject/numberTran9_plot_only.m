%% 失败
function result = numberTran9_plot_only(path)
    % ==========================================================
    
    % 各轴独立节点速度 + 段时间同步
    % 仅计算与绘图，不进行串口传输
    %
    % 主要特点：
    % 1) 每个轴独立计算节点速度
    % 2) 只有折返轴在该节点速度置 0
    % 3) 其它轴保持自身非零节点速度
    % 4) 每段三轴分别规划，再按最大段时间同步
    % 5) 对比：非对称 S 曲线 vs 普通梯形加速
    % ==========================================================

    %% 0. 基本检查
    if nargin < 1
        error('numberTran9_plot_only 需要输入 path');
    end

    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少，至少需要 2 个点。');
    end

    if size(path, 2) < 4
        error('path 至少应包含 4 列：[Z, Y, X, T]');
    end

    %% 1. 物理极限设置
    V_max_Z = 0.0097; A_max_Z = 0.03; D_max_Z = 0.03;
    V_max_Y = 1.05;   A_max_Y = 5.0;  D_max_Y = 4.0;
    V_max_X = 0.68;   A_max_X = 5.0;  D_max_X = 4.0;

    SCALE_Z = 205560;
    SCALE_Y = 945;
    SCALE_X = 1455;

    OFFSET_Z = -11667;
    OFFSET_Y = -1900;
    OFFSET_X_BIAS = 3419.6;

    V_limits = [V_max_Z * SCALE_Z, V_max_Y * SCALE_Y, V_max_X * SCALE_X];
    A_limits = [A_max_Z * SCALE_Z, A_max_Y * SCALE_Y, A_max_X * SCALE_X];
    D_limits = [D_max_Z * SCALE_Z, D_max_Y * SCALE_Y, D_max_X * SCALE_X];

    % 最终硬限制
    V_MAX_LIMITS = [2000, 600, 600];
    V_limits = min(V_limits, V_MAX_LIMITS);

    axis_names = {'J1', 'J2', 'J3'};

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
    delta_pulses = diff(motor_pos(:, 1:3));        % (num_segments x 3)
    abs_delta = abs(delta_pulses);
    num_segments = num_points - 1;

    %% 4. 各轴独立节点速度求解
    % V_node_init_axis / V_node_fwd_axis / V_node_axis : (num_points x 3)
    V_node_init_axis = zeros(num_points, 3);
    V_node_fwd_axis  = zeros(num_points, 3);
    V_node_axis      = zeros(num_points, 3);

    for j = 1:3
        % -------- (1) 初始前瞻 --------
        for k = 2:num_segments
            d_prev = delta_pulses(k-1, j);
            d_curr = delta_pulses(k,   j);

            % 若该轴任一相邻段位移接近 0，则该节点速度置 0
            if abs(d_prev) < 1e-9 || abs(d_curr) < 1e-9
                V_node_init_axis(k, j) = 0;
                continue;
            end

            % 仅当该轴自身发生折返时才置 0
            has_reverse = (sign(d_prev) * sign(d_curr) < 0);

            if has_reverse
                V_node_init_axis(k, j) = 0;
            else
                V_node_init_axis(k, j) = 0.9 * V_limits(j);
            end
        end

        V_node_init_axis(1, j) = 0;
        V_node_init_axis(end, j) = 0;

        % -------- (2) 前向扫描：该轴独立加速约束 --------
        V_node_fwd_axis(:, j) = V_node_init_axis(:, j);
        for k = 2:num_points
            prevSeg = k - 1;
            S_prev = abs_delta(prevSeg, j);
            if S_prev < 1e-9
                continue;
            end

            a_lim = A_limits(j);
            V_node_fwd_axis(k, j) = min(V_node_fwd_axis(k, j), ...
                sqrt(max(V_node_fwd_axis(k-1, j)^2 + 2 * a_lim * S_prev, 0)));
        end

        V_node_fwd_axis(1, j) = 0;
        V_node_fwd_axis(end, j) = 0;

        % -------- (3) 后向扫描：该轴独立减速约束 --------
        V_node_axis(:, j) = V_node_fwd_axis(:, j);
        for k = num_points-1:-1:1
            S_k = abs_delta(k, j);
            if S_k < 1e-9
                continue;
            end

            d_lim = D_limits(j);
            V_node_axis(k, j) = min(V_node_axis(k, j), ...
                sqrt(max(V_node_axis(k+1, j)^2 + 2 * d_lim * S_k, 0)));
        end

        V_node_axis(1, j) = 0;
        V_node_axis(end, j) = 0;
    end

    %% 5. 每段逐轴规划：非对称 S 曲线 + 普通梯形
    % 保存每段各轴原始参数和同步后的参数
    segDataS = repmat(struct( ...
        'S', zeros(1,3), ...
        'signs', zeros(1,3), ...
        'vs_raw', zeros(1,3), ...
        've_raw', zeros(1,3), ...
        'vp_raw', zeros(1,3), ...
        'aa_raw', zeros(1,3), ...
        'ad_raw', zeros(1,3), ...
        'Tja_raw', zeros(1,3), ...
        'Tjd_raw', zeros(1,3), ...
        'Ta_raw', zeros(1,3), ...
        'Tc_raw', zeros(1,3), ...
        'Td_raw', zeros(1,3), ...
        'Ttot_raw', zeros(1,3), ...
        'sync_scale', ones(1,3), ...
        'vs_sync', zeros(1,3), ...
        've_sync', zeros(1,3), ...
        'vp_sync', zeros(1,3), ...
        'aa_sync', zeros(1,3), ...
        'ad_sync', zeros(1,3), ...
        'Tja_sync', zeros(1,3), ...
        'Tjd_sync', zeros(1,3), ...
        'Ta_sync', zeros(1,3), ...
        'Tc_sync', zeros(1,3), ...
        'Td_sync', zeros(1,3), ...
        'Ttot_sync', 0), num_segments, 1);

    segDataTrap = repmat(struct( ...
        'S', zeros(1,3), ...
        'signs', zeros(1,3), ...
        'vs_raw', zeros(1,3), ...
        've_raw', zeros(1,3), ...
        'vp_raw', zeros(1,3), ...
        'aa_raw', zeros(1,3), ...
        'ad_raw', zeros(1,3), ...
        'Ta_raw', zeros(1,3), ...
        'Tc_raw', zeros(1,3), ...
        'Td_raw', zeros(1,3), ...
        'Ttot_raw', zeros(1,3), ...
        'sync_scale', ones(1,3), ...
        'vs_sync', zeros(1,3), ...
        've_sync', zeros(1,3), ...
        'vp_sync', zeros(1,3), ...
        'aa_sync', zeros(1,3), ...
        'ad_sync', zeros(1,3), ...
        'Ta_sync', zeros(1,3), ...
        'Tc_sync', zeros(1,3), ...
        'Td_sync', zeros(1,3), ...
        'Ttot_sync', 0), num_segments, 1);

    for k = 1:num_segments
        Ttot_s_axis = zeros(1,3);
        Ttot_t_axis = zeros(1,3);

        % -------- 先对三轴分别做原始规划 --------
        for j = 1:3
            S_seg = abs_delta(k, j);
            sgn_j = sign(delta_pulses(k, j));
            if sgn_j == 0
                sgn_j = 1;
            end

            vs = V_node_axis(k,   j);
            ve = V_node_axis(k+1, j);
            vmax = V_limits(j);
            aa = A_limits(j);
            ad = D_limits(j);

            segDataS(k).S(j) = S_seg;
            segDataS(k).signs(j) = sgn_j;
            segDataS(k).vs_raw(j) = vs;
            segDataS(k).ve_raw(j) = ve;
            segDataS(k).aa_raw(j) = aa;
            segDataS(k).ad_raw(j) = ad;

            segDataTrap(k).S(j) = S_seg;
            segDataTrap(k).signs(j) = sgn_j;
            segDataTrap(k).vs_raw(j) = vs;
            segDataTrap(k).ve_raw(j) = ve;
            segDataTrap(k).aa_raw(j) = aa;
            segDataTrap(k).ad_raw(j) = ad;

            if S_seg < 1e-9
                % 零位移轴
                segDataS(k).vp_raw(j) = 0;
                segDataS(k).Tja_raw(j) = 0;
                segDataS(k).Tjd_raw(j) = 0;
                segDataS(k).Ta_raw(j) = 0;
                segDataS(k).Tc_raw(j) = 0;
                segDataS(k).Td_raw(j) = 0;
                segDataS(k).Ttot_raw(j) = 0;

                segDataTrap(k).vp_raw(j) = 0;
                segDataTrap(k).Ta_raw(j) = 0;
                segDataTrap(k).Tc_raw(j) = 0;
                segDataTrap(k).Td_raw(j) = 0;
                segDataTrap(k).Ttot_raw(j) = 0;

                continue;
            end

            % 非对称 S 曲线
            [vp_s, Tja_s, Tjd_s, Ta_s, Tc_s, Td_s, Ttot_s] = ...
                calc_asym_logic_cp_axis(S_seg, vmax, aa, ad, vs, ve);

            segDataS(k).vp_raw(j) = vp_s;
            segDataS(k).Tja_raw(j) = Tja_s;
            segDataS(k).Tjd_raw(j) = Tjd_s;
            segDataS(k).Ta_raw(j) = Ta_s;
            segDataS(k).Tc_raw(j) = Tc_s;
            segDataS(k).Td_raw(j) = Td_s;
            segDataS(k).Ttot_raw(j) = Ttot_s;

            Ttot_s_axis(j) = Ttot_s;

            % 普通梯形
            [vp_t, Ta_t, Tc_t, Td_t, Ttot_t] = ...
                calc_trapezoid_cp_axis(S_seg, vmax, aa, ad, vs, ve);

            segDataTrap(k).vp_raw(j) = vp_t;
            segDataTrap(k).Ta_raw(j) = Ta_t;
            segDataTrap(k).Tc_raw(j) = Tc_t;
            segDataTrap(k).Td_raw(j) = Td_t;
            segDataTrap(k).Ttot_raw(j) = Ttot_t;

            Ttot_t_axis(j) = Ttot_t;
        end

        % -------- 段时间同步：取该段三轴最大时间 --------
        T_sync_s = max(Ttot_s_axis);
        T_sync_t = max(Ttot_t_axis);

        if T_sync_s < 1e-6
            T_sync_s = 0.001;
        end
        if T_sync_t < 1e-6
            T_sync_t = 0.001;
        end

        segDataS(k).Ttot_sync = T_sync_s;
        segDataTrap(k).Ttot_sync = T_sync_t;

        % -------- 对非对称 S 曲线按时间缩放同步 --------
        for j = 1:3
            Traw = segDataS(k).Ttot_raw(j);

            if segDataS(k).S(j) < 1e-9 || Traw < 1e-9
                segDataS(k).sync_scale(j) = 1;
                segDataS(k).vs_sync(j) = 0;
                segDataS(k).ve_sync(j) = 0;
                segDataS(k).vp_sync(j) = 0;
                segDataS(k).aa_sync(j) = 0;
                segDataS(k).ad_sync(j) = 0;
                segDataS(k).Tja_sync(j) = 0;
                segDataS(k).Tjd_sync(j) = 0;
                segDataS(k).Ta_sync(j) = 0;
                segDataS(k).Tc_sync(j) = 0;
                segDataS(k).Td_sync(j) = 0;
                continue;
            end

            gamma = T_sync_s / Traw;
            segDataS(k).sync_scale(j) = gamma;

            % 时间放大 gamma 倍，则速度缩小 1/gamma，加速度缩小 1/gamma^2
            segDataS(k).vs_sync(j) = segDataS(k).vs_raw(j) / gamma;
            segDataS(k).ve_sync(j) = segDataS(k).ve_raw(j) / gamma;
            segDataS(k).vp_sync(j) = segDataS(k).vp_raw(j) / gamma;
            segDataS(k).aa_sync(j) = segDataS(k).aa_raw(j) / gamma^2;
            segDataS(k).ad_sync(j) = segDataS(k).ad_raw(j) / gamma^2;

            segDataS(k).Tja_sync(j) = segDataS(k).Tja_raw(j) * gamma;
            segDataS(k).Tjd_sync(j) = segDataS(k).Tjd_raw(j) * gamma;
            segDataS(k).Ta_sync(j)  = segDataS(k).Ta_raw(j)  * gamma;
            segDataS(k).Tc_sync(j)  = segDataS(k).Tc_raw(j)  * gamma;
            segDataS(k).Td_sync(j)  = segDataS(k).Td_raw(j)  * gamma;
        end

        % -------- 对普通梯形按时间缩放同步 --------
        for j = 1:3
            Traw = segDataTrap(k).Ttot_raw(j);

            if segDataTrap(k).S(j) < 1e-9 || Traw < 1e-9
                segDataTrap(k).sync_scale(j) = 1;
                segDataTrap(k).vs_sync(j) = 0;
                segDataTrap(k).ve_sync(j) = 0;
                segDataTrap(k).vp_sync(j) = 0;
                segDataTrap(k).aa_sync(j) = 0;
                segDataTrap(k).ad_sync(j) = 0;
                segDataTrap(k).Ta_sync(j) = 0;
                segDataTrap(k).Tc_sync(j) = 0;
                segDataTrap(k).Td_sync(j) = 0;
                continue;
            end

            gamma = T_sync_t / Traw;
            segDataTrap(k).sync_scale(j) = gamma;

            segDataTrap(k).vs_sync(j) = segDataTrap(k).vs_raw(j) / gamma;
            segDataTrap(k).ve_sync(j) = segDataTrap(k).ve_raw(j) / gamma;
            segDataTrap(k).vp_sync(j) = segDataTrap(k).vp_raw(j) / gamma;
            segDataTrap(k).aa_sync(j) = segDataTrap(k).aa_raw(j) / gamma^2;
            segDataTrap(k).ad_sync(j) = segDataTrap(k).ad_raw(j) / gamma^2;

            segDataTrap(k).Ta_sync(j) = segDataTrap(k).Ta_raw(j) * gamma;
            segDataTrap(k).Tc_sync(j) = segDataTrap(k).Tc_raw(j) * gamma;
            segDataTrap(k).Td_sync(j) = segDataTrap(k).Td_raw(j) * gamma;
        end
    end

    %% 6. 生成整条轨迹绘图数据
    % 非对称 S 曲线
    t_plot_all_s = [];
    V_plot_all_s = cell(1,3);
    A_plot_all_s = cell(1,3);
    for j = 1:3
        V_plot_all_s{j} = [];
        A_plot_all_s{j} = [];
    end

    % 梯形
    t_plot_all_t = [];
    V_plot_all_t = cell(1,3);
    A_plot_all_t = cell(1,3);
    for j = 1:3
        V_plot_all_t{j} = [];
        A_plot_all_t{j} = [];
    end

    % 节点时间
    t_node_s = zeros(num_points,1);
    t_node_t = zeros(num_points,1);
    t_cur_s = 0;
    t_cur_t = 0;

    % 节点速度（同步后，用于画点）
    V_node_sync_s = zeros(num_points,3);
    V_node_sync_t = zeros(num_points,3);

    for k = 1:num_segments
        % ---------- 非对称 S 曲线 ----------
        Tseg = segDataS(k).Ttot_sync;
        dt_arr = linspace(0, Tseg, 100);

        if isempty(t_plot_all_s)
            t_plot_all_s = dt_arr + t_cur_s;
        else
            t_plot_all_s = [t_plot_all_s, dt_arr + t_cur_s];
        end

        t_node_s(k) = t_cur_s;

        for j = 1:3
            v_seg = zeros(size(dt_arr));
            a_seg = zeros(size(dt_arr));

            if segDataS(k).S(j) >= 1e-9
                for ii = 1:length(dt_arr)
                    [v_tmp, a_tmp] = eval_scurve_at_time( ...
                        dt_arr(ii), ...
                        segDataS(k).vs_sync(j), ...
                        segDataS(k).ve_sync(j), ...
                        segDataS(k).vp_sync(j), ...
                        segDataS(k).aa_sync(j), ...
                        segDataS(k).ad_sync(j), ...
                        segDataS(k).Tja_sync(j), ...
                        segDataS(k).Tjd_sync(j), ...
                        segDataS(k).Ta_sync(j), ...
                        segDataS(k).Tc_sync(j), ...
                        segDataS(k).Td_sync(j), ...
                        segDataS(k).Ttot_sync);

                    v_seg(ii) = segDataS(k).signs(j) * v_tmp;
                    a_seg(ii) = segDataS(k).signs(j) * a_tmp;
                end
            end

            V_plot_all_s{j} = [V_plot_all_s{j}, v_seg];
            A_plot_all_s{j} = [A_plot_all_s{j}, a_seg];
            V_node_sync_s(k, j) = segDataS(k).signs(j) * segDataS(k).vs_sync(j);
        end

        t_cur_s = t_cur_s + Tseg;

        % ---------- 梯形 ----------
        Tseg_t = segDataTrap(k).Ttot_sync;
        dt_arr_t = linspace(0, Tseg_t, 100);

        if isempty(t_plot_all_t)
            t_plot_all_t = dt_arr_t + t_cur_t;
        else
            t_plot_all_t = [t_plot_all_t, dt_arr_t + t_cur_t];
        end

        t_node_t(k) = t_cur_t;

        for j = 1:3
            v_seg = zeros(size(dt_arr_t));
            a_seg = zeros(size(dt_arr_t));

            if segDataTrap(k).S(j) >= 1e-9
                for ii = 1:length(dt_arr_t)
                    [v_tmp, a_tmp] = eval_trapezoid_at_time( ...
                        dt_arr_t(ii), ...
                        segDataTrap(k).vs_sync(j), ...
                        segDataTrap(k).ve_sync(j), ...
                        segDataTrap(k).vp_sync(j), ...
                        segDataTrap(k).aa_sync(j), ...
                        segDataTrap(k).ad_sync(j), ...
                        segDataTrap(k).Ta_sync(j), ...
                        segDataTrap(k).Tc_sync(j), ...
                        segDataTrap(k).Td_sync(j), ...
                        segDataTrap(k).Ttot_sync);

                    v_seg(ii) = segDataTrap(k).signs(j) * v_tmp;
                    a_seg(ii) = segDataTrap(k).signs(j) * a_tmp;
                end
            end

            V_plot_all_t{j} = [V_plot_all_t{j}, v_seg];
            A_plot_all_t{j} = [A_plot_all_t{j}, a_seg];
            V_node_sync_t(k, j) = segDataTrap(k).signs(j) * segDataTrap(k).vs_sync(j);
        end

        t_cur_t = t_cur_t + Tseg_t;
    end

    % 终点节点
    t_node_s(end) = t_cur_s;
    t_node_t(end) = t_cur_t;
    V_node_sync_s(end, :) = 0;
    V_node_sync_t(end, :) = 0;

    %% 7. 画图 1：各轴独立节点速度
    figure('Name', 'Independent Node Velocities', ...
           'Position', [80, 80, 1000, 780], 'Color', 'w');

    node_idx = 1:num_points;

    for j = 1:3
        subplot(3,1,j);
        plot(node_idx, V_node_init_axis(:,j), 'bo-', 'LineWidth', 1.2, 'MarkerSize', 5); hold on;
        plot(node_idx, V_node_fwd_axis(:,j), 'ms-', 'LineWidth', 1.2, 'MarkerSize', 5);
        plot(node_idx, V_node_axis(:,j), 'r^-', 'LineWidth', 1.8, 'MarkerSize', 6);

        grid on; box on;
        xlabel('Node Index');
        ylabel('Velocity (Hz)');
        title(['Node Velocity of ', axis_names{j}, ' (Zero Only When This Axis Reverses)']);
        legend('Initial Look-ahead', 'After Forward Pass', 'Final Node Velocity', ...
               'Location', 'best');
        set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    end

    %% 8. 画图 2：非对称 S 曲线三轴速度/加速度
    figure('Name', 'Asymmetric S-Curve Profiles (Axis-independent Node Velocities)', ...
           'Position', [120, 80, 1050, 760], 'Color', 'w');

    subplot(2,1,1);
    h1 = plot(t_plot_all_s, V_plot_all_s{1}, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all_s, V_plot_all_s{2}, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all_s, V_plot_all_s{3}, 'b-', 'LineWidth', 1.5);

    plot(t_node_s, V_node_sync_s(:,1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node_s, V_node_sync_s(:,2), 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node_s, V_node_sync_s(:,3), 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);

    grid on;
    title('(a) Asymmetric S-Curve Velocity Profile');
    xlabel('Time (s)');
    ylabel('Velocity (Hz)');
    legend([h1, h2, h3], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    subplot(2,1,2);
    h4 = plot(t_plot_all_s, A_plot_all_s{1}, 'r-', 'LineWidth', 1.5); hold on;
    h5 = plot(t_plot_all_s, A_plot_all_s{2}, 'g-', 'LineWidth', 1.5);
    h6 = plot(t_plot_all_s, A_plot_all_s{3}, 'b-', 'LineWidth', 1.5);

    grid on;
    title('(b) Asymmetric S-Curve Acceleration Profile');
    xlabel('Time (s)');
    ylabel('Acceleration (Hz/s)');
    legend([h4, h5, h6], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    %% 9. 画图 3：普通梯形三轴速度/加速度
    figure('Name', 'Trapezoidal Profiles (Axis-independent Node Velocities)', ...
           'Position', [160, 100, 1050, 760], 'Color', 'w');

    subplot(2,1,1);
    h1 = plot(t_plot_all_t, V_plot_all_t{1}, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all_t, V_plot_all_t{2}, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all_t, V_plot_all_t{3}, 'b-', 'LineWidth', 1.5);

    plot(t_node_t, V_node_sync_t(:,1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node_t, V_node_sync_t(:,2), 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node_t, V_node_sync_t(:,3), 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);

    grid on;
    title('(a) Trapezoidal Velocity Profile');
    xlabel('Time (s)');
    ylabel('Velocity (Hz)');
    legend([h1, h2, h3], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    subplot(2,1,2);
    h4 = plot(t_plot_all_t, A_plot_all_t{1}, 'r-', 'LineWidth', 1.5); hold on;
    h5 = plot(t_plot_all_t, A_plot_all_t{2}, 'g-', 'LineWidth', 1.5);
    h6 = plot(t_plot_all_t, A_plot_all_t{3}, 'b-', 'LineWidth', 1.5);

    grid on;
    title('(b) Trapezoidal Acceleration Profile');
    xlabel('Time (s)');
    ylabel('Acceleration (Hz/s)');
    legend([h4, h5, h6], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    %% 10. 画图 4：S 曲线 vs 梯形加速对比
    figure('Name', 'Comparison: Asymmetric S-Curve vs Trapezoidal', ...
           'Position', [200, 60, 1100, 820], 'Color', 'w');

    for j = 1:3
        subplot(3,2,2*j-1);
        plot(t_plot_all_s, V_plot_all_s{j}, 'b-', 'LineWidth', 1.8); hold on;
        plot(t_plot_all_t, V_plot_all_t{j}, 'r--', 'LineWidth', 1.4);
        grid on; box on;
        xlabel('Time (s)');
        ylabel('Velocity (Hz)');
        title([axis_names{j}, ' Velocity Comparison']);
        legend('Asymmetric S-Curve', 'Trapezoidal', 'Location', 'best');
        set(gca, 'FontSize', 10.5, 'FontName', 'Times New Roman');

        subplot(3,2,2*j);
        plot(t_plot_all_s, A_plot_all_s{j}, 'b-', 'LineWidth', 1.8); hold on;
        plot(t_plot_all_t, A_plot_all_t{j}, 'r--', 'LineWidth', 1.4);
        grid on; box on;
        xlabel('Time (s)');
        ylabel('Acceleration (Hz/s)');
        title([axis_names{j}, ' Acceleration Comparison']);
        legend('Asymmetric S-Curve', 'Trapezoidal', 'Location', 'best');
        set(gca, 'FontSize', 10.5, 'FontName', 'Times New Roman');
    end

    %% 11. 输出结果
    result = struct();
    result.motor_pos = motor_pos;
    result.delta_pulses = delta_pulses;
    result.abs_delta = abs_delta;

    result.V_limits = V_limits;
    result.A_limits = A_limits;
    result.D_limits = D_limits;

    result.V_node_init_axis = V_node_init_axis;
    result.V_node_fwd_axis = V_node_fwd_axis;
    result.V_node_axis = V_node_axis;

    result.segDataS = segDataS;
    result.segDataTrap = segDataTrap;

    result.t_plot_all_s = t_plot_all_s;
    result.V_plot_all_s = V_plot_all_s;
    result.A_plot_all_s = A_plot_all_s;
    result.t_plot_all_t = t_plot_all_t;
    result.V_plot_all_t = V_plot_all_t;
    result.A_plot_all_t = A_plot_all_t;

    result.t_node_s = t_node_s;
    result.t_node_t = t_node_t;
    result.V_node_sync_s = V_node_sync_s;
    result.V_node_sync_t = V_node_sync_t;
end


%% ==========================================================
% 各轴独立的非对称 S 曲线参数计算
%% ==========================================================
function [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp_axis(S, vmax, aa, ad, vs, ve)
    if S < 1e-6
        S = 1e-6;
    end

    S_l = (0.75 * (vmax^2 - vs^2) / aa) + (0.75 * (vmax^2 - ve^2) / ad);

    if S >= S_l
        vp = vmax;
        Tc = (S - S_l) / max(vp, 1e-9);
    else
        vp = sqrt(max(((4/3) * S * aa * ad + vs^2 * ad + ve^2 * aa) / (aa + ad), 0));
        Tc = 0;
    end

    T_ja = max((vp - vs) / (2 * aa), 0);
    T_jd = max((vp - ve) / (2 * ad), 0);

    Ta = 3 * T_ja;
    Td = 3 * T_jd;
    Ttot = Ta + Tc + Td;
end


%% ==========================================================
% 各轴独立的普通梯形参数计算
%% ==========================================================
function [vp, Ta, Tc, Td, Ttot] = calc_trapezoid_cp_axis(S, vmax, aa, ad, vs, ve)
    if S < 1e-6
        S = 1e-6;
    end

    S_acc = max((vmax^2 - vs^2) / (2 * aa), 0);
    S_dec = max((vmax^2 - ve^2) / (2 * ad), 0);
    S_lim = S_acc + S_dec;

    if S >= S_lim
        vp = vmax;
        Ta = max((vp - vs) / aa, 0);
        Td = max((vp - ve) / ad, 0);
        Tc = (S - S_lim) / max(vp, 1e-9);
    else
        vp = sqrt(max((2*S*aa*ad + ad*vs^2 + aa*ve^2) / (aa + ad), 0));
        Ta = max((vp - vs) / aa, 0);
        Td = max((vp - ve) / ad, 0);
        Tc = 0;
    end

    Ttot = Ta + Tc + Td;
end


%% ==========================================================
% 求某时刻的主导轴 S 曲线速度与加速度
%% ==========================================================
function [v, a] = eval_scurve_at_time(t, vs, ve, vp, aa, ad, T_ja, T_jd, Ta, Tc, Td, Ttot)
    if t <= 0
        v = vs;
        a = 0;
        return;
    end

    if t >= Ttot
        v = ve;
        a = 0;
        return;
    end

    if t < T_ja
        j = aa / max(T_ja, 1e-9);
        a = j * t;
        v = vs + 0.5 * j * t^2;

    elseif t < 2 * T_ja
        dt = t - T_ja;
        a = aa;
        v = vs + 0.5 * aa * T_ja + aa * dt;

    elseif t < Ta
        d_dt_acc = Ta - t;
        j = -aa / max(T_ja, 1e-9);
        a = j * d_dt_acc;
        v = vp - 0.5 * abs(j) * d_dt_acc^2;

    elseif t < (Ta + Tc)
        a = 0;
        v = vp;

    elseif t < (Ta + Tc + T_jd)
        d_dt_dec = t - (Ta + Tc);
        j = -ad / max(T_jd, 1e-9);
        a = j * d_dt_dec;
        v = vp + 0.5 * j * d_dt_dec^2;

    elseif t < (Ta + Tc + 2 * T_jd)
        d_dt_dec = t - (Ta + Tc + T_jd);
        a = -ad;
        v = (vp - 0.5 * ad * T_jd) - ad * d_dt_dec;

    else
        d_dt_end = Ttot - t;
        if d_dt_end < 0
            d_dt_end = 0;
        end
        j = ad / max(T_jd, 1e-9);
        a = -j * d_dt_end;
        v = ve + 0.5 * j * d_dt_end^2;
    end
end


%% ==========================================================
% 求某时刻的普通梯形速度与加速度
%% ==========================================================
function [v, a] = eval_trapezoid_at_time(t, vs, ve, vp, aa, ad, Ta, Tc, Td, Ttot)
    if t <= 0
        v = vs;
        a = 0;
        return;
    end

    if t >= Ttot
        v = ve;
        a = 0;
        return;
    end

    if t < Ta
        a = aa;
        v = vs + aa * t;

    elseif t < (Ta + Tc)
        a = 0;
        v = vp;

    else
        dt = t - (Ta + Tc);
        a = -ad;
        v = vp - ad * dt;
        if v < ve
            v = ve;
            a = 0;
        end
    end
end