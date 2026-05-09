function numberTran9_2(arduino, path)
    % ==========================================================
    % 改进版：各轴独立节点速度 + 各轴独立 S 曲线 + 段时间同步
    % 方案B：发送给 Arduino 前统一放慢时间（timeScale = 1.10）
    % ==========================================================

    %% 0. 基本检查
    if nargin < 2
        error('numberTran9_2 需要两个输入参数：arduino, path');
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

    %% 4. 各轴独立节点速度求解
    V_node_init_axis = zeros(num_points, 3);
    V_node_fwd_axis  = zeros(num_points, 3);
    V_node_axis      = zeros(num_points, 3);

    for j = 1:3
        % -------- (1) 初始前瞻 --------
        for k = 2:num_segments
            d_prev = delta_pulses(k-1, j);
            d_curr = delta_pulses(k,   j);

            if abs(d_prev) < 1e-9 || abs(d_curr) < 1e-9
                V_node_init_axis(k, j) = 0;
                continue;
            end

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

    %% 5. 逐段逐轴规划 + 时间同步
    segData = repmat(struct( ...
        'z0', 0, 'y0', 0, 'x0', 0, ...
        'z1', 0, 'y1', 0, 'x1', 0, ...
        'signs', zeros(1,3), ...
        'S', zeros(1,3), ...
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

    for k = 1:num_segments
        Ttot_axis = zeros(1,3);

        segData(k).z0 = motor_pos(k, 1);
        segData(k).y0 = motor_pos(k, 2);
        segData(k).x0 = motor_pos(k, 3);
        segData(k).z1 = motor_pos(k+1, 1);
        segData(k).y1 = motor_pos(k+1, 2);
        segData(k).x1 = motor_pos(k+1, 3);

        % ------- 每个轴各自规划 -------
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

            segData(k).signs(j)  = sgn_j;
            segData(k).S(j)      = S_seg;
            segData(k).vs_raw(j) = vs;
            segData(k).ve_raw(j) = ve;
            segData(k).aa_raw(j) = aa;
            segData(k).ad_raw(j) = ad;

            if S_seg < 1e-9
                segData(k).vp_raw(j)   = 0;
                segData(k).Tja_raw(j)  = 0;
                segData(k).Tjd_raw(j)  = 0;
                segData(k).Ta_raw(j)   = 0;
                segData(k).Tc_raw(j)   = 0;
                segData(k).Td_raw(j)   = 0;
                segData(k).Ttot_raw(j) = 0;
                continue;
            end

            [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = ...
                calc_asym_logic_cp_axis(S_seg, vmax, aa, ad, vs, ve);

            segData(k).vp_raw(j)   = vp;
            segData(k).Tja_raw(j)  = Tja;
            segData(k).Tjd_raw(j)  = Tjd;
            segData(k).Ta_raw(j)   = Ta;
            segData(k).Tc_raw(j)   = Tc;
            segData(k).Td_raw(j)   = Td;
            segData(k).Ttot_raw(j) = Ttot;

            Ttot_axis(j) = Ttot;
        end

        % ------- 段时间同步 -------
        T_sync = max(Ttot_axis);
        if T_sync < 1e-6
            T_sync = 0.001;
        end
        segData(k).Ttot_sync = T_sync;

        for j = 1:3
            Traw = segData(k).Ttot_raw(j);

            if segData(k).S(j) < 1e-9 || Traw < 1e-9
                segData(k).sync_scale(j) = 1;
                segData(k).vs_sync(j) = 0;
                segData(k).ve_sync(j) = 0;
                segData(k).vp_sync(j) = 0;
                segData(k).aa_sync(j) = 0;
                segData(k).ad_sync(j) = 0;
                segData(k).Tja_sync(j) = 0;
                segData(k).Tjd_sync(j) = 0;
                segData(k).Ta_sync(j) = 0;
                segData(k).Tc_sync(j) = 0;
                segData(k).Td_sync(j) = 0;
                continue;
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



        %% 6. 绘图数据生成与绘图
    % ==========================================================
    % 基于同步后的逐轴独立 S 曲线参数生成速度/加速度图
    % ==========================================================
    t_plot_all = [];
    V_plot_all = cell(1,3);
    A_plot_all = cell(1,3);
    for j = 1:3
        V_plot_all{j} = [];
        A_plot_all{j} = [];
    end

    t_node = zeros(num_points,1);
    V_node_sync = zeros(num_points,3);
    A_node_sync = zeros(num_points,3);

    t_cur = 0;

    for k = 1:num_segments
        Tseg = segData(k).Ttot_sync;
        if Tseg < 1e-9
            Tseg = 0.001;
        end

        dt_arr = linspace(0, Tseg, 100);

        if isempty(t_plot_all)
            t_plot_all = dt_arr + t_cur;
        else
            t_plot_all = [t_plot_all, dt_arr + t_cur];
        end

        t_node(k) = t_cur;

        for j = 1:3
            v_seg = zeros(size(dt_arr));
            a_seg = zeros(size(dt_arr));

            if segData(k).S(j) >= 1e-9
                for ii = 1:length(dt_arr)
                    [v_tmp, a_tmp] = eval_scurve_at_time( ...
                        dt_arr(ii), ...
                        segData(k).vs_sync(j), ...
                        segData(k).ve_sync(j), ...
                        segData(k).vp_sync(j), ...
                        segData(k).aa_sync(j), ...
                        segData(k).ad_sync(j), ...
                        segData(k).Tja_sync(j), ...
                        segData(k).Tjd_sync(j), ...
                        segData(k).Ta_sync(j), ...
                        segData(k).Tc_sync(j), ...
                        segData(k).Td_sync(j), ...
                        segData(k).Ttot_sync);

                    v_seg(ii) = segData(k).signs(j) * v_tmp;
                    a_seg(ii) = segData(k).signs(j) * a_tmp;
                end
            end

            V_plot_all{j} = [V_plot_all{j}, v_seg];
            A_plot_all{j} = [A_plot_all{j}, a_seg];

            V_node_sync(k, j) = segData(k).signs(j) * segData(k).vs_sync(j);
            A_node_sync(k, j) = 0;
        end

        t_cur = t_cur + Tseg;
    end

    % 终点节点
    t_node(end) = t_cur;
    V_node_sync(end,:) = 0;
    A_node_sync(end,:) = 0;

    % ==========================================================
    % 图1：各轴独立节点速度
    % ==========================================================
    figure('Name', 'Independent Node Velocities', ...
           'Position', [80, 80, 1000, 780], 'Color', 'w');

    axis_names = {'J1','J2','J3'};
    node_idx = 1:num_points;

    for j = 1:3
        subplot(3,1,j);
        plot(node_idx, V_node_init_axis(:,j), 'bo-', 'LineWidth', 1.2, 'MarkerSize', 5); hold on;
        plot(node_idx, V_node_fwd_axis(:,j),  'ms-', 'LineWidth', 1.2, 'MarkerSize', 5);
        plot(node_idx, V_node_axis(:,j),      'r^-', 'LineWidth', 1.8, 'MarkerSize', 6);

        grid on; box on;
        xlabel('Node Index');
        ylabel('Velocity (Hz)');
        title(['Node Velocity of ', axis_names{j}, ' (Zero Only When This Axis Reverses)']);
        legend('Initial Look-ahead', 'After Forward Pass', 'Final Node Velocity', ...
               'Location', 'best');
        set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');
    end

    % ==========================================================
    % 图2：逐轴独立 S 曲线速度/加速度图
    % ==========================================================
    figure('Name', 'Axis-independent S-Curve Segment Planning', ...
           'Position', [120, 80, 1050, 760], 'Color', 'w');

    subplot(2,1,1);
    h1 = plot(t_plot_all, V_plot_all{1}, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all, V_plot_all{2}, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all, V_plot_all{3}, 'b-', 'LineWidth', 1.5);

    plot(t_node, V_node_sync(:,1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, V_node_sync(:,2), 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, V_node_sync(:,3), 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);

    grid on;
    title('(a) Axis-independent S-Curve Velocity Profile');
    xlabel('Time (s)');
    ylabel('Velocity (Hz)');
    legend([h1, h2, h3], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    subplot(2,1,2);
    h4 = plot(t_plot_all, A_plot_all{1}, 'r-', 'LineWidth', 1.5); hold on;
    h5 = plot(t_plot_all, A_plot_all{2}, 'g-', 'LineWidth', 1.5);
    h6 = plot(t_plot_all, A_plot_all{3}, 'b-', 'LineWidth', 1.5);

    plot(t_node, A_node_sync(:,1), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, A_node_sync(:,2), 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, A_node_sync(:,3), 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);

    grid on;
    title('(b) Axis-independent S-Curve Acceleration Profile');
    xlabel('Time (s)');
    ylabel('Acceleration (Hz/s)');
    legend([h4, h5, h6], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    %% 6. 串口准备
    if isprop(arduino, 'Status')
        if strcmpi(string(arduino.Status), "closed")
            fopen(arduino);
        end
    end

    flushSerialPort(arduino);
    pause(0.2);

    %% 7. 阶段一：移动到起点
    fprintf('▶ 阶段一：发送起点归位命令...\n');

    writeline(arduino, 'BEGIN_START,1');
    waitForResponse(arduino, "ACK_READY", 3.0);

    startCmd = sprintf('P,%ld,%ld,%ld', ...
        round(motor_pos(1,1)), round(motor_pos(1,2)), round(motor_pos(1,3)));
    writeline(arduino, startCmd);

    waitForOptionalResponse(arduino, "ACK_POINT", 0.5);

    writeline(arduino, 'RUN_START');
    waitForResponse(arduino, "ACK_START", 3.0);
    waitForResponse(arduino, "FINISHED_ALL", 60.0);

    fprintf('  ✓ 已到达起点。\n');
    pause(0.2);

    %% 8. 阶段二：逐轴独立参数发送
    fprintf('▶ 阶段二：发送 %d 个逐轴 S 曲线段参数...\n', num_segments);

    writeline(arduino, sprintf('BEGIN_SEGAX,%d', num_segments));
    waitForResponse(arduino, "ACK_READY", 3.0);

    % 方案B：整体放慢时间
    timeScale = 1.60;

    for k = 1:num_segments
        s = segData(k);

        % ===== 调试输出：第 6 段 =====
        if k == 6
            fprintf('--- Segment 6 ---\n');
            fprintf('p0 = [%ld %ld %ld]\n', s.z0, s.y0, s.x0);
            fprintf('p1 = [%ld %ld %ld]\n', s.z1, s.y1, s.x1);

            fprintf('J1: vs=%.3f ve=%.3f vp=%.3f aa=%.3f ad=%.3f Tja=%.6f Tjd=%.6f Ta=%.6f Tc=%.6f Td=%.6f Ttot=%.6f\n', ...
                s.vs_sync(1), s.ve_sync(1), s.vp_sync(1), s.aa_sync(1), s.ad_sync(1), ...
                s.Tja_sync(1), s.Tjd_sync(1), s.Ta_sync(1), s.Tc_sync(1), s.Td_sync(1), s.Ttot_sync);

            fprintf('J2: vs=%.3f ve=%.3f vp=%.3f aa=%.3f ad=%.3f Tja=%.6f Tjd=%.6f Ta=%.6f Tc=%.6f Td=%.6f Ttot=%.6f\n', ...
                s.vs_sync(2), s.ve_sync(2), s.vp_sync(2), s.aa_sync(2), s.ad_sync(2), ...
                s.Tja_sync(2), s.Tjd_sync(2), s.Ta_sync(2), s.Tc_sync(2), s.Td_sync(2), s.Ttot_sync);

            fprintf('J3: vs=%.3f ve=%.3f vp=%.3f aa=%.3f ad=%.3f Tja=%.6f Tjd=%.6f Ta=%.6f Tc=%.6f Td=%.6f Ttot=%.6f\n', ...
                s.vs_sync(3), s.ve_sync(3), s.vp_sync(3), s.aa_sync(3), s.ad_sync(3), ...
                s.Tja_sync(3), s.Tjd_sync(3), s.Ta_sync(3), s.Tc_sync(3), s.Td_sync(3), s.Ttot_sync);
        end
        % ===========================

        vals = zeros(1, 33);
        idx = 1;

        for j = 1:3
            vs_i  = round(abs(s.vs_sync(j)) * 100);
            ve_i  = round(abs(s.ve_sync(j)) * 100);
            vp_i  = round(abs(s.vp_sync(j)) * 100);
            aa_i  = round(abs(s.aa_sync(j)) * 100);
            ad_i  = round(abs(s.ad_sync(j)) * 100);

            % 方案B：时间统一放慢 10%
            Tja_i = round(s.Tja_sync(j) * timeScale * 1e6);
            Tjd_i = round(s.Tjd_sync(j) * timeScale * 1e6);
            Ta_i  = round(s.Ta_sync(j)  * timeScale * 1e6);
            Tc_i  = round(s.Tc_sync(j)  * timeScale * 1e6);
            Td_i  = round(s.Td_sync(j)  * timeScale * 1e6);
            Tt_i  = round(s.Ttot_sync   * timeScale * 1e6);

            vals(idx:idx+10) = [vs_i, ve_i, vp_i, aa_i, ad_i, Tja_i, Tjd_i, Ta_i, Tc_i, Td_i, Tt_i];
            idx = idx + 11;
        end

        cmd = sprintf(['A,%ld,%ld,%ld,%ld,%ld,%ld,', ...
                       '%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,', ...
                       '%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,', ...
                       '%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld'], ...
                       s.z0, s.y0, s.x0, s.z1, s.y1, s.x1, ...
                       vals(1),  vals(2),  vals(3),  vals(4),  vals(5),  vals(6),  vals(7),  vals(8),  vals(9),  vals(10), vals(11), ...
                       vals(12), vals(13), vals(14), vals(15), vals(16), vals(17), vals(18), vals(19), vals(20), vals(21), vals(22), ...
                       vals(23), vals(24), vals(25), vals(26), vals(27), vals(28), vals(29), vals(30), vals(31), vals(32), vals(33));

        writeline(arduino, cmd);
        waitForOptionalResponse(arduino, "ACK_SEG", 0.2);
    end

    writeline(arduino, 'RUN_SEGAX');
    waitForResponse(arduino, "ACK_START", 3.0);

    fprintf('  ✓ 逐轴 S 曲线段参数下发完毕，Arduino 开始执行。\n');

    waitForResponse(arduino, "FINISHED_ALL", 120.0);

    fprintf('  ✓ 全部轨迹执行完成。\n');
end


%% ==========================================================
% 各轴独立 S 曲线参数计算
%% ==========================================================
function [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp_axis(S, vmax, aa, ad, vs, ve)
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

    Tja = max((vp - vs) / (2 * aa), 0);
    Tjd = max((vp - ve) / (2 * ad), 0);

    Ta = 3 * Tja;
    Td = 3 * Tjd;
    Ttot = Ta + Tc + Td;
end


%% ==========================================================
% 等待指定响应
%% ==========================================================
function waitForResponse(arduino, targetStr, timeoutSec)
    t0 = tic;
    while toc(t0) < timeoutSec
        if arduino.NumBytesAvailable > 0
            resp = strtrim(readline(arduino));

            if ~isempty(resp)
                fprintf('[Arduino] %s\n', resp);
            end

            if strcmp(resp, targetStr)
                return;
            elseif startsWith(resp, "ERR")
                error('Arduino 返回错误：%s', resp);
            end
        end
        pause(0.005);
    end
    error('等待 Arduino 响应超时：%s', targetStr);
end


%% ==========================================================
% 等待可选响应
%% ==========================================================
function waitForOptionalResponse(arduino, targetStr, timeoutSec)
    t0 = tic;
    while toc(t0) < timeoutSec
        if arduino.NumBytesAvailable > 0
            resp = strtrim(readline(arduino));

            if ~isempty(resp)
                fprintf('[Arduino] %s\n', resp);
            end

            if strcmp(resp, targetStr)
                return;
            elseif startsWith(resp, "ERR")
                error('Arduino 返回错误：%s', resp);
            end
        end
        pause(0.003);
    end
end


%% ==========================================================
% 清空串口缓冲
%% ==========================================================
function flushSerialPort(arduino)
    try
        flush(arduino);
    catch
        try
            flushinput(arduino);
            flushoutput(arduino);
        catch
            % 忽略
        end
    end
end

%% ==========================================================
% 求某时刻的单轴 S 曲线速度与加速度（用于绘图）
%% ==========================================================
function [v, a] = eval_scurve_at_time(t, vs, ve, vp, aa, ad, Tja, Tjd, Ta, Tc, Td, Ttot)
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

    if t < Tja
        j = aa / max(Tja, 1e-9);
        a = j * t;
        v = vs + 0.5 * j * t^2;

    elseif t < 2 * Tja
        dt = t - Tja;
        a = aa;
        v = vs + 0.5 * aa * Tja + aa * dt;

    elseif t < Ta
        d_dt_acc = Ta - t;
        j = -aa / max(Tja, 1e-9);
        a = j * d_dt_acc;
        v = vp - 0.5 * abs(j) * d_dt_acc^2;

    elseif t < (Ta + Tc)
        a = 0;
        v = vp;

    elseif t < (Ta + Tc + Tjd)
        d_dt_dec = t - (Ta + Tc);
        j = -ad / max(Tjd, 1e-9);
        a = j * d_dt_dec;
        v = vp + 0.5 * j * d_dt_dec^2;

    elseif t < (Ta + Tc + 2 * Tjd)
        d_dt_dec = t - (Ta + Tc + Tjd);
        a = -ad;
        v = (vp - 0.5 * ad * Tjd) - ad * d_dt_dec;

    else
        d_dt_end = Ttot - t;
        if d_dt_end < 0
            d_dt_end = 0;
        end
        j = ad / max(Tjd, 1e-9);
        a = -j * d_dt_end;
        v = ve + 0.5 * j * d_dt_end^2;
    end
end
