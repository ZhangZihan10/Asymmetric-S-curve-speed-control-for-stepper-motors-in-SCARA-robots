function numberTran9(arduino, path)
    % ==========================================================
    % 改进版：完整 S 曲线段参数下发函数
    % 目标：
    % 1) 阶段一：先安全移动到起点
    % 2) 阶段二：按“段”为单位下发完整 S 曲线参数
    % 3) 为 Arduino 真正执行 S 曲线参数做准备
    %
    % 协议设计：
    %   BEGIN_START,1
    %   P,startZ,startY,startX,startServo
    %   RUN_START
    %
    %   BEGIN_SEG,n
    %   S,z0,y0,x0,z1,y1,x1,servo0,servo1,rZ,rY,rX,vs,ve,vp,aa,ad,Tja,Tjd,Ta,Tc,Td,Ttot
    %   ...
    %   RUN_SEG
    %
    % Arduino 端后续应按段解析并执行 S 曲线
    % ==========================================================

    %% 0. 基本检查
    if nargin < 2
        error('numberTran8 需要两个输入参数：arduino, path');
    end

    num_points = size(path, 1);
    if num_points < 2
        error('路径点数量过少，至少需要 2 个点。');
    end

    if size(path, 2) < 4
        error('path 至少应包含 4 列：[Z, Y, X, T]');
    end

    %% 1. 物理极限设置
    % 你的原始设置保留，但做了更规范整理
    V_max_Z = 0.0097; A_max_Z = 0.03; D_max_Z = 0.03;
    V_max_Y = 1.05;   A_max_Y = 5.0;  D_max_Y = 4.0;
    V_max_X = 0.68;   A_max_X = 5.0;  D_max_X = 4.0;

    % 转换到脉冲空间（Hz / Hz/s）
    SCALE_Z = 205560;
    SCALE_Y = 945;
    SCALE_X = 1455;

    OFFSET_Z = -11667;
    OFFSET_Y = -1900;
    OFFSET_X_BIAS = 3419.6;

    V_limits = [V_max_Z * SCALE_Z, V_max_Y * SCALE_Y, V_max_X * SCALE_X];
    A_limits = [A_max_Z * SCALE_Z, A_max_Y * SCALE_Y, A_max_X * SCALE_X];
    D_limits = [D_max_Z * SCALE_Z, D_max_Y * SCALE_Y, D_max_X * SCALE_X];

    % 最终硬限制（建议与你驱动器/Arduino 实际能力匹配）
    V_MAX_LIMITS = [2000, 600, 600];
    V_limits = min(V_limits, V_MAX_LIMITS);

    %% 2. 路径点映射到电机脉冲空间
    % motor_pos(:,1:3) 用整数脉冲
    % motor_pos(:,4)   舵机角度，单位 degree
    motor_pos = zeros(num_points, 4);

    for i = 1:num_points
        z_pulse = path(i, 1) * SCALE_Z + OFFSET_Z;
        y_pulse = path(i, 2) * SCALE_Y + OFFSET_Y;
        x_pulse = 0.8 * y_pulse + OFFSET_X_BIAS + path(i, 3) * SCALE_X;
        servo_deg = path(i, 4) * 180 / pi;

        motor_pos(i, 1:3) = round([z_pulse, y_pulse, x_pulse]);  % 整数脉冲
        motor_pos(i, 4)   = round(servo_deg, 2);
    end

    %% 3. 逐段位移分析
    delta_pulses = diff(motor_pos(:, 1:3));      % 每段三轴位移
    abs_delta = abs(delta_pulses);
    num_segments = num_points - 1;

    [dS_dom, dom_idx_arr] = max(abs_delta, [], 2);  % 主导轴位移长度与索引

    %% 4. 计算每段最大允许速度
    V_max_seg = zeros(num_segments, 1);

    for k = 1:num_segments
        if dS_dom(k) < 1e-9
            V_max_seg(k) = 0;
            continue;
        end

        r = abs_delta(k, :) / dS_dom(k);   % 三轴相对主导轴比例

        % 各轴按比例约束后允许的主导轴最大速度
        v_allow = [
            safeDiv(V_limits(1), max(r(1), 1e-9)), ...
            safeDiv(V_limits(2), max(r(2), 1e-9)), ...
            safeDiv(V_limits(3), max(r(3), 1e-9)), ...
            V_limits(dom_idx_arr(k))
        ];

        V_max_seg(k) = min(v_allow);
    end

    %% 5. 节点速度前瞻与双向扫描
    V_node = zeros(num_points, 1);   % 节点处主导轴速度

    for k = 2:num_segments
        sameDirection = isequal(sign(delta_pulses(k-1, :)), sign(delta_pulses(k, :)));
        sameDominant  = (dom_idx_arr(k-1) == dom_idx_arr(k));

        if sameDirection && sameDominant
            V_node(k) = min(V_max_seg(k-1), V_max_seg(k)) * 0.9;
        else
            V_node(k) = 0;
        end
    end

    % 前向扫描：加速约束
    for k = 2:num_points
        prevSeg = k - 1;
        if dS_dom(prevSeg) < 1e-9
            continue;
        end
        a_lim = A_limits(dom_idx_arr(prevSeg));
        V_node(k) = min(V_node(k), sqrt(max(V_node(k-1)^2 + 2 * a_lim * dS_dom(prevSeg), 0)));
    end

    % 后向扫描：减速约束
    for k = num_points-1:-1:1
        if dS_dom(k) < 1e-9
            continue;
        end
        d_lim = D_limits(dom_idx_arr(k));
        V_node(k) = min(V_node(k), sqrt(max(V_node(k+1)^2 + 2 * d_lim * dS_dom(k), 0)));
    end

    % 强制首尾节点速度为 0
    V_node(1) = 0;
    V_node(end) = 0;

    %% 6. 生成每段完整 S 曲线参数
    segData = repmat(struct( ...
        'z0', 0, 'y0', 0, 'x0', 0, ...
        'z1', 0, 'y1', 0, 'x1', 0, ...
        'servo0', 0, 'servo1', 0, ...
        'rZ', 0, 'rY', 0, 'rX', 0, ...
        'vs', 0, 've', 0, 'vp', 0, ...
        'aa', 0, 'ad', 0, ...
        'Tja', 0, 'Tjd', 0, ...
        'Ta', 0, 'Tc', 0, 'Td', 0, 'Ttot', 0, ...
        'domIdx', 0, 'Sdom', 0), num_segments, 1);

    % 可视化数据
    t_plot_all = [];
    Vz_plot_all = []; Vy_plot_all = []; Vx_plot_all = [];
    Az_plot_all = []; Ay_plot_all = []; Ax_plot_all = [];

    t_node = zeros(num_points, 1);
    Vz_node = zeros(num_points, 1); Vy_node = zeros(num_points, 1); Vx_node = zeros(num_points, 1);
    Az_node = zeros(num_points, 1); Ay_node = zeros(num_points, 1); Ax_node = zeros(num_points, 1);

    current_time = 0;

    for k = 1:num_segments
        S_seg = dS_dom(k);
        dom_idx = dom_idx_arr(k);

        segData(k).z0 = motor_pos(k, 1);
        segData(k).y0 = motor_pos(k, 2);
        segData(k).x0 = motor_pos(k, 3);
        segData(k).z1 = motor_pos(k+1, 1);
        segData(k).y1 = motor_pos(k+1, 2);
        segData(k).x1 = motor_pos(k+1, 3);

        segData(k).servo0 = motor_pos(k, 4);
        segData(k).servo1 = motor_pos(k+1, 4);

        segData(k).domIdx = dom_idx;
        segData(k).Sdom = S_seg;

        if S_seg < 1e-9
            % 零长度段
            segData(k).rZ = 0;
            segData(k).rY = 0;
            segData(k).rX = 0;
            segData(k).vs = 0;
            segData(k).ve = 0;
            segData(k).vp = 0;
            segData(k).aa = A_limits(dom_idx);
            segData(k).ad = D_limits(dom_idx);
            segData(k).Tja = 0;
            segData(k).Tjd = 0;
            segData(k).Ta = 0;
            segData(k).Tc = 0;
            segData(k).Td = 0;
            segData(k).Ttot = 0.001;

            t_node(k) = current_time;
            continue;
        end

        r = abs_delta(k, :) / S_seg;

        vs = V_node(k);
        ve = V_node(k+1);
        vmax = V_max_seg(k);
        aa = A_limits(dom_idx);
        ad = D_limits(dom_idx);

        [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(0, S_seg, vmax, aa, ad, vs, ve);

        segData(k).rZ = r(1);
        segData(k).rY = r(2);
        segData(k).rX = r(3);
        segData(k).vs = vs;
        segData(k).ve = ve;
        segData(k).vp = vp;
        segData(k).aa = aa;
        segData(k).ad = ad;
        segData(k).Tja = T_ja;
        segData(k).Tjd = T_jd;
        segData(k).Ta = Ta;
        segData(k).Tc = Tc;
        segData(k).Td = Td;
        segData(k).Ttot = max(Ttot, 0.001);

        % 节点图
        t_node(k) = current_time;
        Vz_node(k) = vs * r(1);
        Vy_node(k) = vs * r(2);
        Vx_node(k) = vs * r(3);
        Az_node(k) = 0;
        Ay_node(k) = 0;
        Ax_node(k) = 0;

        % 采样绘图
        dt_arr = linspace(0, segData(k).Ttot, 100);
        v_dom_array = zeros(size(dt_arr));
        a_dom_array = zeros(size(dt_arr));

        for i = 1:length(dt_arr)
            [v_dom_array(i), a_dom_array(i)] = eval_scurve_at_time( ...
                dt_arr(i), vs, ve, vp, aa, ad, T_ja, T_jd, Ta, Tc, Td, segData(k).Ttot);
        end

        t_plot_all  = [t_plot_all,  dt_arr + current_time];
        Vz_plot_all = [Vz_plot_all, v_dom_array * r(1)];
        Vy_plot_all = [Vy_plot_all, v_dom_array * r(2)];
        Vx_plot_all = [Vx_plot_all, v_dom_array * r(3)];

        Az_plot_all = [Az_plot_all, a_dom_array * r(1)];
        Ay_plot_all = [Ay_plot_all, a_dom_array * r(2)];
        Ax_plot_all = [Ax_plot_all, a_dom_array * r(3)];

        current_time = current_time + segData(k).Ttot;
    end

    % 终点节点
    t_node(num_points) = current_time;
    Vz_node(num_points) = 0;
    Vy_node(num_points) = 0;
    Vx_node(num_points) = 0;
    Az_node(num_points) = 0;
    Ay_node(num_points) = 0;
    Ax_node(num_points) = 0;

    %% 7. 绘图
    figure('Name', 'Improved S-Curve Segment Planning', ...
           'Position', [100, 100, 1000, 700], 'Color', 'w');

    subplot(2,1,1);
    h1 = plot(t_plot_all, Vz_plot_all, 'r-', 'LineWidth', 2); hold on;
    h2 = plot(t_plot_all, Vy_plot_all, 'g-', 'LineWidth', 1.5);
    h3 = plot(t_plot_all, Vx_plot_all, 'b-', 'LineWidth', 1.5);

    plot(t_node, Vz_node, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Vy_node, 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Vx_node, 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);

    grid on;
    title('(a) S-Curve Velocity Profile');
    xlabel('Time (s)');
    ylabel('Velocity (Hz)');
    legend([h1, h2, h3], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    subplot(2,1,2);
    h4 = plot(t_plot_all, Az_plot_all, 'r-', 'LineWidth', 1.5); hold on;
    h5 = plot(t_plot_all, Ay_plot_all, 'g-', 'LineWidth', 1.5);
    h6 = plot(t_plot_all, Ax_plot_all, 'b-', 'LineWidth', 1.5);

    plot(t_node, Az_node, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Ay_node, 'gs', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);
    plot(t_node, Ax_node, 'b^', 'MarkerSize', 5, 'MarkerFaceColor', 'w', 'LineWidth', 1);

    grid on;
    title('(b) S-Curve Acceleration Profile');
    xlabel('Time (s)');
    ylabel('Acceleration (Hz/s)');
    legend([h4, h5, h6], 'J1', 'J2', 'J3', 'Location', 'best');
    set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

    %% 8. 串口准备
    if isprop(arduino, 'Status')
        if strcmpi(string(arduino.Status), "closed")
            fopen(arduino);
        end
    end

    flushSerialPort(arduino);
    pause(0.2);

    %% 9. 阶段一：先移动到起点
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

    %% 10. 阶段二：发送完整 S 曲线段参数
    fprintf('▶ 阶段二：发送 %d 个 S 曲线段参数...\n', num_segments);

    writeline(arduino, sprintf('BEGIN_SEG,%d', num_segments));
    waitForResponse(arduino, "ACK_READY", 3.0);

    for k = 1:num_segments
        s = segData(k);
        rZs = round(s.rZ * 10000);
        rYs = round(s.rY * 10000);
        rXs = round(s.rX * 10000);
        
        vs_s = round(s.vs * 100);
        ve_s = round(s.ve * 100);
        vp_s = round(s.vp * 100);
        
        aa_s = round(s.aa * 100);
        ad_s = round(s.ad * 100);
        
        %Tja_us  = round(s.Tja  * 1e6);
        %Tjd_us  = round(s.Tjd  * 1e6);
        %Ta_us   = round(s.Ta   * 1e6);
        %Tc_us   = round(s.Tc   * 1e6);
        %Td_us   = round(s.Td   * 1e6);
        timeScale = 1.25;   % 先试 1.25
        Tja_us  = round(s.Tja  * timeScale * 1e6);
        Tjd_us  = round(s.Tjd  * timeScale * 1e6);
        Ta_us   = round(s.Ta   * timeScale * 1e6);
        Tc_us   = round(s.Tc   * timeScale * 1e6);
        Td_us   = round(s.Td   * timeScale * 1e6);
        Ttot_us = round(s.Ttot * timeScale * 1e6);


        if k == 4
            fprintf('--- Segment 4 ---\n');
            fprintf('z0=%ld y0=%ld x0=%ld\n', s.z0, s.y0, s.x0);
            fprintf('z1=%ld y1=%ld x1=%ld\n', s.z1, s.y1, s.x1);
            fprintf('r=[%.4f %.4f %.4f]\n', s.rZ, s.rY, s.rX);
            fprintf('vs=%.3f ve=%.3f vp=%.3f\n', s.vs, s.ve, s.vp);
            fprintf('aa=%.3f ad=%.3f\n', s.aa, s.ad);
            fprintf('Tja=%.6f Tjd=%.6f Ta=%.6f Tc=%.6f Td=%.6f Ttot=%.6f\n', ...
                s.Tja, s.Tjd, s.Ta, s.Tc, s.Td, s.Ttot);
        end

       cmd = sprintf(['S,%ld,%ld,%ld,%ld,%ld,%ld,', ...
               '%ld,%ld,%ld,', ...
               '%ld,%ld,%ld,%ld,%ld,', ...
               '%ld,%ld,%ld,%ld,%ld,%ld'], ...
               s.z0, s.y0, s.x0, s.z1, s.y1, s.x1, ...
               rZs, rYs, rXs, ...
               vs_s, ve_s, vp_s, aa_s, ad_s, ...
               Tja_us, Tjd_us, Ta_us, Tc_us, Td_us, Ttot_us);

        writeline(arduino, cmd);

        % 建议每段都等待确认，保证绝对稳
        waitForOptionalResponse(arduino, "ACK_SEG", 0.2);
    end

    writeline(arduino, 'RUN_SEG');
    waitForResponse(arduino, "ACK_START", 3.0);

    fprintf('  ✓ S 曲线段参数下发完毕，Arduino 开始执行。\n');

    % 如果你希望 MATLAB 阻塞等待执行完成，保留下面这句
    waitForResponse(arduino, "FINISHED_ALL", 120.0);

    fprintf('  ✓ 全部轨迹执行完成。\n');
end


%% ==========================================================
% 子函数：S 曲线核心计算
% ==========================================================
function [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(p1, p2, vmax, aa, ad, vs, ve)
    S = abs(p2 - p1);
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
% 子函数：求某时刻的主导轴速度与加速度
% ==========================================================
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
% 子函数：安全除法
% ==========================================================
function out = safeDiv(a, b)
    if abs(b) < 1e-12
        out = inf;
    else
        out = a / b;
    end
end


%% ==========================================================
% 子函数：等待指定响应
% ==========================================================
function waitForResponse(arduino, targetStr, timeoutSec)
    t0 = tic;
    while toc(t0) < timeoutSec
        if arduino.NumBytesAvailable > 0
            resp = strtrim(readline(arduino));
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
% 子函数：等待可选响应，不强制
% ==========================================================
function waitForOptionalResponse(arduino, targetStr, timeoutSec)
    t0 = tic;
    while toc(t0) < timeoutSec
        if arduino.NumBytesAvailable > 0
            resp = strtrim(readline(arduino));
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
% 子函数：兼容不同串口对象的清空缓冲
% ==========================================================
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