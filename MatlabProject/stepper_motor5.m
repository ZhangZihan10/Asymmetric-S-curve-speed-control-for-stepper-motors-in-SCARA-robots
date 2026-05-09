% =========================================================================
% 单段步进电机机电联合仿真
% 实验2：S梯形变加速算法 与 普通梯形算法
% 特点：
% 1) 仅针对两个节点之间的单段运动
% 2) 不包含路径前瞻
% 3) 两种算法分别单独出图
% 4) 输出两种算法的脉冲图
% =========================================================================
clear; clc; close all;

%% 1. 仿真参数
dt = 1e-4;                 % 仿真步长 (0.1 ms)
T_end = 4.5;               % 仿真总时长 (s)
N_steps = floor(T_end / dt);
t_vec = (0:N_steps-1) * dt;

% 单段运动：两个节点之间
x0 = 0;                    % 起点 (mm)
x1 = 22;                   % 终点 (mm)

% 速度规划参数
v_max = 20;                % 最大速度 (mm/s)

% S梯形变加速参数（非对称）
acc_a = 50;                % 加速度 (mm/s^2)
acc_d = 100;               % 减速度 (mm/s^2)

% 普通梯形参数（对称）
acc_trap = 50;             % 普通梯形加/减速度 (mm/s^2)

K = 160;                   % 脉冲分辨率 (pulse/mm)

%% 2. 步进电机物理模型参数（二阶线性化磁弹簧模型）
M = 1.0;                   % 等效质量/惯量
f_n = 40;                  % 固有频率 (Hz)
K_s = M * (2*pi*f_n)^2;    % 磁场等效刚度
zeta = 0.15;               % 阻尼比
C = 2 * zeta * sqrt(K_s * M);

%% 3. 分别仿真两种算法
res_s = simulate_single_segment( ...
    's_trapezoid_variable_acc', t_vec, dt, x0, x1, v_max, acc_a, acc_d, acc_trap, ...
    K, M, K_s, C);

res_t = simulate_single_segment( ...
    'trapezoid', t_vec, dt, x0, x1, v_max, acc_a, acc_d, acc_trap, ...
    K, M, K_s, C);

fprintf('仿真完成！\n');

%% 4. 绘图：S梯形变加速算法（单独）
figure('Name', 'S-Trapezoidal Variable-Acceleration Dynamic Simulation', ...
       'Position', [100, 80, 1200, 950], 'Color', 'w');

subplot(5,1,1);
plot(t_vec, res_s.v_ref, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, res_s.v_act, 'r--', 'LineWidth', 1.0);
grid on;
ylabel('Velocity (mm/s)');
title('(a) Velocity Response under S-Trapezoidal Variable-Acceleration Profile');
legend('Reference Velocity', 'Actual Velocity', 'Location', 'best');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,2);
plot(t_vec, res_s.a_ref, 'k-', 'LineWidth', 1.5);
grid on;
ylabel('Acceleration (mm/s^2)');
title('(b) Acceleration Command of S-Trapezoidal Variable-Acceleration Profile');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,3);
plot(t_vec, res_s.force, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
grid on;
ylabel('Motor Force (N)');
title('(c) Dynamic Output Force Response of Stepper Motor');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,4);
plot(t_vec, res_s.vibration_um, 'm-', 'LineWidth', 1.0);
grid on;
ylabel('Error (\mum)');
title('(d) Tracking Error and Vibration Analysis');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,5);
stairs(t_vec, res_s.pulse_train, 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Pulse');
title('(e) Output Pulse Train of S-Trapezoidal Variable-Acceleration Profile');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

%% 5. 绘图：普通梯形算法（单独）
figure('Name', 'Conventional Trapezoidal Dynamic Simulation', ...
       'Position', [150, 100, 1200, 950], 'Color', 'w');

subplot(5,1,1);
plot(t_vec, res_t.v_ref, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, res_t.v_act, 'r--', 'LineWidth', 1.0);
grid on;
ylabel('Velocity (mm/s)');
title('(a) Velocity Response under Conventional Trapezoidal Profile');
legend('Reference Velocity', 'Actual Velocity', 'Location', 'best');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,2);
plot(t_vec, res_t.a_ref, 'k-', 'LineWidth', 1.5);
grid on;
ylabel('Acceleration (mm/s^2)');
title('(b) Acceleration Command of Conventional Trapezoidal Profile');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,3);
plot(t_vec, res_t.force, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
grid on;
ylabel('Motor Force (N)');
title('(c) Dynamic Output Force Response of Stepper Motor');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,4);
plot(t_vec, res_t.vibration_um, 'm-', 'LineWidth', 1.0);
grid on;
ylabel('Error (\mum)');
title('(d) Tracking Error and Vibration Analysis');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(5,1,5);
stairs(t_vec, res_t.pulse_train, 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Pulse');
title('(e) Output Pulse Train of Conventional Trapezoidal Profile');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

%% 6. 输出量化指标
fprintf('\n===== S-Trapezoidal Variable-Acceleration =====\n');
fprintf('Total pulses                 : %d\n', res_s.total_pulses);
fprintf('Peak vibration error (um)    : %.4f\n', max(abs(res_s.vibration_um)));
fprintf('RMS vibration error (um)     : %.4f\n', rms(res_s.vibration_um));
fprintf('Total motion time (s)        : %.6f\n', res_s.Ttot);

fprintf('\n===== Conventional Trapezoidal =====\n');
fprintf('Total pulses                 : %d\n', res_t.total_pulses);
fprintf('Peak vibration error (um)    : %.4f\n', max(abs(res_t.vibration_um)));
fprintf('RMS vibration error (um)     : %.4f\n', rms(res_t.vibration_um));
fprintf('Total motion time (s)        : %.6f\n', res_t.Ttot);

% =========================================================================
% 单段仿真主函数
% =========================================================================
function res = simulate_single_segment(mode, t_vec, dt, x0, x1, v_max, acc_a, acc_d, acc_trap, K, M, K_s, C)

    N_steps = length(t_vec);
    dir_sign = sign(x1 - x0);
    if dir_sign == 0
        dir_sign = 1;
    end
    S_total = abs(x1 - x0);

    % 单段边界条件：无前瞻
    vs = 0;
    ve = 0;

    % ---------- 先生成该算法的单段参数 ----------
    switch lower(mode)
        case 's_trapezoid_variable_acc'
            [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = ...
                calc_s_trapezoid_variable_acc(S_total, v_max, acc_a, acc_d, vs, ve);

        case 'trapezoid'
            [vp, Ta, Tc, Td, Ttot] = ...
                calc_trapezoid_single_segment(S_total, v_max, acc_trap, acc_trap, vs, ve);

            Tja = 0;
            Tjd = 0;

        otherwise
            error('Unknown mode: %s', mode);
    end

    % ---------- 数据记录 ----------
    pulse_train   = zeros(1, N_steps);
    v_ref_rec     = zeros(1, N_steps);
    a_ref_rec     = zeros(1, N_steps);
    x_ref_ideal   = zeros(1, N_steps);
    x_ref_stepped = zeros(1, N_steps);

    x_actual = zeros(1, N_steps);
    v_actual = zeros(1, N_steps);
    a_actual = zeros(1, N_steps);
    force_rec = zeros(1, N_steps);

    % ---------- 脉冲与动力学状态 ----------
    current_stepped_x = x0;
    current_ideal_x = x0;
    x_act = x0;
    v_act = 0;

    last_toggle_time = 0;
    pulse_state = 0;
    pulse_count = 0;

    % ---------- 主循环 ----------
    for i = 1:N_steps
        t = t_vec(i);

        % --- A. 参考轨迹发生器（单段，无前瞻） ---
        switch lower(mode)
            case 's_trapezoid_variable_acc'
                [v_ref_abs, a_ref_abs] = eval_s_trapezoid_variable_acc( ...
                    t, vs, ve, vp, acc_a, acc_d, Tja, Tjd, Ta, Tc, Td, Ttot);

            case 'trapezoid'
                [v_ref_abs, a_ref_abs] = eval_trapezoid_single( ...
                    t, vs, ve, vp, acc_trap, acc_trap, Ta, Tc, Td, Ttot);
        end

        v_ref = dir_sign * v_ref_abs;
        a_ref = dir_sign * a_ref_abs;

        % 理想连续位置
        if i > 1
            current_ideal_x = current_ideal_x + v_ref * dt;
        end

        % --- B. 脉冲发生器 ---
        if abs(v_ref_abs) > 0.05
            T_p = 1 / (K * abs(v_ref_abs));
            if (t - last_toggle_time) >= (T_p / 2)
                pulse_state = 1 - pulse_state;
                last_toggle_time = t;

                % 上升沿计一步
                if pulse_state == 1
                    current_stepped_x = current_stepped_x + dir_sign * (1 / K);
                    pulse_count = pulse_count + 1;
                end
            end
        else
            pulse_state = 0;
        end

        % --- C. 步进电机动力学 ---
        error_x = current_stepped_x - x_act;
        motor_force = K_s * error_x - C * v_act;
        a_act = motor_force / M;
        v_act = v_act + a_act * dt;
        x_act = x_act + v_act * dt;

        % --- D. 记录 ---
        pulse_train(i)   = pulse_state;
        v_ref_rec(i)     = v_ref;
        a_ref_rec(i)     = a_ref;
        x_ref_ideal(i)   = current_ideal_x;
        x_ref_stepped(i) = current_stepped_x;

        x_actual(i) = x_act;
        v_actual(i) = v_act;
        a_actual(i) = a_act;
        force_rec(i) = motor_force;
    end

    vibration = (x_actual - x_ref_ideal) * 1000;   % um

    % ---------- 输出 ----------
    res = struct();
    res.v_ref = v_ref_rec;
    res.a_ref = a_ref_rec;
    res.x_ref_ideal = x_ref_ideal;
    res.x_ref_stepped = x_ref_stepped;

    res.x_act = x_actual;
    res.v_act = v_actual;
    res.a_act = a_actual;
    res.force = force_rec;

    res.vibration_um = vibration;
    res.pulse_train = pulse_train;
    res.total_pulses = pulse_count;

    res.vp = vp;
    res.Tja = Tja;
    res.Tjd = Tjd;
    res.Ta = Ta;
    res.Tc = Tc;
    res.Td = Td;
    res.Ttot = Ttot;
end

% =========================================================================
% S梯形变加速：单段参数计算
% 七段式：加加速-恒加速-减加速-匀速-加减速-恒减速-减减速
% 支持 acc_a ~= acc_d
% =========================================================================
function [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = calc_s_trapezoid_variable_acc(S, vmax, aa, ad, vs, ve)
    if S < 1e-6
        S = 1e-6;
    end

    S_acc = 0.75 * (vmax^2 - vs^2) / aa;
    S_dec = 0.75 * (vmax^2 - ve^2) / ad;
    if S_acc < 0, S_acc = 0; end
    if S_dec < 0, S_dec = 0; end

    S_limit = S_acc + S_dec;

    if S >= S_limit
        vp = vmax;
        Tc = (S - S_limit) / max(vp, 1e-9);
    else
        vp_raw = ((4/3) * S * aa * ad + vs^2 * ad + ve^2 * aa) / (aa + ad);
        if vp_raw < 0, vp_raw = 0; end
        vp = sqrt(vp_raw);
        Tc = 0;
    end

    Tja = max((vp - vs) / (2 * aa), 0);
    Tjd = max((vp - ve) / (2 * ad), 0);

    Ta = 3 * Tja;
    Td = 3 * Tjd;
    Ttot = Ta + Tc + Td;
end

% =========================================================================
% 普通梯形：单段参数计算
% =========================================================================
function [vp, Ta, Tc, Td, Ttot] = calc_trapezoid_single_segment(S, vmax, aa, ad, vs, ve)
    if S < 1e-6
        S = 1e-6;
    end

    S_acc = max((vmax^2 - vs^2) / (2 * aa), 0);
    S_dec = max((vmax^2 - ve^2) / (2 * ad), 0);
    S_limit = S_acc + S_dec;

    if S >= S_limit
        vp = vmax;
        Ta = max((vp - vs) / aa, 0);
        Td = max((vp - ve) / ad, 0);
        Tc = (S - S_limit) / max(vp, 1e-9);
    else
        vp_raw = (2 * S * aa * ad + ad * vs^2 + aa * ve^2) / (aa + ad);
        if vp_raw < 0, vp_raw = 0; end
        vp = sqrt(vp_raw);
        Ta = max((vp - vs) / aa, 0);
        Td = max((vp - ve) / ad, 0);
        Tc = 0;
    end

    Ttot = Ta + Tc + Td;
end

% =========================================================================
% S梯形变加速：单段速度/加速度求值
% =========================================================================
function [v, a] = eval_s_trapezoid_variable_acc(t, vs, ve, vp, aa, ad, Tja, Tjd, Ta, Tc, Td, Ttot)
    if t <= 0
        v = vs; a = 0; return;
    end
    if t >= Ttot
        v = ve; a = 0; return;
    end

    if t < Tja
        j = aa / max(Tja, 1e-9);
        a = j * t;
        v = vs + 0.5 * j * t^2;

    elseif t < 2*Tja
        dt = t - Tja;
        a = aa;
        v = vs + 0.5 * aa * Tja + aa * dt;

    elseif t < Ta
        d = Ta - t;
        j = -aa / max(Tja, 1e-9);
        a = (aa / max(Tja, 1e-9)) * d;
        v = vp - 0.5 * abs(j) * d^2;

    elseif t < (Ta + Tc)
        a = 0;
        v = vp;

    elseif t < (Ta + Tc + Tjd)
        dt = t - (Ta + Tc);
        j = -ad / max(Tjd, 1e-9);
        a = j * dt;
        v = vp + 0.5 * j * dt^2;

    elseif t < (Ta + Tc + 2*Tjd)
        dt = t - (Ta + Tc + Tjd);
        a = -ad;
        v = (vp - 0.5 * ad * Tjd) - ad * dt;

    else
        d = Ttot - t;
        if d < 0, d = 0; end
        j = ad / max(Tjd, 1e-9);
        a = -j * d;
        v = ve + 0.5 * j * d^2;
    end
end

% =========================================================================
% 普通梯形：单段速度/加速度求值
% =========================================================================
function [v, a] = eval_trapezoid_single(t, vs, ve, vp, aa, ad, Ta, Tc, Td, Ttot)
    if t <= 0
        v = vs; a = 0; return;
    end
    if t >= Ttot
        v = ve; a = 0; return;
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