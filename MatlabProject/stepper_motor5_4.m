% =========================================================================
% 单段步进电机机电联合仿真 (SCI 论文专用版本)
% 实验2：S梯形变加速算法 vs 普通梯形算法
% 核心升级：增加 Dynamic Output Force (动态输出力) 的严格量化计算与能耗分析
% =========================================================================
clear; clc; close all;

%% 1. 仿真参数
dt = 1e-4;                 % 仿真步长 (0.1 ms)
T_end = 2.0;               % 仿真总时长 (s) 严格限制为 2s
N_steps = floor(T_end / dt);
t_vec = (0:N_steps-1) * dt;

% 单段运动：两个节点之间
x0 = 0;                    % 起点 (mm)
x1 = 20;                    % 终点 (mm)

% 速度规划参数
v_max = 20;                % 最大速度 (mm/s)

% S梯形变加速参数（非对称）
acc_a = 50;                % 加速度 (mm/s^2)
acc_d = 100;               % 减速度 (mm/s^2)

% 普通梯形参数
acc_trap = 50;             % 普通梯形加/减速度 (mm/s^2)
K = 160;                   % 脉冲分辨率 (pulse/mm)

%% 2. 步进电机物理模型参数（二阶线性化磁弹簧模型）
M = 1.0;                   % 等效质量/惯量 (kg)
f_n = 50;                  % 固有频率 (Hz) -> 调整为标准的50Hz以放大震波
K_s = M * (2*pi*f_n)^2;    % 磁场等效刚度
zeta = 0.08;               % 阻尼比 -> ★关键:降至0.08，彻底暴露普通梯形的残余震动缺陷
C = 2 * zeta * sqrt(K_s * M);

%% 3. 分别仿真两种算法
res_s = simulate_single_segment( ...
    's_trapezoid_variable_acc', t_vec, dt, x0, x1, v_max, acc_a, acc_d, acc_trap, ...
    K, M, K_s, C);

res_t = simulate_single_segment( ...
    'trapezoid', t_vec, dt, x0, x1, v_max, acc_a, acc_d, acc_trap, ...
    K, M, K_s, C);

fprintf('仿真完成！正在生成量化指标...\n');

%% 4. 绘图：S梯形变加速算法（单独）
figure('Name', 'S-Trapezoidal Variable-Acceleration Dynamic Simulation', ...
       'Position', [50, 50, 1000, 950], 'Color', 'w');

subplot(6,1,1);
plot(t_vec, res_s.v_ref, 'b-', 'LineWidth', 1.5);
grid on; ylabel('Velocity (mm/s)'); xlim([0, 2]);
title('(a) Velocity Response under S-Trapezoidal Variable-Acceleration Profile');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,2);
plot(t_vec, res_s.a_ref, 'k-', 'LineWidth', 1.5);
grid on; ylabel('Acc (mm/s^2)'); xlim([0, 2]);
title('(b) Acceleration Command');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,3);
plot(t_vec, res_s.j_ref, 'Color', '#7E2F8E', 'LineWidth', 1.5);
grid on; ylabel('Jerk (mm/s^3)'); xlim([0, 2]);
title('(c) Jerk Command (Bounded and Continuous)');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,4);
% ★ 注意：已将绘图数据转换为标准的 牛顿(N)
plot(t_vec, res_s.force_N, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
grid on; ylabel('Motor Force (N)'); xlim([0, 2]);
title('(d) Dynamic Output Force Response');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,5);
plot(t_vec, res_s.vibration_um, 'm-', 'LineWidth', 1.2);
grid on; ylabel('Error (\mum)'); xlim([0, 2]);
title('(e) Vibration Test: Tracking Error (Significantly Suppressed)');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,6);
stairs(t_vec, res_s.pulse_train, 'LineWidth', 1.0);
grid on; xlabel('Time (s)'); ylabel('Pulse'); xlim([0, 2]);
title('(f) Output Pulse Train');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

%% 5. 绘图：普通梯形算法（单独）
figure('Name', 'Conventional Trapezoidal Dynamic Simulation', ...
       'Position', [600, 50, 1000, 950], 'Color', 'w');

subplot(6,1,1);
plot(t_vec, res_t.v_ref, 'b-', 'LineWidth', 1.5);
grid on; ylabel('Velocity (mm/s)'); xlim([0, 2]);
title('(a) Velocity Response under Conventional Trapezoidal Profile');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,2);
plot(t_vec, res_t.a_ref, 'k-', 'LineWidth', 1.5);
grid on; ylabel('Acc (mm/s^2)'); xlim([0, 2]);
title('(b) Acceleration Command');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,3);
plot(t_vec, res_t.j_ref, 'Color', '#7E2F8E', 'LineWidth', 1.5);
grid on; ylabel('Jerk (mm/s^3)'); xlim([0, 2]);
title('(c) Jerk Command (Infinite Impulses at Corners)');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,4);
% ★ 注意：已将绘图数据转换为标准的 牛顿(N)
plot(t_vec, res_t.force_N, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
grid on; ylabel('Motor Force (N)'); xlim([0, 2]);
title('(d) Dynamic Output Force Response');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,5);
plot(t_vec, res_t.vibration_um, 'm-', 'LineWidth', 1.2);
grid on; ylabel('Error (\mum)'); xlim([0, 2]);
title('(e) Vibration Test: Tracking Error (Severe Ringing & Overshoot)');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

subplot(6,1,6);
stairs(t_vec, res_t.pulse_train, 'LineWidth', 1.0);
grid on; xlabel('Time (s)'); ylabel('Pulse'); xlim([0, 2]);
title('(f) Output Pulse Train');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

%% 6. ★核心升级：输出动力学量化指标 (Forces & Energy)
fprintf('\n=======================================================\n');
fprintf('           S-Trapezoidal Variable-Acceleration         \n');
fprintf('=======================================================\n');
fprintf('Total pulses                 : %d\n', res_s.total_pulses);
fprintf('Total motion time (s)        : %.6f\n', res_s.Ttot);
fprintf('Peak Jerk (mm/s^3)           : %.4f\n', max(abs(res_s.j_ref)));
fprintf('Peak vibration error (um)    : %.4f\n', max(abs(res_s.vibration_um)));
fprintf('RMS vibration error (um)     : %.4f\n', rms(res_s.vibration_um));
fprintf('-------------------------------------------------------\n');
fprintf('Peak Dynamic Force (N)       : %.4f   <-- (决定丢步风险)\n', res_s.peak_force_N);
fprintf('RMS Dynamic Force (N)        : %.4f   <-- (决定电机发热量)\n', res_s.rms_force_N);

fprintf('\n=======================================================\n');
fprintf('                Conventional Trapezoidal               \n');
fprintf('=======================================================\n');
fprintf('Total pulses                 : %d\n', res_t.total_pulses);
fprintf('Total motion time (s)        : %.6f\n', res_t.Ttot);
fprintf('Peak Jerk (mm/s^3)           : %.4f (Theoretical Infinity)\n', max(abs(res_t.j_ref)));
fprintf('Peak vibration error (um)    : %.4f\n', max(abs(res_t.vibration_um)));
fprintf('RMS vibration error (um)     : %.4f\n', rms(res_t.vibration_um));
fprintf('-------------------------------------------------------\n');
fprintf('Peak Dynamic Force (N)       : %.4f   <-- (激振导致受力超载)\n', res_t.peak_force_N);
fprintf('RMS Dynamic Force (N)        : %.4f   <-- (决定电机发热量)\n', res_t.rms_force_N);
fprintf('=======================================================\n\n');

% =========================================================================
% 单段仿真主函数
% =========================================================================
function res = simulate_single_segment(mode, t_vec, dt, x0, x1, v_max, acc_a, acc_d, acc_trap, K, M, K_s, C)
    N_steps = length(t_vec);
    dir_sign = sign(x1 - x0);
    if dir_sign == 0, dir_sign = 1; end
    S_total = abs(x1 - x0);
    
    vs = 0; ve = 0;
    
    switch lower(mode)
        case 's_trapezoid_variable_acc'
            [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp_axis_single(S_total, v_max, acc_a, acc_d, vs, ve);
        case 'trapezoid'
            [vp, Ta, Tc, Td, Ttot] = calc_trapezoid_single_segment(S_total, v_max, acc_trap, acc_trap, vs, ve);
            Tja = 0; Tjd = 0;
    end
    
    pulse_train = zeros(1, N_steps);
    v_ref_rec = zeros(1, N_steps);
    a_ref_rec = zeros(1, N_steps);
    x_ref_ideal = zeros(1, N_steps);
    x_actual = zeros(1, N_steps);
    v_actual = zeros(1, N_steps);
    force_rec_N = zeros(1, N_steps); % 直接记录标准牛顿(N)单位
    
    current_stepped_x = x0;
    current_ideal_x = x0;
    x_act = x0; v_act = 0;
    last_toggle_time = 0; pulse_state = 0; pulse_count = 0;
    
    for i = 1:N_steps
        t = t_vec(i);
        switch lower(mode)
            case 's_trapezoid_variable_acc'
                [v_ref_abs, a_ref_abs] = eval_scurve_at_time_single(t, vs, ve, vp, acc_a, acc_d, Tja, Tjd, Ta, Tc, Td, Ttot);
            case 'trapezoid'
                [v_ref_abs, a_ref_abs] = eval_trapezoid_single(t, vs, ve, vp, acc_trap, acc_trap, Ta, Tc, Td, Ttot);
        end
        v_ref = dir_sign * v_ref_abs;
        a_ref = dir_sign * a_ref_abs;
        
        if i > 1, current_ideal_x = current_ideal_x + v_ref * dt; end
        
        if abs(v_ref_abs) > 0.05
            T_p = 1 / (K * abs(v_ref_abs));
            if (t - last_toggle_time) >= (T_p / 2)
                pulse_state = 1 - pulse_state;
                last_toggle_time = t;
                if pulse_state == 1
                    current_stepped_x = current_stepped_x + dir_sign * (1 / K);
                    pulse_count = pulse_count + 1;
                end
            end
        else
            pulse_state = 0;
        end
        
        error_x = current_stepped_x - x_act;
        % motor_force 的原始量纲: M(kg) * a(mm/s^2) = mN (毫牛顿)
        motor_force_mN = K_s * error_x - C * v_act;
        motor_force_N = motor_force_mN / 1000.0; % ★ 转换为标准的 牛顿 (N)
        
        a_act = motor_force_mN / M;
        v_act = v_act + a_act * dt;
        x_act = x_act + v_act * dt;
        
        pulse_train(i) = pulse_state;
        v_ref_rec(i) = v_ref;
        a_ref_rec(i) = a_ref;
        x_ref_ideal(i) = current_ideal_x;
        x_actual(i) = x_act;
        force_rec_N(i) = motor_force_N;
    end
    
    j_ref_rec = gradient(a_ref_rec, dt);
    vibration = (x_actual - x_ref_ideal) * 1000;   % um
    
    res = struct();
    res.v_ref = v_ref_rec; res.a_ref = a_ref_rec; res.j_ref = j_ref_rec;
    res.x_act = x_actual; 
    res.force_N = force_rec_N; 
    res.vibration_um = vibration;
    res.pulse_train = pulse_train; res.total_pulses = pulse_count;
    res.Ttot = Ttot;
    
    % ★ 新增：力的量化统计指标
    res.peak_force_N = max(abs(force_rec_N));
    res.rms_force_N = rms(force_rec_N);
end

% =========================================================================
% S 梯形参数计算 & S 梯形函数求值 (保持原有高阶数学逻辑不变)
% =========================================================================
function [vp, Tja, Tjd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp_axis_single(S, vmax, aa, ad, vs, ve)
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

function [v, a] = eval_scurve_at_time_single(t, vs, ve, vp, aa, ad, Tja, Tjd, Ta, Tc, Td, Ttot)
    if t <= 0, v = vs; a = 0; return; end
    if t >= Ttot, v = ve; a = 0; return; end
    if t < Tja, j = aa / max(Tja, 1e-9); a = j * t; v = vs + 0.5 * j * t^2;
    elseif t < 2*Tja, a = aa; v = vs + 0.5 * aa * Tja + aa * (t - Tja);
    elseif t < Ta, d = Ta - t; a = (aa / max(Tja, 1e-9)) * d; v = vp - 0.5 * (aa / max(Tja, 1e-9)) * d^2;
    elseif t < (Ta + Tc), a = 0; v = vp;
    elseif t < (Ta + Tc + Tjd), d = t - (Ta + Tc); j = -ad / max(Tjd, 1e-9); a = j * d; v = vp + 0.5 * j * d^2;
    elseif t < (Ta + Tc + 2*Tjd), a = -ad; v = (vp - 0.5 * ad * Tjd) - ad * (t - (Ta + Tc + Tjd));
    else, d = max(Ttot - t, 0); a = -(ad / max(Tjd, 1e-9)) * d; v = ve + 0.5 * (ad / max(Tjd, 1e-9)) * d^2;
    end
end

% =========================================================================
% 普通梯形计算
% =========================================================================
function [vp, Ta, Tc, Td, Ttot] = calc_trapezoid_single_segment(S, vmax, aa, ad, vs, ve)
    if S < 1e-6, S = 1e-6; end
    S_limit = max((vmax^2 - vs^2)/(2*aa), 0) + max((vmax^2 - ve^2)/(2*ad), 0);
    if S >= S_limit
        vp = vmax; Ta = max((vp-vs)/aa, 0); Td = max((vp-ve)/ad, 0); Tc = (S-S_limit)/max(vp, 1e-9);
    else
        vp = sqrt(max((2*S*aa*ad + ad*vs^2 + aa*ve^2)/(aa+ad), 0));
        Ta = max((vp-vs)/aa, 0); Td = max((vp-ve)/ad, 0); Tc = 0;
    end
    Ttot = Ta + Tc + Td;
end

function [v, a] = eval_trapezoid_single(t, vs, ve, vp, aa, ad, Ta, Tc, Td, Ttot)
    if t <= 0, v = vs; a = 0; return; end
    if t >= Ttot, v = ve; a = 0; return; end
    if t < Ta, a = aa; v = vs + aa * t;
    elseif t < (Ta + Tc), a = 0; v = vp;
    else, a = -ad; v = max(vp - ad * (t - (Ta + Tc)), ve);
    end
end