% =========================================================================
% 面向 SCARA 机器人的非对称 S 曲线与步进电机动力学联合仿真 (SCI 论文画图专用)
% =========================================================================
%clear; clc; close all;

% 清除持久化变量，确保每次运行都是全新的状态
clear Asymmetric_S_Curve_CP calc_asym_logic_cp calculate_lookahead_velocity;

%% 1. 仿真参数与轨迹设置
dt = 1e-4;              % 仿真步长 (0.1ms, 相当于单片机 10kHz 中断)
T_end = 4.5;            % 仿真总时长 (秒)
N_steps = floor(T_end / dt);

t_vec = (0:N_steps-1) * dt;

% 路径点与运动学限制 (非对称设计)
target_pos = [0, 22];
v_max = 20;     % 最大速度 (mm/s)
acc_a = 50;     % 加速度 (mm/s^2)
acc_d = 100;    % 减速度 (mm/s^2) - 步进电机减速能力更强
K = 160;        % 脉冲分辨率 (脉冲/mm)

%% 2. 步进电机物理模型参数 (二阶线性化磁弹簧模型)
% 真实步进电机的转子跟随定子磁场，存在弹性环节
M = 1.0;        % 等效负载质量/惯量 (kg)
f_n = 40;       % 系统固有谐振频率 (Hz) - 步进电机常见低频共振区
K_s = M * (2 * pi * f_n)^2; % 磁场等效刚度 (N/mm)
zeta = 0.15;    % 阻尼比 (轻微欠阻尼，模拟真实的机械震荡)
C = 2 * zeta * sqrt(K_s * M); % 阻尼系数

%% 3. 数据记录数组预分配 (加速仿真)
% --- 理论参考指令 ---
pulse_rec = zeros(1, N_steps);
v_ref_rec = zeros(1, N_steps);
a_ref_rec = zeros(1, N_steps);
j_ref_rec = zeros(1, N_steps);
x_ref_ideal = zeros(1, N_steps);    % 完美的连续 S 曲线位置
x_ref_stepped = zeros(1, N_steps);  % 单片机发出的阶跃脉冲位置

% --- 电机实际物理反馈 ---
x_actual = zeros(1, N_steps);       % 转子实际位置
v_actual = zeros(1, N_steps);       % 转子实际速度
a_actual = zeros(1, N_steps);       % 转子实际加速度
Torque_rec = zeros(1, N_steps);     % 电机输出电磁力/转矩

%% 4. 主仿真循环 (ODE Euler 积分)
current_stepped_x = 0;
current_ideal_x = 0;
last_pulse = 0;

x_act = 0; v_act = 0;

fprintf('正在进行机电联合动力学仿真...\n');
for i = 1:N_steps
    t = t_vec(i);
    
    % --- A. 轨迹发生器 (充当单片机) ---
    [pulse, dir, v_ref, a_ref, s_segment, j_ref] = Asymmetric_S_Curve_CP(t, target_pos, v_max, acc_a, acc_d);
    
    % 计算理想的绝对连续位置 (积分)
    if i > 1
        current_ideal_x = current_ideal_x + v_ref * (dir*2-1) * dt; 
    end
    
    % 捕获脉冲上升沿，更新离散的阶跃目标位置
    if pulse == 1 && last_pulse == 0
        if dir == 1
            current_stepped_x = current_stepped_x + 1/K;
        else
            current_stepped_x = current_stepped_x - 1/K;
        end
    end
    last_pulse = pulse;
    
    % --- B. 步进电机动力学计算 (充当真实物理世界) ---
    % 误差 = 磁场指令位置 - 转子实际位置
    error = current_stepped_x - x_act; 
    
    % 产生电磁转矩 (弹簧力) 和 反电动势/机械阻尼力
    Motor_Force = K_s * error - C * v_act; 
    
    % 牛顿第二定律求解实际加速度
    a_act = Motor_Force / M;
    
    % 欧拉积分更新速度和位置
    v_act = v_act + a_act * dt;
    x_act = x_act + v_act * dt;
    
    % --- C. 记录数据 ---
    v_ref_rec(i) = v_ref * (dir*2-1);
    a_ref_rec(i) = a_ref * (dir*2-1);
    j_ref_rec(i) = j_ref;
    x_ref_ideal(i) = current_ideal_x;
    x_ref_stepped(i) = current_stepped_x;
    
    x_actual(i) = x_act;
    v_actual(i) = v_act;
    a_actual(i) = a_act;
    Torque_rec(i) = Motor_Force;
end
fprintf('仿真完成！正在绘制学术图表...\n');

%% 5. 绘制 SCI 论文级别的数据图表
figure('Name', 'Dynamic Analysis of Asymmetric S-Curve for SCARA Stepper Motor', 'Position', [100, 100, 1200, 800], 'Color', 'w');

% 图 1：速度曲线 (展示前瞻与飞越)
subplot(4,1,1);
plot(t_vec, v_ref_rec, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, v_actual, 'r--', 'LineWidth', 1);
grid on; 
ylabel('Velocity (mm/s)'); 
title('(a) Velocity Planning with Look-ahead for Asymmetric Continuous Path');
legend('Reference Velocity', 'Actual Velocity', 'Location', 'best');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

% 图 2：加速度曲线 (展示非对称特性与连续性)
subplot(4,1,2);
plot(t_vec, a_ref_rec, 'k-', 'LineWidth', 1.5);
grid on; 
ylabel('Acceleration (mm/s^2)'); 
title('(b) Asymmetric Acceleration Command Profile');
legend(sprintf('Acc: %d, Dec: %d', acc_a, acc_d), 'Location', 'best');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

% 图 3：电机转矩波形 (展示平滑度)
subplot(4,1,3);
plot(t_vec, Torque_rec, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.2);
grid on; 
ylabel('Motor Force (N)'); 
title('(c) Dynamic Output Force Response of Stepper Motor');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');

% 图 4：★核心指标★ 机械臂末端震动 (跟踪误差)
subplot(4,1,4);
% 震动 = 实际物理位置 - 理论绝对完美位置
vibration = (x_actual - x_ref_ideal) * 1000; % 转换为微米 (um)
plot(t_vec, vibration, 'm-', 'LineWidth', 1);
grid on; 
xlabel('Time (s)'); 
ylabel('Vibration Error (\mum)'); % \mum 会在 MATLAB 中自动渲染为带有希腊字母的 μm
title('(d) Tracking Error and Vibration Analysis of End-Effector');
set(gca, 'FontSize', 11, 'FontName', 'Times New Roman');


% =========================================================================
% 下方为你之前写好的底层函数 (保持不变，支持离散无缝衔接与飞越)
% =========================================================================
function [pulse, dir, v_out, a_out, s_out, j_out] = Asymmetric_S_Curve_CP(t, target_pos, v_max, acc_a, acc_d)
    K = 160; 
    persistent idx startTime last_toggle_time current_state current_dist
    persistent v_peak T_ja T_jd T_acc T_const T_dec T_total
    persistent v_start v_end
    
    num_pts = length(target_pos);
    
    if isempty(idx)
        idx = 2; startTime = 0; last_toggle_time = 0;
        current_state = 0; current_dist = 0;
        v_start = 0;
        v_end = calculate_lookahead_velocity(target_pos, idx, num_pts, v_max);
        [v_peak, T_ja, T_jd, T_acc, T_const, T_dec, T_total] = calc_asym_logic_cp(target_pos(1), target_pos(2), v_max, acc_a, acc_d, v_start, v_end);
    end

    dt = t - startTime;
    safe_idx = min(idx, num_pts);
    S_segment = abs(target_pos(safe_idx) - target_pos(safe_idx-1)); 
    v = 0; a = 0; j = 0;

    if idx <= num_pts
        is_segment_finished = dt >= T_total || (current_dist >= S_segment && dt > (T_total - 0.5*T_jd));
        if is_segment_finished
            if idx < num_pts
                idx = idx + 1; 
                startTime = t; 
                current_dist = 0;
                v_start = v_end; 
                v_end = calculate_lookahead_velocity(target_pos, idx, num_pts, v_max);
                [v_peak, T_ja, T_jd, T_acc, T_const, T_dec, T_total] = calc_asym_logic_cp(target_pos(idx-1), target_pos(idx), v_max, acc_a, acc_d, v_start, v_end);
                dt = 0; 
            else
                idx = num_pts + 1; 
            end
        end
    end

    if idx <= num_pts
        if dt < T_ja
            j = acc_a / T_ja; a = j * dt; v = v_start + 0.5 * j * dt^2; 
        elseif dt < 2.0 * T_ja
            a = acc_a; v = v_start + 0.5 * acc_a * T_ja + acc_a * (dt - T_ja); j = 0;
        elseif dt < T_acc
            d_dt_acc = T_acc - dt; j = -acc_a / T_ja; a = (acc_a / T_ja) * d_dt_acc; v = v_peak - 0.5 * (acc_a / T_ja) * d_dt_acc^2;
        elseif dt < (T_acc + T_const)
            v = v_peak; a = 0; j = 0;
        elseif dt < (T_acc + T_const + T_jd)
            d_dt_dec = dt - (T_acc + T_const); j = -acc_d / T_jd; a = j * d_dt_dec; v = v_peak + 0.5 * j * d_dt_dec^2;
        elseif dt < (T_acc + T_const + 2.0 * T_jd)
            a = -acc_d; d_dt_dec = dt - (T_acc + T_const + T_jd); v_start_dec = v_peak - 0.5 * acc_d * T_jd; v = v_start_dec - acc_d * d_dt_dec; j = 0;
        elseif dt <= T_total
            d_dt_end = T_total - dt; if d_dt_end < 0, d_dt_end = 0; end; j = acc_d / T_jd; a = -j * d_dt_end; v = v_end + 0.5 * j * d_dt_end^2; 
        else
            v = v_end;
        end
    end

    if v > 0.05
        T_p = 1 / (K * v);
        if (t - last_toggle_time) >= (T_p / 2)
            current_state = 1 - current_state;
            last_toggle_time = t;
            if current_state == 1, current_dist = current_dist + (1/K); end
        end
    else
        current_state = 0;
    end

    pulse = double(current_state); 
    dir = double(target_pos(safe_idx) >= target_pos(safe_idx-1));
    v_out = double(v); a_out = double(a); s_out = double(current_dist); j_out = double(j);
end

function ve = calculate_lookahead_velocity(target_pos, current_idx, num_pts, vmax)
    if current_idx >= num_pts
        ve = 0; 
    else
        dir_current = sign(target_pos(current_idx) - target_pos(current_idx-1));
        dir_next = sign(target_pos(current_idx+1) - target_pos(current_idx));
        if dir_current == dir_next
            ve = vmax * 0.6; 
        else
            ve = 0; 
        end
    end
end

function [vp, T_ja, T_jd, Ta, Tc, Td, Ttot] = calc_asym_logic_cp(p1, p2, vmax, aa, ad, vs, ve)
    S = abs(p2 - p1);
    if S < 1e-4, S = 1e-4; end
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