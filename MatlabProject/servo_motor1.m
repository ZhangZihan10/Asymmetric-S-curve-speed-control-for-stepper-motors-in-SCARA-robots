%% 高动态伺服控制系统仿真 (简化一维模型)
clear; clc;

% 1. 定义物理与仿真参数
dt = 1e-4;           % 仿真步长 (0.1ms，模拟 10kHz 的控制频率)
T_sim = 2.0;         % 总仿真时间 2 秒
N = round(T_sim/dt); % 总步数

% 电机物理参数 (PMSM 简化版)
J = 0.01;            % 转子惯量 (Inertia)
B = 0.001;           % 摩擦系数 (Friction)
Kt = 1.2;            % 转矩常数 (Torque constant)

% PI 控制器参数 (高动态要求这些参数调得很激进)
Kp = 0.5;   
Ki = 5.0;   

% 2. 初始化数据记录矩阵
time = (0:N-1)*dt;
speed_ref = zeros(1, N);  % 目标速度
speed_act = zeros(1, N);  % 实际速度
torque_cmd = zeros(1, N); % 指令扭矩 (对应电流 Iq)

% 设定目标速度曲线：第0.5秒时，速度瞬间从0突变到 3000 RPM (测试高动态)
speed_ref(time >= 0.5) = 3000; 

% 3. 仿真主循环 (模拟芯片里的 void loop)
error_sum = 0; % 积分器状态

for k = 1:N-1
    %% --- 控制器部分 (你的算法) ---
    % 计算速度误差
    error = speed_ref(k) - speed_act(k);
    
    % PI 控制计算目标扭矩
    error_sum = error_sum + error * dt; 
    torque_cmd(k) = Kp * error + Ki * error_sum;
    
    % 模拟逆变器输出限制 (电流饱和限制)
    if torque_cmd(k) > 15
        torque_cmd(k) = 15;
    elseif torque_cmd(k) < -15
        torque_cmd(k) = -15;
    end
    
    %% --- 物理被控对象部分 (物理世界的响应) ---
    % 根据牛顿第二定律计算角加速度: T_net = J * alpha
    % 净扭矩 = 电机输出扭矩 - 摩擦阻力
    T_net = torque_cmd(k) * Kt - B * speed_act(k);
    alpha = T_net / J; 
    
    % 欧拉法积分计算下一时刻的速度
    speed_act(k+1) = speed_act(k) + alpha * dt;
end

% 4. 绘制仿真结果
figure;
subplot(2,1,1);
plot(time, speed_ref, 'r--', 'LineWidth', 1.5); hold on;
plot(time, speed_act, 'b', 'LineWidth', 1.5);
title('伺服电机高动态速度响应');
xlabel('时间 (s)'); ylabel('速度 (RPM)');
legend('目标速度', '实际速度');
grid on;

subplot(2,1,2);
plot(time, torque_cmd, 'k', 'LineWidth', 1);
title('控制器输出 (目标转矩/电流指令)');
xlabel('时间 (s)'); ylabel('转矩 (Nm)');
grid on;