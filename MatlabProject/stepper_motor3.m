clear;
clc;

% 1. 建立连接
a = arduino('COM3', 'Mega2560');

% 2. 定义 Stepper2 (Y轴) 的引脚
stepPin   = 'D3';   % Y轴 步进引脚
dirPin    = 'D6';   % Y轴 方向引脚
enablePin = 'D8';   % 共用 使能引脚

% 3. 配置引脚为输出模式
configurePin(a, stepPin, 'DigitalOutput');
configurePin(a, dirPin, 'DigitalOutput');
configurePin(a, enablePin, 'DigitalOutput');

% 4. 激活驱动器并设置方向
writeDigitalPin(a, enablePin, 0);   % LOW: 激活 A4988 使能
writeDigitalPin(a, dirPin, 1);      % HIGH/LOW: 控制正反转

% 5. 运动参数设置
numSteps = 300;      
% 不需要 pulseWidth 了

fprintf('Stepper2 开始全速极速运行 %d 步...\n', numSteps);

% 6. 极速循环发送脉冲 
for k = 1:numSteps
    writeDigitalPin(a, stepPin, 1);
    % 利用 USB 通信自带的延迟，直接翻转电平
    writeDigitalPin(a, stepPin, 0);
end


fprintf('运行结束。\n');

% 7. 关闭使能，释放电机
writeDigitalPin(a, enablePin, 1);   % HIGH: 关闭 A4988 使能
clear a;