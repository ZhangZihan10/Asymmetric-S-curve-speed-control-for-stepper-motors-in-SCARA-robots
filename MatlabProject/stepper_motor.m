% 1. 建立与 Arduino 的连接 (确保指定 Rotary 库)
%clear a;
%a = arduino('COM3', 'Mega2560', 'Libraries', 'Rotary');
% 3. 验证连接
%fprintf('已成功连接到 Arduino Mega 2560 (COM3)\n');

% 1. 建立与 Arduino 的串口连接

s = serialport('COM3', 115200);
configureTerminator(s, "LF");
pause(2); % 等待 Arduino 重置并建立连接

fprintf('已成功连接到 Arduino Mega 2560 (COM3) 串口\n');

% 3. 激活驱动器 (使能端设置为低电平 LOW)
writeline(s, "E"); 
fprintf('电机已使能 (Enabled)\n');

% 5. 设置运动速度 (RPM: 每分钟转数)
% sm3.RPM = 30;
writeline(s, "S30");
fprintf('电机速度已设置为 30 RPM\n');

% 6. 测试运动
fprintf('Z轴电机 (Stepper3) 正在向上移动 1000 步...\n');
writeline(s, "M1000"); % 发送 M3000 指令

% 【重要提醒】：MATLAB 发送指令是瞬间完成的，不会等电机转完。
% 30 RPM = 0.5圈/秒 = 100步/秒。
% 移动 3000 步大约需要 30 秒，加上加速减速时间，我们暂停 32 秒。
pause(32); 

% 你原本代码里的额外暂停 1 秒
pause(1);        

% 7. 释放电机 (切断电流，防止过热)
writeline(s, "R");
fprintf('电机已释放 (Released)\n');

% 结束通信
clear s;