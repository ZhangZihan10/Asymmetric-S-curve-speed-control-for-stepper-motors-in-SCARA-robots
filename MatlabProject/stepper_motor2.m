clear;
clc;

a = arduino('COM3', 'Mega2560');

stepPin   = 'D4';
dirPin    = 'D7';
enablePin = 'D8';

configurePin(a, stepPin, 'DigitalOutput');
configurePin(a, dirPin, 'DigitalOutput');
configurePin(a, enablePin, 'DigitalOutput');

writeDigitalPin(a, enablePin, 0);   % 使能
writeDigitalPin(a, dirPin, 1);      % 方向

numSteps = 300;
pulseWidth = 0.002;

for k = 1:numSteps
    writeDigitalPin(a, stepPin, 1);
    %pause(pulseWidth);
    writeDigitalPin(a, stepPin, 0);
    %pause(pulseWidth);
end

writeDigitalPin(a, enablePin, 1);   % 关闭
clear a;