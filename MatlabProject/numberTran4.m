function numberTran4(arduino, target_z, target_y, target_x, target_t)
    % 输入的 target_z, target_y, target_x, target_t 应当是长度相同的数组（例如长度为 40）
    num_points = length(target_z);
    
    % 1. 发送包头：告知单片机即将发送的点数
    header = sprintf('BEGIN,%d', num_points);
    writeline(arduino, header);
    pause(0.1); % 给单片机一点点时间初始化数组
    
    % 2. 循环计算并发送每一个点
    for i = 1:num_points
        % 取出当前点
        nz = target_z(i);
        ny = target_y(i);
        nx = target_x(i);
        nt = target_t(i);
        
        % 执行你原有的机械臂几何映射计算
        nz = nz * 205560 - 11667;
        nz = round(nz, 2);
        
        y1 = ny; % 备份 ny 用于 x 的计算(如果需要)
        ny = ny * 945 - 1900;
        
        nx = 0.8 * ny + 3419.6 + nx * 1455; % L3 偏角补偿
        nx = round(nx, 2);
        
        ny = round(ny, 2);
        
        nt = nt * 180 / pi;
        nt = round(nt, 3);
        
        % 格式化字符串 (保留你的去零逻辑，虽然 Arduino 解析时不需要，但可减小传输字节数)
        str_z = regexprep(sprintf('%.2f', nz), '\.0*$', '');
        str_y = regexprep(sprintf('%.2f', ny), '\.0*$', '');
        str_x = regexprep(sprintf('%.2f', nx), '\.0*$', '');
        str_t = regexprep(sprintf('%.3f', nt), '\.0*$', '');
        
        % 拼装单行数据包
        combined = sprintf('q,%s,%s,%s,%s', str_z, str_y, str_x, str_t);
        
        % 连续写入（不加长 pause，让数据迅速灌入单片机串口缓存）
        writeline(arduino, combined);
        pause(0.01); % 极短的延时，防止击穿 Arduino 的 64 字节串口硬缓存
    end
    
    % 3. 发送执行指令
    writeline(arduino, 'RUN');
    disp(['已成功下发 ', num2str(num_points), ' 个轨迹点，等待机械臂执行...']);
end