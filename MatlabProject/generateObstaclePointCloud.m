%% GENERATEOBSTACLEPOINTCLOUD 机器人系统避障3D重建
%% 纯色面板只进行3D重建(真实环境)加入相机旋转，直接输出密集点云。
function ptCloud_Recon = generateObstaclePointCloud()
    % GENERATEOBSTACLEPOINTCLOUD 机器人系统避障3D重建
    % 纯色面板只进行3D重建(真实环境)加入相机旋转，直接输出密集点云。
    
    %% 1. 初始化与参数加载
    load('fisheyeCameraParamsReal2old.mat');
    load('Omni_Calib_Results_Real2old1.mat');
    ocam_model = calib_data.ocam_model;
    intrinsics = cameraParams.Intrinsics;
    
    % 激光及相机参数
    camX = 0.312; camY = 3.378; camZ = 0.193;
    camX2 = -2.312; camY2 = 1.378; camZ2 = 0.193;
    lasX = 0; lasY = 0;
    las_dist = 320.99;
    Cube_l = 450;  % 底边实际长度（单位 mm）
    fov = 159;     % 160
    f_max = 611;   % 等效焦距610
    
    % 文件夹路径设置
    % 获取图像列表
    outputFolder = 'RobotBoard';
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
    end
    
    folderPath = outputFolder;
    imageFiles = dir(fullfile(folderPath, '*.jpg'));
    
    outputFolder2 = 'RobotBoardL';%激光识别文件
    folderPath2 = outputFolder2;
    imageFiles2 = dir(fullfile(folderPath2, '*.jpg'));
    
    %% 2. 准备统一显示图窗与点云数据容器
    figure(99); clf;  
    ax = axes;        
    hold(ax, 'on');   
    xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
    axis(ax, 'equal');
    axis(ax, [0 25 0 25 0 25]);
    grid(ax, 'on');
    view(ax, 3);
    title(ax, '所有障碍板贴图结果');
    
    % --- 初始化 3D 密集云图数据容器 ---
    X_final = []; 
    Y_final = []; 
    Z_final = [];
    grid_res = 0.01; % 采样间距 (对应原代码中的缩放比例)

    %% 3. 遍历图像进行建模与点云生成
    for i = 1:length(imageFiles)
        % 读图
        filename = fullfile(outputFolder, imageFiles(i).name);
        testImage = imread(filename);
        filename2 = fullfile(outputFolder2, imageFiles(i).name);
        testImage2 = imread(filename2);
        
        % 激光识别 + 坐标映射
        img = LaserFind(testImage2);
        [x1, y1] = mappingr(img, camY, camX, camZ, lasY, lasX, las_dist, ocam_model);
        x1 = x1(2:end); y1 = y1(2:end);
        [xt1, yt1] = MappingRobotR([x1', y1'], Cube_l);
        
        % PCA拟合获取底边方向
        pts = [xt1(:), yt1(:)]; 
        meanPt = mean(pts, 1);    
        pts_centered = pts - meanPt;
        [~, ~, V] = svd(pts_centered, 0);   
        dir_vector = V(:,1);     
        
        % 方向向量与 y 轴夹角
        v_y = [0, 1] / norm([0, 1]);
        % 【修改这里】：将原本的 dir 改为 dir_vec，避免与系统函数 dir() 冲突
        dir_vec = dir_vector(:)' / norm(dir_vector);
        cos_theta = max(min(dot(v_y, dir_vec), 1), -1);
        theta_deg = rad2deg(acos(cos_theta));

        if dir_vec(1) < 0
            theta_deg = 180 - theta_deg;
        end
        fprintf('线段与 Y 轴的夹角（右侧范围）：%.2f°\n', theta_deg);
        
        % 计算线段长度
        L = 0; p1 = [0, 0]; p2 = [0, 0];
        points = [xt1(:), yt1(:)];
        valid_idx = isfinite(points(:,1)) & isfinite(points(:,2));
        points = points(valid_idx, :);
        N = size(points, 1);
        for i1 = 1:N
            for j = i1+1:N
                d = norm(points(i1,:) - points(j,:));
                if d > L
                    L = d; p1 = points(i1,:); p2 = points(j,:);
                end
            end
        end
        fprintf('线段长度: %.2f\n', L);
        
        % 旋转矩阵设置
        yaw=camY2; pitch=camZ2; roll=camX2;
        R = eul2rotm(deg2rad([yaw, pitch, roll]), 'ZYX');
        
        % 根据图像序号设置姿态角 (保留你的 switch 逻辑)
        switch i
            case 2
                roll = 0; pitch = 90; yaw = 90;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw2 = 0;
                elseif theta_deg>=90, yaw2 = 180-theta_deg;
                else, yaw2 = -(theta_deg);
                end
                R2 = eul2rotm(deg2rad([yaw, pitch,  -1]), 'ZYZ');
            case 1
                roll = 0; pitch = 0;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw = -90;
                elseif theta_deg>=90, yaw = 90-(theta_deg);
                else, yaw = -theta_deg;
                end
                R2 = eul2rotm(deg2rad([roll,pitch,yaw]), 'ZYX');
            case 4
                roll = 90; pitch = 0;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1 || abs(theta_deg - 90) < 1
                    yaw = 0;
                elseif theta_deg<=90, yaw = -theta_deg;
                else, yaw = theta_deg-90;
                end
                R2 = eul2rotm(deg2rad([roll,pitch,yaw]), 'XYZ');
            case 3
                roll = 0; pitch = 90; yaw = -90;
                if abs(theta_deg - 0) < 1 || abs(theta_deg - 180) < 1|| abs(theta_deg - 90) < 1
                    yaw2 = -90;
                elseif theta_deg<=90, yaw2 = -theta_deg;
                else, yaw2 = -theta_deg+90;
                end
                R2 = eul2rotm(deg2rad([yaw2, pitch,  roll]), 'ZYX');
            otherwise
                roll = 0; pitch = -90; yaw = -90;
                R2 = eye(3); % 补充默认 R2，防止报错
        end
        
        % 透视变换
        Wout = 1920; Hout = 1080; aspect = Wout / Hout;
        halfFovY = deg2rad(fov)/2.2;  
        grid_scale_y = tan(halfFovY);
        grid_scale_x = aspect * grid_scale_y;
        
        [u, v] = meshgrid(linspace(-grid_scale_x, grid_scale_x, Wout), ...
                          linspace(-grid_scale_y, grid_scale_y, Hout));
        P = [u(:), ones(numel(u),1), v(:)]';
        P_rotated = R* R2 * P;
        theta = atan2(P_rotated(3,:), P_rotated(1,:));
        phi = atan2(vecnorm(P_rotated([1,3],:), 2, 1), P_rotated(2,:));
        
        maxFOV = 2 * f_max;         
        I = (2 * phi .* cos(theta)) / deg2rad(fov);
        J = (2 * phi .* sin(theta)) / deg2rad(fov);
        S_inv = inv(intrinsics.StretchMatrix);
        I_corrected = S_inv(1,1)*I + S_inv(1,2)*J;
        J_corrected = S_inv(2,1)*I + S_inv(2,2)*J;
        x = I_corrected * f_max + intrinsics.DistortionCenter(1);
        y = J_corrected * f_max + intrinsics.DistortionCenter(2);
        
        perspectiveImg = zeros(size(testImage), 'like', testImage);
        for c = 1:3
            perspectiveImg(:,:,c) = interp2(double(testImage(:,:,c)),...
                reshape(x,Hout, Wout), reshape(y,Hout, Wout), 'linear', 0);
        end
        
        mask = ~(perspectiveImg(:,:,1)<45 & perspectiveImg(:,:,2)<45 & perspectiveImg(:,:,3)<45);
        [row_idx, col_idx] = find(mask);
        if isempty(row_idx), continue; end
        
        h_pixel = max(row_idx) - min(row_idx);
        w_pixel = max(col_idx) - min(col_idx);
        H_real = L * (h_pixel / w_pixel)+1.4;
        fprintf('高度: %.2f\n', H_real);
        
        row_min = min(row_idx); row_max = max(row_idx);
        col_min = min(col_idx); col_max = max(col_idx);
        img_crop = perspectiveImg(row_min:row_max, col_min:col_max, :);
        alpha_crop = double(mask(row_min:row_max, col_min:col_max)); 
        
        switch i
            case 3
                if abs(theta_deg - 0) > 1 || abs(theta_deg - 180) > 1
                    img_crop = imrotate(img_crop, 180); img_crop = fliplr(img_crop);
                    alpha_crop = imrotate(alpha_crop, 180); alpha_crop = fliplr(alpha_crop);
                end
            case 4
                if abs(theta_deg - 0) > 1 || abs(theta_deg - 180) > 1 
                    img_crop = fliplr(img_crop);
                    alpha_crop = fliplr(alpha_crop);
                end
        end
        
        % ---------------- 构建3D空间点 ----------------
        x1 = p2(1); y1 = p2(2);  % 底边左端点
        x2 = p1(1); y2 = p1(2);  % 底边右端点
        v = [x2 - x1, y2 - y1]; v = v / norm(v);
        P0 = [x1, y1, -4];       % 左下角
        P1 = [x2, y2, -4];       % 右下角
        P2 = P1 + [0, 0, H_real];% 右上角
        P3 = P0 + [0, 0, H_real];% 左上角
        
        X = [P0(1), P1(1); P3(1), P2(1)];
        Y = [P0(2), P1(2); P3(2), P2(2)];
        Z = [P0(3), P1(3); P3(3), P2(3)];
        
        % 绘制带纹理和透明度的 surf 贴图
        surf(ax, X, Y, Z, flipud(img_crop), ...
            'FaceColor', 'texturemap', 'EdgeColor', 'none', ...
            'FaceAlpha', 'texturemap', 'AlphaData', flipud(alpha_crop), ...
            'AlphaDataMapping', 'none');

        % ==============================================================
        % 无缝衔接：直接在内存中生成该面板的密集点云数据（替代原代码第2部分）
        % ==============================================================
        
        % 原代码提取 fig 时执行了 /100，这里保持一致对物理坐标进行缩放
        P0_s = P0 / 100;
        P1_s = P1 / 100;
        P3_s = P3 / 100;
        
        % 计算基准方向向量
        v_horizontal = P1_s - P0_s; % 水平边 (对应原代码 v_horizontal)
        v_vertical   = P3_s - P0_s; % 垂直边 (对应原代码 v_vertical)
        
        len_h = norm(v_horizontal);
        len_v = norm(v_vertical);
        
        % 生成参数化坐标网格
        [s_grid, t_grid] = meshgrid(0:(grid_res/len_h):1, 0:(grid_res/len_v):1);
        
        % 利用双线性合成在 3D 空间内计算网格上的每一个点
        X_grid = P0_s(1) + s_grid*v_horizontal(1) + t_grid*v_vertical(1);
        Y_grid = P0_s(2) + s_grid*v_horizontal(2) + t_grid*v_vertical(2);
        Z_grid = P0_s(3) + s_grid*v_horizontal(3) + t_grid*v_vertical(3);
        
        % 拼接到总容器中
        X_final = [X_final; X_grid(:)];
        Y_final = [Y_final; Y_grid(:)];
        Z_final = [Z_final; Z_grid(:)];
        
    end

    %% 4. 生成与展示最终的 3D 点云
    % 清洗 NaN 和 Inf 数据
    valid = isfinite(X_final) & isfinite(Y_final) & isfinite(Z_final);
    ptCloud_Recon = pointCloud([X_final(valid), Y_final(valid), Z_final(valid)]);
    
    % 可视化最终的独立点云
    figure('Name', 'Generated 3D Point Cloud', 'NumberTitle', 'off');
    pcshow(ptCloud_Recon);
    title('自动生成的 3D 密集点云');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    fprintf('成功生成点云数据，总点数: %d\n', ptCloud_Recon.Count);
end