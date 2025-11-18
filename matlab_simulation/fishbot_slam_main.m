%% FishBot二轮差速机器人激光雷达SLAM仿真主程序
% 作者: 欧林海
% 邮箱: franka907@126.com
% 日期: 2025年11月19日
% 
% 功能说明:
% 本程序实现基于Matlab的FishBot二轮差速机器人激光雷达SLAM仿真
% 使用lidarSLAM和位姿图优化实现地图构建和机器人轨迹估计
%
% 技术要点:
% 1. 二轮差速运动学模型
% 2. 2D激光雷达扫描仿真
% 3. lidarSLAM算法
% 4. 回环检测与位姿图优化
% 5. 占据栅格地图构建

clear; close all; clc;

%% 仿真参数配置
% 机器人参数（与ROS2项目一致）
robot.wheelRadius = 0.033;      % 轮子半径 (m)
robot.wheelSeparation = 0.2;    % 轮间距 (m)
robot.maxLinearVel = 0.5;       % 最大线速度 (m/s)
robot.maxAngularVel = 1.0;      % 最大角速度 (rad/s)

% 激光雷达参数（与ROS2项目一致）
lidar.range = 12.0;             % 最大测距范围 (m)
lidar.minRange = 0.12;          % 最小测距范围 (m)
lidar.angleRange = [-pi, pi];   % 扫描角度范围
lidar.numReadings = 360;        % 扫描点数
lidar.resolution = 0.01;        % 距离分辨率 (m)
lidar.noiseStd = 0.01;          % 高斯噪声标准差 (m)

% SLAM参数
slam.mapResolution = 20;        % 地图分辨率 (cells/m)
slam.maxLidarRange = 10.0;      % SLAM使用的最大雷达距离 (m)
slam.loopClosureThreshold = 210;% 回环检测阈值
slam.loopClosureRadius = 3.0;   % 回环搜索半径 (m)

% 仿真参数
sim.dt = 0.1;                   % 时间步长 (s)
sim.totalTime = 100;            % 总仿真时间 (s)
sim.showAnimation = true;       % 是否显示动画

fprintf('=== FishBot SLAM仿真系统 ===\n');
fprintf('作者: 欧林海 (franka907@126.com)\n');
fprintf('日期: 2025年11月19日\n\n');

%% 创建环境地图
fprintf('正在创建仿真环境...\n');
[envMap, obstacles] = createEnvironment();

%% 生成机器人运动轨迹
fprintf('正在生成机器人运动轨迹...\n');
trajectory = generateTrajectory(sim.totalTime, sim.dt);

%% 初始化SLAM算法
fprintf('正在初始化SLAM算法...\n');
slamAlg = lidarSLAM(slam.mapResolution, slam.maxLidarRange);
slamAlg.LoopClosureThreshold = slam.loopClosureThreshold;
slamAlg.LoopClosureSearchRadius = slam.loopClosureRadius;

%% 初始化可视化
if sim.showAnimation
    figure('Name', 'FishBot SLAM仿真', 'Position', [100, 100, 1200, 600]);
end

%% 主仿真循环
fprintf('开始SLAM仿真...\n');
numScans = size(trajectory, 1);
scanData = cell(numScans, 1);
acceptedScans = 0;

for i = 1:numScans
    % 当前位姿
    currentPose = trajectory(i, :);  % [x, y, theta]
    
    % 生成激光雷达扫描数据
    scan = generateLidarScan(currentPose, obstacles, lidar);
    scanData{i} = scan;
    
    % 添加扫描到SLAM算法
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
    
    if isScanAccepted
        acceptedScans = acceptedScans + 1;
        
        % 显示进度
        if mod(acceptedScans, 10) == 0
            fprintf('已处理 %d/%d 个扫描 (接受 %d 个)\n', i, numScans, acceptedScans);
        end
        
        % 检测到回环闭合时显示信息
        if optimizationInfo.IsPerformed
            fprintf('>>> 检测到回环闭合! 扫描 #%d\n', i);
        end
    end
    
    % 动画显示
    if sim.showAnimation && mod(i, 5) == 0
        subplot(1, 2, 1);
        show(slamAlg);
        title(sprintf('SLAM地图构建 (扫描 %d/%d)', i, numScans));
        xlabel('X (m)'); ylabel('Y (m)');
        grid on;
        
        subplot(1, 2, 2);
        plotCurrentScan(currentPose, scan, obstacles);
        title(sprintf('当前激光扫描 (位姿: [%.2f, %.2f, %.2f])', ...
            currentPose(1), currentPose(2), currentPose(3)));
        xlabel('X (m)'); ylabel('Y (m)');
        grid on; axis equal;
        
        drawnow;
    end
end

fprintf('\nSLAM仿真完成!\n');
fprintf('总扫描数: %d, 接受扫描数: %d (%.1f%%)\n', ...
    numScans, acceptedScans, 100*acceptedScans/numScans);

%% 构建最终地图
fprintf('\n正在构建占据栅格地图...\n');
[scans, optimizedPoses] = scansAndPoses(slamAlg);
occupancyMap = buildMap(scans, optimizedPoses, slam.mapResolution, slam.maxLidarRange);

%% 结果可视化
fprintf('正在生成结果可视化...\n');
visualizeResults(slamAlg, occupancyMap, trajectory, scans, optimizedPoses);

%% 保存结果
fprintf('正在保存仿真结果...\n');
saveResults(slamAlg, occupancyMap, trajectory, scans, optimizedPoses);

fprintf('\n=== 仿真完成 ===\n');
fprintf('结果已保存到 matlab_simulation/data/ 目录\n');
