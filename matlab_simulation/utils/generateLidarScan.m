function scan = generateLidarScan(pose, obstacles, lidarParams)
% generateLidarScan - 生成激光雷达扫描数据
%
% 功能: 基于机器人位姿和环境障碍物生成2D激光雷达扫描
%
% 输入:
%   pose        - 机器人位姿 [x, y, theta]
%   obstacles   - 障碍物列表 (N×4矩阵)
%   lidarParams - 雷达参数结构体
%
% 输出:
%   scan - lidarScan对象
%
% 作者: 欧林海 (franka907@126.com)
% 日期: 2025年11月19日

    % 扫描角度
    angles = linspace(lidarParams.angleRange(1), ...
                      lidarParams.angleRange(2), ...
                      lidarParams.numReadings);
    
    % 初始化距离数据
    ranges = lidarParams.range * ones(size(angles));
    
    % 机器人位置和方向
    robotX = pose(1);
    robotY = pose(2);
    robotTheta = pose(3);
    
    % 对每个扫描角度进行射线追踪
    for i = 1:length(angles)
        % 全局坐标系下的射线角度
        rayAngle = robotTheta + angles(i);
        
        % 射线方向
        rayDirX = cos(rayAngle);
        rayDirY = sin(rayAngle);
        
        minDist = lidarParams.range;
        
        % 检查与每个障碍物的交点
        for j = 1:size(obstacles, 1)
            % 障碍物线段
            x1 = obstacles(j, 1);
            y1 = obstacles(j, 2);
            x2 = obstacles(j, 3);
            y2 = obstacles(j, 4);
            
            % 计算射线与线段的交点
            dist = rayLineIntersection(robotX, robotY, rayDirX, rayDirY, ...
                                       x1, y1, x2, y2);
            
            if dist > 0 && dist < minDist
                minDist = dist;
            end
        end
        
        % 限制在最小和最大范围内
        ranges(i) = max(lidarParams.minRange, min(minDist, lidarParams.range));
    end
    
    % 添加高斯噪声
    ranges = ranges + lidarParams.noiseStd * randn(size(ranges));
    
    % 限制范围
    ranges = max(lidarParams.minRange, min(ranges, lidarParams.range));
    
    % 创建lidarScan对象
    scan = lidarScan(ranges, angles);
end

function dist = rayLineIntersection(x0, y0, dx, dy, x1, y1, x2, y2)
% rayLineIntersection - 计算射线与线段的交点距离
%
% 使用参数方程求解:
% 射线: P = (x0, y0) + t*(dx, dy), t >= 0
% 线段: Q = (x1, y1) + s*((x2-x1), (y2-y1)), 0 <= s <= 1

    % 线段方向
    lx = x2 - x1;
    ly = y2 - y1;
    
    % 计算交点参数
    denominator = dx * ly - dy * lx;
    
    if abs(denominator) < 1e-10
        % 平行，无交点
        dist = -1;
        return;
    end
    
    % 计算参数 t 和 s
    t = ((x1 - x0) * ly - (y1 - y0) * lx) / denominator;
    s = ((x1 - x0) * dy - (y1 - y0) * dx) / denominator;
    
    % 检查是否在有效范围内
    if t >= 0 && s >= 0 && s <= 1
        dist = t * sqrt(dx^2 + dy^2);
    else
        dist = -1;
    end
end
