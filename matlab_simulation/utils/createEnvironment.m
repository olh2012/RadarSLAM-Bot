function [envMap, obstacles] = createEnvironment()
% createEnvironment - 创建仿真环境
% 
% 功能: 创建与ROS2 Gazebo世界相似的室内环境，包含墙壁和障碍物
%
% 输出:
%   envMap    - 环境地图结构
%   obstacles - 障碍物列表 (每行: [x1, y1, x2, y2])
%
% 作者: 欧林海 (franka907@126.com)
% 日期: 2025年11月19日

    % 环境边界
    envMap.xLimits = [-5, 5];
    envMap.yLimits = [-5, 5];
    
    % 障碍物定义 (与ROS2 Gazebo世界对应)
    % 每个障碍物: [x_center, y_center, width, height, orientation]
    
    % 墙壁1 (垂直墙)
    wall1 = createWall(2, 0, 0.2, 4, 0);
    
    % 墙壁2 (水平墙)
    wall2 = createWall(0, 2, 0.2, 4, pi/2);
    
    % 墙壁3 (垂直墙)
    wall3 = createWall(-2, 0, 0.2, 4, 0);
    
    % 障碍物盒子
    box1 = createBox(1, 1, 0.5, 0.5);
    
    % 外围边界墙
    boundaryWalls = [
        -5, -5, 5, -5;    % 下边界
        5, -5, 5, 5;      % 右边界
        5, 5, -5, 5;      % 上边界
        -5, 5, -5, -5     % 左边界
    ];
    
    % 合并所有障碍物
    obstacles = [
        wall1;
        wall2;
        wall3;
        box1;
        boundaryWalls
    ];
    
    envMap.obstacles = obstacles;
    envMap.numObstacles = size(obstacles, 1);
    
    fprintf('环境创建完成: %d 个障碍物\n', envMap.numObstacles);
end

function lines = createWall(x, y, width, length, orientation)
% createWall - 创建墙壁障碍物
% 输入:
%   x, y - 墙壁中心坐标
%   width - 墙壁宽度
%   length - 墙壁长度
%   orientation - 墙壁方向 (弧度)
% 输出:
%   lines - 墙壁的线段表示 [x1, y1, x2, y2; ...]

    % 墙壁四个角点（本地坐标系）
    w2 = width / 2;
    l2 = length / 2;
    corners = [
        -w2, -l2;
        w2, -l2;
        w2, l2;
        -w2, l2
    ];
    
    % 旋转矩阵
    R = [cos(orientation), -sin(orientation);
         sin(orientation), cos(orientation)];
    
    % 旋转并平移角点
    rotatedCorners = (R * corners')' + [x, y];
    
    % 创建线段 (矩形的四条边)
    lines = [
        rotatedCorners(1,:), rotatedCorners(2,:);
        rotatedCorners(2,:), rotatedCorners(3,:);
        rotatedCorners(3,:), rotatedCorners(4,:);
        rotatedCorners(4,:), rotatedCorners(1,:)
    ];
end

function lines = createBox(x, y, width, height)
% createBox - 创建矩形障碍物
% 输入:
%   x, y - 盒子中心坐标
%   width - 盒子宽度
%   height - 盒子高度
% 输出:
%   lines - 盒子的线段表示

    w2 = width / 2;
    h2 = height / 2;
    
    % 矩形的四条边
    lines = [
        x-w2, y-h2, x+w2, y-h2;  % 下边
        x+w2, y-h2, x+w2, y+h2;  % 右边
        x+w2, y+h2, x-w2, y+h2;  % 上边
        x-w2, y+h2, x-w2, y-h2   % 左边
    ];
end
