function trajectory = generateTrajectory(totalTime, dt)
% generateTrajectory - 生成机器人运动轨迹
%
% 功能: 生成二轮差速机器人的闭环轨迹，用于SLAM测试
%
% 输入:
%   totalTime - 总仿真时间 (秒)
%   dt        - 时间步长 (秒)
%
% 输出:
%   trajectory - N×3矩阵，每行为[x, y, theta]
%
% 作者: 欧林海 (franka907@126.com)
% 日期: 2025年11月19日

    numSteps = round(totalTime / dt);
    trajectory = zeros(numSteps, 3);  % [x, y, theta]
    
    % 初始位姿
    x = 0;
    y = 0;
    theta = 0;
    
    % 运动参数
    linearVel = 0.3;   % 线速度 (m/s)
    angularVel = 0.0;  % 角速度 (rad/s)
    
    % 定义轨迹段
    % 每段: [持续时间(s), 线速度(m/s), 角速度(rad/s)]
    segments = [
        10, 0.3, 0.0;      % 直线前进
        5,  0.2, 0.3;      % 右转
        8,  0.3, 0.0;      % 直线前进
        5,  0.2, -0.3;     % 左转
        10, 0.3, 0.0;      % 直线前进
        5,  0.2, 0.5;      % 右转
        8,  0.3, 0.0;      % 直线前进
        5,  0.2, -0.5;     % 左转
        10, 0.3, 0.0;      % 直线前进
        8,  0.2, 0.4;      % 右转弧线
        10, 0.3, 0.0;      % 直线前进
        5,  0.2, -0.4;     % 左转
        12, 0.3, 0.0;      % 直线前进
        5,  0.2, 0.3;      % 右转回原点
        10, 0.2, 0.0       % 减速停止
    ];
    
    currentTime = 0;
    segmentIdx = 1;
    
    for i = 1:numSteps
        % 当前位姿
        trajectory(i, :) = [x, y, theta];
        
        % 检查是否需要切换到下一段
        if currentTime >= sum(segments(1:segmentIdx, 1)) && segmentIdx < size(segments, 1)
            segmentIdx = segmentIdx + 1;
        end
        
        % 当前段的速度命令
        linearVel = segments(segmentIdx, 2);
        angularVel = segments(segmentIdx, 3);
        
        % 二轮差速运动学模型
        % dx/dt = v * cos(theta)
        % dy/dt = v * sin(theta)
        % dtheta/dt = omega
        x = x + linearVel * cos(theta) * dt;
        y = y + linearVel * sin(theta) * dt;
        theta = theta + angularVel * dt;
        
        % 限制角度在[-pi, pi]范围内
        theta = wrapToPi(theta);
        
        currentTime = currentTime + dt;
    end
    
    fprintf('轨迹生成完成: %d 个位姿点\n', numSteps);
end
