function plotCurrentScan(pose, scan, obstacles)
% plotCurrentScan - 绘制当前激光扫描
%
% 作者: 欧林海 (franka907@126.com)
% 日期: 2025年11月19日

    cla; hold on;
    
    % 绘制障碍物
    for i = 1:size(obstacles, 1)
        plot([obstacles(i,1), obstacles(i,3)], ...
             [obstacles(i,2), obstacles(i,4)], 'k-', 'LineWidth', 2);
    end
    
    % 绘制机器人位置
    robotSize = 0.2;
    robotCircle = viscircles([pose(1), pose(2)], robotSize/2, ...
        'Color', 'b', 'LineWidth', 1.5);
    
    % 绘制机器人方向
    headingLength = 0.3;
    quiver(pose(1), pose(2), ...
           headingLength*cos(pose(3)), headingLength*sin(pose(3)), ...
           'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % 绘制激光扫描点
    [cartesian] = readCartesian(scan);
    scanX = cartesian(:,1) + pose(1);
    scanY = cartesian(:,2) + pose(2);
    plot(scanX, scanY, 'r.', 'MarkerSize', 3);
    
    axis equal; grid on;
    xlim([-6, 6]); ylim([-6, 6]);
end
