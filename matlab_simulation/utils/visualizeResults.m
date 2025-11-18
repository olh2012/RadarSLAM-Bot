function visualizeResults(slamAlg, occupancyMap, trajectory, scans, optimizedPoses)
% visualizeResults - 可视化SLAM结果
%
% 作者: 欧林海 (franka907@126.com)
% 日期: 2025年11月19日

    figure('Name', 'SLAM结果可视化', 'Position', [100, 100, 1400, 800]);
    
    % 子图1: 最终地图和位姿图
    subplot(2, 3, 1);
    show(slamAlg);
    title('SLAM构建的地图和位姿图');
    xlabel('X (m)'); ylabel('Y (m)');
    grid on;
    
    % 子图2: 占据栅格地图
    subplot(2, 3, 2);
    show(occupancyMap);
    hold on;
    show(slamAlg.PoseGraph, 'IDs', 'off');
    title('占据栅格地图');
    xlabel('X (m)'); ylabel('Y (m)');
    
    % 子图3: 轨迹对比
    subplot(2, 3, 3);
    plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 1.5);
    hold on;
    plot(optimizedPoses(:,1), optimizedPoses(:,2), 'r--', 'LineWidth', 1.5);
    legend('真实轨迹', '优化轨迹');
    title('轨迹对比');
    xlabel('X (m)'); ylabel('Y (m)');
    grid on; axis equal;
    
    % 子图4: X位置误差
    subplot(2, 3, 4);
    errors = trajectory(1:length(optimizedPoses),:) - optimizedPoses;
    plot(errors(:,1), 'LineWidth', 1.5);
    title('X位置误差');
    xlabel('时间步'); ylabel('误差 (m)');
    grid on;
    
    % 子图5: Y位置误差
    subplot(2, 3, 5);
    plot(errors(:,2), 'LineWidth', 1.5);
    title('Y位置误差');
    xlabel('时间步'); ylabel('误差 (m)');
    grid on;
    
    % 子图6: 角度误差
    subplot(2, 3, 6);
    angleErrors = wrapToPi(errors(:,3));
    plot(rad2deg(angleErrors), 'LineWidth', 1.5);
    title('角度误差');
    xlabel('时间步'); ylabel('误差 (度)');
    grid on;
    
    % 计算统计信息
    meanPosError = mean(sqrt(errors(:,1).^2 + errors(:,2).^2));
    meanAngleError = mean(abs(angleErrors));
    
    fprintf('\n=== 精度统计 ===\n');
    fprintf('平均位置误差: %.4f m\n', meanPosError);
    fprintf('平均角度误差: %.4f 度\n', rad2deg(meanAngleError));
end
