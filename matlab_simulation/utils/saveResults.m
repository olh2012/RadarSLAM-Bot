function saveResults(slamAlg, occupancyMap, trajectory, scans, optimizedPoses)
% saveResults - 保存仿真结果
%
% 作者: 欧林海 (franka907@126.com)
% 日期: 2025年11月19日

    % 创建时间戳
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % 保存目录
    saveDir = 'matlab_simulation/data/';
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end
    
    % 保存SLAM对象
    filename = [saveDir, 'slam_result_', timestamp, '.mat'];
    save(filename, 'slamAlg', 'occupancyMap', 'trajectory', 'scans', 'optimizedPoses');
    fprintf('SLAM结果已保存到: %s\n', filename);
    
    % 保存地图为图像
    mapImage = occupancyMatrix(occupancyMap);
    mapFilename = [saveDir, 'map_', timestamp, '.png'];
    imwrite(1 - mapImage, mapFilename);  % 反色以匹配ROS地图
    fprintf('地图图像已保存到: %s\n', mapFilename);
    
    % 保存轨迹数据
    trajFilename = [saveDir, 'trajectory_', timestamp, '.txt'];
    dlmwrite(trajFilename, [trajectory, optimizedPoses], 'delimiter', '\t', 'precision', 6);
    fprintf('轨迹数据已保存到: %s\n', trajFilename);
end
