FishBot SLAM Matlab仿真系统使用说明

作者: 欧林海
邮箱: franka907@126.com
日期: 2025年11月19日

========================================
一、系统简介
========================================

本Matlab仿真系统实现了基于激光雷达的SLAM算法，模拟FishBot二轮差速机器人
在室内环境中进行建图和定位。系统使用Matlab的Robotics System Toolbox和
Navigation Toolbox中的lidarSLAM功能。

========================================
二、环境要求
========================================

1. Matlab R2020b或更高版本
2. 必需工具箱:
   - Robotics System Toolbox
   - Navigation Toolbox
   - Lidar Toolbox (可选)
3. 操作系统: Windows/Linux/macOS均可

========================================
三、文件结构
========================================

matlab_simulation/
├── fishbot_slam_main.m          # 主仿真程序
├── utils/                        # 工具函数目录
│   ├── createEnvironment.m      # 环境创建
│   ├── generateTrajectory.m     # 轨迹生成
│   ├── generateLidarScan.m      # 激光扫描仿真
│   ├── plotCurrentScan.m        # 扫描可视化
│   ├── visualizeResults.m       # 结果可视化
│   └── saveResults.m            # 结果保存
├── data/                         # 结果保存目录
└── README.txt                    # 本文件

========================================
四、快速开始
========================================

1. 启动Matlab

2. 将工作目录设置到matlab_simulation文件夹:
   >> cd('/path/to/RadarSLAM-Bot/matlab_simulation')

3. 添加utils路径:
   >> addpath('utils')

4. 运行主程序:
   >> fishbot_slam_main

5. 仿真将自动运行,实时显示建图过程

========================================
五、主要功能
========================================

1. 环境仿真
   - 创建与ROS2 Gazebo世界相似的环境
   - 包含墙壁和障碍物

2. 机器人运动
   - 二轮差速运动学模型
   - 预定义的闭环轨迹

3. 激光雷达仿真
   - 360度扫描
   - 射线追踪算法
   - 高斯噪声模拟

4. SLAM建图
   - lidarSLAM算法
   - 扫描匹配
   - 回环检测
   - 位姿图优化

5. 结果分析
   - 轨迹对比
   - 误差分析
   - 地图保存

========================================
六、参数配置
========================================

在fishbot_slam_main.m中可以修改以下参数:

机器人参数:
  - wheelRadius: 轮子半径
  - wheelSeparation: 轮间距
  - maxLinearVel: 最大线速度
  - maxAngularVel: 最大角速度

激光雷达参数:
  - range: 最大测距范围
  - numReadings: 扫描点数
  - noiseStd: 噪声标准差

SLAM参数:
  - mapResolution: 地图分辨率
  - loopClosureThreshold: 回环检测阈值
  - loopClosureRadius: 回环搜索半径

仿真参数:
  - dt: 时间步长
  - totalTime: 总仿真时间
  - showAnimation: 是否显示动画

========================================
七、运行结果
========================================

仿真完成后,将在data目录下生成:

1. slam_result_YYYYMMDD_HHMMSS.mat
   - 包含完整的SLAM结果
   - 可用于后续分析

2. map_YYYYMMDD_HHMMSS.png
   - 占据栅格地图图像

3. trajectory_YYYYMMDD_HHMMSS.txt
   - 轨迹数据(真实轨迹和优化轨迹)

========================================
八、技术要点
========================================

1. 二轮差速运动学
   - 正运动学: v = (v_L + v_R)/2, ω = (v_R - v_L)/L
   - 位姿更新: x += v*cos(θ)*dt, y += v*sin(θ)*dt, θ += ω*dt

2. 激光雷达仿真
   - 射线追踪算法
   - 线段相交检测
   - 噪声模型

3. SLAM算法
   - 扫描匹配定位
   - 位姿图构建
   - 回环检测与优化
   - Ceres求解器

4. 与ROS2项目对应
   - 机器人参数一致
   - 环境相似
   - 雷达配置相同

========================================
九、常见问题
========================================

Q1: 提示缺少工具箱怎么办?
A: 需要安装Robotics System Toolbox和Navigation Toolbox

Q2: 仿真运行很慢?
A: 可以减少totalTime或增加dt,或关闭动画显示

Q3: 地图效果不好?
A: 调整SLAM参数,特别是loopClosureThreshold

Q4: 如何加载保存的结果?
A: 使用load('data/slam_result_*.mat')

========================================
十、扩展开发
========================================

1. 修改环境
   - 编辑createEnvironment.m添加更多障碍物

2. 修改轨迹
   - 编辑generateTrajectory.m中的segments变量

3. 添加传感器
   - 在generateLidarScan.m中添加新的传感器模型

4. 优化算法
   - 调整SLAM参数获得更好效果

========================================
十一、参考资料
========================================

1. Matlab lidarSLAM文档:
   https://www.mathworks.com/help/nav/ref/lidarslam.html

2. 二轮差速机器人运动学:
   https://www.mathworks.com/help/robotics/ref/differentialdrivekinematics.html

3. SLAM Toolbox原理:
   https://github.com/SteveMacenski/slam_toolbox

========================================
十二、联系方式
========================================

如有问题或建议,欢迎联系:
作者: 欧林海
邮箱: franka907@126.com

========================================
