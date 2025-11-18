# FishBot SLAM - 基于雷达建图的ROS2机器人项目

## 项目简介

这是一个基于ROS2 Humble的二轮差速机器人激光雷达SLAM建图项目。项目使用鱼香ROS的FishBot二轮机器人平台，通过激光雷达传感器进行同步定位与地图构建（SLAM）。

## 项目特点

- ✅ 基于ROS2 Humble实现
- ✅ 使用2D激光雷达进行环境感知
- ✅ 采用SLAM Toolbox进行实时建图
- ✅ 支持Gazebo仿真环境
- ✅ 二轮差速驱动控制
- ✅ 提供键盘遥控功能
- ✅ 支持地图保存和导出

## 技术架构

### 核心技术栈
- **ROS2版本**: Humble Hawksbill
- **机器人模型**: FishBot 二轮差速机器人
- **SLAM算法**: SLAM Toolbox (基于Karto SLAM)
- **仿真环境**: Gazebo Classic
- **控制系统**: ROS2 Control (diff_drive_controller)

### 系统架构
```
┌─────────────────────────────────────────────────────┐
│                   FishBot SLAM系统                   │
├─────────────────────────────────────────────────────┤
│  Gazebo仿真环境                                      │
│    ├── 机器人模型 (URDF/Xacro)                      │
│    ├── 激光雷达传感器                                │
│    └── 物理世界                                      │
├─────────────────────────────────────────────────────┤
│  ROS2 Control                                        │
│    ├── 差速驱动控制器                                │
│    ├── 关节状态发布器                                │
│    └── 里程计发布                                    │
├─────────────────────────────────────────────────────┤
│  SLAM Toolbox                                        │
│    ├── 激光扫描处理                                  │
│    ├── 位姿估计                                      │
│    ├── 回环检测                                      │
│    └── 地图构建                                      │
├─────────────────────────────────────────────────────┤
│  可视化与交互                                        │
│    ├── RViz2 可视化                                  │
│    ├── 键盘遥控                                      │
│    └── 地图保存                                      │
└─────────────────────────────────────────────────────┘
```

## 项目结构

```
fishbot_slam/
├── config/                          # 配置文件目录
│   ├── diff_drive_controller.yaml   # 差速驱动控制器配置
│   └── slam_toolbox_params.yaml     # SLAM参数配置
├── launch/                          # 启动文件目录
│   ├── gazebo_sim.launch.py         # Gazebo仿真启动
│   ├── slam.launch.py               # SLAM建图启动
│   └── fishbot_slam.launch.py       # 完整系统启动
├── maps/                            # 地图保存目录
├── scripts/                         # 脚本文件目录
│   ├── save_map.py                  # 地图保存脚本
│   └── teleop_keyboard.py           # 键盘遥控脚本
├── urdf/                            # 机器人模型目录
│   ├── fishbot_base.urdf.xacro      # 基础机器人模型
│   └── fishbot.urdf.xacro           # 完整机器人模型（含Gazebo插件）
├── worlds/                          # Gazebo世界文件
│   └── fishbot_world.world          # 测试环境世界
├── CMakeLists.txt                   # CMake构建文件
└── package.xml                      # ROS2包描述文件
```

## 核心功能

### 1. 机器人模型（URDF）
- **底盘**: 圆柱形底盘（半径10cm，高度8cm）
- **驱动轮**: 两个独立驱动轮，支持差速控制
- **支撑轮**: 前后各一个万向支撑轮
- **激光雷达**: 360度2D激光雷达，最大探测距离12米

### 2. 差速驱动控制
- 使用ROS2 Control框架
- diff_drive_controller实现差速控制
- 发布里程计信息（odom话题）
- 发布TF变换（odom -> base_footprint）
- 速度限制：线速度±0.5m/s，角速度±1.0rad/s

### 3. SLAM建图
- 基于SLAM Toolbox的异步SLAM
- 实时激光扫描匹配
- 回环检测优化
- 地图增量更新
- 支持交互式地图编辑

### 4. 可视化与控制
- RViz2实时显示机器人状态和地图
- 键盘遥控支持多方向移动
- 地图实时构建和显示
- 支持保存PGM格式地图

## 使用说明

### 环境要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic 11
- Python 3.10+

### 依赖安装
```bash
# 安装ROS2 Humble核心包
sudo apt install ros-humble-desktop

# 安装Gazebo相关包
sudo apt install ros-humble-gazebo-ros-pkgs

# 安装ROS2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# 安装SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# 安装导航相关包
sudo apt install ros-humble-nav2-bringup
```

### 编译项目
```bash
# 创建工作空间
mkdir -p ~/fishbot_ws/src
cd ~/fishbot_ws/src

# 克隆或复制项目代码
# cp -r /path/to/fishbot_slam .

# 返回工作空间根目录编译
cd ~/fishbot_ws
colcon build --symlink-install

# 加载环境变量
source install/setup.bash
```

### 运行系统

#### 方式1: 一键启动完整系统
```bash
ros2 launch fishbot_slam fishbot_slam.launch.py
```

#### 方式2: 分步启动
```bash
# 终端1: 启动Gazebo仿真
ros2 launch fishbot_slam gazebo_sim.launch.py

# 终端2: 启动SLAM建图
ros2 launch fishbot_slam slam.launch.py

# 终端3: 启动键盘控制
ros2 run fishbot_slam teleop_keyboard.py

# 终端4: 启动RViz2可视化（可选）
rviz2
```

### 键盘控制说明
```
移动控制：
   u    i    o
   j    k    l
   m    ,    .

u/o : 前进+左转/右转
i   : 前进
j/l : 原地左转/右转
k   : 停止
m/. : 后退+左转/右转
,   : 后退

q/z : 增加/减少速度 10%
space : 紧急停止
Ctrl+C : 退出
```

### 保存地图
```bash
# 使用命令行工具保存
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 或使用项目提供的脚本
ros2 run fishbot_slam save_map.py fishbot_map
```

## 参数配置

### 差速控制器参数
- 轮子间距: 0.2米
- 轮子半径: 0.033米
- 最大线速度: 0.5 m/s
- 最大角速度: 1.0 rad/s

### SLAM参数
- 最小移动距离: 0.2米
- 最小旋转角度: 0.2弧度
- 回环检测: 启用
- 地图更新间隔: 5.0秒

## 代码规范

本项目遵循以下规范：
- **Python代码**: PEP 8编码规范
- **C++代码**: Google C++编码规范
- **ROS2**: ROS2官方开发规范
- **文档**: 使用中文注释，遵循Google风格

## 故障排除

### 问题1: Gazebo无法启动
- 检查Gazebo是否正确安装
- 确认ROS2环境变量已加载

### 问题2: 机器人不移动
- 检查控制器是否加载成功
- 确认话题名称是否正确
- 查看`ros2 control list_controllers`输出

### 问题3: SLAM建图效果差
- 调整机器人移动速度（降低速度）
- 检查激光雷达数据质量
- 优化SLAM参数配置

### 问题4: 地图保存失败
- 确认地图保存路径有写权限
- 检查SLAM节点是否正常运行

## 扩展功能

### 未来可添加的功能
- [ ] 导航功能（Nav2集成）
- [ ] 路径规划
- [ ] 自主探索
- [ ] 多机器人协同SLAM
- [ ] 3D SLAM支持

## 技术参考

- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [鱼香ROS](https://fishros.com/)
- [Gazebo官方文档](http://gazebosim.org/)

## 许可证

Apache-2.0 License

## 联系方式

**项目作者**: 欧林海  
**邮箱**: franka907@126.com  

欢迎发送建议或反馈！

