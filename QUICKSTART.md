# FishBot SLAM 快速入门指南

## 快速开始（5分钟上手）

### 步骤1: 环境准备
确保你已经安装了ROS2 Humble环境：
```bash
# 验证ROS2安装
ros2 --version

# 安装必要依赖（如果还没安装）
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup
```

### 步骤2: 构建项目
```bash
# 创建工作空间并复制代码
mkdir -p ~/fishbot_ws/src
cd ~/fishbot_ws/src
# 将fishbot_slam目录复制到这里

# 编译
cd ~/fishbot_ws
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

### 步骤3: 一键启动
```bash
# 启动完整系统（包括Gazebo、SLAM和RViz）
ros2 launch fishbot_slam fishbot_slam.launch.py
```

### 步骤4: 控制机器人建图
在新终端中：
```bash
source ~/fishbot_ws/install/setup.bash
ros2 run fishbot_slam teleop_keyboard.py
```

使用键盘控制机器人移动：
- `i` - 前进
- `,` - 后退  
- `j` - 左转
- `l` - 右转
- `k` - 停止

### 步骤5: 保存地图
建图完成后：
```bash
# 方式1: 使用ROS2命令
ros2 run nav2_map_server map_saver_cli -f ~/fishbot_map

# 方式2: 使用项目脚本
ros2 run fishbot_slam save_map.py my_map
```

## 常见操作

### 仅启动Gazebo仿真
```bash
ros2 launch fishbot_slam gazebo_sim.launch.py
```

### 仅启动SLAM建图
```bash
ros2 launch fishbot_slam slam.launch.py
```

### 查看话题列表
```bash
ros2 topic list
```

### 查看激光雷达数据
```bash
ros2 topic echo /scan
```

### 查看里程计数据
```bash
ros2 topic echo /odom
```

### 查看TF树
```bash
ros2 run tf2_tools view_frames
```

## 可视化

### RViz中查看的内容
1. **Grid** - 网格地图
2. **TF** - 坐标变换树
3. **RobotModel** - 机器人模型
4. **LaserScan** - 激光扫描数据（红色点）
5. **Map** - SLAM构建的地图（灰度）

### 调整RViz视图
- 鼠标滚轮：缩放
- 鼠标中键拖动：平移
- Shift+鼠标左键：旋转视角

## 故障排除速查

### Q1: 启动失败，提示找不到包
```bash
# 确保已经source环境
source ~/fishbot_ws/install/setup.bash
# 或添加到.bashrc
echo "source ~/fishbot_ws/install/setup.bash" >> ~/.bashrc
```

### Q2: 机器人不动
```bash
# 检查控制器状态
ros2 control list_controllers
# 应该看到diff_drive_controller和joint_state_broadcaster都是active

# 手动发送速度命令测试
ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/Twist "{linear: {x: 0.2}}"
```

### Q3: 没有激光数据
```bash
# 检查scan话题
ros2 topic hz /scan
# 应该看到约10Hz的频率

# 查看激光数据
ros2 topic echo /scan --once
```

### Q4: SLAM不工作
```bash
# 检查SLAM节点
ros2 node list | grep slam
# 应该看到/slam_toolbox节点

# 查看地图话题
ros2 topic hz /map
```

## 性能优化建议

### 如果计算机性能较低
1. 减少激光采样点：编辑`urdf/fishbot.urdf.xacro`，将samples从360改为180
2. 降低地图更新频率：编辑`config/slam_toolbox_params.yaml`，增加map_update_interval
3. 关闭Gazebo GUI：启动时添加参数`gui:=false`

### 如果建图精度不够
1. 降低机器人移动速度
2. 增加扫描频率
3. 调整SLAM参数中的minimum_travel_distance和minimum_travel_heading

## 下一步

- 学习ROS2基础概念：话题、服务、参数
- 了解SLAM原理和SLAM Toolbox
- 尝试集成Nav2进行自主导航
- 自定义Gazebo世界进行测试

## 获取帮助

- 查看README.md了解项目详情
- 查看DESIGN.md了解技术设计
- ROS2官方文档：https://docs.ros.org/en/humble/
- 鱼香ROS社区：https://fishros.com/

## 作者信息

**姓名**: 欧林海  
**邮箱**: franka907@126.com  

欢迎发送建议或反馈！

---
祝你使用愉快！🤖
