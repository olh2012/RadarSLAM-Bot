# FishBot SLAM 技术设计文档

## 1. 项目概述

### 1.1 项目背景
本项目实现了一个基于ROS2 Humble的二轮差速机器人激光雷达SLAM系统。项目参考了鱼香ROS的FishBot机器人设计，采用SLAM Toolbox作为建图算法，实现了完整的仿真建图功能。

### 1.2 设计目标
- 实现稳定的2D激光SLAM建图
- 提供完整的仿真环境
- 支持实时可视化和交互控制
- 代码结构清晰，易于扩展

## 2. 系统架构设计

### 2.1 整体架构

系统采用分层架构设计：

```
应用层：RViz2可视化、键盘控制、地图保存
    ↓
算法层：SLAM Toolbox建图算法
    ↓
控制层：ROS2 Control差速驱动控制
    ↓
感知层：激光雷达传感器
    ↓
仿真层：Gazebo物理仿真
```

### 2.2 核心模块

#### 2.2.1 机器人模型模块
- **文件**: `urdf/fishbot_base.urdf.xacro`, `urdf/fishbot.urdf.xacro`
- **功能**: 定义机器人物理结构和参数
- **技术要点**:
  - 使用Xacro宏简化URDF定义
  - 包含碰撞检测和惯性参数
  - 支持Gazebo物理仿真

#### 2.2.2 传感器模块
- **传感器类型**: 2D激光雷达（Ray Sensor）
- **参数配置**:
  - 扫描角度: 360度（-π到π）
  - 采样点数: 360个
  - 扫描频率: 10 Hz
  - 测距范围: 0.12m - 12.0m
  - 高斯噪声: 标准差0.01m

#### 2.2.3 控制模块
- **控制框架**: ROS2 Control
- **控制器类型**: diff_drive_controller
- **功能实现**:
  - 速度命令到轮速的转换
  - 里程计计算和发布
  - TF树维护（odom->base_footprint）
  - 速度限制和加速度限制

#### 2.2.4 SLAM模块
- **算法**: SLAM Toolbox（基于Karto SLAM）
- **工作模式**: 异步建图模式
- **关键特性**:
  - 扫描匹配定位
  - 回环检测
  - 位姿图优化
  - 交互式地图编辑

## 3. 数据流设计

### 3.1 话题通信

```
激光雷达数据流:
/scan [sensor_msgs/LaserScan]
    ↓
SLAM Toolbox
    ↓
/map [nav_msgs/OccupancyGrid]

控制数据流:
/diff_drive_controller/cmd_vel_unstamped [geometry_msgs/Twist]
    ↓
差速驱动控制器
    ↓
/odom [nav_msgs/Odometry]

TF树:
map -> odom -> base_footprint -> base_link -> laser_link
                                           -> left_wheel_link
                                           -> right_wheel_link
```

### 3.2 关键话题说明

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|-----|
| /scan | sensor_msgs/LaserScan | 发布 | 激光雷达扫描数据 |
| /map | nav_msgs/OccupancyGrid | 发布 | SLAM构建的占据栅格地图 |
| /odom | nav_msgs/Odometry | 发布 | 里程计信息 |
| /diff_drive_controller/cmd_vel_unstamped | geometry_msgs/Twist | 订阅 | 速度命令 |
| /joint_states | sensor_msgs/JointState | 发布 | 关节状态 |

## 4. 机器人运动学设计

### 4.1 差速驱动运动学模型

#### 正运动学
已知左右轮速度 v_l 和 v_r，计算机器人线速度v和角速度ω：

```
v = (v_l + v_r) / 2
ω = (v_r - v_l) / L

其中 L 为两轮间距 (0.2m)
```

#### 逆运动学
已知期望线速度v和角速度ω，计算左右轮速度：

```
v_l = v - (ω * L) / 2
v_r = v + (ω * L) / 2
```

### 4.2 里程计计算

基于轮式编码器的位姿估计：
```
Δs = (Δs_l + Δs_r) / 2
Δθ = (Δs_r - Δs_l) / L

x = x + Δs * cos(θ + Δθ/2)
y = y + Δs * sin(θ + Δθ/2)
θ = θ + Δθ
```

## 5. SLAM算法设计

### 5.1 SLAM Toolbox工作流程

```
1. 激光数据预处理
   ↓
2. 扫描匹配（Scan Matching）
   ↓
3. 位姿估计更新
   ↓
4. 地图更新
   ↓
5. 回环检测
   ↓
6. 位姿图优化
```

### 5.2 关键参数说明

| 参数 | 值 | 说明 |
|-----|-----|-----|
| minimum_travel_distance | 0.2m | 触发新扫描处理的最小移动距离 |
| minimum_travel_heading | 0.2rad | 触发新扫描处理的最小旋转角度 |
| do_loop_closing | true | 是否启用回环检测 |
| loop_search_maximum_distance | 3.0m | 回环搜索最大距离 |
| map_update_interval | 5.0s | 地图发布更新间隔 |

### 5.3 优化策略

**扫描匹配优化**:
- 使用相关性搜索空间
- 粗搜索 + 精搜索两阶段策略
- 响应值阈值过滤

**回环检测优化**:
- 链式匹配最小响应阈值
- 协方差约束
- 位姿图优化（使用Ceres Solver）

## 6. 性能优化设计

### 6.1 实时性优化
- 异步SLAM处理，避免阻塞
- 扫描节流（throttle_scans）
- 地图更新频率控制

### 6.2 精度优化
- 高频率轮式里程计（50Hz）
- 激光扫描匹配校正
- 回环检测消除累积误差

## 7. 可扩展性设计

### 7.1 模块化设计
- 独立的Launch文件便于组合使用
- 配置文件与代码分离
- 标准ROS2接口，易于替换组件

### 7.2 扩展方向
- 添加IMU传感器融合
- 集成Nav2导航功能
- 支持3D SLAM
- 多机器人协同建图

## 8. 代码规范

### 8.1 Python代码规范
- 遵循PEP 8
- 使用类型注解
- 详细的文档字符串
- 异常处理

### 8.2 URDF/Xacro规范
- 使用Xacro宏复用代码
- 参数化设计
- 清晰的命名约定
- 完整的物理参数

### 8.3 配置文件规范
- YAML格式
- 层次化组织
- 详细注释说明
- 合理的默认值

## 9. 测试与验证

### 9.1 功能测试
- [x] Gazebo仿真环境启动
- [x] 机器人模型加载
- [x] 激光雷达数据发布
- [x] 差速控制响应
- [x] SLAM建图功能
- [x] 地图保存功能
- [x] 键盘控制

### 9.2 性能测试
- CPU使用率监测
- 内存占用分析
- 话题频率验证
- 建图精度评估

## 10. 已知限制与改进方向

### 10.1 当前限制
- 仅支持2D平面SLAM
- 需要较好的特征环境
- 动态障碍物处理有限

### 10.2 改进方向
1. 增加IMU数据融合提高精度
2. 优化回环检测参数
3. 添加语义信息
4. 支持在线地图编辑
5. 集成自主探索功能

## 11. 参考资料

1. ROS2 Humble官方文档
2. SLAM Toolbox GitHub仓库
3. ROS2 Control设计文档
4. Gazebo仿真教程
5. 鱼香ROS教程系列


---

文档版本: v1.0  
最后更新: 2025年11月19日
