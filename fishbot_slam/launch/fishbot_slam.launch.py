#!/usr/bin/env python3
"""
完整的SLAM建图系统启动文件
功能：一次性启动Gazebo仿真、机器人模型、SLAM建图和RViz可视化
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成Launch描述"""
    
    # 获取软件包路径
    pkg_fishbot_slam = get_package_share_directory('fishbot_slam')
    
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 声明Launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )
    
    # 包含Gazebo仿真Launch文件
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fishbot_slam, 'launch', 'gazebo_sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 包含SLAM Launch文件
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fishbot_slam, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # RViz2节点
    rviz_config_file = os.path.join(pkg_fishbot_slam, 'config', 'fishbot_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 创建Launch描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(slam_launch)
    ld.add_action(rviz_node)
    
    return ld
