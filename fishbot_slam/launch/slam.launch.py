#!/usr/bin/env python3
"""
SLAM建图启动文件
功能：启动SLAM Toolbox进行实时建图
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成Launch描述"""
    
    # 获取软件包路径
    pkg_fishbot_slam = get_package_share_directory('fishbot_slam')
    
    # 配置文件路径
    slam_params_file = os.path.join(
        pkg_fishbot_slam, 'config', 'slam_toolbox_params.yaml'
    )
    
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 声明Launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )
    
    # SLAM Toolbox节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 创建Launch描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(slam_toolbox_node)
    
    return ld
