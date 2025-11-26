#!/usr/bin/env python3
"""
A*导航规划启动文件
功能：启动A*路径规划器和路径跟踪控制器

作者: 欧林海
邮箱: franka907@126.com
日期: 2025年11月19日
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
    astar_params_file = os.path.join(
        pkg_fishbot_slam, 'config', 'astar_planner_params.yaml'
    )
    
    path_follower_params_file = os.path.join(
        pkg_fishbot_slam, 'config', 'path_follower_params.yaml'
    )
    
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 声明Launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )
    
    # A*路径规划器节点
    astar_planner_node = Node(
        package='fishbot_slam',
        executable='astar_planner.py',
        name='astar_planner',
        output='screen',
        parameters=[
            astar_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Pure Pursuit路径跟随器节点
    path_follower_node = Node(
        package='fishbot_slam',
        executable='path_follower.py',
        name='path_follower',
        output='screen',
        parameters=[
            path_follower_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz节点（用于可视化路径）
    rviz_config_file = os.path.join(pkg_fishbot_slam, 'config', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 创建Launch描述
    ld = LaunchDescription()
    
    # 添加动作
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(astar_planner_node)
    ld.add_action(path_follower_node)
    ld.add_action(rviz_node)
    
    return ld
