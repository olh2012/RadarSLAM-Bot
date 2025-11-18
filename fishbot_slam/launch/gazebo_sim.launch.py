#!/usr/bin/env python3
"""
Gazebo仿真启动文件
功能：启动Gazebo仿真环境和FishBot机器人模型
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成Launch描述"""
    
    # 获取软件包路径
    pkg_fishbot_slam = get_package_share_directory('fishbot_slam')
    
    # 定义文件路径
    urdf_file = os.path.join(pkg_fishbot_slam, 'urdf', 'fishbot.urdf.xacro')
    world_file = os.path.join(pkg_fishbot_slam, 'worlds', 'fishbot_world.world')
    
    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    
    # 声明Launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时钟'
    )
    
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='是否启动Gazebo GUI'
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='是否以无头模式运行'
    )
    
    # 启动Gazebo服务器
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    # 启动Gazebo客户端
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gzclient'],
        output='screen'
    )
    
    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # 在Gazebo中生成机器人
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'fishbot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # 启动控制器管理器
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_fishbot_slam, 'config', 'diff_drive_controller.yaml')
        ],
        output='screen'
    )
    
    # 加载关节状态广播器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # 加载差速驱动控制器
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )
    
    # 创建Launch描述并填充
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gui_cmd)
    ld.add_action(declare_headless_cmd)
    
    # 添加节点
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(controller_manager_node)
    ld.add_action(load_joint_state_broadcaster)
    ld.add_action(load_diff_drive_controller)
    
    return ld
