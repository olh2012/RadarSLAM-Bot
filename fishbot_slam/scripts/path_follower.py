#!/usr/bin/env python3
"""
路径跟踪控制器
功能：Pure Pursuit算法跟踪A*规划的路径

作者: 欧林海
邮箱: franka907@126.com
日期: 2025年11月19日

技术要点:
1. Pure Pursuit路径跟踪算法
2. 前视距离自适应
3. 速度控制
4. 路径点管理
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import numpy as np
from math import sqrt, atan2, cos, sin, pi


class PathFollower(Node):
    """Pure Pursuit路径跟踪器"""
    
    def __init__(self):
        super().__init__('path_follower')
        
        # 参数声明
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('min_lookahead_distance', 0.3)
        self.declare_parameter('max_lookahead_distance', 1.0)
        self.declare_parameter('lookahead_gain', 0.3)
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.1)
        
        # 获取参数
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.min_lookahead = self.get_parameter('min_lookahead_distance').value
        self.max_lookahead = self.get_parameter('max_lookahead_distance').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # 路径和状态
        self.path = None
        self.current_pose = None
        self.current_velocity = 0.0
        self.path_index = 0
        self.goal_reached = False
        
        # 订阅路径
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        # 订阅里程计
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )
        
        # 发布目标到达状态
        self.goal_reached_pub = self.create_publisher(Bool, '/goal_reached', 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('路径跟踪控制器已启动')
        self.get_logger().info(f'参数配置:')
        self.get_logger().info(f'  - 前视距离: {self.lookahead_distance}m')
        self.get_logger().info(f'  - 线速度: {self.linear_velocity}m/s')
        self.get_logger().info(f'  - 目标容差: {self.goal_tolerance}m')
    
    def path_callback(self, msg):
        """路径回调"""
        if len(msg.poses) > 0:
            self.path = msg
            self.path_index = 0
            self.goal_reached = False
            self.get_logger().info(f'收到新路径，包含 {len(msg.poses)} 个点')
        else:
            self.get_logger().warn('收到空路径')
    
    def odom_callback(self, msg):
        """里程计回调"""
        self.current_pose = msg.pose.pose
        self.current_velocity = sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
    
    def control_loop(self):
        """控制循环"""
        # 检查是否有路径和当前位置
        if self.path is None or self.current_pose is None:
            return
        
        # 检查是否已到达目标
        if self.goal_reached:
            self.stop_robot()
            return
        
        # 查找前视点
        lookahead_point = self.find_lookahead_point()
        
        if lookahead_point is None:
            # 检查是否到达终点
            goal_pose = self.path.poses[-1].pose
            distance_to_goal = self.distance_to_pose(goal_pose)
            
            if distance_to_goal < self.goal_tolerance:
                self.get_logger().info('已到达目标点！')
                self.goal_reached = True
                self.stop_robot()
                
                # 发布目标到达消息
                msg = Bool()
                msg.data = True
                self.goal_reached_pub.publish(msg)
            else:
                self.get_logger().warn('无法找到前视点')
                self.stop_robot()
            return
        
        # 计算控制命令
        cmd_vel = self.pure_pursuit_control(lookahead_point)
        
        # 发布速度命令
        self.cmd_vel_pub.publish(cmd_vel)
    
    def find_lookahead_point(self):
        """
        查找前视点
        
        Returns:
            前视点位姿或None
        """
        # 自适应前视距离
        adaptive_lookahead = self.lookahead_distance + self.lookahead_gain * self.current_velocity
        adaptive_lookahead = max(self.min_lookahead, min(adaptive_lookahead, self.max_lookahead))
        
        # 从当前索引开始搜索
        for i in range(self.path_index, len(self.path.poses)):
            pose = self.path.poses[i].pose
            distance = self.distance_to_pose(pose)
            
            if distance >= adaptive_lookahead:
                self.path_index = i
                return pose
        
        # 如果没有找到，返回最后一个点
        if len(self.path.poses) > 0:
            return self.path.poses[-1].pose
        
        return None
    
    def pure_pursuit_control(self, target_pose):
        """
        Pure Pursuit控制算法
        
        Args:
            target_pose: 目标位姿
        
        Returns:
            Twist消息
        """
        # 计算目标点在机器人坐标系中的位置
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        
        # 当前朝向
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # 转换到机器人坐标系
        local_x = dx * cos(current_yaw) + dy * sin(current_yaw)
        local_y = -dx * sin(current_yaw) + dy * cos(current_yaw)
        
        # 计算曲率
        distance = sqrt(local_x**2 + local_y**2)
        
        if distance < 0.01:
            # 距离太近，停止
            angular_velocity = 0.0
        else:
            # Pure Pursuit公式
            curvature = 2 * local_y / (distance * distance)
            angular_velocity = curvature * self.linear_velocity
            
            # 限制角速度
            angular_velocity = max(-self.max_angular_velocity, 
                                 min(angular_velocity, self.max_angular_velocity))
        
        # 创建速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        return cmd_vel
    
    def distance_to_pose(self, pose):
        """
        计算到目标位姿的距离
        
        Args:
            pose: 目标位姿
        
        Returns:
            距离
        """
        dx = pose.position.x - self.current_pose.position.x
        dy = pose.position.y - self.current_pose.position.y
        return sqrt(dx*dx + dy*dy)
    
    def get_yaw_from_quaternion(self, q):
        """
        从四元数获取yaw角
        
        Args:
            q: 四元数
        
        Returns:
            yaw角
        """
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)
    
    def stop_robot(self):
        """停止机器人"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    follower = PathFollower()
    
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
