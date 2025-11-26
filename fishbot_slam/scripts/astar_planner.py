#!/usr/bin/env python3
"""
A*路径规划节点
功能：基于占据栅格地图使用A*算法进行全局路径规划

作者: 欧林海
邮箱: franka907@126.com
日期: 2025年11月19日

技术要点:
1. A*启发式搜索算法
2. 占据栅格地图处理
3. 路径平滑优化
4. ROS2服务接口
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Empty
import numpy as np
import heapq
from math import sqrt, atan2, cos, sin


class AStarNode:
    """A*算法节点类"""
    
    def __init__(self, x, y, g=0, h=0, parent=None):
        self.x = x
        self.y = y
        self.g = g  # 从起点到当前节点的实际代价
        self.h = h  # 从当前节点到终点的启发式估计代价
        self.f = g + h  # 总代价
        self.parent = parent
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


class AStarPlanner(Node):
    """A*路径规划器"""
    
    def __init__(self):
        super().__init__('astar_planner')
        
        # 参数声明
        self.declare_parameter('diagonal_movement', True)
        self.declare_parameter('smoothing_iterations', 5)
        self.declare_parameter('obstacle_inflation', 2)
        self.declare_parameter('occupancy_threshold', 50)
        
        # 获取参数
        self.diagonal_movement = self.get_parameter('diagonal_movement').value
        self.smoothing_iterations = self.get_parameter('smoothing_iterations').value
        self.obstacle_inflation = self.get_parameter('obstacle_inflation').value
        self.occupancy_threshold = self.get_parameter('occupancy_threshold').value
        
        # 地图数据
        self.map = None
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # 订阅地图
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # 发布路径
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # 订阅目标点
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # 当前位置
        self.current_pose = None
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            10
        )
        
        self.get_logger().info('A*路径规划器已启动')
        self.get_logger().info(f'参数配置:')
        self.get_logger().info(f'  - 对角线移动: {self.diagonal_movement}')
        self.get_logger().info(f'  - 路径平滑迭代: {self.smoothing_iterations}')
        self.get_logger().info(f'  - 障碍物膨胀: {self.obstacle_inflation}格')
    
    def map_callback(self, msg):
        """地图回调函数"""
        self.map = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
        # 转换为numpy数组
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        # 障碍物膨胀
        if self.obstacle_inflation > 0:
            self.map_data = self.inflate_obstacles(self.map_data)
        
        self.get_logger().info(f'地图已更新: {self.map_width}x{self.map_height}, 分辨率: {self.map_resolution}m')
    
    def current_pose_callback(self, msg):
        """当前位置回调"""
        self.current_pose = msg
    
    def goal_callback(self, msg):
        """目标点回调，触发路径规划"""
        if self.map_data is None:
            self.get_logger().warn('地图数据未准备好，无法规划路径')
            return
        
        if self.current_pose is None:
            self.get_logger().warn('当前位置未知，无法规划路径')
            return
        
        # 世界坐标转换为栅格坐标
        start_x, start_y = self.world_to_grid(
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y
        )
        
        goal_x, goal_y = self.world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        self.get_logger().info(f'开始路径规划: ({start_x}, {start_y}) -> ({goal_x}, {goal_y})')
        
        # 执行A*搜索
        path = self.astar_search((start_x, start_y), (goal_x, goal_y))
        
        if path:
            self.get_logger().info(f'路径规划成功！路径长度: {len(path)} 个点')
            
            # 路径平滑
            if self.smoothing_iterations > 0:
                path = self.smooth_path(path)
                self.get_logger().info(f'路径平滑完成，优化后长度: {len(path)} 个点')
            
            # 发布路径
            self.publish_path(path)
        else:
            self.get_logger().error('路径规划失败！无法找到可行路径')
    
    def astar_search(self, start, goal):
        """
        A*搜索算法
        
        Args:
            start: 起点坐标 (grid_x, grid_y)
            goal: 终点坐标 (grid_x, grid_y)
        
        Returns:
            路径点列表或None
        """
        # 检查起点和终点是否有效
        if not self.is_valid_cell(start[0], start[1]) or \
           not self.is_valid_cell(goal[0], goal[1]):
            self.get_logger().error('起点或终点无效')
            return None
        
        # 开放列表和闭合列表
        open_list = []
        closed_set = set()
        
        # 创建起点节点
        start_node = AStarNode(start[0], start[1], 0, self.heuristic(start, goal))
        heapq.heappush(open_list, start_node)
        
        # 用于快速查找的字典
        open_dict = {start: start_node}
        
        # 移动方向
        if self.diagonal_movement:
            # 8方向移动
            movements = [
                (0, 1, 1.0),    # 上
                (1, 0, 1.0),    # 右
                (0, -1, 1.0),   # 下
                (-1, 0, 1.0),   # 左
                (1, 1, 1.414),  # 右上
                (1, -1, 1.414), # 右下
                (-1, -1, 1.414),# 左下
                (-1, 1, 1.414)  # 左上
            ]
        else:
            # 4方向移动
            movements = [
                (0, 1, 1.0),
                (1, 0, 1.0),
                (0, -1, 1.0),
                (-1, 0, 1.0)
            ]
        
        # A*主循环
        while open_list:
            # 获取f值最小的节点
            current = heapq.heappop(open_list)
            current_pos = (current.x, current.y)
            
            # 从开放字典中移除
            if current_pos in open_dict:
                del open_dict[current_pos]
            
            # 到达终点
            if current_pos == goal:
                return self.reconstruct_path(current)
            
            # 添加到闭合集
            closed_set.add(current_pos)
            
            # 遍历所有可能的移动
            for dx, dy, cost in movements:
                next_x = current.x + dx
                next_y = current.y + dy
                next_pos = (next_x, next_y)
                
                # 检查是否越界或在障碍物上
                if not self.is_valid_cell(next_x, next_y):
                    continue
                
                # 如果已在闭合集中，跳过
                if next_pos in closed_set:
                    continue
                
                # 计算新的g值
                new_g = current.g + cost
                
                # 检查是否在开放列表中
                if next_pos in open_dict:
                    neighbor = open_dict[next_pos]
                    if new_g < neighbor.g:
                        # 找到更好的路径，更新
                        neighbor.g = new_g
                        neighbor.f = new_g + neighbor.h
                        neighbor.parent = current
                        heapq.heapify(open_list)
                else:
                    # 创建新节点
                    h = self.heuristic(next_pos, goal)
                    neighbor = AStarNode(next_x, next_y, new_g, h, current)
                    heapq.heappush(open_list, neighbor)
                    open_dict[next_pos] = neighbor
        
        # 没有找到路径
        return None
    
    def heuristic(self, pos1, pos2):
        """
        启发式函数 (欧几里得距离)
        
        Args:
            pos1: 位置1 (x, y)
            pos2: 位置2 (x, y)
        
        Returns:
            估计距离
        """
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        
        if self.diagonal_movement:
            # 对角距离
            return sqrt(dx * dx + dy * dy)
        else:
            # 曼哈顿距离
            return dx + dy
    
    def is_valid_cell(self, x, y):
        """
        检查栅格是否有效（在地图内且非障碍物）
        
        Args:
            x, y: 栅格坐标
        
        Returns:
            布尔值
        """
        # 检查边界
        if x < 0 or x >= self.map_width or y < 0 or y >= self.map_height:
            return False
        
        # 检查占据状态
        # OccupancyGrid: -1=未知, 0=自由, 100=占据
        cell_value = self.map_data[y, x]
        
        if cell_value == -1:  # 未知区域
            return False
        
        if cell_value > self.occupancy_threshold:  # 障碍物
            return False
        
        return True
    
    def reconstruct_path(self, node):
        """
        重建路径
        
        Args:
            node: 终点节点
        
        Returns:
            路径点列表 [(x, y), ...]
        """
        path = []
        current = node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        
        path.reverse()
        return path
    
    def smooth_path(self, path):
        """
        路径平滑优化（使用梯度下降法）
        
        Args:
            path: 原始路径
        
        Returns:
            平滑后的路径
        """
        if len(path) <= 2:
            return path
        
        # 转换为numpy数组
        smooth_path = np.array(path, dtype=float)
        
        # 权重参数
        weight_data = 0.5      # 保持接近原始路径
        weight_smooth = 0.3    # 平滑程度
        
        # 迭代优化
        for _ in range(self.smoothing_iterations):
            for i in range(1, len(smooth_path) - 1):
                # 保持接近原始路径
                smooth_path[i] += weight_data * (path[i] - smooth_path[i])
                
                # 平滑（向相邻点的平均值移动）
                smooth_path[i] += weight_smooth * (
                    smooth_path[i-1] + smooth_path[i+1] - 2 * smooth_path[i]
                )
        
        # 转回整数坐标
        return [(int(round(x)), int(round(y))) for x, y in smooth_path]
    
    def inflate_obstacles(self, grid_map):
        """
        障碍物膨胀（增加安全距离）
        
        Args:
            grid_map: 原始地图
        
        Returns:
            膨胀后的地图
        """
        inflated_map = grid_map.copy()
        
        # 找到所有障碍物
        obstacles = np.argwhere(grid_map > self.occupancy_threshold)
        
        # 膨胀
        for obs_y, obs_x in obstacles:
            for dy in range(-self.obstacle_inflation, self.obstacle_inflation + 1):
                for dx in range(-self.obstacle_inflation, self.obstacle_inflation + 1):
                    ny, nx = obs_y + dy, obs_x + dx
                    if 0 <= ny < self.map_height and 0 <= nx < self.map_width:
                        # 使用距离作为权重
                        dist = sqrt(dx*dx + dy*dy)
                        if dist <= self.obstacle_inflation:
                            inflated_map[ny, nx] = max(
                                inflated_map[ny, nx],
                                int(100 * (1 - dist / self.obstacle_inflation))
                            )
        
        return inflated_map
    
    def world_to_grid(self, world_x, world_y):
        """
        世界坐标转栅格坐标
        
        Args:
            world_x, world_y: 世界坐标
        
        Returns:
            (grid_x, grid_y)
        """
        grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """
        栅格坐标转世界坐标
        
        Args:
            grid_x, grid_y: 栅格坐标
        
        Returns:
            (world_x, world_y)
        """
        world_x = grid_x * self.map_resolution + self.map_origin_x
        world_y = grid_y * self.map_resolution + self.map_origin_y
        return world_x, world_y
    
    def publish_path(self, path):
        """
        发布路径消息
        
        Args:
            path: 路径点列表
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for grid_x, grid_y in path:
            # 转换为世界坐标
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            
            # 计算朝向（指向下一个点）
            if len(path_msg.poses) > 0:
                prev_pose = path_msg.poses[-1]
                dx = world_x - prev_pose.pose.position.x
                dy = world_y - prev_pose.pose.position.y
                yaw = atan2(dy, dx)
                
                # 转换为四元数
                pose.pose.orientation.z = sin(yaw / 2)
                pose.pose.orientation.w = cos(yaw / 2)
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'路径已发布，包含 {len(path_msg.poses)} 个位姿点')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    planner = AStarPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
