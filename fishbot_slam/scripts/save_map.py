#!/usr/bin/env python3
"""
地图保存脚本
功能：调用map_saver服务保存SLAM建立的地图
"""

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
import sys


class MapSaver(Node):
    """地图保存节点类"""
    
    def __init__(self):
        """初始化节点"""
        super().__init__('map_saver_node')
        
        # 创建服务客户端
        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待地图保存服务...')
        
        self.get_logger().info('地图保存服务已连接')
    
    def save_map(self, map_name='my_map'):
        """
        保存地图
        
        Args:
            map_name: 地图文件名（不包含扩展名）
        """
        # 创建请求
        request = SaveMap.Request()
        request.name = map_name
        
        # 发送异步请求
        self.get_logger().info(f'正在保存地图: {map_name}...')
        future = self.client.call_async(request)
        
        return future


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 获取地图名称参数
    map_name = 'fishbot_map'
    if len(sys.argv) > 1:
        map_name = sys.argv[1]
    
    # 创建地图保存节点
    map_saver = MapSaver()
    
    # 保存地图
    future = map_saver.save_map(map_name)
    
    # 等待服务响应
    rclpy.spin_until_future_complete(map_saver, future)
    
    if future.result() is not None:
        map_saver.get_logger().info(f'地图已成功保存为: {map_name}')
    else:
        map_saver.get_logger().error('地图保存失败!')
    
    # 清理
    map_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
