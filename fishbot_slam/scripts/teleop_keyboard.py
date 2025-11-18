#!/usr/bin/env python3
"""
键盘控制脚本
功能：通过键盘控制FishBot机器人移动进行建图
使用方法：
    i - 前进
    , - 后退
    j - 左转
    l - 右转
    k - 停止
    q/z - 增加/减少速度
    space - 紧急停止
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    """键盘遥控节点类"""
    
    # 键位到速度的映射
    KEY_BINDINGS = {
        'i': (1, 0, 0, 0),   # 前进
        'o': (1, 0, 0, -1),  # 前进+右转
        'u': (1, 0, 0, 1),   # 前进+左转
        ',': (-1, 0, 0, 0),  # 后退
        '.': (-1, 0, 0, 1),  # 后退+左转
        'm': (-1, 0, 0, -1), # 后退+右转
        'j': (0, 0, 0, 1),   # 原地左转
        'l': (0, 0, 0, -1),  # 原地右转
        'k': (0, 0, 0, 0),   # 停止
    }
    
    SPEED_BINDINGS = {
        'q': (1.1, 1.1),     # 增加速度
        'z': (0.9, 0.9),     # 减少速度
    }
    
    def __init__(self):
        """初始化节点"""
        super().__init__('teleop_keyboard')
        
        # 创建速度发布器
        self.publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        
        # 初始速度
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s
        
        # 当前速度
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        
        self.get_logger().info('键盘控制节点已启动')
        self.print_usage()
    
    def print_usage(self):
        """打印使用说明"""
        msg = """
键盘控制说明：
---------------------------
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

当前速度：
  线速度: {:.2f} m/s
  角速度: {:.2f} rad/s
---------------------------
        """.format(self.linear_speed, self.angular_speed)
        print(msg)
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_velocity(self):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.x * self.linear_speed
        twist.linear.y = self.y * self.linear_speed
        twist.linear.z = self.z * self.linear_speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th * self.angular_speed
        
        self.publisher.publish(twist)
    
    def run(self):
        """运行主循环"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while True:
                key = self.get_key()
                
                # 处理移动键
                if key in self.KEY_BINDINGS.keys():
                    self.x, self.y, self.z, self.th = self.KEY_BINDINGS[key]
                    self.publish_velocity()
                    self.get_logger().info(
                        f'速度 - 线速度: {self.x * self.linear_speed:.2f} m/s, '
                        f'角速度: {self.th * self.angular_speed:.2f} rad/s'
                    )
                
                # 处理速度调整键
                elif key in self.SPEED_BINDINGS.keys():
                    self.linear_speed *= self.SPEED_BINDINGS[key][0]
                    self.angular_speed *= self.SPEED_BINDINGS[key][1]
                    self.get_logger().info(
                        f'速度调整 - 线速度: {self.linear_speed:.2f} m/s, '
                        f'角速度: {self.angular_speed:.2f} rad/s'
                    )
                
                # 空格键紧急停止
                elif key == ' ':
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    self.publish_velocity()
                    self.get_logger().info('紧急停止!')
                
                # Ctrl+C退出
                elif key == '\x03':
                    break
                
                else:
                    # 未识别的键，停止运动
                    if key != '':
                        self.get_logger().warn(f'未识别的按键: {key}')
        
        except Exception as e:
            self.get_logger().error(f'错误: {str(e)}')
        
        finally:
            # 停止机器人
            twist = Twist()
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    teleop = TeleopKeyboard()
    teleop.run()
    
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
