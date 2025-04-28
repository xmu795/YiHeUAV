#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RobotController(Node):
    """简单的机器人控制节点，用于测试机器人移动"""
    
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('机器人控制器已启动')
        
    def move_forward(self, speed=0.2, duration=3.0):
        """控制机器人前进"""
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        self.get_logger().info(f'前进，速度: {speed}m/s, 持续时间: {duration}秒')
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()
        
    def turn(self, angular_speed=0.5, duration=3.0):
        """控制机器人旋转"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        
        self.get_logger().info(f'旋转，角速度: {angular_speed}rad/s, 持续时间: {duration}秒')
        self.publisher.publish(twist)
        time.sleep(duration)
        self.stop()
        
    def stop(self):
        """停止机器人"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        self.get_logger().info('停止')
        self.publisher.publish(twist)

def main():
    rclpy.init()
    controller = RobotController()
    
    try:
        # 执行一系列移动测试
        controller.get_logger().info('开始测试机器人移动...')
        
        # 前进3秒
        controller.move_forward(0.2, 3.0)
        time.sleep(1.0)
        
        # 原地旋转3秒
        controller.turn(0.5, 3.0)
        time.sleep(1.0)
        
        # 后退3秒
        controller.move_forward(-0.2, 3.0)
        time.sleep(1.0)
        
        # 反向旋转3秒
        controller.turn(-0.5, 3.0)
        
        controller.get_logger().info('移动测试完成')
        
    except KeyboardInterrupt:
        controller.get_logger().info('用户中断测试')
    
    # 确保机器人停止
    controller.stop()
    
    # 清理
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 