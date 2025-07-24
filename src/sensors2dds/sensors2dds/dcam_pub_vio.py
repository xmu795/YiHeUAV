"""
@file       dcam_pub_vio.py
@brief      桥接器节点，将VINS算法输出的视觉里程计数据转换为PX4可接受的格式
@details    订阅VINS发布的里程计话题/vins_estimator/odometry
            将收到的Odometry消息转换为VehicleVisualOdometry消息
            发布到PX4的/fmu/in/vehicle_visual_odometry话题
@note       需要确保VINS系统输出的坐标系与PX4期望的FRD（前-右-下）坐标系对齐
@author     周鑫鹏
@date       2025-07-24
@version    1.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleVisualOdometry
import numpy as np

class DcamPubVIO(Node):
    """
    节点作为桥接器，将VINS算法输出的视觉里程计数据转换为PX4可接受的格式
    """
    def __init__(self):
        super().__init__('dcam_pub_vio')

        # --- QoS Profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 参数定义 ---
        # VINS-Fusion默认发布到/vins_estimator/odometry话题
        self.declare_parameter('odom_topic', '/vins_estimator/odometry') 
        self.declare_parameter('visual_odometry_topic', '/fmu/in/vehicle_visual_odometry')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        visual_odometry_topic = self.get_parameter('visual_odometry_topic').get_parameter_value().string_value

        # --- 创建订阅者 ---
        # 订阅来自VINS系统的里程计信息
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odometry_callback,
            qos_profile
        )

        # --- 创建发布者 ---
        # 发布转换后的视觉里程计消息给PX4
        self.publisher = self.create_publisher(
            VehicleVisualOdometry,
            visual_odometry_topic,
            10
        )

    def odometry_callback(self, msg: Odometry):
        """
        处理收到的里程计消息的回调函数
        """
        vio_msg = VehicleVisualOdometry()

        # --- 时间戳 ---
        vio_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # 当前时间
        vio_msg.timestamp_sample = int(msg.header.stamp.sec * 1000000 + msg.header.stamp.nanosec / 1000) # 样本时间

        # --- 坐标系定义 ---
        # WARN: 需要上游的VINS节点已经将坐标系处理为FRD
        vio_msg.pose_frame = VehicleVisualOdometry.POSE_FRAME_FRD
        vio_msg.velocity_frame = VehicleVisualOdometry.VELOCITY_FRAME_FRD

        # --- 位置和姿态 ---
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        vio_msg.position[0] = position.x
        vio_msg.position[1] = position.y
        vio_msg.position[2] = position.z
        
        vio_msg.q[0] = orientation.w
        vio_msg.q[1] = orientation.x
        vio_msg.q[2] = orientation.y
        vio_msg.q[3] = orientation.z

        # --- 速度 ---
        linear_velocity = msg.twist.twist.linear
        angular_velocity = msg.twist.twist.angular
        vio_msg.velocity[0] = linear_velocity.x
        vio_msg.velocity[1] = linear_velocity.y
        vio_msg.velocity[2] = linear_velocity.z
        
        vio_msg.angular_velocity[0] = angular_velocity.x
        vio_msg.angular_velocity[1] = angular_velocity.y
        vio_msg.angular_velocity[2] = angular_velocity.z

        # --- 协方差 ---
        pose_covariance = msg.pose.covariance
        twist_covariance = msg.twist.covariance
        
        vio_msg.position_variance[0] = pose_covariance[0]
        vio_msg.position_variance[1] = pose_covariance[7]
        vio_msg.position_variance[2] = pose_covariance[14]

        vio_msg.orientation_variance[0] = pose_covariance[21]
        vio_msg.orientation_variance[1] = pose_covariance[28]
        vio_msg.orientation_variance[2] = pose_covariance[35]

        vio_msg.velocity_variance[0] = twist_covariance[0]
        vio_msg.velocity_variance[1] = twist_covariance[7]
        vio_msg.velocity_variance[2] = twist_covariance[14]

        self.publisher.publish(vio_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DcamPubVIO()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()