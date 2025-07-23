"""
@file       dcam_pub_dist_sensor.py
@brief      将D435i深度相机的数据转换为PX4的距离传感器数据
@details    编写dcam_pub_dist_sensor节点，订阅d435i相机/camera/camera/depth/image_raw深度图话题
            计算中心区域的最小距离，并将其发布到PX4的/fmu/in/distance_sensor话题上
@author     周鑫鹏
@date       2025-07-24
@version    1.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from px4_msgs.msg import DistanceSensor
import cv_bridge
import numpy as np

class DcamPubDistSensor(Node):
    def __init__(self):
        super().__init__('dcam_pub_dist_sensor')

        # --- QoS Profile ---
        # 为传感器数据定义QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 参数定义 ---
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw') # D435i深度图像话题
        self.declare_parameter('distance_sensor_topic', '/fmu/in/distance_sensor')  # PX4距离传感器输入话题
        self.declare_parameter('min_clip_percent_width', 20)  # 图像宽度裁剪百分比 (左右裁减)
        self.declare_parameter('min_clip_percent_height', 20) # 图像高度裁剪百分比 (上下裁减)
        
        # 从参数服务器获取参数值
        depth_image_topic = self.get_parameter('depth_image_topic').get_parameter_value().string_value
        distance_sensor_topic = self.get_parameter('distance_sensor_topic').get_parameter_value().string_value
        self.min_clip_percent_width = self.get_parameter('min_clip_percent_width').get_parameter_value().integer_value
        self.min_clip_percent_height = self.get_parameter('min_clip_percent_height').get_parameter_value().integer_value

        # --- 初始化CV Bridge ---
        self.bridge = cv_bridge.CvBridge()

        # --- 创建订阅者 ---
        # 订阅来自D435i的深度图像
        self.subscription = self.create_subscription(
            Image,
            depth_image_topic,
            self.depth_image_callback,
            qos_profile
        )

        # --- 创建发布者 ---
        # 发布转换后的距离传感器消息给PX4
        self.publisher = self.create_publisher(
            DistanceSensor,
            distance_sensor_topic,
            10
        )


    def depth_image_callback(self, msg: Image):
        """
        处理收到的深度图像消息的回调函数。
        """
        try:
            # 将ROS Image消息转换为OpenCV图像格式（16位无符号整数，单位：毫米）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CV Bridge 转换失败: {e}')
            return

        # 获取图像尺寸
        height, width = depth_image.shape

        # --- 计算中心区域 ---
        # 避免边缘噪声，只关注图像中心区域的距离
        clip_width = int((width * self.min_clip_percent_width) / 100)
        clip_height = int((height * self.min_clip_percent_height) / 100)
        
        center_region = depth_image[clip_height:(height - clip_height), clip_width:(width - clip_width)]
        
        if center_region.size == 0:
            self.get_logger().warning('中心区域为空，请检查裁剪百分比参数。')
            return
            
        # --- 计算最小距离 ---
        # 查找中心区域中所有非零像素的最小值
        # 深度相机中，0通常表示无效读数（太近、太远或无返回）
        min_dist_mm = np.min(center_region[np.nonzero(center_region)])

        # 如果没有有效的距离读数，则不发布消息
        if min_dist_mm == 0:
            return

        # 将距离从毫米(mm)转换为米(m)
        min_dist_m = float(min_dist_mm) / 1000.0

        # --- 构建并发布DistanceSensor消息 ---
        dist_msg = DistanceSensor()
        dist_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # PX4期望微秒时间戳
        dist_msg.device_id = 0 # 可以为0，除非有多个同类传感器
        
        # D435i的有效测距范围，根据实际情况调整
        dist_msg.min_distance = 0.1  # 单位：米
        dist_msg.max_distance = 10.0 # 单位：米
        
        dist_msg.current_distance = min_dist_m
        dist_msg.variance = 0.0 # 未计算方差，设为0
        
        # 传感器方向。MAV_SENSOR_ROTATION_NONE表示前向
        dist_msg.orientation = DistanceSensor.ROTATION_PITCH_270 # 向前
        
        # 传感器类型。MAV_DISTANCE_SENSOR_LASER适用于大多数基于光的传感器
        dist_msg.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER 

        self.publisher.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DcamPubDistSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()