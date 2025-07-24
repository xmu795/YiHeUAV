"""
@file       circle_centre.py
@brief      圆环中心节点，发布圆环中心坐标
@details    编写circle_centre节点，用于订阅d435i相机的/camera/camera/color/image_raw颜色图话题和/camera/camera/depth/image_raw深度图话题
            并使用OpenCV进行圆环检测和深度转换，发布/goal/circle_centre圆环中心坐标话题
@note       连接到d435i相机，修改圆心深度坐标解算方式：获取圆环边缘的深度值并计算平均深度
@author     周鑫鹏
@date       2025-07-11
@version    3.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class CircleCentre(Node):
    def __init__(self):
        super().__init__('circle_centre')
        self.bridge = CvBridge()

        # 订阅颜色图和深度图话题
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_image_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10)

        # 发布圆环中心坐标话题
        self.circle_centre_pub = self.create_publisher(
            PointStamped,
            '/goal/circle_centre',
            10)

        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_received_first_time = False
        self.warning_timer = self.create_timer(1.0, self.check_camera_feed) # 每秒检查一次
        self.get_logger().info('Circle Centre Node has been started.')

    def color_image_callback(self, msg):
        try:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {e}")
        
        if not self.camera_received_first_time:
            self.get_logger().info('Successfully received first camera image.')
            self.camera_received_first_time = True

    def depth_image_callback(self, msg):
        try:
            # 16位的单通道图像深度图
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.process_images()
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
        
        if not self.camera_received_first_time:
            self.get_logger().info('Successfully received first camera image.')
            self.camera_received_first_time = True

    def check_camera_feed(self):
        if not self.camera_received_first_time:
            self.get_logger().warn('No camera feed received yet.')

    def process_images(self):
        if self.latest_color_image is None or self.latest_depth_image is None:
            return

        color_image = self.latest_color_image.copy()
        depth_image = self.latest_depth_image.copy()

        # 转换为灰度图
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # 高斯模糊
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # 使用HoughCircles检测圆环
        # dp: 累加器分辨率与图像分辨率的反比
        # minDist: 两个圆心之间的最小距离
        # param1: Canny边缘检测的高阈值
        # param2: 累加器阈值，越小越容易检测到圆
        # minRadius: 最小圆半径
        # maxRadius: 最大圆半径
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100,
                                   param1=100, param2=50, minRadius=20, maxRadius=200)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            # 只处理检测到的第一个圆
            x, y, r = circles[0]
            # 绘制圆环和圆心
            cv2.circle(color_image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(color_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            # 获取圆心处的深度值
            # 确保坐标在图像范围内
            if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                # 获取圆环边缘的深度值
                edge_depths = []
                num_samples = 36  # 采样点数量
                for i in range(num_samples):
                    angle = 2 * np.pi * i / num_samples
                    # 计算边缘点的坐标
                    ex = int(x + r * np.cos(angle))
                    ey = int(y + r * np.sin(angle))

                    # 确保边缘点在图像范围内
                    if 0 <= ey < depth_image.shape[0] and 0 <= ex < depth_image.shape[1]:
                        edge_depths.append(depth_image[ey, ex])
                
                if edge_depths:
                    # 过滤掉无效深度值（0）并计算平均深度
                    valid_depths = [d for d in edge_depths if d > 0]
                    if valid_depths:
                        depth_value = np.mean(valid_depths)
                        # 深度值通常以毫米为单位，转换为米
                        depth_in_meters = depth_value / 1000.0
                    else:
                        self.get_logger().warn(f"No valid depth values found for circle edges.")
                        return
                else:
                    self.get_logger().warn(f"No edge points found within depth image bounds.")
                    return

                # 相机内参
                fx = 612.0610961914062  # 焦距
                fy = 612.2150268554688  # 焦距
                cx = 319.97662353515625 # 主点
                cy = 248.469970703125   # 主点

                # 将像素坐标转换为相机坐标系下的3D坐标
                point_x = (x - cx) * depth_in_meters / fx
                point_y = (y - cy) * depth_in_meters / fy
                point_z = depth_in_meters

                # 创建PointStamped消息并发布
                circle_centre_msg = PointStamped()
                circle_centre_msg.header.stamp = self.get_clock().now().to_msg()
                
                # 将d435i相机坐标系下的坐标转换为ROS base_link坐标系
                # d435i: x轴向右，y轴向下，z轴向前
                # base_link: x轴向前，y轴向左，z轴向上
                base_link_x = point_z
                base_link_y = -point_x
                base_link_z = -point_y
                circle_centre_msg.point.x = float(base_link_x)
                circle_centre_msg.point.y = float(base_link_y)
                circle_centre_msg.point.z = float(base_link_z)
                self.circle_centre_pub.publish(circle_centre_msg)
                self.get_logger().info(f"Detected circle centre (3D): X={base_link_x:.4f}, Y={base_link_y:.4f}, Z={base_link_z:.4f}")
            
            else:
                self.get_logger().warn(f"Circle center ({x},{y}) is out of depth image bounds.")
        
        # 显示处理后的图像，用于调试
        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CircleCentre()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()