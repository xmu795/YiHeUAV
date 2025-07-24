"""
@file       door_centre.py
@brief      空心门中心节点，发布空心门中心坐标
@details    编写door_centre节点，用于订阅d435i相机的/camera/camera/depth/image_rect_raw深度图话题
            并使用基于深度梯度的方法进行空心门检测和深度转换，发布/goal/door_centre空心门中心坐标话题
@note       尚未连接到d435i相机进行部署测试
@author     周鑫鹏
@date       2025-07-24
@version    1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class DoorCentre(Node):
    def __init__(self):
        super().__init__('door_centre')
        self.bridge = CvBridge()

        # 只订阅深度图话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10)

        # 发布空心门中心坐标话题
        self.door_centre_pub = self.create_publisher(
            PointStamped,
            '/goal/door_centre',
            10)

        self.latest_depth_image = None
        self.camera_received_first_time = False
        self.warning_timer = self.create_timer(1.0, self.check_camera_feed) # 每秒检查一次
        self.get_logger().info('Door Centre Node has been started.')

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
        if self.latest_depth_image is None:
            return

        depth_image = self.latest_depth_image.copy()
        
        # 将深度图转换为浮点型以便计算梯度
        depth_float = depth_image.astype(np.float32)
        
        # 计算梯度
        grad_x = cv2.Sobel(depth_float, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(depth_float, cv2.CV_32F, 0, 1, ksize=3)
        
        # 计算梯度幅值
        grad_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # 应用阈值以突出显著的深度变化
        _, thresh = cv2.threshold(grad_magnitude, 100, 255, cv2.THRESH_BINARY)
        thresh = thresh.astype(np.uint8)
        
        # 形态学操作以连接边缘
        kernel = np.ones((5,5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 如果找到轮廓
        if len(contours) > 0:
            # 过滤掉太小的轮廓
            min_area = 100
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            
            if len(valid_contours) >= 2:
                # 计算每个轮廓的边界框中心点x坐标
                contour_centers = []
                for cnt in valid_contours:
                    x, y, w, h = cv2.boundingRect(cnt)
                    center_x = x + w // 2
                    contour_centers.append((center_x, x, y, w, h))
                
                # 按照x坐标排序
                contour_centers.sort(key=lambda x: x[0])
                
                # 选择最左侧和最右侧的轮廓作为门的两侧边框
                left_contour = contour_centers[0]
                right_contour = contour_centers[-1]
                
                # 计算门的中心x坐标
                center_x = (left_contour[0] + right_contour[0]) // 2
                
                # 使用左侧或右侧轮廓的y坐标和高度作为门的整体y坐标和高度
                # 这里选择左侧轮廓的信息
                _, x_left, y_left, w_left, h_left = left_contour
                center_y = y_left + h_left // 2
                
                # 获取中心点处的深度值
                # 确保坐标在图像范围内
                if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                    # 获取中心点周围的深度值
                    depth_roi = depth_image[max(0, center_y-5):min(depth_image.shape[0], center_y+5),
                                        max(0, center_x-5):min(depth_image.shape[1], center_x+5)]
                    
                    # 过滤掉无效深度值（0）并计算平均深度
                    valid_depths = depth_roi[depth_roi > 0]
                    if valid_depths.size > 0:
                        depth_value = np.mean(valid_depths)
                        # 深度值通常以毫米为单位，转换为米
                        depth_in_meters = depth_value / 1000.0
                    else:
                        self.get_logger().warn(f"No valid depth values found for door center.")
                        return
                else:
                    self.get_logger().warn(f"Door center ({center_x},{center_y}) is out of depth image bounds.")
                    return

                # 相机内参 (需要根据实际相机参数调整)
                fx = 612.0610961914062  # 焦距
                fy = 612.2150268554688  # 焦距
                cx = 319.97662353515625 # 主点
                cy = 248.469970703125   # 主点

                # 将像素坐标转换为相机坐标系下的3D坐标
                point_x = (center_x - cx) * depth_in_meters / fx
                point_y = (center_y - cy) * depth_in_meters / fy
                point_z = depth_in_meters

                # 创建PointStamped消息并发布
                door_centre_msg = PointStamped()
                door_centre_msg.header.stamp = self.get_clock().now().to_msg()
                
                # 将d435i相机坐标系下的坐标转换为ROS base_link坐标系(FLU)
                # d435i: x轴向右，y轴向下，z轴向前
                # base_link (FLU): x轴向前，y轴向左，z轴向上
                base_link_x = point_z
                base_link_y = -point_x
                # 无人机从竖直中轴线穿过即可，实际飞行高度设置好后保持即可，因此z坐标设为0
                base_link_z = 0.0
                door_centre_msg.point.x = float(base_link_x)
                door_centre_msg.point.y = float(base_link_y)
                door_centre_msg.point.z = float(base_link_z)
                self.door_centre_pub.publish(door_centre_msg)
                self.get_logger().info(f"Detected door centre (3D): X={base_link_x:.4f}, Y={base_link_y:.4f}, Z={base_link_z:.4f}")
                
                # 用于调试的可视化
                # 绘制边界矩形和中心点
                color_debug = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
                # 绘制左侧轮廓
                cv2.rectangle(color_debug, (x_left, y_left), (x_left + w_left, y_left + h_left), (0, 255, 0), 2)
                # 绘制右侧轮廓（需要重新计算坐标）
                for cnt_center in contour_centers:
                    if cnt_center[0] == right_contour[0]:
                        _, x_right, y_right, w_right, h_right = cnt_center
                        cv2.rectangle(color_debug, (x_right, y_right), (x_right + w_right, y_right + h_right), (0, 255, 0), 2)
                        break
                # 绘制中心点
                cv2.circle(color_debug, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.imshow("Door Detection Debug", color_debug)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DoorCentre()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()