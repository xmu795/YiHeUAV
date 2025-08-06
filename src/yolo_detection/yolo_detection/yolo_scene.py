#!/usr/bin/env python3

"""
@file       yolo_scene.py
@brief      使用YOLOv11模型检测场景类别和中心坐标，并发布相关话题
@details    订阅usb相机话题/image_raw，通过本地yolov11和权重文件检测场景
            发布场景类别名称话题/yolo/scene/name和ID话题/yolo/scene/id
            发布场景中心坐标话题/yolo/scene/centre和长宽比话题/yolo/scene/aspect_ratio
@note       场景中心坐标为相对相机主点RDF坐标系
@author     FallThrive
@date       2025-08-06
@version    4.0
"""

import sys
import os
# conda_env_path = os.environ.get('CONDA_PREFIX')
# python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
# if conda_env_path:
#     site_packages_path = os.path.join(conda_env_path, 'lib', python_version, 'site-packages')
#     if site_packages_path not in sys.path:
#         sys.path.append(site_packages_path)
#     else:
#         raise

import torch
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
    
class YOLOSceneNode(Node):
    def __init__(self):
        super().__init__('yolo_scene')
        
        # 获取包路径
        package_share_directory = get_package_share_directory('yolo_detection')
        weights_path = os.path.join(package_share_directory, 'weights', 'yolov11m_yihe.pt')
        
        # 检查权重文件是否存在
        if not os.path.exists(weights_path):
            self.get_logger().error(f'Weights file not found: {weights_path}')
            return
            
        # 加载YOLOv11模型
        try:
            self.model = YOLO(weights_path)
            # 获取类别名称
            self.names = self.model.names if hasattr(self.model, 'names') else []
            
            self.get_logger().info('Successfully loaded YOLOv11 model using ultralytics package')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv11 model: {str(e)}')
            return

        # 相机内参矩阵 (硬编码)
        self.camera_matrix = np.array([
            [512.66309, 0,          346.74937],
            [0,         513.6353,   329.43823],
            [0,         0,          1]
        ])
        
        # 已知实物尺寸（硬编码）
        self.real_width = 0.45
        self.real_height = 0.45
        
        # 初始化图像桥接
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
            
        # 发布话题
        self.category_publisher = self.create_publisher(
            String,
            '/yolo/scene/name',
            10)
            
        self.category_id_publisher = self.create_publisher(
            Int32,
            '/yolo/scene/id',
            10)
            
        self.centre_publisher = self.create_publisher(
            PointStamped,
            '/yolo/scene/centre',
            10)
            
        self.aspect_ratio_publisher = self.create_publisher(
            Float32,
            '/yolo/scene/aspect_ratio',
            10)
            
        # 控制处理频率（每秒2帧）
        self.timer_period = 0.5  # 0.5秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 存储最新图像
        self.latest_image = None
        self.image_lock = False
        
        self.get_logger().info('YOLO Scene Node has been started.')

    def image_callback(self, msg):
        # 仅在未处理图像时保存最新图像
        if not self.image_lock:
            self.latest_image = msg

    def timer_callback(self):
        # 如果没有图像则返回
        if self.latest_image is None:
            return
            
        # 锁定图像处理
        self.image_lock = True
        
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            
            # 使用YOLOv11进行目标检测
            results = self.model(cv_image, verbose=False)
            
            # 发布场景类别
            category_name_msg = String()
            category_id_msg = Int32()
            
            # 处理检测结果
            if len(results) > 0:
                # 获取检测结果
                detections = results[0].boxes
                
                if detections is not None and len(detections) > 0:
                    # 只处理置信度最高的一个检测框
                    best_detection = detections[0]
                    for det in detections:
                        if det.conf > best_detection.conf:  # 比较置信度
                            best_detection = det
                    
                    # 获取边界框坐标
                    boxes = best_detection.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = boxes
                    
                    # 计算物体中心点
                    x_centre = (x1 + x2) / 2.0
                    y_centre = (y1 + y2) / 2.0
                    
                    # 计算检测框的宽度和高度
                    width = x2 - x1
                    height = y2 - y1
                    
                    # 计算长宽比
                    aspect_ratio = width / height if height != 0 else 0
                    
                    # 计算相对坐标
                    relative_position = self.calculate_relative_position(
                        x_centre, y_centre, width, height
                    )
                    
                    # 获取检测类别名称和ID
                    scene_name = self.names[int(best_detection.cls)]
                    scene_id = int(best_detection.cls)
                    
                    # 设置消息内容
                    category_name_msg.data = scene_name
                    category_id_msg.data = scene_id
                    
                    # 发布中心点坐标
                    centre_point = PointStamped()
                    centre_point.header.stamp = self.get_clock().now().to_msg()
                    centre_point.header.frame_id = "camera_link"
                    centre_point.point.x = relative_position[0]
                    centre_point.point.y = relative_position[1]
                    centre_point.point.z = relative_position[2]
                    self.centre_publisher.publish(centre_point)
                    
                    # 发布长宽比
                    aspect_ratio_msg = Float32()
                    aspect_ratio_msg.data = float(aspect_ratio)
                    self.aspect_ratio_publisher.publish(aspect_ratio_msg)
                    
                    self.get_logger().info(
                        f'Category: {scene_name}, ID: {scene_id}, '
                        f'Position: ({relative_position[0]:.2f}, {relative_position[1]:.2f}, {relative_position[2]:.2f}), '
                        f'Aspect Ratio: {aspect_ratio:.2f}'
                    )
            
            # 发布检测结果
            self.category_publisher.publish(category_name_msg)
            self.category_id_publisher.publish(category_id_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
        finally:
            # 解锁
            self.image_lock = False

    def calculate_relative_position(self, x_centre, y_centre, width_pixels, height_pixels):
        """
        根据像素坐标和物体尺寸计算相对于相机的3D坐标
        这里相对相机主点RDF坐标系，后续需要坐标变换调试
        
        Args:
            x_centre: 物体中心x坐标（像素）
            y_centre: 物体中心y坐标（像素）
            width_pixels: 物体宽度（像素）
            height_pixels: 物体高度（像素）
            
        Returns:
            相对于相机的3D坐标 [x, y, z]（单位：米）
        """
        # 根据物体在图像中的像素尺寸计算距离
        # 实际尺寸/距离 = 像素尺寸/焦距
        focal_length_x = self.camera_matrix[0, 0]
        focal_length_y = self.camera_matrix[1, 1]
        
        # 通过宽度和高度分别计算距离，然后取平均值
        distance_by_width = (self.real_width * focal_length_x) / width_pixels
        distance_by_height = (self.real_height * focal_length_y) / height_pixels
        z_distance = (distance_by_width + distance_by_height) / 2.0
        
        # 计算相机坐标系中的x, y坐标
        # 使用针孔相机模型: x = (u - cx) * z / fx, y = (v - cy) * z / fy
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        x_position = (x_centre - cx) * z_distance / focal_length_x
        y_position = (y_centre - cy) * z_distance / focal_length_y
        
        return [x_position, y_position, z_distance]


def main(args=None):
    rclpy.init(args=args)
    
    node = YOLOSceneNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()