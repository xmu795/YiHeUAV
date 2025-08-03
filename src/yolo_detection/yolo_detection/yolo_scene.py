#!/usr/bin/env python3

"""
@file       yolo_scene.py
@brief      使用YOLOv5模型检测场景类别和中心坐标，并发布相关话题
@details    订阅usb相机话题/image_raw，通过本地yolov5和权重文件检测场景
            发布场景类别名称话题/yolo/scene/name和ID话题/yolo/scene/name
            发布场景中心坐标话题/yolo/scene/centre
@note       场景中心坐标为相对相机主点RDF坐标系
@author     FallThrive
@date       2025-08-03
@version    2.2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
import sys
import os
from pathlib import Path

try:
    import torch
except ImportError:
    # 如果直接导入失败，尝试添加conda环境路径
    conda_env_path = os.environ.get('CONDA_PREFIX')
    python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    if conda_env_path:
        site_packages_path = os.path.join(conda_env_path, 'lib', python_version, 'site-packages')
        if site_packages_path not in sys.path:
            sys.path.append(site_packages_path)
        import torch
    else:
        raise
    
class YOLOSceneNode(Node):
    def __init__(self):
        super().__init__('yolo_scene')
        
        # 获取包路径
        package_share_directory = get_package_share_directory('yolo_detection')
        weights_path = os.path.join(package_share_directory, 'weights', 'yolov5m_yihe_1.0.pt')
        
        # 检查权重文件是否存在
        if not os.path.exists(weights_path):
            self.get_logger().error(f'Weights file not found: {weights_path}')
            return
            
        # 添加本地yolov5到系统路径
        local_yolov5_path = Path(os.path.expanduser('~/source/yolov5')) # 这里采用绝对路径硬编码，需要根据实际路径修改
        if str(local_yolov5_path) not in sys.path:
            sys.path.insert(0, str(local_yolov5_path))
        
        # 加载YOLOv5模型
        try:
            import models
            from models.common import DetectMultiBackend
            from utils.general import check_img_size
            from utils.torch_utils import select_device
            
            device = select_device('')
            self.model = DetectMultiBackend(weights_path, device=device, dnn=False, data=None, fp16=False)
            self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
            self.imgsz = check_img_size((640, 640), s=self.stride)  # check image size
            
            self.get_logger().info(f'Successfully loaded YOLOv5 model from local repository: {local_yolov5_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOv5 model from local repository: {str(e)}')
            return

        # 相机内参矩阵 (硬编码)
        self.camera_matrix = np.array([
            [520.03381, 0,          330.2527 ],
            [0,         518.90635,  330.02298],
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
            
            # 使用YOLOv5进行目标检测
            import torch
            from utils.general import non_max_suppression, scale_boxes
            from utils.augmentations import letterbox
            
            # 预处理图像
            img = letterbox(cv_image, self.imgsz, stride=self.stride, auto=self.pt)[0]
            img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
            img = np.ascontiguousarray(img)
            
            # 转换为torch张量
            img = torch.from_numpy(img).to(self.model.device)
            img = img.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
            img /= 255  # 0 - 255 to 0.0 - 1.0
            if len(img.shape) == 3:
                img = img[None]  # expand for batch dim
            
            # 推理
            pred = self.model(img, augment=False, visualize=False)
            
            # NMS
            pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False, max_det=1000)
            
            # 发布场景类别
            category_name_msg = String()
            category_id_msg = Int32()
            
            # 处理检测结果
            if len(pred[0]):
                # Rescale boxes from img_size to im0 size
                det = pred[0].clone()
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], cv_image.shape).round()
                
                # 获取检测结果 (xyxy格式: [x1, y1, x2, y2, confidence, class])
                detections = det.cpu().numpy()
                
                if len(detections) > 0:
                    # 只处理置信度最高的一个检测框
                    best_detection = detections[0]
                    for det in detections:
                        if det[4] > best_detection[4]:  # 比较置信度
                            best_detection = det
                    
                    # 计算物体中心点
                    x_centre = (best_detection[0] + best_detection[2]) / 2.0
                    y_centre = (best_detection[1] + best_detection[3]) / 2.0
                    
                    # 计算相对坐标
                    relative_position = self.calculate_relative_position(
                        x_centre, y_centre, 
                        best_detection[2] - best_detection[0],  # 宽度
                        best_detection[3] - best_detection[1]   # 高度
                    )
                    
                    # 获取检测类别名称和ID
                    scene_name = self.names[int(best_detection[5])]
                    scene_id = int(best_detection[5])
                    
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
                    
                    self.get_logger().info(
                        f'Category: {scene_name}, ID: {scene_id}',
                        f'Position: ({relative_position[0]:.2f}, {relative_position[1]:.2f}, {relative_position[2]:.2f})'
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