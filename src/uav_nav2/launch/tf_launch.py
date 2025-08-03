#!/usr/bin/env python3

"""
坐标系:
- base_footprint: 机体底部接触面
- base_link: 机体中心 (PX4中心), 在base_footprint上方4.5cm
- camera_link: 相机坐标系, 在base_link下方4cm, 前方16.5cm
- laser: 雷达坐标系, 在base_link上方8cm, 前方4.3cm
"""

import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # 声明使用仿真时间参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # 发布 base_footprint 到 base_link 的静态变换
        # base_link 在 base_footprint 上方 4.5cm
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.045', '0', '0', '0', 'base_footprint', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 发布 base_link 到 camera_link 的静态变换
        # camera_link 在 base_link 下方 4cm (z=-0.04)，前方 16.5cm (x=0.165)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_link',
            arguments=['0.165', '0', '-0.04', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 发布 base_link 到 laser 的静态变换
        # laser 在 base_link 上方 8cm (z=0.08)，前方 4.3cm (x=0.043)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.043', '0', '0.08', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])