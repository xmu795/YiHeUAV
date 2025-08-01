#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明launch参数
    vslam_odom_topic_arg = DeclareLaunchArgument(
        'vslam_odom_topic',
        default_value='/camera/odom/sample',
        description='VSLAM odometry topic to subscribe to'
    )
    
    px4_odom_topic_arg = DeclareLaunchArgument(
        'px4_odom_topic', 
        default_value='/fmu/out/vehicle_odometry',
        description='PX4 vehicle odometry topic to publish to'
    )
    
    use_pose_topic_arg = DeclareLaunchArgument(
        'use_pose_topic',
        default_value='false',
        description='Whether to use pose topic instead of odometry'
    )
    
    vslam_pose_topic_arg = DeclareLaunchArgument(
        'vslam_pose_topic',
        default_value='/camera/pose',
        description='VSLAM pose topic to subscribe to (if use_pose_topic is true)'
    )

    # 创建节点
    vslam_to_px4_node = Node(
        package='vslam_to_px4',
        executable='vslam_to_px4_node',
        name='vslam_to_px4_node',
        output='screen',
        parameters=[{
            'vslam_odom_topic': LaunchConfiguration('vslam_odom_topic'),
            'px4_odom_topic': LaunchConfiguration('px4_odom_topic'),
            'use_pose_topic': LaunchConfiguration('use_pose_topic'),
            'vslam_pose_topic': LaunchConfiguration('vslam_pose_topic'),
        }],
        remappings=[
            # 可以在这里添加topic重映射
        ]
    )

    return LaunchDescription([
        vslam_odom_topic_arg,
        px4_odom_topic_arg,
        use_pose_topic_arg,
        vslam_pose_topic_arg,
        vslam_to_px4_node
    ])
