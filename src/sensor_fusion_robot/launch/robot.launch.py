import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('sensor_fusion_robot')
    
    # Xacro文件路径
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    
    # 使用xacro处理URDF
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # 世界文件路径
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items(),
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # 在Gazebo中加载机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'sensor_fusion_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )
    
    # 在命令行显示机器人控制提示
    cmd_info = ExecuteProcess(
        cmd=['echo', '机器人已启动! 使用命令 "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \'linear: {x: 0.5}\'" 来使机器人移动'],
        output='screen'
    )
    
    # 创建动作序列
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        cmd_info
    ]) 