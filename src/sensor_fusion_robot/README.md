 # 传感器融合机器人

这个ROS2包提供了一个简单的差速驱动机器人底盘，作为传感器融合项目的载体。机器人模型可以在Gazebo中运行，并配备有适合安装各种传感器的平台。

## 功能

- 简单的差速驱动机器人底盘
- 已集成Gazebo物理仿真
- 通过cmd_vel话题控制机器人移动
- 预留传感器安装平台，便于扩展

## 依赖

- ROS2-humble
- Gazebo
- xacro
- robot_state_publisher
- gazebo_ros

## 使用方法

### 构建包

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_robot
source install/setup.bash
```

### 启动仿真

启动Gazebo和加载机器人模型：

```bash
ros2 launch sensor_fusion_robot robot.launch.py
```

### 控制机器人

可以通过发布速度命令控制机器人移动：

```bash
# 前进（线速度0.5m/s）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 旋转（角速度0.5rad/s）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```


## 机器人结构

基本结构包括:
- 矩形底盘
- 两个驱动轮（差速驱动）
- 前后辅助万向轮（防止倾斜）
- 传感器安装平台（顶部）

## Todolist
 - [ ] 安装复杂传感器
 - [ ] 完成速度控制器的安装 