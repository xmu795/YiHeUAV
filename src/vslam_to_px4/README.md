# VSLAM to PX4 Bridge

这个包提供了一个ROS 2节点，用于将VSLAM系统的定位输出转换为PX4自驾仪可以理解的格式，实现室内定位功能。

## 功能特性

- 订阅VSLAM系统输出的里程计或位姿信息
- 自动进行坐标系转换 (ENU -> NED)
- 发布PX4兼容的VehicleOdometry消息
- 支持协方差矩阵转换
- 可配置的质量评分系统
- 支持多种VSLAM输入格式

## 支持的消息类型

### 输入 (VSLAM)
- `nav_msgs/msg/Odometry` - 完整的里程计信息（位置、速度、协方差）
- `geometry_msgs/msg/PoseWithCovarianceStamped` - 位姿信息（仅位置和姿态）

### 输出 (PX4)
- `px4_msgs/msg/VehicleOdometry` - PX4飞控的标准定位消息

## 坐标系转换

### ROS (ENU) 到 PX4 (NED)
- **位置**: 
  - North = ROS_Y
  - East = ROS_X  
  - Down = -ROS_Z
- **姿态**: 通过四元数变换实现ENU到NED的转换
- **速度**: 相同的转换规则

## 构建和安装

```bash
# 在ROS 2工作空间中
cd ~/ros2_ws
colcon build --packages-select vslam_to_px4
source install/setup.bash
```

## 使用方法

### 1. 基本启动
```bash
ros2 run vslam_to_px4 vslam_to_px4_node
```

### 2. 使用launch文件
```bash
ros2 launch vslam_to_px4 vslam_to_px4.launch.py
```

### 3. 自定义话题名称
```bash
ros2 launch vslam_to_px4 vslam_to_px4.launch.py \
    vslam_odom_topic:=/your_vslam/odometry \
    px4_odom_topic:=/fmu/in/vehicle_visual_odometry
```

### 4. 使用位姿话题而非里程计话题
```bash
ros2 launch vslam_to_px4 vslam_to_px4.launch.py \
    use_pose_topic:=true \
    vslam_pose_topic:=/your_vslam/pose
```

### 5. 使用配置文件
```bash
ros2 run vslam_to_px4 vslam_to_px4_node --ros-args --params-file src/vslam_to_px4/config/params.yaml
```

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `vslam_odom_topic` | string | `/camera/odom/sample` | VSLAM里程计话题 |
| `vslam_pose_topic` | string | `/camera/pose` | VSLAM位姿话题 |
| `px4_odom_topic` | string | `/fmu/out/vehicle_odometry` | PX4输出话题 |
| `use_pose_topic` | bool | `false` | 是否使用位姿话题而非里程计 |

## 话题接口

### 订阅话题
- `vslam_odom_topic` (nav_msgs/msg/Odometry) - VSLAM里程计数据
- `vslam_pose_topic` (geometry_msgs/msg/PoseWithCovarianceStamped) - VSLAM位姿数据

### 发布话题  
- `px4_odom_topic` (px4_msgs/msg/VehicleOdometry) - PX4定位数据

## 质量评分

节点会根据输入数据的协方差矩阵自动计算质量分数（0-100）：
- 100: 最高质量
- 50-99: 良好质量  
- 1-49: 较差质量

## 常见VSLAM系统集成

### 1. Intel RealSense T265
```bash
# T265通常发布到 /camera/odom/sample
ros2 launch vslam_to_px4 vslam_to_px4.launch.py \
    vslam_odom_topic:=/camera/odom/sample
```

### 2. ORB-SLAM3
```bash
# ORB-SLAM3通常发布位姿到 /orb_slam3/camera_pose
ros2 launch vslam_to_px4 vslam_to_px4.launch.py \
    use_pose_topic:=true \
    vslam_pose_topic:=/orb_slam3/camera_pose
```

### 3. RTAB-Map
```bash
# RTAB-Map发布里程计到 /rtabmap/odom
ros2 launch vslam_to_px4 vslam_to_px4.launch.py \
    vslam_odom_topic:=/rtabmap/odom
```

## 调试和监控

### 查看发布的消息
```bash
ros2 topic echo /fmu/out/vehicle_odometry
```

### 检查话题连接
```bash
ros2 node info /vslam_to_px4_node
```

### 监控消息频率
```bash
ros2 topic hz /fmu/out/vehicle_odometry
```

## 故障排除

1. **没有输出消息**: 检查VSLAM话题是否正在发布数据
2. **PX4不接受定位数据**: 确保PX4参数`EKF2_AID_MASK`包含视觉定位
3. **坐标系错误**: 检查VSLAM系统的坐标系设置
4. **质量分数过低**: 调整VSLAM系统参数或环境光照条件

## 许可证

待定义
