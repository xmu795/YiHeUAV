# YiHeUAV

一个基于ROS2的无人机自主飞行系统，集成了行为树控制、视觉感知和自主导航功能。

## 项目结构

```
ros2_ws/
├── src/
│   ├── uav_interfaces/          # UAV通信接口定义
│   │   ├── action/              # Action接口
│   │   │   ├── MoveToTarget.action
│   │   │   └── SearchingPicture.action
│   │   ├── srv/                 # Service接口
│   │   │   ├── Arm.srv
│   │   │   ├── ReleaseCargo.srv
│   │   │   └── SetMode.srv
│   │   └── msg/                 # Message接口 (预留)
│   ├── uav_bt/                  # 行为树控制包
│   │   ├── include/uav_bt/
│   │   │   └── action_nodes.hpp # 行为树节点定义
│   │   ├── src/
│   │   │   └── action_nodes.cpp # 行为树节点实现
│   │   └── README.md            # 详细使用文档
│   ├── vslam_to_px4/           # VSLAM定位到PX4桥接
│   └── third_party/            # 第三方库
├── build/                      # 编译输出
├── install/                    # 安装文件
└── log/                       # 日志文件
```

## 已完成功能

### ✅ 接口定义 (uav_interfaces)
- **Action接口**:
  - `MoveToTarget`: 无人机移动到目标点
  - `SearchingPicture`: 视觉目标搜索
- **Service接口**:
  - `Arm`: 无人机解锁/上锁
  - `SetMode`: 设置飞行模式
  - `ReleaseCargo`: 货物投放

### ✅ 行为树控制系统 (uav_bt)
- **动作节点**:
  - `MoveToTargetNode`: 目标点导航
  - `SearchingPictureNode`: 视觉搜索
- **服务节点**:
  - `ArmNode`: 解锁控制
  - `SetModeNode`: 模式设置
  - `ReleaseCargoNode`: 货物投放
- **工具节点**:
  - `PrintMessage`: 日志打印
  - `PublishStatusNode`: 状态发布

### ✅ 第三方集成
- BehaviorTree.CPP V4.6
- RealSense D435相机支持
- PX4飞控通信
- IMU校准工具

## 开发进展

### 当前状态 (2025-08-01)
- [x] 接口定义完成并编译通过
- [x] 行为树节点库实现完成
- [x] 基础框架搭建完成
- [x] 编译系统配置完成

### 进行中
- [ ] 硬件设备驱动集成
- [ ] 具体任务行为树设计
- [ ] 系统集成测试

## TodoList

### 高优先级
- [ ] 实现具体的无人机服务端 (Action/Service Servers)
- [ ] 部署思岚M2A8雷达和其ROS节点
- [ ] 实现圆环的精确识别和发布
- [ ] 创建完整的任务执行行为树

### 中优先级  
- [ ] 添加安全检查节点 (电池、GPS、传感器状态)
- [ ] 实现异常处理和恢复机制
- [ ] 添加任务监控和可视化
- [ ] 性能优化和代码重构

### 低优先级
- [ ] 增加仿真环境支持
- [ ] 编写自动化测试
- [ ] 添加配置文件管理
- [ ] 文档完善和示例程序

## 快速开始

### 1. 编译项目
```bash
cd /home/wwm/ros2_ws
colcon build
source install/setup.zsh
```

### 2. 运行基础测试
```bash
# 测试接口定义
ros2 interface list | grep uav_interfaces

# 运行模拟测试节点
ros2 run uav_bt mock_uav_tester
```

### 3. 查看详细文档
参考 [uav_bt/README.md](src/uav_bt/README.md) 获取详细的API使用指南。

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Behavior Tree │    │  UAV Interfaces │    │   Hardware      │
│   Control       │◄──►│   (Actions/Srvs) │◄──►│   Drivers       │
│   (uav_bt)      │    │                 │    │   (Sensors/FC)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Mission       │    │   Communication│    │   External      │
│   Planning      │    │   Middleware    │    │   Systems       │
│                 │    │   (ROS2)        │    │   (PX4, etc.)   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 技术栈

- **框架**: ROS2 Humble
- **控制**: BehaviorTree.CPP V4.6 + BehaviorTree.ROS2
- **视觉**: RealSense D435 + OpenCV
- **飞控**: PX4 + MAVROS
- **定位**: VSLAM
- **语言**: C++17, Python3

## 贡献指南

1. 查看 [uav_bt/README.md](src/uav_bt/README.md) 了解如何添加新的行为树节点
2. 遵循ROS2编码规范
3. 添加适当的测试和文档
4. 提交前确保编译通过

## 许可证

MIT License

## 联系方式

- 作者: 王为鸣 (wwm)
- 邮箱: 1120231016@bit.edu.cn
- 项目: YiHeUAV

