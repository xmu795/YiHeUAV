# UAV Behavior Tree Package

## 概述

`uav_bt` 是一个基于 ROS2 和 BehaviorTree.CPP 的无人机行为树控制包。该包提供了一系列预定义的行为树节点，用于实现复杂的无人机任务逻辑。

## 包结构

```
uav_bt/
├── include/uav_bt/
│   └── action_nodes.hpp          # 行为树节点头文件
├── src/
│   ├── action_nodes.cpp          # 行为树节点实现
│   ├── mock_uav_node.cpp         # 模拟无人机节点
│   ├── mock_uav_runner.cpp       # 模拟节点启动器
│   └── mock_uav_tester.cpp       # 测试节点
├── config/                       # 配置文件目录（用于存放行为树XML文件）
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 依赖项

- **ROS2 Humble**
- **BehaviorTree.CPP V4.6+**
- **BehaviorTree.ROS2**
- **uav_interfaces** (自定义接口包)
- **geometry_msgs**
- **std_msgs**

## 已实现的行为树节点

### 1. 动作节点 (Action Nodes)

#### MoveToTargetNode
- **功能**: 控制无人机移动到指定的目标点
- **类型**: ROS Action 客户端节点
- **输入端口**:
  - `target_point` (geometry_msgs::msg::PointStamped): 目标点位置
- **Action类型**: `uav_interfaces::action::MoveToTarget`

#### SearchingPictureNode
- **功能**: 控制无人机在指定区域搜索视觉标签
- **类型**: ROS Action 客户端节点
- **输入端口**:
  - `proposed_point` (geometry_msgs::msg::PointStamped): 搜索中心点
  - `search_radius` (double): 搜索半径(米)
  - `tag_name` (string): 要搜索的标签名称
- **输出端口**:
  - `found_point` (geometry_msgs::msg::PointStamped): 找到标签的位置
- **Action类型**: `uav_interfaces::action::SearchingPicture`

### 2. 服务节点 (Service Nodes)

#### ArmNode
- **功能**: 控制无人机解锁/上锁
- **类型**: ROS Service 客户端节点
- **输入端口**:
  - `arm` (bool): true为解锁，false为上锁
- **Service类型**: `uav_interfaces::srv::Arm`

#### SetModeNode
- **功能**: 设置无人机飞行模式
- **类型**: ROS Service 客户端节点
- **输入端口**:
  - `mode` (string): 飞行模式名称 (如 "GUIDED", "LOITER", "RTL")
- **输出端口**:
  - `current_mode` (string): 设置后的当前模式
- **Service类型**: `uav_interfaces::srv::SetMode`

#### ReleaseCargoNode
- **功能**: 控制无人机释放指定货物
- **类型**: ROS Service 客户端节点
- **输入端口**:
  - `cargo_id` (int16): 要释放的货物ID
- **Service类型**: `uav_interfaces::srv::ReleaseCargo`

### 3. 实用工具节点

#### PrintMessage
- **功能**: 使用ROS2日志系统打印消息
- **类型**: 同步动作节点
- **输入端口**:
  - `message` (string): 要打印的消息

#### PublishStatusNode
- **功能**: 发布无人机状态到ROS话题
- **类型**: ROS话题发布节点
- **输入端口**:
  - `status` (string): 要发布的状态消息
- **话题类型**: `std_msgs::msg::String`

## 使用方法

### 1. 编译包

```bash
cd /home/wwm/ros2_ws
colcon build --packages-select uav_interfaces uav_bt
source install/setup.zsh
```

### 2. 在C++代码中使用

```cpp
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "uav_bt/action_nodes.hpp"

class MyBehaviorTreeNode : public rclcpp::Node
{
public:
    MyBehaviorTreeNode() : Node("my_bt_node")
    {
        // 创建行为树工厂
        BT::BehaviorTreeFactory factory;
        
        // 设置ROS参数
        BT::RosNodeParams params;
        params.nh = shared_from_this();
        params.default_port_value = "my_bt";
        
        // 注册节点类型
        factory.registerNodeType<uav_bt::MoveToTargetNode>("MoveToTarget", params);
        factory.registerNodeType<uav_bt::ArmNode>("ArmDrone", params);
        factory.registerNodeType<uav_bt::SetModeNode>("SetMode", params);
        factory.registerNodeType<uav_bt::SearchingPictureNode>("SearchTag", params);
        factory.registerNodeType<uav_bt::ReleaseCargoNode>("ReleaseCargo", params);
        factory.registerNodeType<uav_bt::PrintMessage>("PrintMessage", shared_from_this());
        
        // 从XML文件创建行为树
        auto tree = factory.createTreeFromFile("path/to/your/tree.xml");
        
        // 执行行为树
        auto status = tree.tickOnce();
    }
};
```

### 3. 行为树XML示例

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
    <BehaviorTree ID="UAVMission">
        <Sequence name="MainSequence">
            <!-- 解锁无人机 -->
            <ArmDrone arm="true"/>
            
            <!-- 设置引导模式 -->
            <SetMode mode="GUIDED"/>
            
            <!-- 移动到搜索区域 -->
            <MoveToTarget target_point="{search_point}"/>
            
            <!-- 搜索目标 -->
            <SearchTag proposed_point="{search_point}" 
                       search_radius="50.0" 
                       tag_name="landing_pad" 
                       found_point="{target_location}"/>
            
            <!-- 移动到找到的目标位置 -->
            <MoveToTarget target_point="{target_location}"/>
            
            <!-- 释放货物 -->
            <ReleaseCargo cargo_id="1"/>
            
            <!-- 打印完成消息 -->
            <PrintMessage message="Mission completed successfully!"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## 进一步开发指南

### 1. 添加新的行为树节点

#### 步骤1: 在头文件中声明
在 `include/uav_bt/action_nodes.hpp` 中添加新的节点类：

```cpp
class MyNewNode : public BT::SyncActionNode  // 或其他基类
{
public:
    MyNewNode(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    
private:
    // 私有成员变量
};
```

#### 步骤2: 在源文件中实现
在 `src/action_nodes.cpp` 中实现：

```cpp
MyNewNode::MyNewNode(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList MyNewNode::providedPorts()
{
    return {
        BT::InputPort<std::string>("input_param"),
        BT::OutputPort<int>("output_result")
    };
}

BT::NodeStatus MyNewNode::tick()
{
    // 实现节点逻辑
    return BT::NodeStatus::SUCCESS;
}
```

#### 步骤3: 注册节点
在使用时注册新节点：

```cpp
factory.registerNodeType<uav_bt::MyNewNode>("MyNewNode");
```

### 2. 节点类型选择

- **SyncActionNode**: 同步执行的简单动作
- **AsyncActionNode**: 异步执行的动作  
- **StatefulActionNode**: 带状态的动作节点
- **RosActionNode**: ROS Action客户端
- **RosServiceNode**: ROS Service客户端
- **RosTopicPubNode**: ROS话题发布
- **RosTopicSubNode**: ROS话题订阅

### 3. 端口类型

- **InputPort**: 从行为树黑板读取输入
- **OutputPort**: 向行为树黑板写入输出
- **BidirectionalPort**: 双向端口

### 4. 常用模式

#### 条件检查节点
```cpp
class CheckBatteryLevel : public BT::SyncActionNode
{
    BT::NodeStatus tick() override
    {
        double battery_level = getBatteryLevel();
        return (battery_level > 20.0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
```

#### 带参数的动作节点
```cpp
class WaitNode : public BT::SyncActionNode
{
    BT::NodeStatus tick() override
    {
        double wait_time;
        if (!getInput("duration", wait_time)) {
            throw BT::RuntimeError("Missing duration parameter");
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(wait_time));
        return BT::NodeStatus::SUCCESS;
    }
};
```

### 5. 调试和监控

- 使用 `BT::printTreeRecursively()` 打印树结构
- 使用 `RCLCPP_INFO/WARN/ERROR` 进行日志记录
- 考虑使用 Groot2 工具进行可视化调试

### 6. 最佳实践

1. **错误处理**: 始终检查输入参数的有效性
2. **日志记录**: 在关键操作点添加适当的日志
3. **超时处理**: 为长时间运行的操作设置超时
4. **状态检查**: 在执行操作前检查系统状态
5. **模块化设计**: 保持节点功能单一，易于测试和重用

## 测试

运行测试程序：
```bash
ros2 run uav_bt mock_uav_tester
```

## 许可证

MIT License

## 作者

王为鸣 (wwm) - 1120231016@bit.edu.cn

## 版本历史

- v1.0.0 (2025-08-01): 初始版本，实现基础行为树节点
