/**
 * @brief 实现 UAV 行为树节点的头文件
 * 
 * 该文件定义了无人机行为树的各种动作节点，支持两种通信模式：
 * 1. PX4 直接命令模式：通过发布 VehicleCommand 消息到 /fmu/in/vehicle_command 话题
 * 2. ROS2 服务/动作模式：通过 ROS2 的服务和动作接口进行通信
 * 
 * PX4 命令模式的优势：
 * - 直接与 PX4 飞控通信，无需中间层
 * - 一次性命令发布，无需等待响应
 * - 更低的延迟和更高的实时性
 * - 符合 PX4 标准的 MAVLink 命令协议
 * 
 * 该文件依赖于：
 * - ROS2 和 BehaviorTree.CPP V4.6 库
 * - px4_msgs 消息包（用于 PX4 飞控通信）
 * - uav_interfaces 自定义接口包
 * 
 * @file action_nodes.hpp
 * @author 王为鸣
 * @date 2025-07-08
 * @version 2.0 - 添加 PX4 直接命令支持
 */
#ifndef UAV_BT_ACTION_NODES_HPP
#define UAV_BT_ACTION_NODES_HPP


#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include "uav_interfaces/action/searching_picture.hpp"
#include "uav_interfaces/srv/release_cargo.hpp"
#include "uav_interfaces/srv/arm.hpp"
#include "uav_interfaces/srv/set_mode.hpp"


using SearchingPicture = uav_interfaces::action::SearchingPicture;
using ReleaseCargo = uav_interfaces::srv::ReleaseCargo;
using ArmService = uav_interfaces::srv::Arm;
using SetModeService = uav_interfaces::srv::SetMode;

namespace uav_bt
{

/**
 * @brief PublishStatusNode 类
 * 该类用于发布无人机状态到指定话题。
 * @note 这个类被多次继承，创造多个publisher发布向同一个话题。
 * 相比于在共同使用一个publisher句柄，这种结构开销更高，但是相对解耦和简单。
 */
class PublishStatusNode : public BT::RosTopicPubNode<std_msgs::msg::String>
{
public:
  PublishStatusNode(const std::string& name, const BT::NodeConfig& config,
                    const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  //  setMessage() 纯虚函数
  bool setMessage(std_msgs::msg::String& msg) override;
};
/**
 * @brief PrintMessage 类
 * 该类用于使用 ROS2 打印消息。
 * @param name 节点名称
 * @param config 节点配置
 * @param node_handle ROS2 节点句柄
 * @note 该类继承自 SyncActionNode 类
 */
class PrintMessage : public BT::SyncActionNode
{
public:
  PrintMessage(const std::string& name,  const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle); 

  static BT::PortsList providedPorts();

  //  tick() 函数
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief StreamPositionNode 类 - PX4 Offboard 模式位置流发布节点
 * 
 * 设计说明：
 * 该节点专为 PX4 Offboard 模式设计，通过发布 TrajectorySetpoint 消息来实现高频位置控制。
 * 
 * 为什么使用 SyncActionNode：
 * 1. 高频发布需求：PX4 Offboard 模式要求至少 2Hz 的设定点频率，推荐 10-20Hz
 * 2. 持续控制：每次 tick 发布一次消息，通过行为树的循环 tick 机制实现高频发布
 * 3. 实时性：避免 Action 机制的额外开销，直接发布消息
 * 4. 简化设计：无需复杂的状态管理，每次 tick 都是独立的发布操作
 * 5. 灵活控制：可通过返回 RUNNING 状态保持节点活跃，持续发布
 * 
 * TrajectorySetpoint 消息优势：
 * - PX4 Offboard 模式的标准接口
 * - 支持位置、速度、加速度和偏航角控制
 * - 包含时间戳，确保消息新鲜度
 * - 兼容性好，支持所有 PX4 版本
 * 
 * 使用方式：
 * 1. 设置目标位置通过输入端口
 * 2. 将此节点放在 Repeat 或 KeepRunningUntilFailure 等循环节点中实现高频发布
 * 3. 每次 tick 发布一次 TrajectorySetpoint 消息并返回 SUCCESS
 * 4. 通过行为树的循环控制实现所需的发布频率（如 20Hz）
 * 
 * @param name 节点名称
 * @param config 节点配置
 * @param node_handle ROS2 节点句柄，用于创建发布者
 */
class StreamPositionNode : public BT::SyncActionNode
{
public:
    StreamPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
};

/**
 * @brief ReleaseCargoNode 类
 * 该类用于控制无人机释放货物。
 * @param name 节点名称
 * @param config 节点配置
 * @param params ROS2 节点参数(包含了ROS2节点句柄)
 * @note 该类继承自 BT::RosServiceNode 类
 */
class ReleaseCargoNode : public BT::RosServiceNode<ReleaseCargo>
{
public:
    ReleaseCargoNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts();

    bool setRequest(Request::SharedPtr& request) override;

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

/**
 * @brief ArmNode 类 - PX4 解锁/上锁控制节点
 * 
 * 设计说明：
 * 该节点通过发布 VehicleCommand 消息到 /fmu/in/vehicle_command 话题来直接控制 PX4 飞控的解锁状态。
 * 
 * 为什么使用 SyncActionNode 而不是 RosServiceNode：
 * 1. PX4 命令模式：PX4 期望接收一次性的 VehicleCommand 消息，而不是请求-响应模式
 * 2. 实时性要求：解锁/上锁是安全关键操作，需要最低延迟
 * 3. 简化架构：无需额外的服务服务器，直接与飞控通信
 * 4. 标准兼容：符合 MAVLink COMPONENT_ARM_DISARM 命令标准
 * 5. 可靠性：避免了服务调用可能的超时和失败情况
 * 
 * 消息参数：
 * - command: VEHICLE_CMD_COMPONENT_ARM_DISARM (400)
 * - param1: 1.0 表示解锁，0.0 表示上锁
 * - target_system/component: 飞控系统和组件 ID
 * 
 * @param name 节点名称
 * @param config 节点配置
 * @param node_handle ROS2 节点句柄，用于创建发布者
 */
class ArmNode : public BT::SyncActionNode
{
public:
    ArmNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
};

/**
 * @brief SetModeNode 类 - PX4 飞行模式设置节点
 * 
 * 设计说明：
 * 该节点通过发布 VehicleCommand 消息到 /fmu/in/vehicle_command 话题来直接设置 PX4 飞控的飞行模式。
 * 
 * 为什么使用 SyncActionNode 而不是 RosServiceNode：
 * 1. PX4 命令协议：PX4 使用 MAVLink 命令协议，期望一次性命令而非请求-响应
 * 2. 模式切换速度：飞行模式切换需要快速响应，避免服务调用的额外开销
 * 3. 系统简化：减少中间层，直接与飞控的标准接口通信
 * 4. 错误处理：PX4 内部会处理无效模式，无需复杂的验证逻辑
 * 5. 并发安全：避免了多个节点同时调用服务可能的竞争问题
 * 
 * 支持的飞行模式：
 * - MANUAL: 手动模式 (nav_state = 0)
 * - ACRO: 特技模式 (nav_state = 1) 
 * - ALTCTL: 高度控制模式 (nav_state = 2)
 * - POSCTL: 位置控制模式 (nav_state = 3)
 * - AUTO_MISSION: 自动任务模式 (nav_state = 4)
 * - AUTO_LOITER: 自动悬停模式 (nav_state = 5)
 * - AUTO_RTL: 自动返回模式 (nav_state = 6)
 * - OFFBOARD: 外部控制模式 (nav_state = 14)
 * 
 * 消息参数：
 * - command: VEHICLE_CMD_SET_NAV_STATE (100001)
 * - param1: 对应的导航状态值
 * 
 * @param name 节点名称
 * @param config 节点配置  
 * @param node_handle ROS2 节点句柄，用于创建发布者
 */
class SetModeNode : public BT::SyncActionNode
{
public:
    SetModeNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
};

/**
 * @brief SearchingPictureNode 类
 * 该类用于控制无人机搜索图像目标。
 */
class SearchingPictureNode : public BT::RosActionNode<SearchingPicture>
{
public:
    SearchingPictureNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts();

    bool setGoal(Goal& goal) override;

    BT::NodeStatus onResultReceived(const WrappedResult& result) override;

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

}

#endif // UAV_BT_ACTION_NODES_HPP