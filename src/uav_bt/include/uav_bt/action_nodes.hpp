#ifndef UAV_BT_ACTION_NODES_HPP
#define UAV_BT_ACTION_NODES_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/**
 * @brief 动作节点的基类
 */
// 条件节点：检查是否在地面
class IsOnGroundNode : public BT::ConditionNode {
public:
    IsOnGroundNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
};

// 条件节点：检查是否在空中
class IsInAirNode : public BT::ConditionNode {
public:
    IsInAirNode(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
};

// 条件节点：检查命令
class CheckCommandNode : public BT::ConditionNode {
public:
    CheckCommandNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// 条件节点：检查超时
class IsTimeoutReachedNode : public BT::ConditionNode {
public:
    IsTimeoutReachedNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// 动作节点：运动到目标位置
class MoveToTargetNode : public BT::StatefulActionNode {
public:
    MoveToTargetNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
    double target_altitude_;
    std::chrono::steady_clock::time_point start_time_;
};

// 动作节点：降落
class LandNode : public BT::StatefulActionNode {
public:
    LandNode(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::chrono::steady_clock::time_point start_time_;
};

// 动作节点：更新时间戳
class UpdateTimestampNode : public BT::SyncActionNode {
public:
    UpdateTimestampNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// 动作节点：清除命令
class ClearCommandNode : public BT::SyncActionNode {
public:
    ClearCommandNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// 动作节点：打印消息
class PrintMessageNode : public BT::SyncActionNode {
public:
    PrintMessageNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
};