/**
 * @brief 实现 UAV 行为树节点的头文件
 *  该文件定义了一些 UAV 行为树节点，包括条件节点和动作节点。
 *  这些节点用于检查 UAV 的状态、执行动作等。
 *  @note 该文件依赖于 ROS2 和 BehaviorTree.CPP V4.6库。
 *  @file bt_nodes.hpp
 *  @author 王为鸣
 *  @date 2025-07-08
 *  @version 1.0
 **/
#ifndef UAV_BT_BT_NODES_HPP
#define UAV_BT_BT_NODES_HPP


#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros2/behavior_tree_ros2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


 /**
  * @brief 条件节点：检查是否在地面
  * 该节点检查 UAV 是否在地面上。
  * @note 如果 UAV 在地面上，返回成功状态；否则返回失败状态。
  **/
class IsOnGroundNode : public BT::ConditionNode {
public:
    IsOnGroundNode(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
 
/**
 * @brief 条件节点：检查是否在空中
 * 该节点检查 UAV 是否在空中。
 * @note 如果 UAV 在空中，返回成功状态；否则返回失败状态。
 **/
class IsInAirNode : public BT::ConditionNode {
public:
    IsInAirNode(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};


#endif // UAV_BT_BT_NODES_HPP