/**
 * @brief 实现 UAV 行为树节点的头文件
 *  该文件定义了一些 UAV 行为树节点动作节点。
 *  这些节点用于执行动作等。
 *  该文件依赖于 ROS2 和 BehaviorTree.CPP V4.6库。
 *  BT中的ConditionNode通过SimpleConditionNode，直接传入lambda函数即可。
 *  由于ROS2的通信机制远比BlackBoard完善，因此在节点设计中使用将主要以行为树之间通过ROS2节点通信。
 *  @file action_nodes.hpp
 *  @author 王为鸣
 *  @date 2025-07-08
 *  @version 1.0
 **/
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "uav_interfaces/action/move_to_target.hpp"


using MoveToTarget = uav_interfaces::action::MoveToTarget;
using GoalHandleMoveToTarget = rclcpp_action::ClientGoalHandle<MoveToTarget>;

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
                    const BT::RosNodeParams& params)

  static BT::PortsList providedPorts()

  //  setMessage() 纯虚函数
  bool setMessage(std_msgs::msg::String& msg) override
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
  PrintMessage(const std::string& name,  const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node_handle); 

  static BT::PortsList providedPorts();

  //  tick() 函数
  BT::NodeStatus tick() override;
};

/**
 * @brief MoveToTargetNode 类
 * 该类用于控制无人机移动到指定目标点。
 * @param name 节点名称
 * @param config 节点配置
 * @param params ROS2 节点参数(包含了ROS2节点句柄)
 * @note 该类继承自 BT::RosActionNode 类
 */
class MoveToTargetNode : public BT::RosActionNode<MoveToTarget>
{
public:
    MoveToTargetNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts();

    bool setGoal(Goal& goal) override;

    BT::NodeStatus onResultReceived(const WrappedResult& result) override;

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

    BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};

/**
 * @brief ReleaseCargoNode 类
 * 该类用于控制无人机释放货物。
 * @param name 节点名称
 * @param config 节点配置
 * @param params ROS2 节点参数(包含了ROS2节点句柄)
 * @note 该类继承自 BT::RosActionNode 类
 */
class ReleaseCargoNode : public BT::RosActionNode<ReleaseCargo>

}

#endif // UAV_BT_ACTION_NODES_HPP