/**
 * @brief 实现 UAV 行为树节点的头文件
 *  该文件定义了一些 UAV 行为树节点动作节点。
 *  这些节点用于执行动作等。
 *  @note 该文件依赖于 ROS2 和 BehaviorTree.CPP V4.6库。BT中的ConditionNode通过SimpleConditionNode，直接传入lambda函数即可。
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace uav_bt
{
/**
 * @brief PublishStatusNode 类
 * 该类用于发布无人机状态到指定话题。
 * 
 */


#endif // UAV_BT_ACTION_NODES_HPP