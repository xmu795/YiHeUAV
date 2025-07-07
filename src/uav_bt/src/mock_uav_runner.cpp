/**
 * @file mock_uav_runner.cpp
 * @brief 运行mockuavnode，用于测试行为树
 * @details 该文件实现一个运行mockuavnode的main函数，包含一个位置发布器和一个目标点订阅器。
 *         通过模拟目标点的发布，验证无人机节点是否能够正确接收目标点并发布位置信息。
 * @note 该文件仅用于测试行为树控制器的功能，实际无人机的坐标订阅和坐标发布通过MAVLink实现。
 * @author 王为鸣
 * @date 2025-07-04
 * @version 1.0
 * @related uav_bt/src/mock_uav_node.cpp
 * @related uav_bt/test/mock_uav_node_test.cpp
 */

#include "uav_bt/mock_uav_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<uav_bt::MockUAVNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}