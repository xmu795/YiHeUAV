/**
 * @file mock_uav_node.hpp
 * @brief 模拟无人机节点头文件
 * @details 该文件定义了MockUAVNode类，用于模拟无人机的基本功能，
 *          包括位置发布器和MoveToTarget Action服务器。
 * @author 王为鸣
 * @date 2025-07-04
 * @version 2.0
 */

#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "uav_interfaces/action/move_to_target.hpp"

namespace uav_bt
{
    /**
     * @class MockUAVNode
     * @brief 模拟无人机节点，提供位置发布和MoveToTarget Action服务
     * @details 该类继承自rclcpp::Node，模拟无人机的基本功能：
     *          - 定期发布当前位置信息
     *          - 提供MoveToTarget Action服务器
     *          - 模拟无人机移动到目标点的行为
     */
    class MockUAVNode : public rclcpp::Node
    {
    public:
        /**
         * @brief 构造函数
         * @details 初始化ROS2节点，创建发布器、Action服务器和定时器
         */
        MockUAVNode();

        /**
         * @brief 析构函数
         */
        ~MockUAVNode() = default;

        // 类型别名定义
        using MoveToTarget = uav_interfaces::action::MoveToTarget;
        using MoveToTargetGoalHandle = rclcpp_action::ServerGoalHandle<MoveToTarget>;

    private:
        /**
         * @brief 发布当前位置信息
         * @details 定期调用此函数发布无人机当前位置到uav_position话题
         */
        void publish_position();

        /**
         * @brief 处理Action目标请求
         * @param uuid 目标的唯一标识符
         * @param goal 目标点信息
         * @return GoalResponse 是否接受该目标
         */
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const MoveToTarget::Goal> goal);

        /**
         * @brief 处理Action取消请求
         * @param goal_handle 目标句柄
         * @return CancelResponse 是否接受取消请求
         */
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<MoveToTargetGoalHandle> goal_handle);

        /**
         * @brief 处理已接受的Action目标
         * @param goal_handle 目标句柄
         * @details 启动新线程执行移动操作
         */
        void handle_accepted(const std::shared_ptr<MoveToTargetGoalHandle> goal_handle);

        /**
         * @brief 执行移动到目标点的操作
         * @param goal_handle 目标句柄
         * @details 在独立线程中模拟无人机移动过程，提供实时反馈
         */
        void execute_move(const std::shared_ptr<MoveToTargetGoalHandle> goal_handle);

        // ROS2组件
        /// Action服务器，处理MoveToTarget请求
        rclcpp_action::Server<MoveToTarget>::SharedPtr action_server_;
        
        /// 位置信息发布器，发布到uav_position话题
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_publisher_;
        
        /// 定时器，控制位置发布频率
        rclcpp::TimerBase::SharedPtr timer_;

        // 状态变量
        /// 当前位置信息（线程共享数据）
        geometry_msgs::msg::Point current_position_;
        
        /// 保护current_position_的互斥锁
        std::mutex position_mutex_;
    };

} // namespace uav_bt

