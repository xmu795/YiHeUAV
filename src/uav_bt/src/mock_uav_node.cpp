/** 
* @file mock_uav_node.cpp
* @brief 模拟无人机节点，用于测试行为树
* @details 该文件实现一个模拟无人机节点，继承自rclcpp::Node，
*          用于模拟无人机的基本功能。其中包括目标点坐标订阅器，位置信息发布器，以及UAV应该实现的Action Server。
* @note 该文件仅用于测试行为树控制器的功能，实际无人机的坐标订阅和坐标发布通过MAVLink实现。
* @author 王为鸣 
* @date 2025-07-04
* @version 2.0
* @related uav_bt/test/mock_uav_node_test.cpp
**/

#include "uav_bt/mock_uav_node.hpp"

#include <chrono>
#include <thread>
#include <cmath> // For std::sqrt

using namespace std::chrono_literals; // for 100ms
using namespace uav_bt;


constexpr double UAV_SIM_SPEED = 0.5; // m/s
constexpr double GOAL_TOLERANCE = 0.1; // meters
constexpr int ACTION_LOOP_RATE = 20; // Hz

MockUAVNode::MockUAVNode() : Node("mock_uav_node")
{
    // 初始化当前位置
    current_position_.x = 0.0;
    current_position_.y = 0.0;
    current_position_.z = 0.0;
    
    //  禁用了订阅器，避免与Action Server冲突。测试行为树时，应只使用Action接口。
    /*
    target_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "target_point", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
            this->target_callback(msg);
        });
    */
    
    position_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "uav_position", 10);
    
    timer_ = this->create_wall_timer(
        100ms, // C++14 字面量
        [this]() { this->publish_position(); });

    using MoveToTarget = uav_interfaces::action::MoveToTarget;
    action_server_ = rclcpp_action::create_server<MoveToTarget>(
        this,
        "MoveToTarget",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToTarget::Goal> goal) {
            return this->handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<MoveToTargetGoalHandle> goal_handle) {
            return this->handle_cancel(goal_handle);
        },
        [this](const std::shared_ptr<MoveToTargetGoalHandle> goal_handle) {
            this->handle_accepted(goal_handle);
        });
    
    RCLCPP_INFO(this->get_logger(), "Mock UAV Node initialized and ready for action goals.");
}

// 注意共享数据 current_position_
void MockUAVNode::publish_position()
{
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "world";
    
    { // 锁定互斥锁
        std::lock_guard<std::mutex> lock(position_mutex_);
        msg->pose.position = current_position_;
    }
    
    msg->pose.orientation.w = 1.0;
    
    position_publisher_->publish(std::move(msg));
}

rclcpp_action::GoalResponse MockUAVNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const uav_interfaces::action::MoveToTarget::Goal> goal)
{
    (void)uuid; // 避免未使用变量的警告
    RCLCPP_INFO(this->get_logger(), "Received goal request to move to [%.2f, %.2f, %.2f]",
        goal->target_point.point.x, goal->target_point.point.y, goal->target_point.point.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MockUAVNode::handle_cancel(
    const std::shared_ptr<MoveToTargetGoalHandle> goal_handle)
{
    (void)goal_handle; // 避免未使用变量的警告
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MockUAVNode::handle_accepted(const std::shared_ptr<MoveToTargetGoalHandle> goal_handle)
{
    // 独立的线程来执行Action
    std::thread{ [this, goal_handle]() {
        this->execute_move(goal_handle);
    }}.detach(); // detach() 让线程在后台运行
}

void MockUAVNode::execute_move(const std::shared_ptr<MoveToTargetGoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    // 使用 rclcpp::Rate 控制循环频率，避免CPU空转
    rclcpp::Rate loop_rate(ACTION_LOOP_RATE);
    
    const auto goal = goal_handle->get_goal();
    const auto target_pos = goal->target_point.point;
    
    auto feedback = std::make_shared<uav_interfaces::action::MoveToTarget::Feedback>();
    auto result = std::make_shared<uav_interfaces::action::MoveToTarget::Result>();
    
    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        double distance;
        { // 作用域锁
            std::lock_guard<std::mutex> lock(position_mutex_);
            
            double dx = target_pos.x - current_position_.x;
            double dy = target_pos.y - current_position_.y;
            double dz = target_pos.z - current_position_.z;
            distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (distance < GOAL_TOLERANCE) {
                break; // 跳出循环，准备成功返回
            }

            double move_step = UAV_SIM_SPEED / ACTION_LOOP_RATE;
            current_position_.x += (dx / distance) * move_step;
            current_position_.y += (dy / distance) * move_step;
            current_position_.z += (dz / distance) * move_step;

            
            feedback->current_distance = distance;
            goal_handle->publish_feedback(feedback);
        } // 互斥锁在这里自动释放

        loop_rate.sleep();
    }

    // 检查循环退出的原因
    if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
    } else {
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "Goal aborted due to node shutdown.");
    }
}