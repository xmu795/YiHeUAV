#include <iostream>
#include <thread>
#include <atomic>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uav_interfaces/action/move_to_target.hpp"

using MoveToTarget = uav_interfaces::action::MoveToTarget;
using GoalHandleMoveToTarget = rclcpp_action::ClientGoalHandle<MoveToTarget>;

class MockUAVTesterNode : public rclcpp::Node
{
public:
    explicit MockUAVTesterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("mock_uav_tester", options), goal_active_(false)
    {
        // 创建Action Client
        this->action_client_ = rclcpp_action::create_client<MoveToTarget>(
            this,
            "MoveToTarget");
    }

    void send_goal(const MoveToTarget::Goal & test_point)
    {
        // 1. 等待Action Server连接
        RCLCPP_INFO(this->get_logger(), "Waiting for action server to be available...");
        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        // 2. 创建一个目标(Goal)消息
        auto goal_msg = MoveToTarget::Goal();
        goal_msg.target_point.header.frame_id = "world";
        goal_msg.target_point.point.x = test_point.target_point.point.x;
        goal_msg.target_point.point.y = test_point.target_point.point.y;
        goal_msg.target_point.point.z = test_point.target_point.point.z;

        RCLCPP_INFO(this->get_logger(), "Sending goal to target: [%.2f, %.2f, %.2f]",
            goal_msg.target_point.point.x,
            goal_msg.target_point.point.y,
            goal_msg.target_point.point.z);

        // 设置goal为活跃状态
        goal_active_ = true;

        // 3. 设置回调函数
        auto send_goal_options = rclcpp_action::Client<MoveToTarget>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            [this](const GoalHandleMoveToTarget::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                    goal_active_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleMoveToTarget::SharedPtr, const std::shared_ptr<const MoveToTarget::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Feedback received: Distance to target = %.2f meters", feedback->current_distance);
            };

        send_goal_options.result_callback =
            [this](const GoalHandleMoveToTarget::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal SUCCEEDED! Final success flag: %s", result.result->success ? "true" : "false");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was ABORTED");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Goal was CANCELED");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        break;
                }
                goal_active_ = false;
                std::cout << "\n=== Ready for next command ===" << std::endl;
                std::cout << "Enter target coordinates (x y z) or 'q' to quit: ";
                std::cout.flush();
            };

        // 4. 异步发送目标
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    bool is_goal_active() const { return goal_active_; }

private:
    rclcpp_action::Client<MoveToTarget>::SharedPtr action_client_;
    std::atomic<bool> goal_active_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto tester_node = std::make_shared<MockUAVTesterNode>();

    // 创建一个单独的线程来处理ROS2回调
    std::thread spin_thread([&tester_node]() {
        rclcpp::spin(tester_node);
    });

    // 主线程处理用户输入
    std::cout << "\n=== UAV Movement Tester ===" << std::endl;
    std::cout << "Enter target coordinates (x y z) or 'q' to quit: ";
    
    std::string input;
    while (rclcpp::ok() && std::getline(std::cin, input)) {
        // 检查是否退出
        if (input == "q" || input == "quit") {
            std::cout << "Exiting..." << std::endl;
            break;
        }
        
        // 检查是否有goal正在执行
        if (tester_node->is_goal_active()) {
            std::cout << "Goal is still active, please wait..." << std::endl;
            std::cout << "Enter target coordinates (x y z) or 'q' to quit: ";
            continue;
        }
        
        // 解析坐标
        std::istringstream iss(input);
        double x, y, z;
        if (!(iss >> x >> y >> z)) {
            std::cout << "Invalid input format. Please enter three numbers (x y z)" << std::endl;
            std::cout << "Enter target coordinates (x y z) or 'q' to quit: ";
            continue;
        }
        
        // 创建目标点
        auto test_point = MoveToTarget::Goal();
        test_point.target_point.point.x = x;
        test_point.target_point.point.y = y;
        test_point.target_point.point.z = z;
        
        // 发送目标指令
        std::cout << "Sending UAV to target: [" << x << ", " << y << ", " << z << "]" << std::endl;
        tester_node->send_goal(test_point);
    }

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    return 0;
}