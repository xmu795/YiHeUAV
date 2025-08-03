#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/actions/sleep_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "uav_bt/action_nodes.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

class UAVBehaviorTreeNode : public rclcpp::Node
{
public:
    UAVBehaviorTreeNode() : Node("uav_behavior_tree")
    {
        // 创建行为树工厂
        auto node_shared = shared_from_this();
        factory_.registerNodeType<uav_bt::PrintMessage>("PrintMessage", node_shared);
        
        // 注册BehaviorTree.ROS2节点
        BT::RosNodeParams params;
        params.nh = shared_from_this();
        params.default_port_value = "uav_bt";
        
        // 注册动作节点
        factory_.registerNodeType<uav_bt::SearchingPictureNode>("SearchingPictureNode", params);
        
        // 注册服务节点 (旧的服务模式)
        factory_.registerNodeType<uav_bt::ReleaseCargoNode>("ReleaseCargoNode", params);
        
        // 注册新的PX4命令节点 (直接使用 node_shared)
        factory_.registerNodeType<uav_bt::ArmNode>("ArmNode", node_shared);
        factory_.registerNodeType<uav_bt::SetModeNode>("SetModeNode", node_shared);
        factory_.registerNodeType<uav_bt::StreamPositionNode>("StreamPositionNode", node_shared);
        
        // 注册BehaviorTree.CPP内置节点
        factory_.registerNodeType<BT::SleepNode>("Sleep");
        
        // 设置黑板上的初始值
        setupBlackboard();
        
        RCLCPP_INFO(this->get_logger(), "UAV Behavior Tree Node initialized");
    }
    
    void loadTreeFromFile(const std::string& tree_file)
    {
        try {
            tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
            RCLCPP_INFO(this->get_logger(), "Behavior tree loaded from: %s", tree_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load tree: %s", e.what());
        }
    }
    
    void executeBehaviorTree()
    {
        if (!tree_.rootNode()) {
            RCLCPP_ERROR(this->get_logger(), "No behavior tree loaded!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting behavior tree execution...");
        
        auto status = BT::NodeStatus::RUNNING;
        while (status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
            status = tree_.tickOnce();
            
            // 等待一小段时间再继续
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // 处理ROS2事件
            rclcpp::spin_some(shared_from_this());
        }
        
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Behavior tree execution completed successfully!");
        } else if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_ERROR(this->get_logger(), "Behavior tree execution failed!");
        }
    }

private:
    void setupBlackboard()
    {
        blackboard_ = BT::Blackboard::create();
        
        // 设置搜索区域点
        geometry_msgs::msg::PointStamped search_point;
        search_point.header.frame_id = "map";
        search_point.header.stamp = this->now();
        search_point.point.x = 10.0;
        search_point.point.y = 5.0;
        search_point.point.z = 20.0;  // 飞行高度
        
        blackboard_->set("search_area_point", search_point);
        
        RCLCPP_INFO(this->get_logger(), "Blackboard initialized with search area at (%.1f, %.1f, %.1f)",
                   search_point.point.x, search_point.point.y, search_point.point.z);
    }
    
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<UAVBehaviorTreeNode>();
    
    // 从参数或默认路径加载行为树
    std::string tree_file = "/home/wwm/ros2_ws/src/uav_bt/config/arm_setmode_test.xml";
    if (argc > 1) {
        tree_file = argv[1];
    }
    
    node->loadTreeFromFile(tree_file);
    
    // 执行行为树
    std::thread bt_thread([node]() {
        node->executeBehaviorTree();
    });
    
    // 保持ROS2节点运行
    rclcpp::spin(node);
    
    bt_thread.join();
    rclcpp::shutdown();
    
    return 0;
}
