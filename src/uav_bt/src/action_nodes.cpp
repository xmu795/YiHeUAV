

#include "uav_bt/action_nodes.hpp"


namespace uav_bt
{

    PublishStatusNode::PublishStatusNode(const std::string& name, const BT::NodeConfig& config,
                                           const BT::RosNodeParams& params)
        : RosTopicPubNode<std_msgs::msg::String>(name, config, params){}

    BT::PortsList PublishStatusNode::providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("status"),
        });
    }
    bool PublishStatusNode::setMessage(std_msgs::msg::String& msg)
    {
        std::string to_send;
        if (!getInput("status", to_send)) {
            return false;
        }
        msg.data = to_send;
        return true;
    }

    PrintMessage::PrintMessage(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle)
        :  BT::SyncActionNode(name, config), node_(node_handle) {}
    BT::PortsList PrintMessage::providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<std::string>("message"),
        });
    }
    BT::NodeStatus PrintMessage::tick()
    {
        std::string message;
        if (!getInput("message", message)) {
            throw BT::RuntimeError("Missing required input [message]");
        }
        RCLCPP_INFO(node_->get_logger("uav_bt"), "PrintMessage: %s", message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    MoveToTargetNode::MoveToTargetNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
        : RosActionNode<MoveToTarget>(name, config, params) {}
    BT::PortsList MoveToTargetNode::providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<geometry_msgs::msg::PoseStamped>("target_point", "Target pose to move to"),
        });
    }

    bool MoveToTargetNode::setGoal(MoveToTarget::Goal& goal)
    {
        geometry_msgs::msg::PoseStamped target_point;
        if (!getInput("target_point", target_point)) {
            return false;
        }
        goal.target_pose = target_point;
        RCLCPP_INFO(logger(), "MoveToTargetNode: Setting goal to target point at (%f, %f, %f)",
                    target_point.pose.position.x, target_point.pose.position.y, target_point.pose.position.z);
        return true;
    }
    BT::NodeStatus MoveToTargetNode::onResultReceived(const MoveToTarget::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(logger(), "MoveToTargetNode: Action succeeded");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(logger(), "MoveToTargetNode: Action failed with code %d", result.code);
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus MoveToTargetNode::onFeedback(const std::shared_ptr<const Feedback> feedback) 
    {
        if (!feedback) {
            RCLCPP_WARN(logger(), "MoveToTargetNode: Received null feedback");
            return BT::NodeStatus::FAILURE; 
        }
        RCLCPP_INFO(logger(), "MoveToTargetNode: Feedback - Current position: (%f, %f, %f)",
                    feedback->current_position.x, feedback->current_position.y, feedback->current_position.z);
        
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus MoveToTargetNode::onFailure(ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "Action failed with error: %s", toStr(error));
        return BT::NodeStatus::FAILURE;
    }


}