/**
 * @brief UAV 行为树节点实现文件
 * 
 * 实现了双模式通信的行为树节点：
 * 1. PX4 直接命令节点：ArmNode, SetModeNode
 * 2. ROS2 服务/动作节点：其他复杂功能节点
 * 
 * 设计理念说明：
 * 
 * PX4 直接命令模式的优势：
 * ==========================================
 * 1. 消除中间层：直接与飞控通信，减少延迟和故障点
 * 2. 标准兼容性：使用标准 MAVLink 命令，确保与所有 PX4 版本兼容
 * 3. 实时性：一次性命令发布，无阻塞等待，适合安全关键操作
 * 4. 简化部署：无需额外的服务服务器，减少系统复杂度
 * 5. 资源效率：避免服务调用的额外线程和内存开销
 * 
 * 为什么不使用 RosServiceNode：
 * ==========================================
 * 1. PX4 原生不支持 ROS 服务：需要额外的适配层
 * 2. 延迟增加：请求-响应模式增加了不必要的往返时间
 * 3. 复杂性：需要维护额外的服务服务器代码
 * 4. 故障点：服务调用可能超时、失败，需要额外的错误处理
 * 5. 资源消耗：每个服务需要额外的线程和回调管理
 * 
 * 适用场景选择：
 * ==========================================
 * - 飞控基础命令（解锁、模式切换、起飞、降落）→ PX4 直接模式
 * - 复杂任务（路径规划、视觉搜索、载荷操作）→ ROS2 服务/动作模式
 * 
 * @file action_nodes.cpp
 * @author 王为鸣
 * @date 2025-08-02
 * @version 2.0
 */

#include "uav_bt/action_nodes.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <limits>


namespace uav_bt
{

    PublishStatusNode::PublishStatusNode(const std::string& name, const BT::NodeConfig& config,
                                           const BT::RosNodeParams& params)
        : RosTopicPubNode<std_msgs::msg::String>(name, config, params){}

    BT::PortsList PublishStatusNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("status"),
        };
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
        return {
            BT::InputPort<std::string>("message"),
        };
    }
    BT::NodeStatus PrintMessage::tick()
    {
        std::string message;
        if (!getInput("message", message)) {
            throw BT::RuntimeError("Missing required input [message]");
        }
        RCLCPP_INFO(node_->get_logger(), "PrintMessage: %s", message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    // ========================================================================
    // StreamPositionNode 实现 - PX4 Offboard 高频位置控制节点
    // ========================================================================
    
    StreamPositionNode::StreamPositionNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle)
        : SyncActionNode(name, config), node_(node_handle) 
    {
        // 创建发布者，发送 TrajectorySetpoint 消息到 PX4
        // /fmu/in/trajectory_setpoint 是 PX4 Offboard 模式的标准接口
        setpoint_pub_ = node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        
        RCLCPP_INFO(node_->get_logger(), "StreamPositionNode: Initialized, ready for Offboard position streaming");
    }

    BT::PortsList StreamPositionNode::providedPorts()
    {
        return {
            BT::InputPort<double>("x", "Target X position in meters (NED frame)"),
            BT::InputPort<double>("y", "Target Y position in meters (NED frame)"),
            BT::InputPort<double>("z", "Target Z position in meters (NED frame, negative for altitude)"),
            BT::InputPort<double>("yaw", "Target yaw angle in radians (optional)"),
        };
    }

    BT::NodeStatus StreamPositionNode::tick()
    {
        double x, y, z;
        double yaw = 0.0;  // 默认偏航角为 0
        
        // 获取必需的位置参数
        if (!getInput("x", x)) {
            RCLCPP_ERROR(node_->get_logger(), "StreamPositionNode: Missing x position input");
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("y", y)) {
            RCLCPP_ERROR(node_->get_logger(), "StreamPositionNode: Missing y position input");
            return BT::NodeStatus::FAILURE;
        }
        if (!getInput("z", z)) {
            RCLCPP_ERROR(node_->get_logger(), "StreamPositionNode: Missing z position input");
            return BT::NodeStatus::FAILURE;
        }
        
        // 获取可选的偏航角参数
        getInput("yaw", yaw);  // 如果没有提供，保持默认值 0.0
        
        // 构造 TrajectorySetpoint 消息
        auto setpoint_msg = px4_msgs::msg::TrajectorySetpoint();
        
        // 时间戳：PX4 需要微秒精度的时间戳
        setpoint_msg.timestamp = node_->now().nanoseconds() / 1000;
        
        // 位置设定点 (NED 坐标系)
        // NED: North-East-Down，北-东-下坐标系
        // x: 北向位置 (正值向北)
        // y: 东向位置 (正值向东)  
        // z: 下向位置 (负值表示高度，如 z=-10 表示离地10米)
        setpoint_msg.position[0] = static_cast<float>(x);
        setpoint_msg.position[1] = static_cast<float>(y);
        setpoint_msg.position[2] = static_cast<float>(z);
        
        // 速度设定点：设置为 NaN 表示不控制速度，只控制位置
        setpoint_msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
        setpoint_msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
        setpoint_msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();
        
        // 加速度设定点：设置为 NaN 表示不控制加速度
        setpoint_msg.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
        setpoint_msg.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
        setpoint_msg.acceleration[2] = std::numeric_limits<float>::quiet_NaN();
        
        // 偏航角设定点
        setpoint_msg.yaw = static_cast<float>(yaw);
        
        // 偏航速度：设置为 NaN 表示不控制偏航速度
        setpoint_msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
        
        // 发布设定点消息
        // 这是关键：每次 tick 都发布一次消息，确保 PX4 Offboard 模式的 2Hz+ 要求
        setpoint_pub_->publish(setpoint_msg);
        
        // 调试信息：可以注释掉以减少日志输出
        static int publish_count = 0;
        if (++publish_count % 20 == 0) {  // 每20次发布打印一次（降低日志频率）
            RCLCPP_DEBUG(node_->get_logger(), 
                        "StreamPositionNode: Published setpoint #%d - Position: (%.2f, %.2f, %.2f), Yaw: %.2f", 
                        publish_count, x, y, z, yaw);
        }
        
        // 返回 SUCCESS 状态：
        // SyncActionNode 不允许返回 RUNNING，每次调用都必须完成
        // 高频发布需要通过行为树的外部循环机制实现（例如 Repeat 节点或主循环中的高频 tick）
        return BT::NodeStatus::SUCCESS;
    }

    // ReleaseCargoNode 实现
    ReleaseCargoNode::ReleaseCargoNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
        : RosServiceNode<ReleaseCargo>(name, config, params) {}

    BT::PortsList ReleaseCargoNode::providedPorts()
    {
        return {
            BT::InputPort<int16_t>("cargo_id", "ID of the cargo to release"),
        };
    }

    bool ReleaseCargoNode::setRequest(Request::SharedPtr& request)
    {
        int16_t cargo_id;
        if (!getInput("cargo_id", cargo_id)) {
            RCLCPP_ERROR(logger(), "ReleaseCargoNode: Missing cargo_id input");
            return false;
        }
        request->cargo_id = cargo_id;
        RCLCPP_INFO(logger(), "ReleaseCargoNode: Requesting release of cargo %d", cargo_id);
        return true;
    }

    BT::NodeStatus ReleaseCargoNode::onResponseReceived(const Response::SharedPtr& response)
    {
        if (response->success) {
            RCLCPP_INFO(logger(), "ReleaseCargoNode: Successfully released cargo - %s", response->message.c_str());
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(logger(), "ReleaseCargoNode: Failed to release cargo - %s", response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus ReleaseCargoNode::onFailure(BT::ServiceNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "ReleaseCargoNode: Service call failed with error: %s", toStr(error));
        return BT::NodeStatus::FAILURE;
    }

    // ========================================================================
    // ArmNode 实现 - PX4 直接命令模式
    // ========================================================================
    
    ArmNode::ArmNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle)
        : SyncActionNode(name, config), node_(node_handle) 
    {
        // 创建发布者，直接发送到 PX4 的标准命令接口
        // /fmu/in/vehicle_command 是 PX4 接收外部命令的标准话题
        command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    }

    BT::PortsList ArmNode::providedPorts()
    {
        return {
            BT::InputPort<bool>("arm", "True to arm, false to disarm"),
        };
    }

    BT::NodeStatus ArmNode::tick()
    {
        bool arm;
        if (!getInput("arm", arm)) {
            RCLCPP_ERROR(node_->get_logger(), "ArmNode: Missing arm input");
            return BT::NodeStatus::FAILURE;
        }

        // 构造 PX4 VehicleCommand 消息
        // 这个消息格式完全符合 MAVLink COMPONENT_ARM_DISARM 命令规范
        auto command_msg = px4_msgs::msg::VehicleCommand();
        
        // 时间戳：PX4 需要微秒精度的时间戳
        command_msg.timestamp = node_->now().nanoseconds() / 1000;
        
        // 命令 ID：使用标准的 MAVLink COMPONENT_ARM_DISARM 命令
        command_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        
        // 参数设置：param1 用于指定解锁(1.0)或上锁(0.0)状态
        command_msg.param1 = arm ? 1.0f : 0.0f;
        command_msg.param2 = 0.0f;  // 未使用
        command_msg.param3 = 0.0f;  // 未使用
        command_msg.param4 = 0.0f;  // 未使用
        command_msg.param5 = 0.0;   // 未使用
        command_msg.param6 = 0.0;   // 未使用
        command_msg.param7 = 0.0f;  // 未使用
        
        // 目标系统设置：1 表示主飞控系统
        command_msg.target_system = 1;
        command_msg.target_component = 1;
        
        // 源系统设置：标识命令来源
        command_msg.source_system = 1;
        command_msg.source_component = 1;
        
        // 确认标志：0 表示首次发送
        command_msg.confirmation = 0;
        
        // 外部标志：true 表示来自外部系统（ROS节点）
        command_msg.from_external = true;

        // 发布命令：一次性发送，无需等待响应
        // PX4 会在内部处理这个命令并更新系统状态
        command_pub_->publish(command_msg);
        
        RCLCPP_INFO(node_->get_logger(), "ArmNode: Sent %s command to PX4", arm ? "ARM" : "DISARM");
        
        // 立即返回成功：命令已发送，PX4 会异步处理
        // 如果需要确认，可以通过订阅 /fmu/out/vehicle_status 获取实际状态
        return BT::NodeStatus::SUCCESS;
    }

    // ========================================================================
    // SetModeNode 实现 - PX4 直接命令模式
    // ========================================================================
    
    SetModeNode::SetModeNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Node::SharedPtr node_handle)
        : SyncActionNode(name, config), node_(node_handle) 
    {
        // 创建发布者，发送模式切换命令到 PX4
        command_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    }

    BT::PortsList SetModeNode::providedPorts()
    {
        return {
            BT::InputPort<std::string>("mode", "Flight mode to set (MANUAL, ACRO, ALTCTL, POSCTL, AUTO_MISSION, AUTO_LOITER, AUTO_RTL, AUTO_TAKEOFF, AUTO_LAND, AUTO_FOLLOW_TARGET, AUTO_PRECLAND, OFFBOARD, STABILIZED, RATTITUDE)"),
        };
    }

    BT::NodeStatus SetModeNode::tick()
    {
        std::string mode;
        if (!getInput("mode", mode)) {
            RCLCPP_ERROR(node_->get_logger(), "SetModeNode: Missing mode input");
            return BT::NodeStatus::FAILURE;
        }

        // PX4 飞行模式映射表
        // 将用户友好的字符串模式转换为 MAVLink 标准模式值
        // 参考 MAVLink MAV_MODE 枚举定义
        uint8_t base_mode = 0;
        uint32_t custom_main_mode = 0;
        uint32_t custom_sub_mode = 0;
        
        if (mode == "MANUAL") {
            base_mode = 1;  // MAV_MODE_MANUAL_ARMED 或 MAV_MODE_MANUAL_DISARMED
            custom_main_mode = 1;  // PX4_CUSTOM_MAIN_MODE_MANUAL
        } else if (mode == "ACRO") {
            base_mode = 1;
            custom_main_mode = 2;  // PX4_CUSTOM_MAIN_MODE_ACRO
        } else if (mode == "ALTCTL") {
            base_mode = 1;
            custom_main_mode = 3;  // PX4_CUSTOM_MAIN_MODE_ALTCTL
        } else if (mode == "POSCTL") {
            base_mode = 1;
            custom_main_mode = 4;  // PX4_CUSTOM_MAIN_MODE_POSCTL
        } else if (mode == "AUTO_MISSION") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 1;   // PX4_CUSTOM_SUB_MODE_AUTO_MISSION
        } else if (mode == "AUTO_LOITER") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 3;   // PX4_CUSTOM_SUB_MODE_AUTO_LOITER
        } else if (mode == "AUTO_RTL") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 4;   // PX4_CUSTOM_SUB_MODE_AUTO_RTL
        } else if (mode == "AUTO_TAKEOFF") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 2;   // PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF
        } else if (mode == "AUTO_LAND") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 5;   // PX4_CUSTOM_SUB_MODE_AUTO_LAND
        } else if (mode == "AUTO_FOLLOW_TARGET") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 6;   // PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET
        } else if (mode == "AUTO_PRECLAND") {
            base_mode = 1;
            custom_main_mode = 5;  // PX4_CUSTOM_MAIN_MODE_AUTO
            custom_sub_mode = 7;   // PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND
        } else if (mode == "OFFBOARD") {
            base_mode = 1;
            custom_main_mode = 6;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        } else if (mode == "STABILIZED") {
            base_mode = 1;
            custom_main_mode = 7;  // PX4_CUSTOM_MAIN_MODE_STABILIZED
        } else if (mode == "RATTITUDE") {
            base_mode = 1;
            custom_main_mode = 8;  // PX4_CUSTOM_MAIN_MODE_RATTITUDE {
            base_mode = 1;
            custom_main_mode = 6;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        } else {
            RCLCPP_ERROR(node_->get_logger(), "SetModeNode: Unknown mode '%s'. Supported modes: MANUAL, ACRO, ALTCTL, POSCTL, AUTO_MISSION, AUTO_LOITER, AUTO_RTL, AUTO_TAKEOFF, AUTO_LAND, AUTO_FOLLOW_TARGET, AUTO_PRECLAND, OFFBOARD, STABILIZED, RATTITUDE", mode.c_str());
            return BT::NodeStatus::FAILURE;
        }

        // 构造 PX4 VehicleCommand 消息
        // 使用标准 MAVLink DO_SET_MODE 命令
        auto command_msg = px4_msgs::msg::VehicleCommand();
        
        // 时间戳设置
        command_msg.timestamp = node_->now().nanoseconds() / 1000;
        
        // 使用标准 MAVLink DO_SET_MODE 命令
        command_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        
        // param1: Base mode (MAV_MODE)
        command_msg.param1 = static_cast<float>(base_mode);
        // param2: Custom main mode
        command_msg.param2 = static_cast<float>(custom_main_mode);
        // param3: Custom sub mode  
        command_msg.param3 = static_cast<float>(custom_sub_mode);
        command_msg.param4 = 0.0f;  // 未使用
        command_msg.param5 = 0.0;   // 未使用
        command_msg.param6 = 0.0;   // 未使用
        command_msg.param7 = 0.0f;  // 未使用
        
        // 目标和源系统设置
        command_msg.target_system = 1;
        command_msg.target_component = 1;
        command_msg.source_system = 1;
        command_msg.source_component = 1;
        command_msg.confirmation = 0;
        command_msg.from_external = true;

        // 发布命令：PX4 会验证模式切换的前置条件
        // 例如：某些模式需要GPS锁定、传感器健康等
        command_pub_->publish(command_msg);
        
        RCLCPP_INFO(node_->get_logger(), "SetModeNode: Sent mode change command to PX4, mode: %s (base_mode: %d, custom_main: %d, custom_sub: %d)", 
                    mode.c_str(), base_mode, custom_main_mode, custom_sub_mode);
        
        // 立即返回成功：命令已发送
        // 实际的模式切换状态可通过 /fmu/out/vehicle_status 监控
        return BT::NodeStatus::SUCCESS;
    }

    // SearchingPictureNode 实现
    SearchingPictureNode::SearchingPictureNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
        : RosActionNode<SearchingPicture>(name, config, params) {}

    BT::PortsList SearchingPictureNode::providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PointStamped>("proposed_point", "Central point of search area"),
            BT::InputPort<double>("search_radius", "Search radius in meters"),
            BT::InputPort<std::string>("tag_name", "Name of the tag to search for"),
            BT::OutputPort<geometry_msgs::msg::PointStamped>("found_point", "Location where tag was found"),
        };
    }

    bool SearchingPictureNode::setGoal(SearchingPicture::Goal& goal)
    {
        geometry_msgs::msg::PointStamped proposed_point;
        double search_radius;
        std::string tag_name;
        
        if (!getInput("proposed_point", proposed_point)) {
            RCLCPP_ERROR(logger(), "SearchingPictureNode: Missing proposed_point input");
            return false;
        }
        if (!getInput("search_radius", search_radius)) {
            RCLCPP_ERROR(logger(), "SearchingPictureNode: Missing search_radius input");
            return false;
        }
        if (!getInput("tag_name", tag_name)) {
            RCLCPP_ERROR(logger(), "SearchingPictureNode: Missing tag_name input");
            return false;
        }
        
        goal.proposed_point = proposed_point;
        goal.search_radius = search_radius;
        goal.tag_name = tag_name;
        
        RCLCPP_INFO(logger(), "SearchingPictureNode: Searching for tag '%s' around (%f, %f, %f) with radius %f",
                    tag_name.c_str(), proposed_point.point.x, proposed_point.point.y, 
                    proposed_point.point.z, search_radius);
        return true;
    }

    BT::NodeStatus SearchingPictureNode::onResultReceived(const WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
            RCLCPP_INFO(logger(), "SearchingPictureNode: Tag found at (%f, %f, %f) - %s",
                       result.result->found_point.point.x, result.result->found_point.point.y,
                       result.result->found_point.point.z, result.result->message.c_str());
            setOutput("found_point", result.result->found_point);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(logger(), "SearchingPictureNode: Search failed - %s", 
                        result.result ? result.result->message.c_str() : "Unknown error");
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus SearchingPictureNode::onFeedback(const std::shared_ptr<const SearchingPicture::Feedback> feedback)
    {
        if (!feedback) {
            RCLCPP_WARN(logger(), "SearchingPictureNode: Received null feedback");
            return BT::NodeStatus::RUNNING;
        }
        RCLCPP_INFO(logger(), "SearchingPictureNode: Current position: (%f, %f, %f)",
                    feedback->current_position.point.x, feedback->current_position.point.y,
                    feedback->current_position.point.z);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus SearchingPictureNode::onFailure(BT::ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(logger(), "SearchingPictureNode: Action failed with error: %s", toStr(error));
        return BT::NodeStatus::FAILURE;
    }


}