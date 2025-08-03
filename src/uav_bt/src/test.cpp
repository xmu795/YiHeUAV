#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

// 包含你自定义节点定义的头文件
// 请确保路径正确，例如 "uav_bt_project/action_nodes.hpp"
#include "uav_bt/action_nodes.hpp"

// 使用你定义的命名空间
using namespace BT;
using namespace uav_bt;

// 将之前定义的行为树 XML 嵌入为字符串
static const char* xml_text = R"(
<root BTCPP_format="4">
    <BehaviorTree ID="UAVTestTree">
        <Sequence name="MainTestSequence">
            <PrintMessage message="=== UAV BT Test Started: Testing ArmNode and SetModeNode ===" />
            
            <PrintMessage message="Step 1: Arming the UAV..." />
            <ArmNode arm="true" />
            <PrintMessage message="UAV armed successfully!" />
            <SetModeNode mode="OFFBOARD" />
            <Sleep msec="2000" />
            
            <PrintMessage message="Step 2: Testing StreamPositionNode for 5 seconds..." />
            <PrintMessage message="Note: StreamPositionNode will publish at each tick. For high-frequency publishing, use Repeat node." />
            
            <Repeat num_cycles="-1">
                <Sequence>
                    <StreamPositionNode x="3.2" y="2.5" z="-5.0" yaw="0.0" />
                    <Sleep msec="50" />  <!-- 50ms = 20Hz publishing rate -->
                </Sequence>
            </Repeat>
            
            <PrintMessage message="Position streaming completed!" />
            <Sleep msec="2000" />
            <PrintMessage message="Step 3: Disarming the UAV..." />
            <ArmNode arm="false" />
            <PrintMessage message="=== UAV BT Test Completed Successfully! ===" />
        </Sequence>
    </BehaviorTree>
    <TreeNodesModel>
        <Action ID="PrintMessage" editable="true">
            <input_port name="message" default="Hello"/>
        </Action>
        <Action ID="ArmNode" editable="true">
            <input_port name="arm" default="true" description="True to arm, false to disarm"/>
        </Action>
        <Action ID="SetModeNode" editable="true">
            <input_port name="mode" default="MANUAL" description="Flight mode: MANUAL, ACRO, ALTCTL, POSCTL, AUTO_MISSION, AUTO_LOITER, AUTO_RTL, OFFBOARD"/>
        </Action>
        <Action ID="StreamPositionNode" editable="true">
            <input_port name="x" default="0.0" description="Target X position in meters"/>
            <input_port name="y" default="0.0" description="Target Y position in meters "/>
            <input_port name="z" default="-5.0" description="Target Z position in meters "/>
            <input_port name="yaw" default="0.0" description="Target yaw angle in radians "/>
        </Action>
        <Action ID="Sleep" editable="true">
            <input_port name="msec" default="1000" description="Sleep time in milliseconds"/>
        </Action>
    </TreeNodesModel>
</root>
)";

int main(int argc, char** argv)
{
    // 1. Initialize ROS2
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("uav_bt_test_runner");
    RCLCPP_INFO(ros_node->get_logger(), "ROS2 node initialized for BT execution.");

    // 2. Create BehaviorTreeFactory
    BehaviorTreeFactory factory;

    // 3. Register custom nodes using the CORRECT method: registerBuilder
    
    // Register ArmNode
    factory.registerBuilder<ArmNode>(
        "ArmNode", 
        [&](const std::string& name, const NodeConfig& config) {
            return std::make_unique<ArmNode>(name, config, ros_node);
        });

    // Register SetModeNode
    factory.registerBuilder<SetModeNode>(
        "SetModeNode", 
        [&](const std::string& name, const NodeConfig& config) {
            return std::make_unique<SetModeNode>(name, config, ros_node);
        });

    // Register StreamPositionNode
    factory.registerBuilder<StreamPositionNode>(
        "StreamPositionNode", 
        [&](const std::string& name, const NodeConfig& config) {
            return std::make_unique<StreamPositionNode>(name, config, ros_node);
        });
        
    // Register PrintMessage
    factory.registerBuilder<PrintMessage>(
        "PrintMessage", 
        [&](const std::string& name, const NodeConfig& config) {
            return std::make_unique<PrintMessage>(name, config, ros_node);
        });

    // 4. Create Behavior Tree
    RCLCPP_INFO(ros_node->get_logger(), "Creating Behavior Tree from XML.");
    auto tree = factory.createTreeFromText(xml_text);

    // 5. Add logger
    StdCoutLogger logger(tree);

    // 6. Run the tree
    RCLCPP_INFO(ros_node->get_logger(), "Ticking the Behavior Tree...");
    NodeStatus status = tree.tickWhileRunning();
    
    RCLCPP_INFO(ros_node->get_logger(), "Behavior Tree execution finished with status: %s", toStr(status).c_str());

    // 7. Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}