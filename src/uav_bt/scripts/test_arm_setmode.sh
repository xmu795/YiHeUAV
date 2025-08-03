#!/bin/bash

# ARM和SetMode节点测试脚本
# 这个脚本用于测试ArmNode和SetModeNode在Behavior Tree中的功能

echo "=== UAV Behavior Tree ARM and SetMode Test ==="
echo "This test will:"
echo "1. Arm the UAV"
echo "2. Switch flight modes 5 times with 20s intervals"
echo "3. Disarm the UAV"
echo ""
echo "Test modes sequence:"
echo "  1. MANUAL"
echo "  2. ALTCTL" 
echo "  3. POSCTL"
echo "  4. AUTO_LOITER"
echo "  5. OFFBOARD"
echo ""
echo "Total test duration: approximately 2 minutes"
echo ""

# 进入ROS2工作空间
cd /home/wwm/ros2_ws

# 检查是否已编译
if [ ! -f "install/uav_bt/lib/uav_bt/uav_bt_runner" ]; then
    echo "Error: uav_bt_runner not found. Please build the workspace first:"
    echo "  colcon build --packages-select uav_bt"
    exit 1
fi

# 检查XML文件是否存在
if [ ! -f "src/uav_bt/config/arm_setmode_test.xml" ]; then
    echo "Error: Test XML file not found: src/uav_bt/config/arm_setmode_test.xml"
    exit 1
fi

# 设置ROS2环境
source install/setup.bash

echo "Starting UAV Behavior Tree test..."
echo "Press Ctrl+C to stop the test at any time."
echo ""

# 运行测试
./install/uav_bt/lib/uav_bt/uav_bt_runner src/uav_bt/config/arm_setmode_test.xml

echo ""
echo "=== Test completed ==="
