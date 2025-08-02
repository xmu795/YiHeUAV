#!/bin/bash

# 简单 PX4 行为树演示启动脚本
# 该脚本展示如何使用 PX4 直接命令节点

echo "========================================="
echo "🚁 启动简单 PX4 行为树演示"
echo "========================================="

# 设置工作空间环境
cd /home/wwm/ros2_ws
source install/setup.bash

echo "📋 演示内容："
echo "  1. 解锁无人机 (ArmNode)"
echo "  2. 设置为 OFFBOARD 模式 (SetModeNode)"
echo "  3. 发布状态消息到 /uav/status"
echo "  4. 上锁无人机 (ArmNode)"
echo ""

echo "📡 监听话题 (另开终端查看)："
echo "  ros2 topic echo /uav/status"
echo "  ros2 topic echo /fmu/in/vehicle_command"
echo ""

echo "⚠️  注意：此演示会发布真实的 PX4 命令！"
echo "   确保在安全环境中运行，或连接模拟器"
echo ""

read -p "按 Enter 键继续启动演示..."

echo ""
echo "🚀 启动行为树演示..."
echo ""

# 运行演示程序
ros2 run uav_bt simple_px4_demo

echo ""
echo "✅ 演示完成！"
echo "========================================="
