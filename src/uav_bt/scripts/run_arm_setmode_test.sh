#!/bin/bash

# ArmNode 和 SetModeNode 测试脚本
# 
# 这个脚本提供了多种测试选项：
# 1. 简单测试 - 快速验证基本功能
# 2. 完整测试 - 详细的多模式测试
# 3. 自定义测试 - 使用用户指定的行为树文件

echo "=================================================="
echo "    ArmNode 和 SetModeNode 行为树测试工具"
echo "=================================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取包路径
PACKAGE_SHARE_DIR=$(ros2 pkg prefix uav_bt)/share/uav_bt

# 检查包是否已构建
if [ ! -d "$PACKAGE_SHARE_DIR" ]; then
    echo -e "${RED}错误：uav_bt 包未找到或未构建${NC}"
    echo "请先编译包："
    echo "  cd /home/wwm/ros2_ws"
    echo "  colcon build --packages-select uav_bt"
    exit 1
fi

# 显示菜单
echo "请选择测试模式："
echo -e "${GREEN}1.${NC} 简单测试（推荐用于初次验证）"
echo -e "${BLUE}2.${NC} 完整测试（详细的多模式测试）"
echo -e "${YELLOW}3.${NC} 自定义测试（指定 XML 文件）"
echo -e "${RED}4.${NC} 退出"
echo ""

# 读取用户选择
read -p "请输入选择 (1-4): " choice

case $choice in
    1)
        echo -e "${GREEN}运行简单测试...${NC}"
        TREE_FILE="$PACKAGE_SHARE_DIR/config/simple_test_tree.xml"
        ;;
    2)
        echo -e "${BLUE}运行完整测试...${NC}"
        TREE_FILE="$PACKAGE_SHARE_DIR/config/test_arm_setmode_tree.xml"
        ;;
    3)
        echo -e "${YELLOW}请输入 XML 文件路径：${NC}"
        read -p "> " TREE_FILE
        if [ ! -f "$TREE_FILE" ]; then
            echo -e "${RED}错误：文件不存在 - $TREE_FILE${NC}"
            exit 1
        fi
        ;;
    4)
        echo "退出测试"
        exit 0
        ;;
    *)
        echo -e "${RED}无效选择${NC}"
        exit 1
        ;;
esac

# 检查行为树文件是否存在
if [ ! -f "$TREE_FILE" ]; then
    echo -e "${RED}错误：行为树文件不存在 - $TREE_FILE${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}使用行为树文件：${NC} $TREE_FILE"
echo ""

# 显示监控提示
echo -e "${YELLOW}提示：${NC}"
echo "在另一个终端运行以下命令来监控发送的 PX4 命令："
echo -e "${GREEN}ros2 topic echo /fmu/in/vehicle_command${NC}"
echo ""

# 等待用户确认
read -p "按 Enter 键开始测试，或 Ctrl+C 取消..."

# 运行测试
echo -e "${GREEN}启动测试程序...${NC}"
ros2 run uav_bt test_arm_setmode_nodes "$TREE_FILE"

# 检查退出状态
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ 测试程序正常退出${NC}"
else
    echo ""
    echo -e "${RED}✗ 测试程序异常退出${NC}"
fi

echo ""
echo "测试完成！"
