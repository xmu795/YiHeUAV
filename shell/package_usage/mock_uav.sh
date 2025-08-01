#!/bin/bash

# 脚本功能：启动 mock_uav 包中的节点
#
# 使用方法:
#   ./run.sh main - 启动 mock_uav_node
#   ./run.sh test - 启动 test_mock_uav
#   ./run.sh both - 同时启动 mock_uav_node 和 test_mock_uav

# 设置工作空间路径
WORKSPACE_PATH="/home/wwm/ros2_ws"

# 检查 setup.bash 是否存在
if [ ! -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    echo "错误: 在 $WORKSPACE_PATH/install/setup.bash 中找不到ROS2工作区设置文件。"
    echo "请先在 $WORKSPACE_PATH 目录下运行 'colcon build' 来构建您的工作空间。"
    exit 1
fi

# 加载ROS2工作空间环境
source "$WORKSPACE_PATH/install/setup.bash"

# 显示使用说明
usage() {
    echo "使用: $0 {main|test|both}"
    echo "  main: 启动 mock_uav_node"
    echo "  test: 启动 test_mock_uav"
    echo "  both: 在后台同时启动 mock_uav_node 和 test_mock_uav"
    exit 1
}

# 检查参数
if [ "$#" -ne 1 ]; then
    usage
fi

# 根据参数执行命令
case "$1" in
    main)
        echo "正在启动 mock_uav_node..."
        ros2 run mock_uav mock_uav_node
        ;;
    test)
        echo "正在启动 test_mock_uav..."
        ros2 run mock_uav test_mock_uav
        ;;
    both)
        echo "正在后台启动 mock_uav_node..."
        ros2 run mock_uav mock_uav_node &
        NODE_PID=$!

        echo "正在后台启动 test_mock_uav..."
        ros2 run mock_uav test_mock_uav &
        TEST_PID=$!

        echo "mock_uav_node (PID: $NODE_PID) 和 test_mock_uav (PID: $TEST_PID) 已启动。"
        echo "按 Ctrl+C 停止所有节点。"

        # 等待任一进程结束
        wait -n
        # 杀死另一个进程
        kill $NODE_PID $TEST_PID 2>/dev/null
        ;;
    *)
        usage
        ;;
esac

echo "脚本执行完毕。"