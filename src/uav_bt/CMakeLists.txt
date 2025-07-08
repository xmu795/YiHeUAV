cmake_minimum_required(VERSION 3.8)
project(uav_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uav_interfaces REQUIRED)



# 包含头文件目录
include_directories(include)

# 添加源文件
add_executable(mock_uav_node 
               src/mock_uav_runner.cpp
               src/mock_uav_node.cpp)
# 添加测试节点
add_executable(mock_uav_tester 
               src/mock_uav_tester.cpp)

# 链接依赖库
ament_target_dependencies(mock_uav_node
  rclcpp
  behaviortree_cpp
  geometry_msgs
  std_msgs
  rclcpp_action
  uav_interfaces
)

# 链接测试节点依赖库
ament_target_dependencies(mock_uav_tester
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  uav_interfaces
)

ament_target_dependencies(mock_uav_tester
  rclcpp
  rclcpp_action
  uav_interfaces
)

# 安装目标
install(TARGETS mock_uav_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mock_uav_tester
  DESTINATION lib/${PROJECT_NAME}
)

# 安装Launch文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)



ament_package()