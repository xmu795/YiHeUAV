cmake_minimum_required(VERSION 3.8)
project(uav_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(uav_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)



# 包含头文件目录
include_directories(include)

# 创建行为树节点库
add_library(${PROJECT_NAME}_nodes
    src/action_nodes.cpp
)

# 链接行为树节点库的依赖
ament_target_dependencies(${PROJECT_NAME}_nodes
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  geometry_msgs
  std_msgs
  rclcpp_action
  uav_interfaces
  px4_msgs
)

# 创建主执行程序
add_executable(uav_bt_runner
    src/test.cpp
)

# 链接主程序的依赖
ament_target_dependencies(uav_bt_runner
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  geometry_msgs
  std_msgs
  rclcpp_action
  uav_interfaces
  px4_msgs
)

# 链接节点库
target_link_libraries(uav_bt_runner
    ${PROJECT_NAME}_nodes
)

# 安装目标
install(TARGETS ${PROJECT_NAME}_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件
install(TARGETS uav_bt_runner
  DESTINATION lib/${PROJECT_NAME}
)


# 安装头文件
install(DIRECTORY include/
  DESTINATION include/
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  FILES_MATCHING PATTERN "*.xml"
)

# 安装脚本文件
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts/
  USE_SOURCE_PERMISSIONS
)

# 安装Launch文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()



ament_package()