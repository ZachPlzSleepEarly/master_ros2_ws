cmake_minimum_required(VERSION 3.8)
project(bt_action_server)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成 action interface
rosidl_generate_interfaces(
  ${PROJECT_NAME}
    "action/ReachLocation.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# 定义 action server 可执行文件
add_executable(reach_location_server src/reach_location_server.cpp)
ament_target_dependencies(reach_location_server rclcpp rclcpp_action std_msgs geometry_msgs)
# 把自定义 ROS 2 Action 的 typesupport 库链接到这个节点
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(reach_location_server "${cpp_typesupport_target}")

# 定义 action client 可执行文件
add_executable(reach_location_client src/reach_location_client.cpp)
ament_target_dependencies(reach_location_client rclcpp rclcpp_action std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(reach_location_client "${cpp_typesupport_target}")

add_executable(bt_action src/bt_action.cpp)
ament_target_dependencies(bt_action rclcpp rclcpp_action std_msgs geometry_msgs behaviortree_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(bt_action "${cpp_typesupport_target}")

install(
  DIRECTORY
    trees
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    reach_location_server
    reach_location_client
    bt_action
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
