cmake_minimum_required(VERSION 3.8)
project(hyy_actions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# REQUIRED to -> Launch move_group interface:
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# REQUIRED to -> Include hyy_message package:
find_package(hyy_message REQUIRED)

# REQUIRED to -> Execute ACTIONS:
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# ===== ACTIONS ===== #
# moveJ_action.cpp:
add_executable(moveJ_action scripts/moveJ_action.cpp)
ament_target_dependencies(
  moveJ_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  hyy_message)
install(TARGETS
  moveJ_action
  DESTINATION lib/${PROJECT_NAME})
# moveJs_action.cpp:
add_executable(moveJs_action scripts/moveJs_action.cpp)
ament_target_dependencies(
  moveJs_action
  moveit_ros_planning_interface
  std_msgs
  rclcpp
  rclcpp_action
  hyy_message)
install(TARGETS
  moveJs_action
  DESTINATION lib/${PROJECT_NAME})
# moveXYZW_action.cpp:
add_executable(moveXYZW_action scripts/moveXYZW_action.cpp)
ament_target_dependencies(
  moveXYZW_action
  moveit_ros_planning_interface
  moveit_visual_tools
  std_msgs
  rclcpp
  rclcpp_action
  hyy_message)
install(TARGETS
  moveXYZW_action
  DESTINATION lib/${PROJECT_NAME})
# gripper_action.cpp:
add_executable(gripper_action scripts/gripper_action.cpp)
ament_target_dependencies(
  gripper_action
  moveit_ros_planning_interface
  moveit_visual_tools
  std_msgs
  control_msgs
  rclcpp
  rclcpp_action
  hyy_message)
install(TARGETS
  gripper_action
  DESTINATION lib/${PROJECT_NAME})
ament_package()
