cmake_minimum_required(VERSION 3.8)
project(robotiq_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(gazebo_grasp_plugin REQUIRED)

# add executable
add_executable(test_grpper src/test_grpper.cpp)
ament_target_dependencies(
  test_grpper
  rclcpp rclcpp_action control_msgs
)

install(
  TARGETS
    test_grpper
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    meshes
    rviz
    urdf
    gazebo
    srdf
    src
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
