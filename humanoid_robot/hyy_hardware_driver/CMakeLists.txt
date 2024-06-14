cmake_minimum_required(VERSION 3.8)
project(hyy_hardware_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    realtime_tools
    controller_manager
    single_arm_hardware_interface
    hyy_controller
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#Create a executable target.
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
#Add include directories for the specified target and link required libraries.
ament_target_dependencies(
  ${PROJECT_NAME} 
  PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#Link binary libraries (i.e. dynamic libraries or static libraries) to a target.
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC 
  # /home/robotck/Buaa_ws/HYYRobotX64GRIP/depend/robot_base/lib/libRobotControl.so
  RobotControl
  modbus #Implement Modbus communication protocol.
  pthread #POSIX thread ibrary is used to implement multi-threaded programming.
  rt #Real-time library provides a series of functions related to real-time programming.
  m #Mathematics library, including various standard mathematical operation functions.
  dl #Dynamic loading library provides functions for dynamically loading and linking shared libraries.
  xml2 #Library for parsing XML documents.
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
