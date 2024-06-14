cmake_minimum_required(VERSION 3.8)
project(hyy_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hyy_message REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/debugging.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
#Specifies the directory where the compiler should look for header files when compiling a given target (such as an executable or library).
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #used during the build(compilation) process.
$<INSTALL_INTERFACE:include>) #used after the target is installed.
ament_target_dependencies(
  ${PROJECT_NAME} 
  PUBLIC 
  rclcpp
  hyy_message
  sensor_msgs
)

#Copy the project's header files or included directories to the target installation path during the installation phase.
install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

#Specifies the directory where runtime components of executable files and dynamic libraries should be installed.
install( 
  TARGETS ${PROJECT_NAME} 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
