cmake_minimum_required(VERSION 3.8)

project(hyy_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#***************************************************************************/
#*                                                                         */
#*   Standard application structure   dependencies                         */
#*                                                                         */
#***************************************************************************/

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hyy_message REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fcl REQUIRED)
find_package(ccd REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

#***************************************************************************/
#*                                                                         */
#*   Standard application structure   hyy_robot_control configure          */
#*                                                                         */
#***************************************************************************/

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
)
  
#***************************************************************************/
#*                                                                         */
#*   Standard application structure   collision_detect configure           */
#*                                                                         */
#***************************************************************************/

add_executable(collision_detect src/collision_detect.cpp)
ament_target_dependencies(
  collision_detect
  PUBLIC 
  sensor_msgs
  rclcpp
  hyy_message
  kdl_parser
  orocos_kdl
  Eigen3 #只需要处理头文件目录
)
target_link_libraries(
  collision_detect
  PUBLIC 
  fcl
  ccd
)

#***************************************************************************/
#*                                                                         */
#*   Standard application structure   moveit motion planning configure     */
#*                                                                         */
#***************************************************************************/

add_executable(motion_plan_demo src/motion_plan_demo.cpp)
ament_target_dependencies(
  motion_plan_demo 
  PUBLIC
  rclcpp 
  std_msgs 
  moveit_ros_planning_interface 
  moveit_visual_tools
)

#***************************************************************************/
#*                                                                         */
#*   Standard application structure   install                              */
#*                                                                         */
#***************************************************************************/

#Copy the project's header files or included directories to the target installation path during the installation phase.
install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
  )
  
#Specifies the directory where runtime components of executable files and dynamic libraries should be installed.
install( 
  TARGETS ${PROJECT_NAME} collision_detect motion_plan_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#***************************************************************************/
#*                                                                         */
#*   Standard application structure   test                                 */
#*                                                                         */
#***************************************************************************/

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
