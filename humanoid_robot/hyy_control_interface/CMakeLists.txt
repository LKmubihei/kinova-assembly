#***************************************************************************/
#*                                                                         */
#*   CMake project configuration for hyy_control_interface                 */
#*                                                                         */
#*   - Adds compiler options if using GCC or Clang                         */
#*   - Finds and links required packages                                   */
#*                                                                         */
#***************************************************************************/

cmake_minimum_required(VERSION 3.8)
project(hyy_control_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
#*   Standard application structure - hyyrobot motion planning             */
#*                                                                         */
#*   target_include_directories: Specifies the directory where the         */
#*   compiler should look for header files when compiling a given          */
#*   target (such as an executable or library).                            */
#*                                                                         */
#*   BUILD_INTERFACE: used during the build(compilation) process.          */
#*   INSTALL_INTERFACE: used after the target is installed.                */
#*                                                                         */
#***************************************************************************/

add_executable(h1_demo_cart src/hyy_control_interface.cpp src/h1_demo_cart.cpp)
target_include_directories(h1_demo_cart PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
$<INSTALL_INTERFACE:include>) 
ament_target_dependencies(h1_demo_cart PUBLIC rclcpp hyy_message)

add_executable(h1_demo_pos src/hyy_control_interface.cpp src/h1_demo_pos.cpp)
target_include_directories(h1_demo_pos PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
$<INSTALL_INTERFACE:include>) 
ament_target_dependencies(h1_demo_pos PUBLIC rclcpp hyy_message)

add_executable(t1_demo_pos src/hyy_control_interface.cpp src/t1_demo_pos.cpp)
target_include_directories(t1_demo_pos PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(t1_demo_pos PUBLIC rclcpp hyy_message)

#***************************************************************************/
#*                                                                         */
#*   Standard application structure - Collision detection                  */
#*                                                                         */
#*   This section configures the collision detection executable.           */
#*   The target `collision_detect` is created, and its dependencies are    */
#*   specified for both compilation and linking stages.                    */
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
  Eigen3 # Only the header files are required
)
target_link_libraries(
  collision_detect
  PUBLIC 
  fcl
  ccd
)

#***************************************************************************/
#*                                                                         */
#*   Standard application structure - MoveIt motion planning               */
#*                                                                         */
#*   This section configures the motion planning demo executable for the   */
#*   the MoveIt framework. The target `motion_plan_demo` is created        */
#*   and linked with necessary dependencies.                               */
#*                                                                         */
#***************************************************************************/

add_executable(motion_plan_demo src/h1_moveit_demo.cpp)
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
#*   The following lines specify the required C and C++ language           */
#*   standards for the specified targets. Each target is configured        */
#*   to use the C99 standard for C files and the C++17 standard            */
#*   for C++ files.                                                        */
#*                                                                         */
#***************************************************************************/

target_compile_features(h1_demo_cart PUBLIC c_std_99 cxx_std_17)
target_compile_features(h1_demo_pos  PUBLIC c_std_99 cxx_std_17)
target_compile_features(t1_demo_pos  PUBLIC c_std_99 cxx_std_17)
target_compile_features(collision_detect PUBLIC c_std_99 cxx_std_17)
target_compile_features(motion_plan_demo PUBLIC c_std_99 cxx_std_17)

#***************************************************************************/
#*                                                                         */
#*   Copy the project's header files or included directories to            */
#*   the target installation path during the installation phase.           */
#*                                                                         */
#*   Specifies the directory where runtime components of executable        */
#*   files and dynamic libraries should be installed.                      */
#*                                                                         */
#***************************************************************************/

install(
  DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME}
  )
  
install( 
  TARGETS 
  h1_demo_cart 
  h1_demo_pos 
  t1_demo_pos 
  collision_detect  
  motion_plan_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#***************************************************************************/
#*                                                                         */
#*   If testing is enabled, perform the following steps                    */
#*   This block is executed only when the BUILD_TESTING option is ON.      */
#*                                                                         */
#*   1. Locate and require the ament_lint_auto package, which is           */
#*      necessary for automated linting and code quality checks.           */
#*                                                                         */
#*   2. Skip the linter that checks for copyright and license information  */
#*      in source files by setting ament_cmake_copyright_FOUND to TRUE.    */
#*      This line should be commented out when all source files have       */
#*      proper copyright and license headers.                              */
#*                                                                         */
#*   3. Skip cpplint checks by setting ament_cmake_cpplint_FOUND to TRUE.  */
#*      This setting only takes effect in a git repository environment.    */
#*      Uncomment this line when the package is in a git repository and    */
#*      all source files have appropriate copyright and license headers.   */
#*                                                                         */
#*   4. Automatically locate and configure test dependencies needed        */
#*      for running the tests in this package using                        */
#*      ament_lint_auto_find_test_dependencies().                          */
#*                                                                         */
#***************************************************************************/

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#***************************************************************************/
#*                                                                         */
#*   Declare and configure the ament package                               */
#*   This command sets up the package so that it can be recognized         */
#*   and utilized by the ROS 2 ecosystem.                                  */
#*                                                                         */
#***************************************************************************/

ament_package() 
