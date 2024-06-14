cmake_minimum_required(VERSION 3.8)
project(single_arm_hardware_interface)

#Add additional compilation options for all targets when the compiler is GNU GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall #Enable all warning messages
    -Wextra #Enable additional warning messages
    -Wpedantic #Requires the compiler to follow the C++ standard and report any code that does not conform to the standard
  )
endif()

#find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#Create a library target.
add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
#Specify necessary compiler features for the target(Require C99 and C++17).
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

#Specifies the directory where the compiler should look for header files when compiling a given target (such as an executable or library).
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #used during the build(compilation) process.
$<INSTALL_INTERFACE:include>) #used after the target is installed.

#Add the library search directory for linking.
# LINK_DIRECTORIES(/home/robotck/Buaa_ws/HYYRobotX64GRIP/depend/robot_base/lib) 
# find_library(ROBOT_CONTROL_LIB RobotControl 
#              PATHS /home/robotck/Buaa_ws/HYYRobotX64GRIP/depend/robot_base/lib  
#              NO_DEFAULT_PATH)

#Add include directories for the specified target and link required libraries.
ament_target_dependencies(
  ${PROJECT_NAME} 
  PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# set_target_properties(${PROJECT_NAME} PROPERTIES 
#   BUILD_RPATH "/home/robotck/Buaa_ws/HYYRobotX64GRIP/depend/robot_base/lib"
#   INSTALL_RPATH "/home/robotck/Buaa_ws/HYYRobotX64GRIP/depend/robot_base/lib"
# )

#Link binary libraries (i.e. dynamic libraries or static libraries) to a target.
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC 
  # /home/robotck/Buaa_ws/HYYRobotX64GRIP/depend/robot_base/lib/libRobotControl.so
  RobotControl
  modbus #Implement Modbus communication protocol.
  #nlopt #Nonlinear optimization library.
  pthread #POSIX thread ibrary is used to implement multi-threaded programming.
  rt #Real-time library provides a series of functions related to real-time programming.
  m #Mathematics library, including various standard mathematical operation functions.
  dl #Dynamic loading library provides functions for dynamically loading and linking shared libraries.
  xml2 #Library for parsing XML documents.
)

#prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#Help export plugin description files when using the pluginlib library
pluginlib_export_plugin_description_file(hardware_interface single_arm_hardware_interface_plugin.xml)

#Copy the project's header files or included directories to the target installation path during the installation phase.
install(
  DIRECTORY include/
  DESTINATION include
)

#Export the target as a CMake export file (usually a .cmake file), so that other CMake projects can find and use it via the find_package command.
#Specifies the directory where static libraries (.a or .lib files) should be installed.
#Specifies the directory where dynamic libraries (shared libraries, .so, .dll or .dylib files) should be installed.
#Specifies the directory where runtime components of executable files and dynamic libraries should be installed.
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME} 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#Set test options
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)comment the line when this package is in a git repo 
  # and when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the include directory so that other packages that depend on this package can find and include the package's header files at compile time.
ament_export_include_directories(include)

# Export libraries, enabling other packages that depend on this package to find and link the libraries provided by this package at link time.
ament_export_libraries(${PROJECT_NAME})

# Export a CMake target (e.g., a library or executable) and associate it with an export name (in this case, export_${PROJECT_NAME}).
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export other packages this package depends for other packages depend on this package to automatically get this dependency information.
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Generate and install all necessary CMake configuration files so that other packages can find and use this package using find_package.
ament_package()
