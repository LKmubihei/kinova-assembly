cmake_minimum_required(VERSION 3.8)
project(hyy_message)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Robotgrip.srv"
  "srv/Robotio.srv"
  "srv/Robotmove.srv"
  "srv/Robotmovedata.srv" 
  "srv/Robotgeneralcontrol.srv"
  "srv/Setangle.srv"
  "srv/Getangleact.srv"
  "srv/Getangleset.srv"
  "srv/Setpos.srv"
  "srv/Getposact.srv"
  "srv/Getposset.srv"
  "srv/Setspeed.srv"
  "srv/Getspeedset.srv"
  "srv/Setforce.srv"
  "srv/Getforceact.srv"
  "srv/Getforceset.srv"
  "srv/Getcurrentact.srv"
  "srv/Geterror.srv"
  "srv/Gettemp.srv"
  "msg/Detect.msg"
  "msg/JointPose.msg"
  "msg/JointPoseS.msg"
  "action/MoveJ.action"
  "action/MoveJs.action"
  "action/MoveG.action"
  "action/MoveXYZW.action"
  "action/MoveL.action"
  "action/MoveR.action"
  "action/MoveXYZ.action"
  "action/MoveYPR.action"
  "action/MoveROT.action"
  "action/MoveRP.action"
  DEPENDENCIES std_msgs
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
