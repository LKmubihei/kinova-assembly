cmake_minimum_required(VERSION 3.5)
project(ur_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_ursim.sh
  scripts/arm_demo.py
  scripts/arm_gripper_demo.py
  scripts/arm_gripper_visual_demo.py
  scripts/ur_base_tf_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
