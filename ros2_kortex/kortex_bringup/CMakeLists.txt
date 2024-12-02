cmake_minimum_required(VERSION 3.5)
project(kortex_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  scripts/arm_demo.py
  scripts/arm_gripper_demo.py
  scripts/arm_gripper_visual_demo.py
  scripts/arm_objectdet_demo.py
  scripts/gripper_server_test.py
  scripts/topic_visual.py
  scripts/test_huge_joint_space_jump.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
