cmake_minimum_required(VERSION 3.5)
project(kortex_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables
install(PROGRAMS
  # original
  scripts/test_huge_joint_space_jump.py
  # added here
  scripts/realrobot/arm_demo.py
  scripts/realrobot/arm_gripper_demo.py
  scripts/realrobot/arm_gripper_visual_demo.py
  scripts/realrobot/arm_objectdet_demo.py
  scripts/realrobot/assembly_demo.py
  # sim
  scripts/sim/arm_sim_demo.py
  scripts/sim/grasp_green_battery.py
  scripts/sim/arm_gripper_sim_demo.py
  scripts/sim/arm_gripper_visual_sim_demo.py
  scripts/sim/arm_objectdet_sim_demo.py
  scripts/sim/assembly_part_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
