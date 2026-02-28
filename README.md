# 安装说明
ros 要求 humble 
python包主要是  unified_planning 这个库。其余的包，如果运行出问题再安装

1. 第一次所有包编译使用  colcon build --cmake-args -DPROTOBUF_PROTOC_EXECUTABLE=/usr/bin/protoc    命令编译， 之后可以直接使用 colcon build
2.编译会报错
/home/lk/workspace/src/hyy_actions/scripts/moveJs_action.cpp:43:10: fatal error: moveit/move_group_interface/move_group_interface_improved.h: 没有那个文件或目录
   43 | #include <moveit/move_group_interface/move_group_interface_improved.h>
解决：
sudo mv /home/lk/workspace/src/move_group_interface_improved.h /opt/ros/humble/include/moveit/move_group_interface/
(换一下 /home/lk/workspace 为你自己的路径)



# 启动机器人界面
gazebo+moveit仿真（gen3机械臂+robotiq85+vision）：
ros2 launch kortex_bringup kortex_sim_moveit_control.launch.py
     
单gazebo仿真环境：
ros2 launch kortex_bringup kortex_sim_control.launch.py

真机使用：
ros2 launch kortex_bringup kortex_real.launch.py 

启动相机话题发布
ros2 launch kinova_vision kinova_vision.launch.py 

<!-- # 开启夹爪服务(大寰夹爪)
sudo chmod 777 /dev/ttyUSB0 
ros2 run vcu_bridge Test -->

# 控制机器人
真机：
ros2 run kortex_bringup arm_demo.py 
ros2 run kortex_bringup arm_gripper_demo.py 
ros2 run kortex_bringup arm_gripper_visual_demo.py 
ros2 run kortex_bringup arm_objectdet_demo.py 
仿真：
ros2 run kortex_bringup grasp_green_battery.py  ## 抓绿色电池
ros2 run kortex_bringup arm_sim_demo.py 
ros2 run kortex_bringup arm_gripper_sim_demo.py 
ros2 run kortex_bringup arm_gripper_visual_sim_demo.py 
ros2 run kortex_bringup arm_objectdet_sim_demo.py 