步骤	详解
安装ros2-humble	按照官网https://docs.ros.org/en/humble/Installation.html
安装ros2-control相关	命令 sudo apt install ros-humble-ros2-control*
安装gazebo相关	命令 sudo apt install ros-humble-gazebo*
安装moveit2相关	命令 sudo apt install ros-humble-moveit*
确保安装colcon工具	命令 sudo apt install python3-colcon-common-extensions
安装Ur10e开发环境	参考教程 https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/README.md
	安装cycloneDDS命令 sudo apt install ros-humble-rmw-cyclonedds-cpp
	配置DDS命令 export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
	构建工作空间命令 mkdir -p ~/ur_ros2_ws/src/
	cd ~/ur_ros2_ws/src/
	git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git -b humble
	git clone https://github.com/UniversalRobots/Universal_Robots_Client_Library.git -b master
	git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b humble
	git clone https://github.com/ros-industrial/ur_msgs.git -b foxy-devel
安装相关依赖	cd ~/ur_ros2_ws
	rosdep install --ignore-src --from-paths src -y 
编译工作空间	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3
	编译结果如右图----------------------------------->
可能会遇到的问题1	Ur_controller功能包依赖于joint-trajectory-controllers包，但是编译报错，在于现有的tolerances.hpp中与源码存在版本差异
	替换步骤：sudo apt remove ros-humble-joint-trajectory-controller
	从ros2 controllers中找到humble分支手动下载joint-trajectory-controller功能包，放在arm_ws工作空间下
	执行命令：colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 3 --allow-overriding joint_trajectory_controller
可能会遇到的问题2	Ur_controller功能包中的scaled_joint_trajectory_controller.cpp依赖于joint-trajectory-controllers包，但是编译报错，部分变量缺少命名空间
	修改步骤：加入using namespace joint_trajectory_controller

