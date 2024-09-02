## hyy_hardware_driver
# This package is mainly used to start the driver node, take h1 robot as an example.
1. Start the ros2 driver master node, using the hyy controller.
Controller-side: ros2 launch hyy_hardware_driver h1_driver.launch.py use_default_controllers:=false if_add_external_device:=true

2. Start the ros2 driver master node, using the moveit2(default controller).
Controller-side: ros2 launch hyy_hardware_driver h1_driver.launch.py use_default_controllers:=true if_add_external_device:=true
PC-side: ros2 launch hyy_hardware_driver h1_moveit_driver.launch.py if_add_external_device:=true ifstartRviz:=true

3. Start the PC-side Gazebo driver master node, using the Moveit controller, for simulation.
PC-side: ros2 launch hyy_hardware_driver h1_moveit_gazbo_driver.launch.py if_add_external_device:=true ifstartRviz:=true use_sim_time:=true

Note: The parameters of each launch file can be adjusted according to actual conditions.