import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    
    # declare some variables.
    package_name = 'dual_arm_gazebo'
    robot_name = 'dual_arm'
    controllor_config_name = "dual_arm_control.yaml"
    urdf_config_name = "dual_arm_gazebo.urdf.xacro"
    
    dual_arm_gazebo_path = os.path.join(get_package_share_directory(package_name))
    controllor_config_path = os.path.join(dual_arm_gazebo_path, 'config', controllor_config_name)

    # 启动gazebo
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both')
    
	# 读取xacro
    xacro_file = os.path.join(dual_arm_gazebo_path, 'urdf', urdf_config_name)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = {'robot_description': doc.toxml()}
        
	# 启动机器人状态发布节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_xml],
        output='screen'
    )
    
	# 把机器人模型加载到gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', robot_name],
        output='screen'
    )
    
    # 关节状态发布器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 机器人右臂/左臂/腰部/头部运动控制器
    # load_right_arm_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'right_arm_controller'],
    #     output='screen'
    # )
    # load_left_arm_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'left_arm_controller'],
    #     output='screen'
    # )
    # load_body_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'body_controller'],
    #     output='screen'
    # )
    # load_head_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'head_controller'],
    #     output='screen'
    # )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_state_broadcaster']
    )

    load_right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['right_arm_controller']
    )

    load_left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['left_arm_controller']
    )

    load_body_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['body_controller']
    )

    load_head_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['head_controller']
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([

        RegisterEventHandler(
                    OnProcessExit(
                        target_action = spawn_entity,
                        on_exit = [
                            load_joint_state_broadcaster
                        ]
                    )
                ),

        RegisterEventHandler(
                    OnProcessExit(
                        target_action = load_joint_state_broadcaster,
                        on_exit = [
                            rviz_cmd,
                            load_right_arm_controller,
                            load_left_arm_controller,
                            load_body_controller,
                            load_head_controller
                        ]
                    )
                ),
        start_gazebo_cmd,
        node_robot_state_publisher,
        spawn_entity
    ]
)
