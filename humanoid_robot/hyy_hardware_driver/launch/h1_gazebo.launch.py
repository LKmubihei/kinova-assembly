from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="h1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="h1.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_default_controllers",
            default_value="true",
            description="If true then use default controllers, otherwise use HYY controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "add_external_devices",
            default_value="false",
            description="If true, add external devices to the robot description.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_default_controllers = LaunchConfiguration("use_default_controllers")
    add_external_devices = LaunchConfiguration("add_external_devices")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # 启动gazebo
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both')

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', "h1_robot"],
        output='screen'
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
        
    load_default_left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['default_left_arm_controller',"-c", "/controller_manager"],
        condition=IfCondition(use_default_controllers)
    )
    load_default_right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['default_right_arm_controller',"-c", "/controller_manager"],
        condition=IfCondition(use_default_controllers)
    )
    load_default_body_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['default_body_controller',"-c", "/controller_manager"],
        condition=IfCondition(use_default_controllers)
    )
    load_default_head_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['default_head_controller',"-c", "/controller_manager"],
        condition=IfCondition(use_default_controllers)
    )
    
    load_hyy_left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_left_arm_controller',"-c", "/controller_manager"],
        condition=UnlessCondition(use_default_controllers)
    )

    load_hyy_right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_right_arm_controller',"-c", "/controller_manager"],
        condition=UnlessCondition(use_default_controllers)
    )

    load_hyy_body_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_body_controller',"-c", "/controller_manager"],
        condition=UnlessCondition(use_default_controllers)
    )

    load_hyy_head_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_head_controller',"-c", "/controller_manager"],
        condition=UnlessCondition(use_default_controllers)
    )
    
    # if add_external_devices & !use_default_controllers
    load_hyy_external_device_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_external_device_controller',"-c", "/controller_manager"],
        condition= UnlessCondition(use_default_controllers) and IfCondition(add_external_devices)
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit = [
                    TimerAction(
                        period=2.0,
                        actions=[joint_state_broadcaster_spawner],
                    ),
                ],
            ),
        ),
    )

    # Delay rviz start after Joint State Broadcaster to avoid unnecessary warning output.
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                            load_default_left_arm_controller, 
                            load_default_right_arm_controller,
                            load_default_body_controller,
                            load_default_head_controller,
                            load_hyy_left_arm_controller,
                            load_hyy_right_arm_controller,
                            load_hyy_body_controller,
                            load_hyy_head_controller,
                            load_hyy_external_device_controller
                             ]
                )
            ]
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            start_gazebo_cmd,
            robot_state_pub_node,
            spawn_entity,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner,
        ]
    )
