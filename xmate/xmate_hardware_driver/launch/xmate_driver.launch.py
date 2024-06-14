from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="xmate_hardware_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="xmate_controller.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="xmate_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="xmate_robot.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_default_controllers",
            default_value="false",
            description="If true then use default controllers, otherwise use HYY controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_flag",
            default_value="false",
            description="If true robot run in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "system_arg",
            default_value="--path /home/robot/Work/system/robot_config --iscopy true",
            description="args for system initialize ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "communication_time",
            default_value="1000000",
            description="communication time",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "prefix",
    #         default_value='""',
    #         description="Prefix of the joint names, useful for \
    #     multi-robot setup. If changed than also joint names in the controllers' configuration \
    #     have to be updated.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_mock_hardware",
    #         default_value="true",
    #         description="Start robot with fake hardware mirroring command to its states.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "mock_sensor_commands",
    #         default_value="false",
    #         description="Enable fake command interfaces for sensors used for simple simulations. \
    #         Used only if 'use_mock_hardware' parameter is true.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "robot_controller",
    #         default_value="forward_position_controller",
    #         choices=["forward_position_controller", "joint_trajectory_controller"],
    #         description="Robot controller to start.",
    #     )
    # )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_default_controllers = LaunchConfiguration("use_default_controllers")
    system_arg = LaunchConfiguration("system_arg")
    communication_time = LaunchConfiguration("communication_time")
    # sim_flag = LaunchConfiguration("sim_flag")
    # prefix = LaunchConfiguration("prefix")
    # use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    # mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    # robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            )
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     output="both",
    #     parameters=[robot_description, robot_controllers],
    # )

    control_node = Node(
        package="xmate_hardware_driver",
        executable="xmate_hardware_driver",
        output="both",
        parameters=[
                    {'system_arg':system_arg},
                    {'communication_time':communication_time},
                    robot_description,
                    robot_controllers
                    ],
        # prefix="gdb --args"
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
        # emulate_tty=True,
        # additional_env ={'RCUTILS_LOGGING_SEVERITY_THRESHOLD':"DEBUG"}
    )
        
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['arm_controller',"-c", "/controller_manager"],
        condition=IfCondition(use_default_controllers)
    )
    
    load_hyy_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_arm_controller',"-c", "/controller_manager"],
        condition=UnlessCondition(use_default_controllers)
    )

    # load_left_arm_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['left_arm_controller',"-c", "/controller_manager"],
    #     condition=IfCondition(use_default_controllers)
    # )

    # load_body_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['body_controller',"-c", "/controller_manager"],
    #     condition=IfCondition(use_default_controllers)
    # )

    # load_head_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['head_controller',"-c", "/controller_manager"],
    #     condition=IfCondition(use_default_controllers)
    # )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = (
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[joint_state_broadcaster_spawner],
                    ),
                ],
            )
        )
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
                    actions=[load_arm_controller, load_hyy_arm_controller]
                )
            ]
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        ]
    )
