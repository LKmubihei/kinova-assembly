import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    SetLaunchConfiguration,
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
            'sim_gazebo_classic', 
            default_value='false', 
            description='if true, then start simulation with Gazebo classic'
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
            "communication_time",
            default_value="1000000",
            description="hyy timer period in ns",
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'device_mode', 
            default_value='8', 
            description='device mode, 8-position 9-velocity 10-effort'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'sim_flag', 
            default_value='false', 
            description='if true, robot run in simulation mode'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'system_arg', 
            default_value='--path /home/robot/Work/system/robot_config --iscopy true', 
            description='System argument for initialize'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'if_add_axisgroups', 
            default_value='true', 
            description='if true, then add axis groups'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'if_add_external_device', 
            default_value='false', 
            description='if true, then add external devices'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'external_device_dof', 
            default_value='7', 
            description='external devices DOF'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'ifstartRviz', 
            default_value='false', 
            description='if true then start Rviz'
        )
    )

    # Initialize Arguments
    sim_gazebo_classic = LaunchConfiguration('sim_gazebo_classic')
    use_default_controllers = LaunchConfiguration("use_default_controllers")
    communication_time = LaunchConfiguration("communication_time")
    device_mode = LaunchConfiguration('device_mode')
    sim_flag = LaunchConfiguration('sim_flag')
    system_arg = LaunchConfiguration('system_arg')
    if_add_axisgroups = LaunchConfiguration('if_add_axisgroups')
    if_add_external_device = LaunchConfiguration('if_add_external_device')
    external_device_dof = LaunchConfiguration('external_device_dof')
    ifstartRviz = LaunchConfiguration('ifstartRviz')

    # Only the default controller is supported.
    set_use_default_controllers = SetLaunchConfiguration(
        'use_default_controllers',
        'true',
        condition=IfCondition(sim_gazebo_classic)
    )
    # Gazebo simulation of external devices is not supported yet.
    set_if_add_external_device = SetLaunchConfiguration(
        'if_add_external_device',
        'false',
        condition=IfCondition(sim_gazebo_classic)
    )

    # Get robot_description(URDF) via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name = "xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("t1_description"), "urdf", "t1.xacro"]
            ),
            ' sim_gazebo_classic:=', sim_gazebo_classic,
            ' device_mode:=', device_mode,
            ' sim_flag:=', sim_flag,
            ' if_add_axisgroups:=', if_add_axisgroups,
            ' if_add_external_device:=', if_add_external_device,
            ' external_device_dof:=', external_device_dof
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get robot_controller config 
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("hyy_hardware_driver"), "config", "t1_controller.yaml"]
    )

    # Start the master driver node 
    control_node = Node(
        package="hyy_hardware_driver",
        executable="hyy_hardware_driver",
        output="both",
        parameters=[{'communication_time':communication_time},
                    robot_description,
                    robot_controllers],
        condition = UnlessCondition(sim_gazebo_classic)
    )
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', '/usr/share/gazebo-11/worlds/empty.world', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both',
        condition =IfCondition(sim_gazebo_classic),
    )
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 't1_robot'],
        output='screen',
        condition =IfCondition(sim_gazebo_classic)
    )
    
    # Start the necessary controller 
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
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
    
    load_hyy_external_device_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_external_device_controller',"-c", "/controller_manager"],
        condition= IfCondition(if_add_external_device)
    )

    # Start rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )
    
    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_spawn_master_driver_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        ),
        condition = UnlessCondition(sim_gazebo_classic)
    )
    
    # Delay spawn_entity after start of gazebo
    delay_spawn_entity_after_start_gazebo_cmd = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_gazebo_cmd,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[spawn_entity],
                ),
            ],
        ),
        condition = IfCondition(sim_gazebo_classic)
    )
    
    # Delay loading and activation of `joint_state_broadcaster` after start of spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        ),
        condition = IfCondition(sim_gazebo_classic)
    )
       
    # Delay rviz start after Joint State Broadcaster to avoid unnecessary warning output.
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[rviz_node],
                ),
            ],
        ),
        condition = IfCondition(ifstartRviz)
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[load_default_left_arm_controller, 
                             load_default_right_arm_controller,
                             load_hyy_left_arm_controller,
                             load_hyy_right_arm_controller,
                             load_hyy_external_device_controller]
                )
            ]
        )
    )
    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_addaxis_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                             load_default_body_controller,
                             load_default_head_controller,
                             load_hyy_body_controller,
                             load_hyy_head_controller]
                            )
            ]
        ),
        condition = IfCondition(if_add_axisgroups)
    )

    return LaunchDescription(
        declared_arguments +
        [set_use_default_controllers, set_if_add_external_device] +
        [
            start_gazebo_cmd,
            control_node,
            robot_state_pub_node,
            delay_spawn_entity_after_start_gazebo_cmd,
            delay_joint_state_broadcaster_spawner_after_spawn_master_driver_node,
            delay_joint_state_broadcaster_spawner_after_spawn_entity,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner,
            delay_addaxis_controller_spawners_after_joint_state_broadcaster_spawner
        ]
    )
