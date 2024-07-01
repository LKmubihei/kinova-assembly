import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():
    
    package_share_directory = get_package_share_directory('h1_robot_moveit_config')

    urdf_file_path = os.path.join(package_share_directory, 'config', 'h1_robot.urdf.xacro')
    yaml_file_path = os.path.join(package_share_directory, 'config', 'moveit_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder("h1_robot")
        .robot_description(file_path=urdf_file_path)
        .trajectory_execution(file_path=yaml_file_path)
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[moveit_config.to_dict(),
                    planning_scene_monitor_parameters,
                    {"use_sim_time": True}],
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory("h1_robot_moveit_config"), 'config', "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2555",
        output="both",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True}
        ], 
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="both",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "fake_link"],
        parameters=[{"use_sim_time": True}],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # Load controllers
    load_controllers = []
    controllers = [
        'head_controller',
        'body_controller',
        'left_arm_controller',
        'right_arm_controller',
        'gripper_controller',
        'hand_J1_controller',
        'hand_J2_controller',
        'hand_J3_controller',
        'hand_J4_controller',
        'hand_J5_controller',
        'hand_J6_controller',
    ]
    for controller in controllers:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                output="both"
            )
        )

    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', '/usr/share/gazebo-11/worlds/empty.world', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both',
    )
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'h1_robot'],
        output='both',
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
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=load_controllers
                )
            ]
        )
    )
    
    # Delay loading and activation of move_group after load_controllers
    delay_run_move_group_node_after_load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_controllers[-1],  # Ensure the last controller loaded triggers this event
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[run_move_group_node]  # Wrap the Node in a list
                )
            ]
        )
    )
    
    # Delay loading and activation of move_group after load_controllers
    delay_rviz_node_after_run_move_group_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=run_move_group_node,  # Ensure the last controller loaded triggers this event
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[rviz_node]  # Wrap the Node in a list
                )
            ]
        )
    )
    
    return LaunchDescription(
        [
            start_gazebo_cmd,
            robot_state_publisher,
            static_tf,
            delay_spawn_entity_after_start_gazebo_cmd,
            delay_joint_state_broadcaster_spawner_after_spawn_entity,
            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner,
            delay_run_move_group_node_after_load_controllers,
            delay_rviz_node_after_run_move_group_node
        ]
    )
