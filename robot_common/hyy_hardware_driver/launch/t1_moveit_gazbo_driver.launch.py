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

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
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
            default_value='true', 
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
            default_value='true', 
            description='if true then start Rviz'
        )
    )
    declared_arguments.append( 
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='if true then start Rviz'
        )
    )

    # Initialize Arguments
    if_add_axisgroups = LaunchConfiguration('if_add_axisgroups')
    if_add_external_device = LaunchConfiguration('if_add_external_device')
    external_device_dof = LaunchConfiguration('external_device_dof')
    ifstartRviz = LaunchConfiguration('ifstartRviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get robot_description(URDF) via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name = "xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("t1_description"), "urdf", "t1.xacro"]
            ),
            ' sim_gazebo_classic:=', "true",
            ' if_add_axisgroups:=', if_add_axisgroups,
            ' if_add_external_device:=', if_add_external_device,
            ' external_device_dof:=', external_device_dof
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    #**********************************#
    #         Moveit2 config           #
    #**********************************#
    
    yaml_file_path = os.path.join(get_package_share_directory('t1_robot_moveit_config'), 'config', 'moveit_controllers.yaml')
    moveit_config = (
        MoveItConfigsBuilder("t1_robot")
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
                    robot_description,
                    {"use_sim_time": use_sim_time}
        ],
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory("t1_robot_moveit_config"), 'config', "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2555",
        output="both",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(),
                    robot_description,
                    {"use_sim_time": use_sim_time}
        ], 
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="both",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "fake_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    #**********************************#
    #         Hradware config          #
    #**********************************#

    # Start the master driver node 
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose', '/usr/share/gazebo-11/worlds/empty.world', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both',
    )
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 't1_robot'],
        output='screen',
    )
    
    #**********************************#
    #        Controller config         #
    #**********************************#

    # Start the necessary controller 
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{"use_sim_time": use_sim_time}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
        
    load_default_left_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['left_arm_controller',"-c", "/controller_manager"],
    )
    
    load_default_right_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['right_arm_controller',"-c", "/controller_manager"],
    )
    
    load_default_body_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['body_controller',"-c", "/controller_manager"], 
    )
    
    load_default_head_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['head_controller',"-c", "/controller_manager"], 
    )
    
    load_default_hand_controller_1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hand_J1_controller',"-c", "/controller_manager"],
    )
    
    load_default_hand_controller_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hand_J2_controller',"-c", "/controller_manager"],
    )
    
    load_default_hand_controller_3 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hand_J3_controller',"-c", "/controller_manager"],
    )
    
    load_default_hand_controller_4 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hand_J4_controller',"-c", "/controller_manager"],
    )
    
    load_default_hand_controller_5 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hand_J5_controller',"-c", "/controller_manager"],
    )
    
    load_default_hand_controller_6 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hand_J6_controller',"-c", "/controller_manager"],
    )
    
    load_default_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['gripper_controller',"-c", "/controller_manager"],
    )

    load_hyy_external_device_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_external_device_controller',"-c", "/controller_manager"],
    )

    #**********************************#
    #       Boot sequence config       #
    #**********************************#
    
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

    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[load_default_left_arm_controller, 
                             load_default_right_arm_controller
                             ]
                )
            ]
        )
    )

    delay_default_external_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[load_default_hand_controller_1,
                             load_default_hand_controller_2,
                             load_default_hand_controller_3,
                             load_default_hand_controller_4,
                             load_default_hand_controller_5,
                             load_default_hand_controller_6,
                             load_default_gripper_controller,
                             ]
                )
            ]
        ),
    )
 
    delay_additionaxis_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                             load_default_body_controller,
                             load_default_head_controller,]
                            )
            ]
        ),
        condition = IfCondition(if_add_axisgroups)
    )
    
    delay_rviz_node_after_run_move_group_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=run_move_group_node,  # Ensure the last controller loaded triggers this event
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[rviz_node]  # Wrap the Node in a list
                )
            ]
        ),
        condition = IfCondition(ifstartRviz)
    )

    #**********************************#
    #              Boot                #
    #**********************************#

    return LaunchDescription(
        declared_arguments +
        [
            start_gazebo_cmd,
            robot_state_pub_node,
            static_tf,
            delay_spawn_entity_after_start_gazebo_cmd,
            delay_joint_state_broadcaster_spawner_after_spawn_entity,
            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner,
            delay_additionaxis_controller_spawners_after_joint_state_broadcaster_spawner,
            delay_default_external_controller_spawners_after_joint_state_broadcaster_spawner,
            run_move_group_node,
            delay_rviz_node_after_run_move_group_node
        ]
    )
