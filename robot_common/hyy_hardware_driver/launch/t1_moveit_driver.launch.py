import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction
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
            'ifstartRviz', 
            default_value='true', 
            description='if true then start Rviz'
        )
    )

    # Initialize Arguments
    if_add_axisgroups = LaunchConfiguration('if_add_axisgroups')
    if_add_external_device = LaunchConfiguration('if_add_external_device')
    ifstartRviz = LaunchConfiguration('ifstartRviz')
    
    # Get robot_description(URDF) via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name = "xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("t1_description"), "urdf", "t1.xacro"]
            ),
            ' if_add_axisgroups:=', if_add_axisgroups,
            ' if_add_external_device:=', if_add_external_device,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
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
        ],
    )

    # RViz
    rviz_config_file = os.path.join(get_package_share_directory("t1_robot_moveit_config"), 'config', "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config_file]
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="both",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "fake_link"],
        parameters=[],
    )

    # t1 base TF
    t1_base_tf = Node(
        package="hyy_control_interface",
        executable="t1_base_tf_pub.py",
        output="both",
    )
  
    moveXYZW_interface = Node(
        package="hyy_actions",
        executable="moveXYZW_action",
        output="screen",
        parameters=[moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    robot_description,
                     {"use_sim_time": False},
                     {"control_group": 'left_arm'},
                     {"base_frame": 't1_base'}
                    ]
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

    return LaunchDescription(
        declared_arguments +
        [
            static_tf,
            t1_base_tf,
            run_move_group_node,
            moveXYZW_interface,
            delay_rviz_node_after_run_move_group_node
        ]
    )
