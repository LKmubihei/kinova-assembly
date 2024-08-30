import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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

    # Initialize Arguments
    if_add_axisgroups = LaunchConfiguration('if_add_axisgroups')
    if_add_external_device = LaunchConfiguration('if_add_external_device')
    
    # Get robot_description(URDF) via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name = "xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("h1_description"), "urdf", "h1.xacro"]
            ),
            ' if_add_axisgroups:=', if_add_axisgroups,
            ' if_add_external_device:=', if_add_external_device,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    moveit_config = MoveItConfigsBuilder("h1_robot").to_moveit_configs()
    
    run_motion_plan_demo_node = Node(
        package="hyy_control_interface",
        executable="motion_plan_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description
        ],
    )
    
    return LaunchDescription(
        declared_arguments +
        [
            run_motion_plan_demo_node
        ]
    )
    
    
    
    
    