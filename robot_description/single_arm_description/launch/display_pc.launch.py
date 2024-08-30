from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="single_arm_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="TC7_in_pc.rviz",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    rviz_file = LaunchConfiguration("rviz_file")

    rviz_config_file_path=PathJoinSubstitution([FindPackageShare(description_package), "config", rviz_file])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d', rviz_config_file_path],
        output="log",
    )

    return LaunchDescription(
        declared_arguments + 
        [
            rviz_node
        ]
    )

