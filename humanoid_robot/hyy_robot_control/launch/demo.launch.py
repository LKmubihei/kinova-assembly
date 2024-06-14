from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    hyy_robot_control_node = Node(
        package="hyy_robot_control",
        executable="hyy_robot_control",
        output="both",
    )
    
    collision_detect_node = Node(
        package="hyy_robot_control",
        executable="collision_detect",
        output="both",
    )
    
    return LaunchDescription([
        hyy_robot_control_node,
        collision_detect_node]
    )
