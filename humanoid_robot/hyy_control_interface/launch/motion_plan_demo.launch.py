from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("h1_robot").to_moveit_configs()

    run_motion_plan_demo_node = Node(
        package="hyy_robot_control",
        executable="motion_plan_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
    )
    
    return LaunchDescription([run_motion_plan_demo_node])
    
    
    
    
    
    
    