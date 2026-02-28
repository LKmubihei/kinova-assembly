from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='robotiq_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='robotiq_2f_85_gripper.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    start_rviz = LaunchConfiguration('start_rviz')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ', 
            PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
            ' ',
            'prefix:=', 
            prefix, 
        ]
    )

    robot_description = {'robot_description': robot_description_content}


    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "srdf", "robotiq_2f_85.srdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            'description_package:=',
            description_package,
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare(description_package), "config", "robotiq_2f_85_joint_limits.yaml",
        ]
    )

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare(description_package), "config", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare(description_package), "config", "robotiq_2f_85_ompl_planning.yaml",
        ]
    )
    
    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package),
            "config", "robotiq_2f_85_moveit_controllers.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description":True,
        "publish_robot_description_semantic":True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            # robot_description_kinematics,
            robot_description_planning_joint_limits,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'robotiq_2f_85.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_joint_limits,
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        move_group_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)