import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
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

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="h1_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="h1.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_default_controllers",
            default_value="true",
            description="If true then use default controllers, otherwise use HYY controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "add_external_devices",
            default_value="false",
            description="If true, add external devices to the robot description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "load_rviz", 
            default_value="true", 
            description="If true, load RVIZ file."
        )
    )
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_default_controllers = LaunchConfiguration("use_default_controllers")
    add_external_devices = LaunchConfiguration("add_external_devices")
    load_rviz = LaunchConfiguration("load_rviz")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # 启动gazebo
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both')

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', "h1_robot"],
        output='screen'
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
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
    
    # if add_external_devices & !use_default_controllers
    load_hyy_external_device_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['hyy_external_device_controller',"-c", "/controller_manager"],
        condition= UnlessCondition(use_default_controllers) and IfCondition(add_external_devices)
    )
    
    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    robot_description_semantic_config = load_file("h1_moveit2", "config/h1_robot.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }
    
    kinematics_yaml = load_yaml("h1_moveit2", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("h1_moveit2", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("h1_moveit2", "config/h1_robot_controller.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
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
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # {"monitor_dynamics": False},
            {"use_sim_time": True} 
        ],
    )

    # RVIZ:
    
    rviz_base = os.path.join(get_package_share_directory("dual_arm_moveit"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(load_rviz),
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit = [
                    TimerAction(
                        period=5.0,
                        actions=[
                            rviz_node_full,
                            run_move_group_node
                        ]
                    ),
                    joint_state_broadcaster_spawner
                ],
            )
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                            load_default_left_arm_controller, 
                            load_default_right_arm_controller,
                            load_default_body_controller,
                            load_default_head_controller,
                            load_hyy_left_arm_controller,
                            load_hyy_right_arm_controller,
                            load_hyy_body_controller,
                            load_hyy_head_controller,
                            load_hyy_external_device_controller
                             ]
                )
            ]
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            start_gazebo_cmd,
            robot_state_pub_node,
            spawn_entity,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            delay_robot_controller_spawners_after_joint_state_broadcaster_spawner,
        ]
    )
