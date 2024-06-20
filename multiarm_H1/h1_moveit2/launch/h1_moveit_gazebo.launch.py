import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
    IncludeLaunchDescription
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
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
            "sim_gazebo_classic",
            default_value="true",
            description="Use simulation (Gazebo).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "load_rviz",
            default_value="true",
            description="if load rviz file.",
        )
    )
    sim_gazebo_classic = LaunchConfiguration('sim_gazebo_classic')
    load_rviz = LaunchConfiguration('load_rviz')

    h1_driver_launch_file = os.path.join(get_package_share_directory('hyy_hardware_driver'), 'launch', 'h1_driver.launch.py')

    gazebo_driver = IncludeLaunchDescription(PythonLaunchDescriptionSource(h1_driver_launch_file),
        launch_arguments={'sim_gazebo_classic': sim_gazebo_classic}.items())

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name = "xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("h1_description"), "urdf", "h1.xacro"]
            ),
            ' sim_gazebo_classic:=', sim_gazebo_classic,
        ]
    )
    robot_description = {"robot_description": robot_description_content}


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
    delay_move_group_after_gazebo_driver = TimerAction(
        period=10.0,
        actions=[run_move_group_node, rviz_node_full]
    )

    return LaunchDescription(
        declared_arguments+
        [
            gazebo_driver,
            delay_move_group_after_gazebo_driver
        ]
    )
