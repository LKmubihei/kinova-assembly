# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
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
    
    
# ========== **GENERATE LAUNCH DESCRIPTION** ========== #

def generate_launch_description():

    # *********************** Gazebo *********************** # 
    
    # declare some variables.
    package_name = 'dual_arm_gazebo'
    robot_name = 'dual_arm'
    controllor_config_name = "dual_arm_control.yaml"
    urdf_config_name = "dual_arm_gazebo.urdf.xacro"
    
    dual_arm_gazebo_path = os.path.join(get_package_share_directory(package_name))
    controllor_config_path = os.path.join(dual_arm_gazebo_path, 'config', controllor_config_name)

    # 启动gazebo
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='both')
    
	# 读取xacro
    xacro_file = os.path.join(dual_arm_gazebo_path, 'urdf', urdf_config_name)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = {'robot_description': doc.toxml()}
    
	# 把机器人模型加载到gazebo
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', robot_name],
        output='screen'
    )
    
	# 启动机器人状态发布节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # parameters=[robot_description_xml,{'use_sim_time': True}],
        parameters=[robot_description_xml],
        output='screen'
    )
    
    # ***** ROS2_CONTROL -> LOAD CONTROLLERS ***** #
    
    # 关节状态发布器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # 机器人右臂运动控制器
    load_right_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_arm_controller'],
        output='screen'
    )
    # 机器人左臂运动控制器
    load_left_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_arm_controller'],
        output='screen'
    )
    # 机器人腰部控制器
    load_body_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'body_controller'],
        output='screen'
    )
    # 机器人头部控制器
    load_head_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'head_controller'],
        output='screen'
    )
    
    # *********************** MoveIt!2 *********************** #   
    
    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", 
        default_value="False", 
        description="Load RVIZ file."
    )
    
    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    robot_description_semantic_config = load_file("dual_arm_moveit", "config/dual_arm.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config }
    
    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("dual_arm_moveit", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("dual_arm_moveit", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml("dual_arm_moveit", "config/dual_arm_controllers.yaml")
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
            robot_description_xml,
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
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("dual_arm_moveit"), "config")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description_xml,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(load_RVIZfile),
    )
    
    return LaunchDescription([
        
        RegisterEventHandler(
                    OnProcessExit(
                        target_action = spawn_entity,
                        on_exit = [
                            # MoveIt!2:
                            TimerAction(
                                period=5.0,
                                actions=[
                                    rviz_arg,
                                    rviz_node_full,
                                    run_move_group_node
                                ]
                            ),
                            load_joint_state_broadcaster
                        ]
                    )
                ),

        RegisterEventHandler(
                    OnProcessExit(
                        target_action = load_joint_state_broadcaster,
                        on_exit = [
                            load_right_arm_controller,
                            load_left_arm_controller,
                            load_body_controller,
                            load_head_controller
                        ]
                    )
                ),
        start_gazebo_cmd,
        node_robot_state_publisher,
        spawn_entity
        
    ])