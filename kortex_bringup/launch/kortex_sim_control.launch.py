# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Marq Rasmussen

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")
    robot_type = LaunchConfiguration("robot_type")
    dof = LaunchConfiguration("dof")
    vision = LaunchConfiguration("vision")
    # General arguments
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    robot_traj_controller = LaunchConfiguration("robot_controller")
    robot_pos_controller = LaunchConfiguration("robot_pos_controller")
    robot_hand_controller = LaunchConfiguration("robot_hand_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gripper = LaunchConfiguration("gripper")
    robot_controllers = PathJoinSubstitution(
        # https://answers.ros.org/question/397123/how-to-access-the-runtime-value-of-a-launchconfiguration-instance-within-custom-launch-code-injected-via-an-opaquefunction-in-ros2/
        [
            FindPackageShare(description_package),
            "arms/" + robot_type.perform(context) + "/" + dof.perform(context) + "dof/config",
            controllers_file,
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "robots", description_file]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "name:=",
            robot_name,
            " ",
            "arm:=",
            robot_type,
            " ",
            "dof:=",
            dof,
            " ",
            "vision:=",
            vision,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=",
            sim_gazebo,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "simulation_controllers:=",
            robot_controllers,
            " ",
            "gripper:=",
            gripper,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content.perform(context)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_traj_controller, "-c", "/controller_manager"],
    )

    robot_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_pos_controller, "--inactive", "-c", "/controller_manager"],
    )

    robot_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_hand_controller, "-c", "/controller_manager"],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": "true",
            # "world": "/home/robotck/kinova_ws/src/ros2_kortex/kortex_bringup/worlds/geer17.world",
            # "world": "/home/lk/kinova_ws/src/ros2_kortex/kortex_bringup/worlds/assembly.world",
            "world": os.path.join(get_package_share_directory("kortex_bringup"), "models", "ee.world"),
            # "verbose": "True"
        }.items(),
    )

    # Spawn robot
    # 机械臂生成位置：将机械臂底座放在装配台上
    # 桌面位置: x=-0.25, y=0, 桌面高度: z=1.01
    # 机械臂放在桌子边缘附近，方便够到零件
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        arguments=[
                    "-entity", robot_name, 
                    "-topic", "robot_description",
                    "-x", "-1.0",      # X 位置(红色线)：桌子前方
                    "-y", "-0.12",      # Y 位置（绿色线）：桌子中央
                    "-z", "1.01",     # Z 位置（蓝色线）：桌面高度
                    "-Y", "1.57"       # Yaw  0 ， 1.57 ， 3.14
                ],
        output="screen",
        condition=IfCondition(sim_gazebo),
    )

    assembly_part_spawner_node = Node(
        package="kortex_bringup",
        executable="assembly_part_spawner.py",
        name="assembly_part_spawner",
        output="screen",
        parameters=[
            {
                "models_path": os.path.join(get_package_share_directory("kortex_bringup"), "models"),
                "parts_yaml": "\n".join(
                    [
                        "green_battery: [-0.95, 0.35, 1.025, 0.0, 0.0, 0.0]",
                        # "blue_pump: [-0.95, 0.35, 1.025, 0.0, 0.0, 0.0]",
                        # "red_regulator: [-0.35, -0.10, 1.025, 0.0, 0.0, 0.0]",
                        # "orange_sensor: [-0.35, 0.10, 1.025, 0.0, 0.0, 0.0]",
                    ]
                ),
            }
        ],
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        rviz_node,
        robot_traj_controller_spawner,
        robot_pos_controller_spawner,
        robot_hand_controller_spawner,
        gazebo,
        gazebo_spawn_robot,
        assembly_part_spawner_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # 设置 Gazebo 模型路径，添加自定义模型目录
    models_path = os.path.join(get_package_share_directory("kortex_bringup"), "models")
    current_gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if current_gazebo_model_path:
        new_gazebo_model_path = models_path + ':' + current_gazebo_model_path
    else:
        new_gazebo_model_path = models_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_gazebo_model_path
    )

    # 设置 Gazebo 插件路径，确保 libgazebo_grasp_fix.so 可以被加载
    # gazebo_grasp_plugin 与 kortex_bringup 在同一 workspace，通过 install 目录推导
    _install_base = os.path.dirname(get_package_prefix("kortex_bringup"))
    plugin_path = os.path.join(_install_base, "gazebo_grasp_plugin", "lib")
    current_gazebo_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    if current_gazebo_plugin_path:
        new_gazebo_plugin_path = plugin_path + ':' + current_gazebo_plugin_path
    else:
        new_gazebo_plugin_path = plugin_path

    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=new_gazebo_plugin_path
    )
    
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Use Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="true",
            description="Use Gazebo Classic for simulation",
        )
    )
    # Robot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            description="Type/series of robot.",
            choices=["gen3", "gen3_lite"],
            default_value="gen3",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            description="DoF of robot.",
            choices=["6", "7"],
            default_value="7",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            description="Use arm mounted realsense",
            choices=["true", "false"],
            default_value="true",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="kortex_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="kinova.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="gen3",
            description="Robot name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_pos_controller",
            default_value="twist_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_hand_controller",
            default_value="robotiq_gripper_controller",
            description="Robot hand controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            choices=["robotiq_2f_85", "robotiq_2f_140", "gen3_lite_2f"],
            description="Gripper to use",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )

    return LaunchDescription([set_gazebo_model_path, set_gazebo_plugin_path] + declared_arguments + [OpaqueFunction(function=launch_setup)])
