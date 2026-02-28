from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 必须导入

def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")
    sim_gazebo = LaunchConfiguration("sim_gazebo")
    robot_type = LaunchConfiguration("robot_type")
    robot_name = LaunchConfiguration("robot_name")

    kinova_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kortex_bringup"), "/launch", "/kortex_sim_control.launch.py"]
        ),
        launch_arguments={
            "sim_gazebo": "false",  # 不使用 Gazebo 仿真
            "robot_ip": robot_ip,
            "robot_type": robot_type,
            "robot_name": robot_name,
            "use_sim_time": "true",  # 使用模拟时间
        }.items(),
    )

    kinova_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kinova_gen3_7dof_robotiq_2f_85_moveit_config"), "/launch", "/kinova_moveit_driver.launch.py"]
        ),
        launch_arguments={
            "robot_ip": robot_ip,  # 连接到实际机器人
            "use_fake_hardware": "false",  # 不使用虚拟硬件
            "use_sim_time": "true",  # 使用模拟时间
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        kinova_control_launch,
        kinova_moveit_launch
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo",
            default_value="false",  # 禁用 Gazebo 仿真
            description="Use Gazebo Classic for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="gen3",
            description="Robot type",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="gen3",
            description="Robot name.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
