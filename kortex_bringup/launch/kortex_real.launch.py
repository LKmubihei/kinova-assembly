from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip")

    kinova_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kinova_gen3_7dof_robotiq_2f_85_moveit_config"), "/launch", "/kinova_moveit_driver.launch.py"]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": "false",
            "use_sim_time": "false",
            "sim_gazebo": "false",
            "launch_rviz": "true",
        }.items(),
    )

    return [kinova_moveit_launch]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address by which the robot can be reached.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
