from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bringup_dir = get_package_share_directory('t1_description')
    urdf_config_name = "t1_robot.urdf.xacro"
    
    xacro_file = os.path.join(bringup_dir, 'urdf', urdf_config_name)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        start_joint_state_publisher_cmd,
        start_robot_state_publisher_cmd,
        rviz_cmd
    ])
