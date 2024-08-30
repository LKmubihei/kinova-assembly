import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('xmate_description')
    urdf_config_name = "xmate.urdf.xacro"
    
    xacro_file = os.path.join(bringup_dir, 'urdf', urdf_config_name)
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = {'robot_description': doc.toxml()}
    
    # Launch configuration variables specific to simulation
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub_gui = LaunchConfiguration('use_joint_state_pub_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub_gui',
        default_value='false',
        description='Whether to start the joint state publisher')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_xml]
    )
    
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_pub_gui),
    )
    
    start_joint_state_publisher_cmd = Node(
        package='xmate_description',
        executable='publisher',
        name='publisher',
        output='screen',
        condition=UnlessCondition(use_joint_state_pub_gui),
    )
    
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')
        
  
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)


    # Add any conditioned actions
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld   
    
