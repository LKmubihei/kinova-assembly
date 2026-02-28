# What is done in this launch file
#   * Load robotiq gripper description from xacro file
#   * Launch rviz to visualize
#   * Launch gazebo to simulate
#
# Note: ros2 control node is launched by the libgazebo_ros_control.so in 
#   robotiq_2f_85_gripper.gazebo.urdf.xacro`, which is loaded by the gazebo
#   plugin in the gazebo.launch.py file.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='robotiq_description',
            description='Package containing robotiq gripper description.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='robotiq_2f_85_gripper.urdf.xacro',
            description='Robotiq gripper description file.'
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
            default_value='false',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit', 
            default_value='true', 
            description='Whether to use moveit2 or not.'
        )
    )

    # Initialize arguments
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    start_rviz = LaunchConfiguration('start_rviz')
    use_moveit = LaunchConfiguration('use_moveit')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ', 
            PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
            ' ',
            'prefix:=', 
            prefix, 
            ' ', 
            'sim_gazebo:=',
            'true',
            ' ',
            'use_fake_hardware:=',
            'false',
            ' ',
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare(description_package), '/launch', '/moveit.launch.py'
        ]), 
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'prefix': prefix,
            'namespace': namespace,
            'start_rviz':start_rviz,
            'use_sim': 'true',
        }.items(),
        condition=IfCondition(use_moveit),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'robotiq_2f_85.rviz']
    )
    gazebo_world_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'gazebo', 'empty.world']
    )

    # ----------- Rviz -----------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
        ],
    )

    # ----------- Gazebo -----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'true', 'world': gazebo_world_file}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', [namespace, 'robot_description'], '-entity', [namespace, 'robotiq_2f_85_gripper']],
        output='screen',
    )

    # ----------- robot state publisher -----------
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    # ----------- ros2 control: controller manager -----------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    robotiq_gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager',
                   [namespace, 'controller_manager']],
    )

    # Delay `joint_state_broadcaster` after spawn_entity
    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        ),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robotiq_gripper_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        spawn_entity,
        moveit_launch,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_spawn_entity,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # robotiq_gripper_controller_spawner, # use this instead of `delay_robot_controller_spawner_after_joint_state_broadcaster_spawner`, 
                                            # we can set initial position of gripper in 2f_85.ros2_control.xacro
    ]

    return LaunchDescription(declared_arguments + nodes)