from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare('moonpie_lari').find('moonpie_lari')
    gazebo_ros_pkg_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    gazebo_pkg_share = FindPackageShare('gazebo_ros_pkgs').find('gazebo_ros_pkgs')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')

    # Launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to execute gzclient')

    declare_use_gui_cmd = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Set "false" to run headless')

    # Specify the actions
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'empty.world')}.items(),
        condition=IfCondition(use_gui))

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(use_gui))

    # Spawn the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'moonpie_robot',
            '-file', os.path.join(pkg_share, 'urdf', 'robot.urdf')
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(os.path.join(pkg_share, 'urdf', 'robot.urdf')).read()}]
    )

    # Joint state publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_gui_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    return ld 