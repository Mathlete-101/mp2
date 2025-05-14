import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    nav_test_dir = get_package_share_directory('nav_test')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_yaml_file = LaunchConfiguration('map')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')


    #default_params_filename = 'nav2_params_dwb_controller.yaml'
    default_params_filename = 'nav2_params_graceful_controller.yaml'
    default_params_file = os.path.join(nav_test_dir, 'config', default_params_filename)

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',  # Empty string since we're not using a static map
        description='Dummy map path (not used)')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Include the launch files
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
            'bt_xml_file': bt_xml_file,
            'map': map_yaml_file,
            'slam': 'False',
            'use_lifecycle_mgr': 'true',
            'map_server': 'true',  # Enable map server to provide map for costmap
            'amcl': 'false',  # Disable AMCL since we're not using a static map
            'controller_server': 'true',
            'planner_server': 'true',
            'recoveries_server': 'true',
            'bt_navigator': 'true',
            'waypoint_follower': 'true',
            'use_rviz': 'false',  # Disable RViz since we're launching it separately
        }.items())

    ld.add_action(nav2_bringup_cmd)

    return ld 