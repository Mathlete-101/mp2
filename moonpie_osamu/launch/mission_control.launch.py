import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    moonpie_osamu_dir = get_package_share_directory('moonpie_osamu')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml_file = LaunchConfiguration('bt_xml_file')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(moonpie_osamu_dir, 'behavior_trees', 'mission_tree.xml'),
        description='Full path to the behavior tree xml file to use')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Launch the mission control node
    mission_control_node = Node(
        package='moonpie_osamu',
        executable='mission_control',
        name='mission_control',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'bt_xml_file': bt_xml_file,
        }]
    )

    # Launch the digging sequence node
    digging_sequence_node = Node(
        package='moonpie_osamu',
        executable='digging_sequence_node',
        name='digging_sequence',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    ld.add_action(mission_control_node)
    ld.add_action(digging_sequence_node)

    return ld 