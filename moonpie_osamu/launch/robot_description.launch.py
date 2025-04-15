"""
This launch file launches the robot state publisher with the robot's URDF description.
It publishes the robot's URDF to the /robot_description topic and transforms between
robot links to the /tf topic. This is required for visualizing the robot model in RViz
and other tools that need to know the robot's structure and state.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('moonpie_osamu')
    
    # Get the absolute path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    
    # Read the URDF file content
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Create the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # Create the joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Create the static transform publisher for the camera
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera1_transform',
        arguments=['0.2', '0', '0.05', '0', '0', '0', 'base_link', 'camera1_link']
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_transform
    ]) 