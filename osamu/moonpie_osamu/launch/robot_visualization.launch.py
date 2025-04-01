"""
This launch file launches the visualization components for the robot and camera data.
It launches:
1. The robot state publisher with the robot's URDF description
2. RViz configured to show both the robot model and camera pointcloud
This is used when you want to visualize the robot without starting the actual camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare('moonpie_osamu').find('moonpie_osamu')
    
    # Declare the launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz'
    )
    
    # Include the robot description launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        ])
    )
    
    # Launch RViz if enabled
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'realsense_d435.rviz')],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_description_launch,
        rviz_node
    ]) 