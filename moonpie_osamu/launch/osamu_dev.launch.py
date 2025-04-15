"""
This launch file is for development purposes.
It launches:
1. The osamu.launch.py file which includes robot visualization and camera
2. The robot_visualization.launch.py file which includes robot visualization without camera
This is useful for development and testing when you want to have both setups running simultaneously.
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
    
    # Include the osamu launch file
    osamu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'osamu.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )
    
    # Include the robot visualization launch file
    robot_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_visualization.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )
    
    return LaunchDescription([
        use_rviz_arg,
        osamu_launch,
        robot_visualization_launch
    ]) 