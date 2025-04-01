"""
This launch file combines the robot visualization with the RealSense camera.
It launches:
1. The robot visualization components (robot state publisher and RViz)
2. The RealSense D435 camera driver with pointcloud enabled
This is the main launch file to use when you want to visualize both the robot and camera data.
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
    
    # Include the robot visualization launch file
    robot_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_visualization.launch.py')
        ]),
        launch_arguments={
            'use_rviz': use_rviz
        }.items()
    )
    
    # Include the camera launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'camera_only.launch.py')
        ])
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_visualization_launch,
        camera_launch
    ]) 