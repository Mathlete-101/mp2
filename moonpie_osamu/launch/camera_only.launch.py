"""
This launch file launches only the RealSense D435 camera node.
It launches:
1. The RealSense camera driver with pointcloud enabled
2. Static transforms to connect camera frames to robot base_link
3. Image transport nodes for color and depth images
This is used when you want to start the camera without visualization.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the realsense_ros2 package share
    realsense_share = FindPackageShare('realsense_ros2').find('realsense_ros2')

    # Include the RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_share, 'launch', 'realsense_launch.py')
        ])
    )
    
    return LaunchDescription([
        realsense_launch
    ]) 
