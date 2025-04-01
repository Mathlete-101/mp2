"""
This launch file launches the RealSense D435 camera node and RViz for visualization.
It:
1. Launches the RealSense camera driver with pointcloud enabled
2. Launches RViz with a pre-configured view for the camera's pointcloud
3. Allows toggling RViz on/off with the use_rviz parameter
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
    
    # Include the RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                FindPackageShare('realsense2_camera').find('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'pointcloud.enable': 'true'
        }.items()
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
        realsense_launch,
        rviz_node
    ]) 