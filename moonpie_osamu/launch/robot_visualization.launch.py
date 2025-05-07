"""
This launch file launches the visualization components for the robot and camera data.
It launches:
1. The robot state publisher with the robot's URDF description
2. RViz configured to show both the robot model, camera pointcloud, and map
3. Image transport nodes to decompress compressed image streams
It should not launch the camera or slam nodes. Those should be launched in robot_with_camera.launch.py
This is used when you want to visualize the robot without starting the actual camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition 
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare('moonpie_osamu').find('moonpie_osamu')
    # Launch RViz if enabled
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'config', 'navigation.rviz'), '--ros-args', '--log-level', 'rviz2:=warn'],
    )

    return LaunchDescription([
        rviz_node
    ]) 
