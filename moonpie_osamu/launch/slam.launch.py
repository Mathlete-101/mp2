"""
This launch file launches RTAB-Map for SLAM using a depth camera.
It launches:
1. RGBD Synchronization node
2. RTAB-Map core node configured for depth input
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare('moonpie_osamu').find('moonpie_osamu')

    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ORB-SLAM3 node
    orbslam_node = Node(
        package='orbslam3_ros2',
        executable='orbslam3_ros2_node',
        name='orbslam3',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'slam_mode': 'RGBD',  # Options: MONO, STEREO, RGBD
            'vocabulary_file': os.path.join(FindPackageShare('orbslam3_ros2').find('orbslam3_ros2'), 'vocabulary', 'ORBvoc.txt'),
            'settings_file': os.path.join(pkg_share, 'config', 'orbslam3_rgbd_config.yaml'),
            'publish_tf': True,
            'publish_pointcloud': True,
            'map_frame_id': 'map',
            'camera_frame_id': 'camera1_link'
        }],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_raw', '/camera/depth/image_rect_raw')
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        orbslam_node
    ]) 