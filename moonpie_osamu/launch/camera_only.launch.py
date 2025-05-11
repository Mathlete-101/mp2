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
            'pointcloud.enable': 'true',
            'camera_name': 'camera1',
            'camera_namespace': '/camera1',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_sync': 'true',
            'depth_module.emitter_enabled': 'true'
        }.items()
    )

    # Add static transform from camera link to camera optical frame
    camera_link_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=['0', '0', '0', '0', '0', '0', 'camera1_link', 'camera1_depth_optical_frame']
    )


    
    return LaunchDescription([
        realsense_launch,
        camera_link_to_optical
    ]) 
