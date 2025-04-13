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
            'camera_namespace': '/camera1'
        }.items()
    )

    # Add static transform from camera link to base_link
    camera_to_robot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_robot',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera1_link']
    )

    # Add static transform from camera link to camera optical frame
    camera_link_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'camera1_depth_frame']
    )
    
    # Add image transport nodes for color and depth images
    color_image_transport = Node(
        package='image_transport',
        executable='republish',
        name='color_image_transport',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera1/color/image_raw'),
            ('out', '/camera1/color/image_raw/compressed')
        ]
    )
    
    depth_image_transport = Node(
        package='image_transport',
        executable='republish',
        name='depth_image_transport',
        arguments=['raw', 'compressedDepth'],
        remappings=[
            ('in', '/camera1/depth/image_raw'),
            ('out', '/camera1/depth/image_raw/compressedDepth')
        ]
    )
    
    return LaunchDescription([
        realsense_launch,
        camera_to_robot,
        camera_link_to_optical,
        color_image_transport,
        depth_image_transport
    ]) 