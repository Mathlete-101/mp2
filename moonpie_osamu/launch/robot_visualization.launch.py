"""
This launch file launches the visualization components for the robot and camera data.
It launches:
1. The robot state publisher with the robot's URDF description
2. SLAM Toolbox for mapping
3. RViz configured to show both the robot model, camera pointcloud, and map
4. Image transport nodes to decompress compressed image streams
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
    
    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ])
    )
    
    # Add static transform from world to map
    world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )
    
    # Add static transform from map to odom (initial pose)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Add static transform from odom to base_link (initial pose)
    odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    
    # Add static transform from base_link to camera link
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_transform',
        arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'camera1_link']
    )
    
    # Add static transform from camera link to camera optical frame
    camera_link_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=['0', '0', '0', '0', '0', '0', 'camera1_link', 'camera1_depth_optical_frame']
    )
    
    # Add static transform from camera1_link to four_wheeled_robot/camera_link/depth_camera1
    camera_link_to_sim_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_sim_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'camera1_link', 'four_wheeled_robot/camera_link/depth_camera1']
    )
    
    # Launch RViz if enabled
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'realsense_d435.rviz')],
        condition=IfCondition(use_rviz)
    )
    
    # Add image transport nodes to decompress compressed streams
    color_image_decompress = Node(
        package='image_transport',
        executable='republish',
        name='color_image_decompress',
        arguments=['compressed', 'raw'],
        remappings=[
            ('in', '/camera1/color/image_raw/compressed'),
            ('out', '/camera1/color/image_raw/decompressed')
        ]
    )
    
    depth_image_decompress = Node(
        package='image_transport',
        executable='republish',
        name='depth_image_decompress',
        arguments=['compressedDepth', 'raw'],
        remappings=[
            ('in', '/camera1/depth/image_raw/compressedDepth'),
            ('out', '/camera1/depth/image_raw/decompressed')
        ]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_description_launch,
        slam_launch,
        world_to_map,
        map_to_odom,
        odom_to_base_link,
        camera_transform,
        camera_link_to_optical,
        camera_link_to_sim_camera,
        rviz_node,
        color_image_decompress,
        depth_image_decompress
    ]) 