"""
This launch file launches RTAB-Map for SLAM using a depth camera.
It launches:
1. RGBD Synchronization node
2. RTAB-Map core node configured for depth input
"""

# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch moonpie_osamu slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'publish_tf':True,
          'publish_map':True,
          'publish_grid_map':True,
          'publish_cloud_map':True,
          'publish_global_path':True,
          'cloud_noise_filtering_radius':0.05,
          'cloud_noise_filtering_min_neighbors':2,
          'cloud_ceiling_height':2.0,
          'cloud_floor_height':0.1,
          'cloud_max_depth':3.0,
          'cloud_min_depth':0.1,
          'cloud_voxel_size':0.05,
          'cloud_output_frame':'map',
          'cloud_ceiling_filtering':True,
          'cloud_floor_filtering':True,
          'cloud_noise_filtering':True}]

    remappings=[
          ('left/image_rect', '/camera1/camera1/infra1/image_rect_raw'),
          ('left/camera_info', '/camera1/camera1/infra1/camera_info'),
          ('right/image_rect', '/camera1/camera1/infra2/image_rect_raw'),
          ('right/camera_info', '/camera1/camera1/infra2/camera_info')]

    return LaunchDescription([

        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ]) 