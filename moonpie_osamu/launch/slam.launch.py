"""
This launch file launches RTAB-Map for SLAM using a depth camera.
It launches:
1. RGBD Synchronization node
2. RTAB-Map core node configured for depth input
3. RTAB-Map visualization node (optional)
4. Robot Localization EKF node (optional)
"""

# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
#   Install robot_localization package (ros-$ROS_DISTRO-robot-localization)
# Example:
#   $ ros2 launch moonpie_osamu slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    use_rtabmap_viz_arg = DeclareLaunchArgument(
        'use_rtabmap_viz',
        default_value='False',
        description='Whether to launch rtabmap_viz'
    )
    
    use_imu = LaunchConfiguration('use_imu')
    use_imu_arg = DeclareLaunchArgument(
        'use_imu',
        default_value='False',
        description='Whether to use IMU data for odometry fusion'
    )
    
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
          'cloud_noise_filtering':True,
          # QoS settings for map topics
          'qos_map': 'Reliable,Transient',
          'qos_grid_map': 'Reliable,Transient',
          'qos_cloud_map': 'Reliable,Transient'}]

    remappings=[
          ('left/image_rect', '/camera1/camera1/infra1/image_rect_raw'),
          ('left/camera_info', '/camera1/camera1/infra1/camera_info'),
          ('right/image_rect', '/camera1/camera1/infra2/image_rect_raw'),
          ('right/camera_info', '/camera1/camera1/infra2/camera_info'),
          # Remap grid_map to map
          ('grid_map', 'map'),
          ('grid_map_updates', 'map_updates')]

    # Get the path to the EKF config file
    ekf_config_path = os.path.join(
        get_package_share_directory('moonpie_osamu'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        use_rtabmap_viz_arg,
        use_imu_arg,
        
        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),

        # Original stereo odometry node (only when not using IMU)
        Node(
            package='rtabmap_odom', 
            executable='stereo_odometry', 
            output='log',
            parameters=parameters,
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'stereo_odometry:=warn'],
            condition=UnlessCondition(use_imu)),

        # Robot Localization system nodes (only when using IMU)
        Node(
            package='rtabmap_odom', 
            executable='stereo_odometry', 
            output='log',
            parameters=parameters,
            remappings=remappings + [('odom', '/stereo_odometry/odom')],
            arguments=['--ros-args', '--log-level', 'stereo_odometry:=warn'],
            condition=IfCondition(use_imu)),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('odometry/filtered', 'odom')],
            condition=IfCondition(use_imu)),

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='log',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d', '--ros-args', '--log-level', 'rtabmap:=warn']),

        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(use_rtabmap_viz)),
    ]) 
