"""
This launch file combines the robot visualization with the RealSense camera.
It launches:
1. The robot state publisher with the robot's URDF description
2. The RealSense D435 camera driver with pointcloud enabled
3. RViz configured to show both the robot model and camera pointcloud
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
    
    # Include the robot description launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        ])
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

    # Add static transform from camera to robot base_link
    camera_to_robot = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_robot',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
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
        robot_description_launch,
        realsense_launch,
        camera_to_robot,
        rviz_node
    ]) 