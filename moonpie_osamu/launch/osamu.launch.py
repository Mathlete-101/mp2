"""
This launch file combines the robot visualization with the RealSense camera.
It launches:
1. The robot visualization components (robot state publisher and RViz)
2. The RealSense D435 camera driver with pointcloud enabled
3. SLAM nodes for mapping
This is the main launch file to use when you want to visualize both the robot and camera data with SLAM.
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
        default_value='False',
        description='Whether to launch RViz'
    )
    
    # Declare the launch argument for rtabmap_viz
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    use_rtabmap_viz_arg = DeclareLaunchArgument(
        'use_rtabmap_viz',
        default_value='False',
        description='Whether to launch rtabmap_viz'
    )
    
    # Include the robot description launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        ])
    )
    
    # Include the camera launch file
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'camera_only.launch.py')
        ])
    )
    
    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_rtabmap_viz': use_rtabmap_viz
        }.items()
    )
    
    # Launch RViz if enabled
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'navigation.rviz')],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        use_rviz_arg,
        use_rtabmap_viz_arg,
        robot_description_launch,
        camera_launch,
        slam_launch,
        rviz_node
    ]) 