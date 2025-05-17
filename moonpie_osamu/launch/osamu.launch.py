"""
This launch file combines the robot visualization with the RealSense camera.
It launches:
1. The robot visualization components (robot state publisher and RViz)
2. The RealSense D435 camera driver with pointcloud enabled
3. SLAM nodes for mapping
4. ArUco marker detection
5. Navigation system (optional)
6. Mission control system
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

    # Declare the launch argument for navigation
    use_navigation = LaunchConfiguration('use_navigation')
    use_navigation_arg = DeclareLaunchArgument(
        'use_navigation',
        default_value='False',
        description='Whether to launch navigation'
    )

    # Include the RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera_only.launch.py')
        )
    )

    # Include the robot description launch file
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        ])
    )

    # Include the SLAM launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_rtabmap_viz': use_rtabmap_viz
        }.items()
    )

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aruco.launch.py')
        )
    )

    # Launch the mission control launch file
    mission_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mission_control.launch.py')
        )
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(use_rviz_arg)
    ld.add_action(use_rtabmap_viz_arg)
    ld.add_action(use_navigation_arg)

    # Add the launch files
    ld.add_action(robot_description_launch)
    ld.add_action(realsense_launch)
    ld.add_action(slam_launch)
    ld.add_action(aruco_launch)
    ld.add_action(mission_control_launch)

    return ld 
