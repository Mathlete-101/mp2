"""
Launch file for ArUco marker detection using the RealSense camera.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create the ArUco detection node
    aruco_node = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',  # Use the autostart version
        name='aruco_tracker',
        parameters=[{
            'marker_size': 0.139,  # Size in meters
            'marker_dict': '5X5_250',  # Removed DICT_ prefix
            'cam_base_topic': '/rs_d435/image_raw'  # Updated to use RealSense D435 topic
        }],
        remappings=[
            ('/camera1/camera1/color/image_raw', '/rs_d435/image_raw'),
            ('/camera1/camera1/color/camera_info', '/rs_d435/image_raw/camera_info')
        ]
    )

    # Create the launch description
    ld = LaunchDescription()
    
    # Add the nodes
    ld.add_action(aruco_node)

    return ld 