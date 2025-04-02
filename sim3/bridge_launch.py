from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Include the camera and control bridge launch files
    camera_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_dir, 'camera_bridge_launch.py')
        ])
    )
    
    control_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_dir, 'control_bridge_launch.py')
        ])
    )
    
    return LaunchDescription([
        camera_bridge,
        control_bridge
    ]) 