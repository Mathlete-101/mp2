from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'moonpie_lari'
    package_share_dir = get_package_share_directory(package_name)

    imu_config = os.path.join(package_share_dir, 'config', 'imu_params.yaml')
    
    parameters = [{}]
    remappings = []
    # Include the RealSense launch file
    return LaunchDescription([
        Node(
            package='moonpie_lari', executable='arduino_comms', output='screen',
            parameters=parameters,
            remappings=remappings),
        Node(
            package='moonpie_lari', executable='arduino_control', output='screen',
            parameters=parameters,
            remappings=remappings),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros2_mpu6050'), 'launch', 'ros2_mpu6050.launch.py'),
            ),
            launch_arguments={
                'param_file': imu_config
            }.items()
        ),
    ])
