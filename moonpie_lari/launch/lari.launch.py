from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{}]
    remappings = []
    return LaunchDescription([
        Node(
            package='moonpie_lari', executable='arduino_comms', output='screen',
            parameters=parameters,
            remappings=remappings),
        Node(
            package='moonpie_lari', executable='arduino_control', output='screen',
            parameters=parameters,
            remappings=remappings),
        Node(
            package='ros2_mpu6050', executable='ros2_mpu6050', output='screen',
            parameters=parameters,
            remappings=remappings),
            
    ])
