from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the Arduino control node
        Node(
            package='moonpie_lari',
            executable='arduino_control',
            name='arduino_control'
        ),

        # Launch the Arduino communication node
        Node(
            package='moonpie_lari',
            executable='arduino_comms',
            name='arduino_comms',
            output="screen",
            emulate_tty=True
        ),
        
        # Launch the IMU node
        Node(
            package='ros2_mpu6050',
            executable='ros2_mpu6050',
            name='mpu6050_sensor',
            output="screen",
            emulate_tty=True
        )
    ]) 