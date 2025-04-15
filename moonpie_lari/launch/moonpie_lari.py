from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    arduino_control = Node(
        package='moonpie-lari',
        executable='arduino_control'
    )

    #imu node
