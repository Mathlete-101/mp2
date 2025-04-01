from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge for cmd_vel
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/model/vehicle_blue/cmd_vel', '/model/vehicle_blue/cmd_vel'),
            ],
            output='screen',
            parameters=[{
                'config_file': '',
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./tf_static.publisher.history': 'keep_last',
                'qos_overrides./tf_static.publisher.reliability': 'reliable',
                'qos_overrides./tf_static.publisher.depth': 1,
                'qos_overrides./tf_static.publisher.history_policy': 'keep_last',
                'qos_overrides./tf_static.publisher.reliable': True,
                'qos_overrides./tf_static.publisher.transient_local': True,
            }]
        ),
        # Bridge for odometry
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='odom_bridge',
            arguments=['/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            remappings=[
                ('/odom', '/odom'),
            ],
            output='screen',
            parameters=[{
                'config_file': '',
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./tf_static.publisher.history': 'keep_last',
                'qos_overrides./tf_static.publisher.reliability': 'reliable',
                'qos_overrides./tf_static.publisher.depth': 1,
                'qos_overrides./tf_static.publisher.history_policy': 'keep_last',
                'qos_overrides./tf_static.publisher.reliable': True,
                'qos_overrides./tf_static.publisher.transient_local': True,
            }]
        ),
        # Teleop keyboard
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            parameters=[{
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/cmd_vel'
            }],
            output='screen'
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay_blue',
            parameters=[{
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/model/vehicle_blue/cmd_vel'
            }],
            output='screen'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel_raw')]
        )
    ]) 