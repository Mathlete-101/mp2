from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge for camera1 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera1_bridge',
            arguments=[
                '/camera1/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/camera1/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera1/depth@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera1/depth/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'
            ],
            remappings=[
                ('/camera1/camera_info', '/camera1/camera1/color/camera_info'),
                ('/camera1/image', '/camera1/camera1/color/image_raw'),
                ('/camera1/depth', '/camera1/camera1/depth/image_rect_raw'),
                ('/camera1/depth/points', '/camera1/camera1/depth/color/points'),
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
        )
    ]) 