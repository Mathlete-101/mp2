"""
This launch file is for development purposes.
It launches:
1. The osamu.launch.py file which includes robot visualization and camera
2. The robot_visualization.launch.py file which includes robot visualization without camera
This is useful for development and testing when you want to have both setups running simultaneously.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare('moonpie_osamu').find('moonpie_osamu')
    
    # Declare the launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rtabmap = LaunchConfiguration('use_rtabmap')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_aruco = LaunchConfiguration('use_aruco')
    use_mission_control = LaunchConfiguration('use_mission_control')
    use_digging = LaunchConfiguration('use_digging')
    use_rviz = LaunchConfiguration('use_rviz')
    use_mission_panel = LaunchConfiguration('use_mission_panel')
    depth_module_emitter_enabled = LaunchConfiguration('depth_module_emitter_enabled')

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_rtabmap', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_nav2', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_aruco', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_mission_control', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_digging', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_mission_panel', default_value='true'))
    ld.add_action(DeclareLaunchArgument('depth_module_emitter_enabled', default_value='0'))

    # Include the osamu launch file with RViz enabled
    osamu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'osamu.launch.py')
        ]),
        launch_arguments={
            'use_rviz': 'False',
            'use_rtabmap_viz': 'True',  # Enable rtabmap_viz for development
        }.items()
    )
    
    # Include the robot visualization launch file with RViz disabled
    robot_visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'robot_visualization.launch.py')
        ]),
        launch_arguments={
            'rviz_config': 'navigation.rviz'
        }.items()
    )
    
    # Add the mission control panel node
    mission_control_panel_node = Node(
        package='moonpie_osamu',
        executable='mission_control_panel',
        name='mission_control_panel',
        output='screen'
    )

    # Add robot state publisher
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    )

    # Add joint state publisher
    ld.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    )

    # Add static transform publishers
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera1_transform',
            arguments=['0.2', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'camera1_link']
        )
    )

    # Add RealSense T265 node
    ld.add_action(
        Node(
            package='realsense_ros2',
            executable='rs_t265_node',
            name='rs_t265_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_module.emitter_enabled': depth_module_emitter_enabled
            }]
        )
    )

    # Add RealSense D435 node
    ld.add_action(
        Node(
            package='realsense_ros2',
            executable='rs_d435_node',
            name='rs_d435_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'depth_module.emitter_enabled': depth_module_emitter_enabled
            }]
        )
    )

    # Add static transform publishers for D435
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435_transform',
            arguments=['0.0', '0.025', '0.03', '0.0', '0.0', '0.0', 'base_link', 'camera_link_d435']
        )
    )

    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='d435_pcl_transform',
            arguments=['0.0', '0.025', '0.03', '-0.5', '0.5', '-0.5', '0.5', 'base_link', 'camera_link_d435_pcl']
        )
    )

    # Add static transform publisher for T265
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='t265_transform',
            arguments=['0.0', '0.0', '0.04', '0.0', '0.0', '0.0', 'base_link', 't265_frame']
        )
    )

    # Add RTABMap nodes if enabled
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonpie_osamu'),
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_module.emitter_enabled': depth_module_emitter_enabled
        }.items(),
        condition=LaunchConfiguration('use_rtabmap')
    )
    ld.add_action(rtabmap_launch)

    # Add ArUco tracker if enabled
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonpie_osamu'),
                'launch',
                'aruco.launch.py'
            ])
        ]),
        condition=LaunchConfiguration('use_aruco')
    )
    ld.add_action(aruco_launch)

    # Add Mission Control if enabled
    mission_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonpie_osamu'),
                'launch',
                'mission_control.launch.py'
            ])
        ]),
        condition=LaunchConfiguration('use_mission_control')
    )
    ld.add_action(mission_control_launch)

    # Add Digging Sequence if enabled
    digging_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonpie_osamu'),
                'launch',
                'digging_sequence.launch.py'
            ])
        ]),
        condition=LaunchConfiguration('use_digging')
    )
    ld.add_action(digging_launch)

    # Add RViz if enabled
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonpie_osamu'),
                'launch',
                'rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'depth_module.emitter_enabled': depth_module_emitter_enabled
        }.items(),
        condition=LaunchConfiguration('use_rviz')
    )
    ld.add_action(rviz_launch)

    # Add Mission Control Panel if enabled
    mission_panel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonpie_osamu'),
                'launch',
                'mission_control_panel.launch.py'
            ])
        ]),
        launch_arguments={
            'depth_module.emitter_enabled': depth_module_emitter_enabled
        }.items(),
        condition=LaunchConfiguration('use_mission_panel')
    )
    ld.add_action(mission_panel_launch)

    return ld 
