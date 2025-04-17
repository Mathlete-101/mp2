"""
Navigation launch file for moonpie_osamu that sets up Nav2 components.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    pkg_share = FindPackageShare('moonpie_osamu').find('moonpie_osamu')
    
    # Nav2 nodes
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[os.path.join(pkg_share, 'config', 'nav2_params.yaml')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{'autostart': True,
                    'node_names': ['controller_server',
                                 'planner_server',
                                 'behavior_server',
                                 'bt_navigator']}]
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager
    ]) 