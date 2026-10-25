#!/usr/bin/env python3
"""circe_perception: terrain_node (/terrain/grid) and scan_pitch_filter (/scan_filtered).

    ros2 launch circe_perception terrain.launch.py [use_sim_time:=true]

Needs odom -> base_link from localization.launch.py.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory('circe_perception'))
    default_params = pkg / 'config' / 'terrain.yaml'
    assert default_params.is_file(), f'missing terrain config: {default_params}'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),
        DeclareLaunchArgument(
            'params_file', default_value=str(default_params),
            description='Terrain traversability thresholds'),
        DeclareLaunchArgument(
            'use_scan_filter', default_value='true',
            description='Run the lidar ground-return filter. Set false to A/B it: with '
                        'it off, driving onto a ramp fills the costmap with phantom '
                        'obstacles and the controller gives up.'),
        DeclareLaunchArgument(
            'camera_namespace', default_value='/camera/camera',
            description='Where the RealSense driver publishes. sensors.launch.py runs it '
                        'as namespace `camera` + name `camera`, hence the doubled name.'),

        Node(
            package='circe_perception',
            executable='terrain_node.py',
            name='terrain_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('depth/image_rect_raw',
                 [LaunchConfiguration('camera_namespace'), '/depth/image_rect_raw']),
                ('depth/camera_info',
                 [LaunchConfiguration('camera_namespace'), '/depth/camera_info']),
                ('grid', '/terrain/grid'),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),

        # Cuts lidar ground returns using IMU attitude. SLAM and both costmaps read its
        # output, so it respawns.
        Node(
            package='circe_perception',
            executable='scan_pitch_filter.py',
            name='scan_pitch_filter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_scan_filter')),
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
