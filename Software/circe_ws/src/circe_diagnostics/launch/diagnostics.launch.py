#!/usr/bin/env python3
"""CIRCE health monitoring: circe_health_monitor + diagnostic_aggregator.

Started by circe_bringup.launch.py and circe_sim_nav.launch.py with the same layer
arguments, so disabled layers aren't reported as broken.
Output: /diagnostics_agg (tree) and /diagnostics_toplevel_state (worst status).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (AndSubstitution, EqualsSubstitution,
                                  LaunchConfiguration, NotSubstitution,
                                  OrSubstitution)
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory('circe_diagnostics'))
    analyzers = pkg / 'config' / 'analyzers.yaml'
    checks = pkg / 'config' / 'health_monitor.yaml'

    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    imu_source = LaunchConfiguration('imu_source')
    use_perception = LaunchConfiguration('use_perception')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Same as circe_bringup.launch.py's imu_will_publish.
    imu_will_publish = AndSubstitution(
        use_imu,
        OrSubstitution(
            AndSubstitution(use_camera, EqualsSubstitution(imu_source, 'd456')),
            EqualsSubstitution(imu_source, 'bno055')))

    # One entry per check in health_monitor.yaml; missing = always on.
    enabled = {
        'lidar_raw.enabled': use_lidar,
        'lidar_masked.enabled': use_lidar,
        # scan_pitch_filter publishes /scan_filtered only in response to /scan.
        'lidar_filtered.enabled': AndSubstitution(use_perception, use_lidar),
        'camera_depth.enabled': use_camera,
        # The simulated camera has no raw IMU stream.
        'imu_raw.enabled': AndSubstitution(use_camera, LaunchConfiguration('camera_imu')),
        'imu_fused.enabled': imu_will_publish,
        'odom_wheels.enabled': NotSubstitution(LaunchConfiguration('bench_mode')),
        'slam.enabled': LaunchConfiguration('use_slam'),
        'host.enabled': LaunchConfiguration('monitor_host'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('imu_source', default_value='d456',
                              choices=['d456', 'bno055']),
        DeclareLaunchArgument('use_perception', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),
        DeclareLaunchArgument(
            'bench_mode', default_value='false',
            description='No rover attached (circe_bringup bench_mode): do not expect '
                        '/odometry/wheels.'),
        DeclareLaunchArgument(
            'camera_imu', default_value='true',
            description='The camera publishes its raw IMU on /camera/camera/imu. True '
                        'for the real D456; false under Gazebo, whose rgbd_camera has '
                        'no IMU stream.'),
        DeclareLaunchArgument(
            'monitor_host', default_value='true',
            description='Report the computer this runs on: CPU temperature, '
                        'under-voltage and throttling, CPU load, memory, disk.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),

        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            output='screen',
            parameters=[str(analyzers), {'use_sim_time': use_sim_time}],
        ),

        # Name must match analyzers.yaml's "circe_health_monitor: ..." prefixes.
        Node(
            package='circe_diagnostics',
            executable='health_monitor.py',
            name='circe_health_monitor',
            output='screen',
            parameters=[str(checks), enabled, {'use_sim_time': use_sim_time}],
        ),
    ])
