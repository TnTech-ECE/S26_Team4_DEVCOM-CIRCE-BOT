#!/usr/bin/env python3
"""CIRCE health monitoring: circe_health_monitor + diagnostic_aggregator.

    ros2 launch circe_diagnostics diagnostics.launch.py
    ros2 launch circe_diagnostics diagnostics.launch.py use_camera:=false

Started by BOTH top-level launch files -- circe_bringup.launch.py on the robot and
circe_gazebo's circe_sim_nav.launch.py under Gazebo -- with the same config files and
the same node names, so the tree reads identically in both. Each passes its own layer
arguments through. The arguments here have the same names and defaults as in
circe_bringup.launch.py, and they mean the same thing: a layer switched off on purpose
is not a layer that is broken, so its checks are disabled rather than left to report
silence as an error.

The simulation differs in exactly two facts, and each is an argument rather than a
forked config: use_sim_time:=true, and camera_imu:=false (Gazebo's rgbd_camera has no
IMU stream; the simulated IMU publishes /imu/data directly).

Output:
    /diagnostics_agg              the tree   (ros2 run rqt_robot_monitor rqt_robot_monitor)
    /diagnostics_toplevel_state   one status: the worst thing on the robot right now

The EKF and Nav2 publish their own diagnostics and need no argument here: when they are
not running their groups are simply empty, which config/analyzers.yaml reads as OK.
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

    # Same expression as circe_bringup.launch.py's imu_will_publish, and for the same
    # reason: with imu_source:=d456 the publisher is imu_fusion.py, which has nothing
    # to republish when the camera is off.
    imu_will_publish = AndSubstitution(
        use_imu,
        OrSubstitution(
            AndSubstitution(use_camera, EqualsSubstitution(imu_source, 'd456')),
            EqualsSubstitution(imu_source, 'bno055')))

    # One entry per check in config/health_monitor.yaml. A check missing from here is
    # always on, so a new check in the YAML is watched until somebody decides otherwise.
    enabled = {
        'lidar_raw.enabled': use_lidar,
        'lidar_masked.enabled': use_lidar,
        # scan_pitch_filter publishes /scan_filtered only in response to /scan.
        'lidar_filtered.enabled': AndSubstitution(use_perception, use_lidar),
        'camera_depth.enabled': use_camera,
        # realsense_d456.yaml enables the gyro and accelerometer unconditionally, so the
        # raw IMU stream exists whenever the real camera does, whichever device backs
        # /imu/data. The simulated camera has none -- hence camera_imu.
        'imu_raw.enabled': AndSubstitution(use_camera, LaunchConfiguration('camera_imu')),
        'imu_fused.enabled': imu_will_publish,
        # No motor driver runs in bench_mode, so nothing publishes /odometry/wheels.
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

        # *** THE NAME IS LOAD-BEARING. *** diagnostic_updater prefixes every status
        # with it, and config/analyzers.yaml matches on "circe_health_monitor: ...".
        # Rename it here and every check drops out of its group into "Other".
        Node(
            package='circe_diagnostics',
            executable='health_monitor.py',
            name='circe_health_monitor',
            output='screen',
            parameters=[str(checks), enabled, {'use_sim_time': use_sim_time}],
        ),
    ])
