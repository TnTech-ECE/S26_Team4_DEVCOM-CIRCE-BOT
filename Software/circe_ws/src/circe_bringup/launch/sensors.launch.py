#!/usr/bin/env python3
"""CIRCE sensor suite: RPLidar A1 + RealSense D456 (+ optional BNO055).

    ros2 launch circe_bringup sensors.launch.py use_camera:=false   # lidar alone

Lidar:  /scan_raw (driver) -> scan_sector_filter -> /scan (rear masked, lidar_link)
Camera: /camera/camera/depth/image_rect_raw, /camera/camera/imu (raw, no orientation)
IMU:    /imu/data (imu_fusion.py from the D456 IMU, or BNO055)

Sensors start one at a time, each gated by wait_for.py, so the log names the broken
stage and the USB devices don't enumerate at once.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def _gate(label, timeout, checks, hint='', required='true'):
    """Node that blocks until `checks` pass, then exits (scripts/wait_for.py).

    Each check carries its own enabled flag, so a gate for a disabled sensor still
    runs and starts the next stage.
    """
    arguments = ['--label', label,
                 '--timeout', str(float(timeout)),
                 '--required', required]
    for check in checks:
        arguments += check
    if hint:
        arguments += ['--hint', hint]

    return Node(package='circe_bringup', executable='wait_for.py',
                name=f'gate_{label}', arguments=arguments, output='screen')


def _then(gate, label, next_label, *actions):
    """Start `actions` once `gate` exits successfully; say why if it did not."""
    def _on_exit(event, context):
        if event.returncode == 0:
            return list(actions)
        return [LogInfo(msg=(
            f'[sensors] {next_label} NOT STARTED: the "{label}" gate timed out.'))]

    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=_on_exit))


def _launch_setup(context):
    """Build the sensor chain with every launch configuration resolved to a literal.

    The OnProcessExit handlers run after the include's scope is popped, so
    LaunchConfigurations must be resolved here, not in the handlers.
    """
    def resolved(substitution):
        return substitution.perform(context)

    config = Path(get_package_share_directory('circe_bringup'), 'config')

    # Shared with circe_gazebo, so it lives in circe_perception.
    sector_config = Path(get_package_share_directory('circe_perception'),
                         'config', 'scan_sector_filter.yaml')
    assert sector_config.is_file(), f'missing sector filter config: {sector_config}'

    lidar_config = config / 'lidar_a1.yaml'
    camera_config = config / 'realsense_d456.yaml'
    imu_config = config / 'bno055.yaml'
    d456_imu_config = config / 'imu_d456.yaml'
    for f in (lidar_config, camera_config, imu_config, d456_imu_config):
        assert f.is_file(), f'missing sensor config: {f}'

    # use_imu: whether to publish /imu/data; imu_source: which device (one publisher only).
    imu_on = LaunchConfiguration('use_imu')
    source = LaunchConfiguration('imu_source')
    use_lidar = resolved(LaunchConfiguration('use_lidar'))
    use_camera = resolved(LaunchConfiguration('use_camera'))
    strict_order = resolved(LaunchConfiguration('strict_order'))

    d456_imu = resolved(AndSubstitution(imu_on, EqualsSubstitution(source, 'd456')))
    bno055_imu = resolved(AndSubstitution(imu_on, EqualsSubstitution(source, 'bno055')))
    camera_imu = resolved(AndSubstitution(use_camera, d456_imu))
    use_d456_imu = IfCondition(d456_imu)
    use_bno055 = IfCondition(bno055_imu)

    # respawn: sllidar_node exits if it loses /dev/rplidar.
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_a1',
        parameters=[lidar_config],
        output='screen',
        # Raw sweep goes to /scan_raw; /scan comes from the sector filter.
        remappings=[('/scan', '/scan_raw')],
        condition=IfCondition(use_lidar),
        respawn=True,
        respawn_delay=2.0,
    )

    gate_lidar = _gate('lidar', 45, hint=(
        'The RPLidar driver is publishing nothing on /scan_raw. Check that '
        '/dev/rplidar exists (udev/60-rplidar.rules), that the motor is spinning, and '
        'that the baud rate in config/lidar_a1.yaml is 115200 for an A1 -- '
        'sllidar_node\'s own default is 1000000 and silently reads nothing.'),
        checks=[['--topic', use_lidar, '/scan_raw']], required=strict_order)

    # Masks the rear of the sweep (spool and tether) before /scan, so every consumer
    # gets the masked scan. Both remaps are needed to avoid colliding with /scan_filtered.
    sector_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_sector_filter',
        parameters=[str(sector_config)],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen',
        condition=IfCondition(use_lidar),
        respawn=True,
        respawn_delay=1.0,
    )

    gate_scan = _gate('scan', 30, hint=(
        '/scan_raw is flowing but /scan is not, so the fault is in the sector filter, '
        'not the lidar. Check config/scan_sector_filter.yaml in circe_perception and '
        'that laser_filters is installed.'),
        checks=[['--topic', use_lidar, '/scan']], required=strict_order)

    # respawn covers a driver crash; reconnect_timeout covers an unplug.
    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        parameters=[camera_config],
        output='screen',
        condition=IfCondition(use_camera),
        respawn=True,
        respawn_delay=5.0,
    )

    # Only wait on the IMU topic when the D456 IMU is in use.
    gate_camera = _gate('camera', 90, hint=(
        'The D456 did not produce data. It needs a USB 3 port -- on USB 2 the driver '
        'starts and then fails to open the depth stream. initial_reset: true in '
        'config/realsense_d456.yaml makes a cold start genuinely slow, so a timeout '
        'here is worth confirming with `rs-enumerate-devices` before changing '
        'anything.'), checks=[
            ['--topic', use_camera, '/camera/camera/depth/image_rect_raw'],
            ['--topic', camera_imu, '/camera/camera/imu'],
        ], required=strict_order)

    # The D456 IMU is raw (no orientation, optical frame); imu_fusion.py rotates it
    # into base_link and adds roll/pitch for /imu/data.
    imu_fusion = Node(
        package='circe_perception',
        executable='imu_fusion.py',
        name='imu_fusion',
        parameters=[str(d456_imu_config)],
        remappings=[('imu_in', '/camera/camera/imu'), ('imu_out', '/imu/data')],
        output='screen',
        condition=use_d456_imu,
        respawn=True,
        respawn_delay=1.0,
    )

    # BNO055 fuses on-chip, so it publishes /imu/data directly.
    bno055 = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[imu_config],
        remappings=[('/imu', '/imu/data')],
        output='screen',
        condition=use_bno055,
    )

    return [
        lidar,
        gate_lidar,
        _then(gate_lidar, 'lidar', 'the sector filter', sector_filter, gate_scan),
        _then(gate_scan, 'scan', 'the camera', camera, gate_camera),
        _then(gate_camera, 'camera', 'IMU fusion', imu_fusion, bno055),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the RPLidar A1 (needs /dev/rplidar)'),
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='Start the RealSense D456 (needs a USB 3 port)'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Publish /imu/data. See imu_source for which device. Set false '
                        'to run deliberately without an IMU -- note that this silently '
                        'degrades the EKF heading, scan_pitch_filter and terrain_node.'),
        DeclareLaunchArgument(
            'imu_source', default_value='d456',
            choices=['d456', 'bno055'],
            description='Which device publishes /imu/data. d456 = the camera\'s own '
                        'IMU, fused into an attitude by circe_perception/imu_fusion.py '
                        '(requires use_camera:=true). bno055 = the external part, which '
                        'is not fitted and whose driver is not installed.'),
        DeclareLaunchArgument(
            'strict_order', default_value='true',
            description='Stop at the first sensor that fails to come up, instead of '
                        'starting the rest on top of it.'),

        OpaqueFunction(function=_launch_setup),
    ])
