#!/usr/bin/env python3
"""State estimation: robot_localization EKF fusing wheel odometry.

    ros2 launch circe_nav localization.launch.py

Ported from roverrobotics_driver/launch/robot_localizer.launch.py.

Publishes: /odometry/filtered, and the odom -> base_link transform.
Subscribes: /odometry/wheels (from circe_bringup's rover.launch.py)

*** THIS NODE OWNS odom -> base_link. ***
Nothing else may publish that edge. In particular
circe_bringup/config/rover_zero_4wd.yaml keeps the motor driver's publish_tf false,
and there is no static publisher for it outside bench_mode. Two publishers on this
edge is the single most common way to get a robot that jitters or sits at the origin.

See config/localization_ekf.yaml for the open question about the imu0 input.

*** fuse_wheel_yaw:=true IS FOR RUNNING WITHOUT AN IMU. ***
localization_ekf.yaml takes vx/vy from the wheels and yaw rate ONLY from imu0, so with
no /imu/data (use_camera:=false on circe_bringup, whose D456 carries the IMU) nothing
ever rotates the filter: odom -> base_link keeps its starting heading while the rover
turns, and SLAM and Nav2 see it drive in straight lines. This adds the wheels' yaw rate
to odom0_config. Skid-steer wheels scrub sideways in a turn and over-report rotation,
so the heading is worse than the IMU's; slam_toolbox's scan matching corrects map ->
odom, not this. Leave it false whenever /imu/data is live, or both sources fuse yaw
rate. circe_bringup.launch.py sets it from whether an IMU will publish.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# localization_ekf.yaml's odom0_config plus yaw_vel (row 4, last column).
WHEEL_YAW_ODOM0_CONFIG = [False, False, False,
                          False, False, False,
                          True,  True,  False,
                          False, False, True,
                          False, False, False]


def _ekf(condition, overrides):
    return Node(
        package='robot_localization',
        executable='ekf_node',
        # Must match the top-level key in localization_ekf.yaml.
        name='ekf_filter_node',
        output='screen',
        condition=condition,
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            *overrides,
        ],
    )


def generate_launch_description():
    default_params = Path(
        get_package_share_directory('circe_nav'), 'config', 'localization_ekf.yaml')
    assert default_params.is_file(), f'missing {default_params}'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),
        DeclareLaunchArgument(
            'params_file', default_value=str(default_params),
            description='robot_localization EKF parameters'),
        DeclareLaunchArgument(
            'fuse_wheel_yaw', default_value='false',
            description='Also fuse wheel yaw rate. Only for running with no /imu/data, '
                        'which is otherwise the filter\'s sole heading source.'),

        _ekf(UnlessCondition(LaunchConfiguration('fuse_wheel_yaw')), []),
        _ekf(IfCondition(LaunchConfiguration('fuse_wheel_yaw')),
             [{'odom0_config': WHEEL_YAW_ODOM0_CONFIG}]),
    ])
