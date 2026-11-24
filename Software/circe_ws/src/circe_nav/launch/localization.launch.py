#!/usr/bin/env python3
"""robot_localization EKF: /odometry/wheels (+ /imu/data) -> /odometry/filtered.

    ros2 launch circe_nav localization.launch.py

Sole owner of odom -> base_link. fuse_wheel_yaw:=true is only for running with no IMU.
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
