#!/usr/bin/env python3
"""Gamepad teleop for CIRCE: joy_node -> roverrobotics_input_manager -> /cmd_vel.

    ros2 launch circe_control teleop.launch.py controller:=ps5   # default ps4
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    config = Path(get_package_share_directory('circe_control'), 'config')

    controller = LaunchConfiguration('controller').perform(context)
    controller_config = config / f'{controller}_controller.yaml'
    assert controller_config.is_file(), (
        f'unknown controller {controller!r}: no such file {controller_config}')

    topics_config = config / 'joy_topics.yaml'
    assert topics_config.is_file(), f'missing {topics_config}'

    return [
        # respawn: pads get unplugged mid-session.
        Node(
            package='roverrobotics_input_manager',
            executable='joys_manager.py',
            output='screen',
            parameters=[{
                'controller': str(controller_config),
                'topics': str(topics_config),
            }],
            respawn=True,
            respawn_delay=1,
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('joy_device_id'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            respawn=True,
            respawn_delay=1,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='ps4',
            description='Gamepad layout: ps4 or ps5'),
        DeclareLaunchArgument(
            'joy_device_id', default_value='0',
            description='Which /dev/input/js* to open'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function=_launch_setup),
    ])
