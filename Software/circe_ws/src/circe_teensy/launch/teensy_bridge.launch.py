#!/usr/bin/env python3
"""Teensy USB bridge on its own (not yet part of circe_bringup)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('circe_teensy')) / 'config' / 'teensy_bridge.yaml'

    return LaunchDescription([
        Node(
            package='circe_teensy',
            executable='teensy_bridge.py',
            name='teensy_bridge',
            output='screen',
            parameters=[str(config)],
        ),
    ])
