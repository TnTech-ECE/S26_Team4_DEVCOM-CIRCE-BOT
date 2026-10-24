#!/usr/bin/env python3
"""Gamepad teleop for CIRCE.

    ros2 launch circe_control teleop.launch.py                  # PS4 pad
    ros2 launch circe_control teleop.launch.py controller:=ps5

Ported from roverrobotics_driver/launch/ps4_controller.launch.py, with the pad choice
turned into a launch argument instead of a second near-identical launch file, and one
substantive fix:

  *** joy_linux -> joy ***
  Upstream runs `joy_linux/joy_linux_node`. That package is NOT installed on this
  machine and is not in circe.repos, so the upstream launch file fails outright with a
  package-not-found error. `joy/joy_node` is installed, is the maintained
  ros2 driver, and publishes the same sensor_msgs/Joy on /joy, so the input manager
  downstream does not know the difference.

Publishes: /cmd_vel (via roverrobotics_input_manager, see config/joy_topics.yaml)

This is also where the twist_mux and the software e-stop belong when they arrive
(IMPLEMENTATION_PLAN.md Phase 5). At that point /cmd_vel here should be remapped to a
teleop-specific input of the mux rather than driving the base directly, so an
autonomous goal and a human on the stick cannot both command the motors at once.
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
        # respawn: the pad is a USB device and users unplug them mid-session.
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
                # Below this the stick is treated as centred. Matches the
                # exclusion bands in the *_controller.yaml files.
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
        # Upstream sets this so log lines from the respawning nodes are not lost
        # when they die; keep it.
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function=_launch_setup),
    ])
