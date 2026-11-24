#!/usr/bin/env python3
"""Rover Zero 4WD base: robot_state_publisher, joint_state_publisher, motor driver.

Nodes start in order, each gated by wait_for.py, so the log ends at the failing one.
Publishes /odometry/wheels, /joint_states, /tf; subscribes /cmd_vel. The EKF owns odom -> base_link.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gate(label, timeout, checks, hint='', required='true'):
    """Node that blocks until `checks` pass, then exits (scripts/wait_for.py)."""
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
    """Start `actions` once `gate` exits successfully."""
    def _on_exit(event, context):
        if event.returncode == 0:
            return list(actions)
        return [LogInfo(msg=(
            f'[rover] {next_label} NOT STARTED: the "{label}" gate timed out.'))]

    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=_on_exit))


def _launch_setup(context):
    """Build the base chain with strict_order resolved (see sensors.launch.py)."""
    strict_order = LaunchConfiguration('strict_order').perform(context)

    hardware_config = Path(
        get_package_share_directory('circe_bringup'), 'config', 'rover_zero_4wd.yaml')
    assert hardware_config.is_file(), f'missing rover config: {hardware_config}'

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Fixed joint on /tf_static: proves the xacro built.
    gate_model = _gate('robot_model', 30, hint=(
        'robot_state_publisher published no TF, which nearly always means the xacro '
        'failed to expand. Run `xacro $(ros2 pkg prefix circe_description)/share/'
        'circe_description/urdf/circe_rover.urdf.xacro` and read the error directly.'),
        checks=[['--tf', 'true', 'base_link', 'lidar_link']], required=strict_order)

    # Wheel joints are continuous and the driver publishes no /joint_states.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[hardware_config],
    )

    gate_joints = _gate('joint_states', 30, hint=(
        'joint_state_publisher is not publishing. It takes the model from '
        'robot_state_publisher\'s latched /robot_description; if that topic is empty '
        'the node waits forever without erroring.'),
        checks=[['--topic', 'true', '/joint_states']], required=strict_order)

    # respawn: survives rover power-cycles / USB re-enumeration.
    driver = Node(
        package='roverrobotics_driver',
        name='roverrobotics_driver',
        executable='roverrobotics_driver',
        parameters=[hardware_config],
        output='screen',
        respawn=True,
        respawn_delay=1,
    )

    return [
        robot_state_publisher,
        gate_model,
        _then(gate_model, 'robot_model', 'joint_state_publisher',
              joint_state_publisher, gate_joints),
        _then(gate_joints, 'joint_states', 'the motor driver', driver),
    ]


def generate_launch_description():
    default_model_path = get_package_share_path('circe_description') / 'urdf' / 'circe_rover.urdf.xacro'

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model', default_value=str(default_model_path),
            description='Absolute path to the robot xacro/urdf'),
        DeclareLaunchArgument(
            'strict_order', default_value='true',
            description='Stop at the first node that fails to come up, instead of '
                        'starting the rest on top of it.'),

        OpaqueFunction(function=_launch_setup),
    ])
