#!/usr/bin/env python3
"""Rover Zero 4WD base: motor driver + robot_state_publisher.

Ported from roverrobotics_driver/launch/zero.launch.py, with two deliberate changes:

  * the robot model comes from circe_description, not roverrobotics_description
  * accessories.launch.py is NOT included here

Upstream, zero.launch.py pulled the sensors in as a side effect, so you could not
start the base without also starting the lidar and camera. That makes the bottom-up
bring-up in the checklist impossible. Sensors now live in sensors.launch.py and both
are composed by circe_bringup.launch.py.

    ros2 launch circe_bringup rover.launch.py

Publishes: /odometry/wheels, /joint_states, /tf (base_link and below), /robot_description
Subscribes: /cmd_vel
Does NOT publish odom -> base_link -- that edge belongs to the EKF in circe_nav.

*** THE THREE NODES START IN ORDER, EACH AFTER THE LAST IS OBSERVED WORKING. ***
Sequenced with scripts/wait_for.py, the same mechanism circe_bringup.launch.py uses
between layers, for one reason: FAILURE ISOLATION. Started together, these three
interleave their output and a broken base looks like one wall of log. Started in order,
the last line before the silence names the fault:

    robot_state_publisher   xacro built and the model is being published
    joint_state_publisher   /joint_states is flowing, so the wheels can render
                            (it takes the URDF from robot_state_publisher's latched
                            /robot_description, so it genuinely wants RSP up first)
    roverrobotics_driver    /dev/rover-control is open and odometry is arriving

The driver goes last although it depends on neither of the others. It is the node that
actually fails on hardware -- power, USB enumeration, udev, a port held by a stale
process -- and putting it after the two that always work means the console ends on the
thing worth reading.
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
    """Block until `checks` pass, then exit. See scripts/wait_for.py for the details.

    `required` is strict_order, already resolved -- see _launch_setup for why.
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
            f'[rover] {next_label} NOT STARTED: the "{label}" gate timed out.'))]

    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=_on_exit))


def _launch_setup(context):
    """Build the base chain with strict_order already RESOLVED to a literal.

    The joint_states gate is created by an OnProcessExit handler, after
    circe_bringup.launch.py's scoped include of this file has been popped, so a
    LaunchConfiguration('strict_order') evaluated there reads the includer's value --
    or kills the launch if the includer has none. The long version, reproduced, is in
    sensors.launch.py's _launch_setup.
    """
    strict_order = LaunchConfiguration('strict_order').perform(context)

    hardware_config = Path(
        get_package_share_directory('circe_bringup'), 'config', 'rover_zero_4wd.yaml')
    # Fail at launch with a clear message rather than letting the driver come up with
    # robot_type "NONE" and device_port "NONE" and report a confusing serial error.
    assert hardware_config.is_file(), f'missing rover config: {hardware_config}'

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # Publishes base_link -> chassis_link -> {wheels, imu_link, lidar_link,
    # sensor_shelf_link -> camera_bottom_screw_frame -> camera_link}.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # base_link -> lidar_link is a FIXED joint, so robot_state_publisher puts it on
    # /tf_static as soon as the xacro is parsed -- before and independently of any
    # joint state. That makes it the cleanest possible "the model built" signal: it
    # cannot be satisfied by a half-started node and it does not depend on the
    # joint_state_publisher that has not started yet.
    gate_model = _gate('robot_model', 30, hint=(
        'robot_state_publisher published no TF, which nearly always means the xacro '
        'failed to expand. Run `xacro $(ros2 pkg prefix circe_description)/share/'
        'circe_description/urdf/circe_rover.urdf.xacro` and read the error directly.'),
        checks=[['--tf', 'true', 'base_link', 'lidar_link']], required=strict_order)

    # The wheel joints are `continuous` in the URDF and the driver publishes no
    # /joint_states, so without this robot_state_publisher warns about missing
    # joint states and the wheels never render.
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

    # respawn: the serial link to the motor controller drops if the rover is
    # power-cycled or the USB re-enumerates. Upstream respawns, and so do we --
    # without it a brownout ends the drive session.
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
