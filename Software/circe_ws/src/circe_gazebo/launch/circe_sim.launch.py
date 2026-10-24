#!/usr/bin/env python3
"""CIRCE in Gazebo Harmonic: the simulated hardware layer, and nothing above it.

This is the sim substitute for circe_bringup's rover.launch.py + sensors.launch.py.
It starts Gazebo, spawns the rover, bridges the topics, and publishes the TF tree
below base_link. It does NOT start the EKF, SLAM, or Nav2 -- use
circe_sim_nav.launch.py for those, exactly as the real stack layers up.

    ros2 launch circe_gazebo circe_sim.launch.py
    ros2 launch circe_gazebo circe_sim.launch.py world:=empty.sdf
    ros2 launch circe_gazebo circe_sim.launch.py gui:=false        # headless

Drive it with:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

circe_control's teleop is not usable here: it needs a physical gamepad.

Publishes: /clock, /scan, /imu/data, /odometry/wheels, /joint_states,
           /camera/camera/{color,depth}/..., /tf (base_link and below)
Subscribes: /cmd_vel

Does NOT publish odom -> base_link. That edge belongs to the EKF in circe_nav, the
same as on hardware. The gz DiffDrive plugin's own TF goes to /tf_gazebo and is
deliberately not bridged -- see config/gz_bridge.yaml.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = Path(get_package_share_directory('circe_gazebo'))
    pkg_description = Path(get_package_share_directory('circe_description'))

    model_path = pkg_description / 'urdf' / 'circe_rover.urdf.xacro'
    bridge_config = pkg_gazebo / 'config' / 'gz_bridge.yaml'
    sector_config = Path(get_package_share_directory('circe_perception'),
                         'config', 'scan_sector_filter.yaml')
    rviz_config = pkg_gazebo / 'rviz' / 'circe_sim.rviz'
    for f in (model_path, bridge_config, rviz_config, sector_config):
        assert f.is_file(), f'missing sim resource: {f}'

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')

    # sim_mode:=true is what switches on every <gazebo> block in the description.
    # Without it the model spawns with no plugins and no sensors, sits inert, and
    # gives no obvious clue why.
    robot_description = ParameterValue(
        Command(['xacro ', str(model_path), ' sim_mode:=true']), value_type=str)

    # Mesh resolution. Every mesh in the expanded URDF is a package:// URI (the xacros
    # used to say file://$(find ...), which baked the build machine's absolute path into
    # /robot_description and broke RViz on any other machine). gz can only resolve
    # package:// via GZ_SIM_RESOURCE_PATH, which is not set to this workspace by
    # default, so without the append below the robot silently renders as nothing.
    #
    # gz resolves package://foo/bar by looking for a directory named foo on the path,
    # so what has to be appended is the PARENT of each package's share directory, not
    # the share directory itself.
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.pathsep.join({
            str(Path(get_package_share_directory(pkg)).parent)
            for pkg in ('circe_description',
                        'realsense2_description',
                        'roverrobotics_description')
        }),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # -r starts the world running. Without it gz comes up PAUSED, /clock never
    # advances, and every node with use_sim_time sits at t=0 looking hung -- which is
    # what the upstream rover launch files do and it confuses everyone once.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim'),
                     'launch', 'gz_sim.launch.py'))),
        launch_arguments={
            'gz_args': [
                '-r ',
                # -s runs the server only. Headless is substantially faster and still
                # publishes every topic, which is what you want for SLAM/Nav2 runs.
                PythonExpression(["'-s ' if '", gui, "'.lower() == 'false' else ''"]),
                PathJoinSubstitution([str(pkg_gazebo), 'worlds', world]),
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'circe',
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # Same mask, same config file, same position in the chain as the hardware path in
    # circe_bringup/launch/sensors.launch.py: the bridge publishes /scan_raw and this
    # republishes /scan with the rear half removed. Keeping one mechanism across sim and
    # hardware is the whole point -- see the note in config/gz_bridge.yaml.
    #
    # Both remappings matter: the chain node's own topics are `scan` and `scan_filtered`,
    # and /scan_filtered already belongs to circe_perception's scan_pitch_filter.
    scan_sector_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_sector_filter',
        parameters=[str(sector_config), {'use_sim_time': True}],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen',
        # Same recovery as the hardware path: /scan has no other publisher.
        respawn=True,
        respawn_delay=1.0,
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': str(bridge_config),
            'use_sim_time': True,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_sim',
        output='screen',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='circe_indoor.sdf',
            description='World file in circe_gazebo/worlds: circe_indoor.sdf has '
                        'walls and obstacles for SLAM/Nav2; empty.sdf is a bare '
                        'ground plane for drive and TF checks.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo GUI. false runs the server headless, which '
                        'is much faster and still publishes every topic.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the simulation layout'),
        DeclareLaunchArgument(
            'x', default_value='0.0', description='Spawn x'),
        DeclareLaunchArgument(
            'y', default_value='0.0', description='Spawn y'),
        DeclareLaunchArgument(
            'z', default_value='0.15',
            description='Spawn z. Slightly above ground so the wheels settle rather '
                        'than start interpenetrating the floor.'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0', description='Spawn yaw'),

        resource_path,
        gz_sim,

        # Order matters here, though not as a hard barrier: `create -topic
        # /robot_description` subscribes and blocks ("Waiting messages on topic
        # [/robot_description]") until robot_state_publisher latches the model, so
        # declaring the publisher first is enough and the spawn cannot lose the race.
        #
        # An earlier version wrapped the spawn in RegisterEventHandler(OnProcessStart)
        # to force the ordering. Do not reintroduce that: when this file is included
        # from a scoped GroupAction (which circe_sim_nav.launch.py does, and must, to
        # stop params_file leaking between the circe_nav includes) the group's
        # launch-configuration scope is popped before the deferred handler fires, and
        # the spawn dies with "launch configuration 'x' does not exist".
        robot_state_publisher,
        spawn,

        bridge,
        scan_sector_filter,
        rviz,
    ])
