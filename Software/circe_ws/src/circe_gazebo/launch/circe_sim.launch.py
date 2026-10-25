#!/usr/bin/env python3
"""CIRCE in Gazebo Harmonic: the sim stand-in for rover.launch.py + sensors.launch.py.

Starts Gazebo, spawns the rover, bridges topics, publishes TF below base_link.
EKF/SLAM/Nav2 live in circe_sim_nav.launch.py. odom -> base_link is left to the EKF.

    ros2 launch circe_gazebo circe_sim.launch.py [world:=empty.sdf] [gui:=false]
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
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

    # sim_mode:=true enables the <gazebo> plugin/sensor blocks.
    robot_description = ParameterValue(
        Command(['xacro ', str(model_path), ' sim_mode:=true']), value_type=str)

    # gz resolves package:// URIs via GZ_SIM_RESOURCE_PATH (parent of each share dir).
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

    # -r: start unpaused, otherwise /clock never advances.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim'),
                     'launch', 'gz_sim.launch.py'))),
        launch_arguments={
            'gz_args': [
                '-r ',
                # -s: headless server.
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

    # Same rear-sector mask as hardware: /scan_raw -> /scan.
    scan_sector_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_sector_filter',
        parameters=[str(sector_config), {'use_sim_time': True}],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen',
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

        # `create` blocks until /robot_description is latched, so no event handler is
        # needed (one breaks when included from a scoped GroupAction).
        robot_state_publisher,
        spawn,

        bridge,
        scan_sector_filter,
        rviz,
    ])
