#!/usr/bin/env python3
"""Terrain traversability node for CIRCE.

    ros2 launch circe_perception terrain.launch.py
    ros2 launch circe_perception terrain.launch.py use_sim_time:=true

Reads the D456 depth image and publishes /terrain/grid, a graded OccupancyGrid that Nav2
consumes as a StaticLayer in both costmaps (see the terrain_layer blocks in
circe_nav/config/nav2_params.yaml).

This is a leaf node: it publishes no TF and no /cmd_vel, and it does not touch the
costmaps. Deleting the terrain_layer entries from nav2_params.yaml disables the whole
feature with this still running.

Requires odom -> base_link, so localization.launch.py must be up first. Without it the
node logs "no transform to odom" once and idles.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg = Path(get_package_share_directory('circe_perception'))
    default_params = pkg / 'config' / 'terrain.yaml'
    assert default_params.is_file(), f'missing terrain config: {default_params}'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),
        DeclareLaunchArgument(
            'params_file', default_value=str(default_params),
            description='Terrain traversability thresholds'),
        DeclareLaunchArgument(
            'use_scan_filter', default_value='true',
            description='Run the lidar ground-return filter. Set false to A/B it: with '
                        'it off, driving onto a ramp fills the costmap with phantom '
                        'obstacles and the controller gives up.'),
        DeclareLaunchArgument(
            'camera_namespace', default_value='/camera/camera',
            description='Where the RealSense driver publishes. sensors.launch.py runs it '
                        'as namespace `camera` + name `camera`, hence the doubled name.'),

        Node(
            package='circe_perception',
            executable='terrain_node.py',
            name='terrain_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            # Relative names in the node, resolved here, so the camera can be renamed or
            # re-namespaced without touching the source.
            remappings=[
                ('depth/image_rect_raw',
                 [LaunchConfiguration('camera_namespace'), '/depth/image_rect_raw']),
                ('depth/camera_info',
                 [LaunchConfiguration('camera_namespace'), '/depth/camera_info']),
                ('grid', '/terrain/grid'),
            ],
            # Its grid is persistent, so a restart forgets the terrain behind the rover --
            # still far better than the terrain_layer going permanently silent.
            respawn=True,
            respawn_delay=2.0,
        ),

        # *** LIDAR GROUND-RETURN FILTER ***
        # Sits between the lidar driver and the costmap. On an incline the chassis-fixed
        # scan plane tilts into the ground and the down-slope beams come back as a solid
        # wall a metre or two out; the EKF's two_d_mode hides the tilt from TF, so the
        # costmap cannot reject those points by height. This node uses the IMU attitude,
        # which is never flattened, to cut each beam at its own ground-strike range.
        # Publishing to a NEW topic rather than remapping /scan in place is deliberate:
        # a consumer sees the filtered scan only when explicitly pointed here. Today that
        # is BOTH Nav2 costmaps AND slam_toolbox (scan_topic in
        # circe_nav/config/mapper_params_circe.yaml); collision_monitor still reads /scan.
        #
        # respawn: this node is a single point of failure for all three of those
        # consumers. It fails safe on a missing IMU, but if the process dies /scan_filtered
        # goes silent and SLAM and the costmaps go blind together, with nothing logged
        # after the crash. It holds no state worth keeping, so restarting is free.
        Node(
            package='circe_perception',
            executable='scan_pitch_filter.py',
            name='scan_pitch_filter',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_scan_filter')),
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            respawn=True,
            respawn_delay=1.0,
        ),
    ])
