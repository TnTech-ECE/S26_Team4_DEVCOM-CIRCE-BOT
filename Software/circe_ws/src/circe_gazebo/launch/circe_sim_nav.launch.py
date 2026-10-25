#!/usr/bin/env python3
"""CIRCE in Gazebo with EKF, SLAM and Nav2, layered like circe_bringup.launch.py.

    ros2 launch circe_gazebo circe_sim_nav.launch.py [use_slam:=false] [use_nav2:=false]

Reuses circe_nav launch files unchanged (use_sim_time is passed as an override).
Layers are chained by circe_bringup's wait_for.py gates, not timers:
sim -> EKF -> perception (/scan_filtered) -> SLAM (map -> odom) -> Nav2.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, OrSubstitution
from launch_ros.actions import Node


def _include(package, launch_file, condition=None, arguments=None):
    """Include a launch file in a scoped GroupAction.

    Required: the circe_nav launch files all declare `params_file`, and without scoping
    the first include's value leaks into the rest (Nav2 then fails with "No critics").
    """
    return GroupAction(
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory(package),
                             'launch', launch_file))),
                launch_arguments=arguments or {},
            ),
        ],
    )


def _gate(label, timeout, checks, hint=''):
    """Node that blocks until `checks` pass, then exits (wait_for.py).

    Always runs; each check carries an ENABLED flag so disabled layers just skip checks.
    """
    arguments = ['--label', label,
                 '--timeout', str(float(timeout)),
                 '--required', LaunchConfiguration('strict_order')]
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
            f'[sim] {next_label} NOT STARTED: the "{label}" gate above timed out. '
            f'Everything up to and including {label} is still running -- inspect it '
            f'with `ros2 topic list`, `ros2 topic hz`, and `ros2 run tf2_tools '
            f'view_frames`. Re-run with strict_order:=false to start the remaining '
            f'layers anyway.'))]

    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=_on_exit))


def generate_launch_description():
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_terrain = LaunchConfiguration('use_terrain')

    sim_time = {'use_sim_time': 'true'}

    # Pass use_rviz=false explicitly, or child launches inherit true and open extra RViz.
    sim_no_rviz = dict(sim_time, use_rviz='false')

    # ---- Layer 1: simulated hardware -------------------------------------------------
    sim = _include('circe_gazebo', 'circe_sim.launch.py', arguments={
        'world': LaunchConfiguration('world'),
        'gui': LaunchConfiguration('gui'),
        'use_rviz': LaunchConfiguration('use_rviz'),
    }.items())

    gate_sim = _gate('simulation', 120, hint=(
        'If NOTHING arrived: Gazebo is not running, is paused (/clock not advancing; '
        'circe_sim.launch.py passes -r for this), or a server left over from an earlier '
        'run swallowed the spawn -- run `ros2 run circe_gazebo stop_sim.sh` and relaunch. '
        '/scan_raw but no /scan means the sector filter, not the bridge. Topic names come '
        'from config/gz_bridge.yaml. A cold first start compiles shaders and can be slow.'),
        checks=[
            ['--tf', 'true', 'base_link', 'lidar_link'],
            ['--topic', 'true', '/odometry/wheels'],
            ['--topic', 'true', '/scan'],
            ['--topic', 'true', '/camera/camera/depth/image_rect_raw'],
            ['--topic', 'true', '/imu/data'],
        ])

    # ---- Layer 2: state estimation ---------------------------------------------------
    ekf = _include('circe_nav', 'localization.launch.py', arguments=sim_time.items())

    gate_ekf = _gate('ekf', 30, hint=(
        'The EKF has /odometry/wheels but is not publishing odom -> base_link. Check '
        'that it got use_sim_time (a wall-clock EKF rejects every sim-stamped message), '
        'and that no /tf bridge row in config/gz_bridge.yaml has given this edge a second '
        'publisher.'),
        checks=[
            ['--tf', 'true', 'odom', 'base_link'],
            ['--topic', 'true', '/odometry/filtered'],
        ])

    # ---- Layer 3: terrain traversability + the lidar ground-return filter -------------
    perception = _include('circe_perception', 'terrain.launch.py',
                          condition=IfCondition(use_terrain),
                          arguments=sim_time.items())

    gate_perception = _gate('perception', 45, hint=(
        'No /scan_filtered while /scan is flowing means scan_pitch_filter is not '
        'running or crashed on import (it needs numpy and scipy). SLAM and both '
        'costmaps read /scan_filtered, so nothing above this works without it.'),
        checks=[
            ['--node', use_terrain, 'scan_pitch_filter'],
            ['--node', use_terrain, 'terrain_node'],
            ['--topic', use_terrain, '/scan_filtered'],
        ])

    # ---- Layer 4: SLAM ---------------------------------------------------------------
    # lifecycle manager makes RViz's Nav2 panel show "Localization: active".
    slam = _include('circe_nav', 'slam.launch.py',
                    condition=IfCondition(use_slam),
                    arguments=dict(sim_no_rviz, use_lifecycle_manager='true').items())

    # map -> odom is also needed when SLAM is off but Nav2 is on.
    gate_slam = _gate('localization', 90, hint=(
        'Run `ros2 lifecycle get /slam_toolbox`. "active [3]" with no map -> odom means '
        'it is getting no scans -- check /scan_filtered, which use_terrain:=false '
        'silences. If you passed use_slam:=false, nothing publishes map -> odom at all '
        'and Nav2 cannot run: add use_nav2:=false.'),
        checks=[
            ['--node', use_slam, 'slam_toolbox'],
            ['--tf', OrSubstitution(use_slam, use_nav2), 'map', 'odom'],
        ])

    # ---- Layer 5: Nav2 ---------------------------------------------------------------
    nav2 = _include('circe_nav', 'navigation.launch.py',
                    condition=IfCondition(use_nav2),
                    arguments=dict(
                        sim_no_rviz,
                        use_mission=LaunchConfiguration('use_mission'),
                        mission_leg_max=LaunchConfiguration('mission_leg_max'),
                        mission_handoff=LaunchConfiguration('mission_handoff'),
                        use_respawn=LaunchConfiguration('use_respawn'),
                    ).items())

    gate_nav2 = _gate('navigation', 120, hint=(
        'Nav2 aborts its whole bringup if any server fails to activate. Read the log '
        'for the FIRST "Failed to activate" line. "No critics defined for FollowPath" '
        'means a params file other than nav2_params.yaml reached controller_server -- '
        'see the _include docstring in this file.'), checks=[
            ['--node', use_nav2, 'bt_navigator'],
            ['--topic', use_nav2, '/local_costmap/costmap'],
            ['--topic', use_nav2, '/global_costmap/costmap'],
        ])

    # ---- Health monitoring -------------------------------------------------------------
    # Outside the chain so it keeps running when a gate fails.
    diagnostics = _include('circe_diagnostics', 'diagnostics.launch.py',
                           condition=IfCondition(LaunchConfiguration('use_diagnostics')),
                           arguments=dict(
                               sim_time,
                               camera_imu='false',
                               use_perception=use_terrain,
                               use_slam=use_slam,
                           ).items())

    ready = LogInfo(msg=(
        '\n'
        '========================================================================\n'
        ' CIRCE SIM IS UP. Every layer was verified before the next one started.\n'
        '========================================================================\n'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='circe_indoor.sdf',
            description='World file in circe_gazebo/worlds'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo GUI'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the simulation layout'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Start slam_toolbox. Needs a live odom -> base_link first, '
                        'which the EKF below provides.'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2. Only meaningful once TF, /scan and odometry are '
                        'all verified clean -- which the gates now do before it starts.'),
        DeclareLaunchArgument(
            'use_mission', default_value='false',
            description='Start cable_mission.py: one far goal on /mission_goal becomes '
                        'a chain of legs Nav2 can plan, driven out on odom and home on '
                        'map. Retarget RViz 2D Goal Pose to /mission_goal to use it.'),
        DeclareLaunchArgument('mission_leg_max', default_value='20.0'),
        DeclareLaunchArgument('mission_handoff', default_value='3.0'),
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='Respawn crashed Nav2 servers; the Nav2 lifecycle manager then '
                        're-activates the stack. Same as circe_bringup.launch.py.'),

        DeclareLaunchArgument(
            'use_terrain', default_value='true',
            description='Start circe_perception: terrain_node (/terrain/grid) AND '
                        'scan_pitch_filter (/scan_filtered). SLAM and both costmaps read '
                        '/scan_filtered, so false blinds them as well as the terrain '
                        'layer.'),
        DeclareLaunchArgument(
            'use_diagnostics', default_value='true',
            description='Run circe_diagnostics, exactly as on the robot: the tree on '
                        '/diagnostics_agg and one overall level on '
                        '/diagnostics_toplevel_state.'),
        DeclareLaunchArgument(
            'strict_order', default_value='true',
            description='Stop the bringup at the first layer that fails to come up. '
                        'false makes the gates warn and continue.'),

        sim,
        diagnostics,
        gate_sim,
        _then(gate_sim, 'simulation', 'the EKF', ekf, gate_ekf),
        # Perception before SLAM: slam_toolbox reads /scan_filtered.
        _then(gate_ekf, 'ekf', 'perception', perception, gate_perception),
        _then(gate_perception, 'perception', 'SLAM', slam, gate_slam),
        _then(gate_slam, 'localization', 'Nav2', nav2, gate_nav2),
        _then(gate_nav2, 'navigation', 'the ready banner', ready),
    ])
