#!/usr/bin/env python3
"""CIRCE top-level bring-up. One entry point, layered by launch argument.

    ros2 launch circe_bringup circe_bringup.launch.py          # full stack
    ... use_nav2:=false use_slam:=false                        # peel layers off
    ... bench_mode:=true                                       # no rover attached

Frame ownership (one publisher per edge):
    map -> odom          slam_toolbox
    odom -> base_link    robot_localization EKF
    base_link -> ...     robot_state_publisher (URDF)

Each layer is followed by a wait_for.py gate; the next layer starts only once the
previous one's output is observed. Order:
    base -> sensors -> EKF -> perception -> SLAM -> Nav2
Perception must come before SLAM (slam_toolbox reads /scan_filtered).
On a gate timeout the chain stops; strict_order:=false makes gates warn and continue.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (AndSubstitution, EqualsSubstitution,
                                  LaunchConfiguration, NotSubstitution,
                                  OrSubstitution)
from launch_ros.actions import Node


def _include(package, launch_file, condition=None, arguments=None):
    """Include a launch file in a scoped GroupAction.

    The scope is required: the circe_nav launch files all declare `params_file`, and
    without it the first include's value leaks into the others.
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
    """Gate node (scripts/wait_for.py). The gate is unconditional; each check carries
    its own enabled flag so disabled layers pass straight through."""
    arguments = ['--label', label,
                 '--timeout', str(float(timeout)),
                 '--required', LaunchConfiguration('strict_order')]
    for check in checks:
        arguments += check
    if hint:
        arguments += ['--hint', hint]

    return Node(
        package='circe_bringup',
        executable='wait_for.py',
        name=f'gate_{label}',
        arguments=arguments,
        output='screen',
    )


def _then(gate, label, next_label, *actions):
    """Start `actions` once `gate` exits successfully; otherwise log and stop the chain."""
    def _on_exit(event, context):
        if event.returncode == 0:
            return list(actions)
        return [LogInfo(msg=(
            f'[bringup] {next_label} NOT STARTED: the "{label}" gate above timed out. '
            f'Everything up to and including {label} is still running -- inspect it '
            f'with `ros2 topic list`, `ros2 topic hz`, and `ros2 run tf2_tools '
            f'view_frames`. Re-run with strict_order:=false to start the remaining '
            f'layers anyway.'))]

    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=_on_exit))


def generate_launch_description():
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_imu = LaunchConfiguration('use_imu')
    imu_source = LaunchConfiguration('imu_source')
    use_perception = LaunchConfiguration('use_perception')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_teleop = LaunchConfiguration('use_teleop')
    bench_mode = LaunchConfiguration('bench_mode')
    strict_order = LaunchConfiguration('strict_order')

    on_rover = NotSubstitution(bench_mode)

    # /imu/data only publishes if its source device is running.
    imu_will_publish = AndSubstitution(
        use_imu,
        OrSubstitution(
            AndSubstitution(use_camera, EqualsSubstitution(imu_source, 'd456')),
            EqualsSubstitution(imu_source, 'bno055')))

    scan_filtered_live = AndSubstitution(use_perception, use_lidar)

    # ---- Layer 1: base (bench_mode: display.launch.py + static odom -> base_link) ----
    base = [
        _include('circe_bringup', 'rover.launch.py',
                 condition=UnlessCondition(bench_mode),
                 arguments={'strict_order': strict_order}.items()),

        GroupAction(
            condition=IfCondition(bench_mode),
            actions=[
                _include('circe_description', 'display.launch.py'),
                # Never on the real robot: it fights the EKF.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='bench_odom_to_base_link',
                    arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
                ),
            ],
        ),
    ]

    # Fixed-joint TF proves the URDF built; /odometry/wheels proves the motor link.
    gate_base = _gate('base', 60, hint=(
        'No TF means robot_state_publisher did not start: run `xacro '
        '$(ros2 pkg prefix circe_description)/share/circe_description/urdf/'
        'circe_rover.urdf.xacro` and read the error. No /odometry/wheels means the '
        'motor driver is not talking: check that /dev/rover-control exists (udev/'
        '61-roverrobotics.rules), that the rover is powered on, and that nothing else '
        'holds the port.'), checks=[
            ['--tf', 'true', 'base_link', 'lidar_link'],
            ['--topic', on_rover, '/odometry/wheels'],
        ])

    # ---- Layer 2: sensors ----
    sensors = _include('circe_bringup', 'sensors.launch.py', arguments={
        'use_lidar': use_lidar,
        'use_camera': use_camera,
        'use_imu': use_imu,
        'imu_source': imu_source,
        'strict_order': strict_order,
    }.items())

    gate_sensors = _gate('sensors', 90, hint=(
        'No /scan: check /scan_raw first -- traffic there isolates the fault to the '
        'sector filter rather than the lidar (/dev/rplidar, udev/60-rplidar.rules). '
        'No camera depth: the D456 needs a USB 3 port, and initial_reset: true in '
        'realsense_d456.yaml makes a cold start slow. No /imu/data with '
        'imu_source:=d456: imu_fusion.py is running but /camera/camera/imu is not '
        'arriving, so this is the camera, not the fusion.'), checks=[
            ['--topic', use_lidar, '/scan'],
            ['--topic', use_camera, '/camera/camera/depth/image_rect_raw'],
            ['--topic', imu_will_publish, '/imu/data'],
        ])

    # ---- Layer 3: state estimation (fuse wheel yaw when there's no IMU) ----
    ekf = _include('circe_nav', 'localization.launch.py',
                   condition=UnlessCondition(bench_mode),
                   arguments={'fuse_wheel_yaw': NotSubstitution(imu_will_publish)}.items())

    # odom -> base_link is what Nav2's costmaps wait on.
    gate_ekf = _gate('ekf', 30, hint=(
        'The EKF has /odometry/wheels but is not publishing odom -> base_link. Check '
        'that publish_tf is true in circe_nav/config/localization_ekf.yaml, that the '
        'top-level key there is still ekf_filter_node (it must match the node name in '
        'localization.launch.py), and that /odometry/wheels carries a sane frame_id.'),
        checks=[
            ['--tf', 'true', 'odom', 'base_link'],
            ['--topic', on_rover, '/odometry/filtered'],
        ])

    # ---- Layer 5: SLAM (driven by lifecycle_manager_localization) ----
    slam = _include('circe_nav', 'slam.launch.py',
                    condition=IfCondition(use_slam),
                    arguments={'use_lifecycle_manager': 'true'}.items())

    # map -> odom is also required when SLAM is off but Nav2 is on.
    gate_slam = _gate('localization', 90, hint=(
        'Run `ros2 lifecycle get /slam_toolbox`. "unconfigured [1]" means nothing sent '
        'it a transition -- on Jazzy async_slam_toolbox_node is a LIFECYCLE node and a '
        'plain launch_ros Node leaves it parked there silently. "active [3]" with no '
        'map -> odom means it is getting no scans -- check /scan_filtered, NOT /scan, '
        'because mapper_params_circe.yaml points scan_topic at the filtered topic. '
        'If you passed use_slam:=false, nothing publishes map -> odom at all and Nav2 '
        'cannot run: add use_nav2:=false.'),
        checks=[
            ['--node', use_slam, 'slam_toolbox'],
            ['--tf', OrSubstitution(use_slam, use_nav2), 'map', 'odom'],
        ])

    # ---- Layer 4: perception ----
    # Required by Nav2: scan_pitch_filter -> /scan_filtered (costmaps + SLAM),
    # terrain_node -> /terrain/grid (terrain_layer).
    perception = _include('circe_perception', 'terrain.launch.py',
                          condition=IfCondition(use_perception))

    # terrain_node only publishes once Nav2 subscribes, so just check the node is up.
    gate_perception = _gate('perception', 45, hint=(
        'No /scan_filtered while /scan is flowing means scan_pitch_filter is not '
        'running or crashed on import (it needs numpy and scipy). Nav2 reads '
        '/scan_filtered, so do NOT start it until this is fixed: its costmaps would '
        'have no obstacle data at all and the rover would drive blind.'), checks=[
            ['--node', use_perception, 'scan_pitch_filter'],
            ['--node', use_perception, 'terrain_node'],
            ['--topic', scan_filtered_live, '/scan_filtered'],
        ])

    # ---- Layer 6: Nav2 ----
    nav2 = _include('circe_nav', 'navigation.launch.py',
                    condition=IfCondition(use_nav2),
                    arguments={
                        'use_mission': LaunchConfiguration('use_mission'),
                        'mission_leg_max': LaunchConfiguration('mission_leg_max'),
                        'mission_handoff': LaunchConfiguration('mission_handoff'),
                        'use_respawn': LaunchConfiguration('use_respawn'),
                    }.items())

    # Both costmaps publish only once activated (local needs EKF, global needs SLAM).
    gate_nav2 = _gate('navigation', 120, hint=(
        'Nav2 aborts its whole bringup if any server fails to activate. Read the log '
        'for the FIRST "Failed to activate" line. "No critics defined for FollowPath" '
        'means a params file other than nav2_params.yaml reached controller_server -- '
        'see the _include docstring in this file.'), checks=[
            ['--node', use_nav2, 'bt_navigator'],
            ['--topic', use_nav2, '/local_costmap/costmap'],
            ['--topic', use_nav2, '/global_costmap/costmap'],
        ])

    # ---- Optional: teleop (outside the chain; starts after the base gate) ----
    teleop = _include('circe_control', 'teleop.launch.py',
                      condition=IfCondition(use_teleop))

    # ---- Health monitoring (outside the chain; starts with Layer 1) ----
    diagnostics = _include('circe_diagnostics', 'diagnostics.launch.py',
                           condition=IfCondition(LaunchConfiguration('use_diagnostics')),
                           arguments={
                               'use_lidar': use_lidar,
                               'use_camera': use_camera,
                               'use_imu': use_imu,
                               'imu_source': imu_source,
                               'use_perception': use_perception,
                               'use_slam': use_slam,
                               'bench_mode': bench_mode,
                           }.items())

    ready = LogInfo(msg=(
        '\n'
        '========================================================================\n'
        ' CIRCE IS UP. Every layer was verified before the next one started.\n'
        '========================================================================\n'))

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        # False until the D456's USB 3 cable is fitted.
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Publish /imu/data. Without it the EKF loses its only heading '
                        'reference, and scan_pitch_filter and terrain_node lose the '
                        'attitude that keeps ramps out of the costmap.'),
        DeclareLaunchArgument(
            'imu_source', default_value='d456',
            choices=['d456', 'bno055'],
            description='Which device backs /imu/data. d456 is the camera\'s own IMU, '
                        'fused by circe_perception/imu_fusion.py; it needs '
                        'use_camera:=true. bno055 is the external part, not fitted.'),
        DeclareLaunchArgument(
            'use_perception', default_value='true',
            description='Run circe_perception (scan_pitch_filter + terrain_node). '
                        'Nav2 gets its lidar through /scan_filtered from this package, '
                        'so turning it off blinds the costmaps unless nav2_params.yaml '
                        'is also pointed back at /scan.'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Start slam_toolbox, which owns map -> odom. Needs a live '
                        'odom -> base_link first, or it blocks waiting for the '
                        'transform and publishes no map -- the ekf gate below is what '
                        'guarantees it has one. Set false to drive and map nothing.'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2. Only meaningful once TF, /scan and odometry are '
                        'all clean, which the gates below now verify rather than '
                        'assume. Set false to bring the robot up to SLAM and drive it '
                        'manually.'),
        DeclareLaunchArgument(
            'use_mission', default_value='false',
            description='Start cable_mission.py: one far goal on /mission_goal becomes '
                        'a chain of legs Nav2 can plan, driven out on odom and home on '
                        'map. Needed for any goal past the global costmap half-width.'),
        DeclareLaunchArgument('mission_leg_max', default_value='20.0'),
        DeclareLaunchArgument('mission_handoff', default_value='3.0'),
        DeclareLaunchArgument(
            'use_respawn', default_value='false',
            description='Respawn crashed Nav2 servers; the Nav2 lifecycle manager then '
                        're-activates the stack.'),

        DeclareLaunchArgument(
            'use_teleop', default_value='false',
            description='Start gamepad teleop from circe_control'),
        DeclareLaunchArgument(
            'use_diagnostics', default_value='true',
            description='Run circe_diagnostics: live health of the sensors, EKF, SLAM, '
                        'Nav2 and the Pi on /diagnostics_agg. View it with '
                        '`ros2 run rqt_robot_monitor rqt_robot_monitor`.'),
        DeclareLaunchArgument(
            'bench_mode', default_value='false',
            description='Desk testing with no rover attached: skip the motor driver '
                        'and the EKF, and publish a static odom -> base_link so tools '
                        'that need a complete tree still work. NEVER set this on the '
                        'real robot -- it pins the robot at the origin and fights the '
                        'EKF, which is exactly the bug the old circe_tf package caused.'),
        DeclareLaunchArgument(
            'strict_order', default_value='true',
            description='Stop the bringup at the first layer that fails to come up, '
                        'instead of starting the layers above it on top of a broken '
                        'one. Set false to make the gates warn and continue -- useful '
                        'when you are deliberately running degraded and know which '
                        'signal is missing, and nothing else.'),

        # Layer 1 starts now; each later layer is started by the gate before it.
        *base,
        diagnostics,
        gate_base,
        _then(gate_base, 'base', 'sensors', sensors, gate_sensors, teleop),
        _then(gate_sensors, 'sensors', 'the EKF', ekf, gate_ekf),
        _then(gate_ekf, 'ekf', 'perception', perception, gate_perception),
        _then(gate_perception, 'perception', 'SLAM', slam, gate_slam),
        _then(gate_slam, 'localization', 'Nav2', nav2, gate_nav2),
        _then(gate_nav2, 'navigation', 'the ready banner', ready),
    ])
