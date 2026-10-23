#!/usr/bin/env python3
"""CIRCE in Gazebo, with the full navigation stack on top.

Layered exactly like circe_bringup.launch.py, so the same debugging discipline
applies: bring it up bottom-up and stop at the layer you are debugging.

    ros2 launch circe_gazebo circe_sim_nav.launch.py                       # sim + EKF + SLAM + Nav2
    ros2 launch circe_gazebo circe_sim_nav.launch.py use_nav2:=false       # sim + EKF + SLAM
    ros2 launch circe_gazebo circe_sim_nav.launch.py use_slam:=false use_nav2:=false   # sim + EKF
    ros2 launch circe_gazebo circe_sim.launch.py                           # sim alone

Do not skip ahead to Nav2. Almost every "Nav2 doesn't work" report is really a bad TF
tree or a bad /scan, and both are far easier to see in isolation. That is as true in
sim as on the robot -- more so, because in sim it is tempting to assume the physics is
the problem when it is really the frame graph.

*** WHY THIS REUSES circe_nav UNCHANGED ***
localization.launch.py, slam.launch.py and navigation.launch.py each already declare a
use_sim_time argument and apply it as an override dict AFTER their params file, so the
launch value beats the `use_sim_time: false` sitting in the YAML. Nav2 gets it from
nav2_bringup's SetParameter, which is why nav2_params.yaml must carry no use_sim_time
keys at all (see the note at the top of that file). So there is no sim-specific copy of
localization_ekf.yaml or nav2_params.yaml anywhere, and there should not be one -- a
forked params file is how sim and hardware quietly drift apart.

circe_bringup.launch.py itself is NOT reused: it declares no use_sim_time and its first
layer starts the real motor driver. Its sequencer, scripts/wait_for.py, IS reused --
see below.

Frame ownership, identical to hardware:
    map -> odom             slam_toolbox           (use_slam:=true)
    odom -> base_link       robot_localization EKF
    base_link -> ...        robot_state_publisher, from the URDF
The gz DiffDrive plugin is deliberately not on that list; see config/gz_bridge.yaml.

*** SEQUENCED BY THE SAME GATES AS THE ROBOT, NOT BY A TIMER. ***
This file used to start the EKF, SLAM and terrain all at once, and Nav2 after a fixed
TimerAction(period=12.0) whose comment admitted the delay was load-bearing. It was also
too short. Measured on the dev laptop: Nav2 activated on the 12 s mark and global_costmap
then spent 4.5 s logging

    Timed out waiting for transform from base_link to map to become available,
    tf error: Invalid frame ID "map" ... - frame does not exist

because slam_toolbox had not yet published map -> odom. It survived only because the
costmap waits 60 s before failing its activation -- a slower host or a heavier world
loses that race outright, and lifecycle_manager_navigation then aborts all of Nav2.

The layers are now chained through circe_bringup/scripts/wait_for.py exactly as
circe_bringup.launch.py chains them -- same order, same signals, same failure reports --
so the simulation exercises the sequencing the hardware depends on instead of hiding it:

    sim hardware    gz, robot_state_publisher, bridge, sector filter
      gate          base_link -> lidar_link, /odometry/wheels, /scan, depth, /imu/data
    EKF             robot_localization
      gate          odom -> base_link, /odometry/filtered
    perception      scan_pitch_filter + terrain_node                  (use_terrain)
      gate          /scan_filtered              <-- what SLAM AND the costmaps read
    SLAM            slam_toolbox, via lifecycle_manager_localization  (use_slam)
      gate          map -> odom
    Nav2            the full navigation stack                         (use_nav2)
      gate          bt_navigator up, both costmaps published

Perception comes before SLAM for the reason circe_bringup.launch.py's docstring gives:
slam_toolbox reads /scan_filtered, which perception publishes. Each check carries an
ENABLED flag instead of its gate carrying a condition, so every gate always runs and
the chain survives any combination of the layer arguments. strict_order:=false makes a
timed-out gate warn and continue.

*** DIAGNOSTICS RUN HERE TOO, OUTSIDE THE CHAIN, STARTED FIRST. ***
circe_diagnostics is included the way circe_bringup.launch.py includes it -- the same
launch file, the same analyzers.yaml and health_monitor.yaml, the same node names -- so
/diagnostics_agg and /diagnostics_toplevel_state read the same in sim as on the robot.
The sim differs only in arguments: use_sim_time:=true, and camera_imu:=false because
Gazebo's rgbd_camera has no IMU stream (the simulated IMU publishes /imu/data itself).
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
    """Include a launch file with its launch configurations isolated.

    *** THE GroupAction IS LOAD-BEARING. DO NOT REMOVE IT. ***

    IncludeLaunchDescription does not scope launch configurations. All three circe_nav
    launch files declare an argument named `params_file`, each defaulting to its own
    YAML, and DeclareLaunchArgument only applies its default when the configuration is
    not already set. So without scoping, the first include to run wins and every later
    one silently inherits its value:

        localization.launch.py  sets params_file = localization_ekf.yaml
        slam.launch.py          inherits it, never sees mapper_params_circe.yaml
        navigation.launch.py    inherits it, never sees nav2_params.yaml

    Nav2 then finds no controller_server key in localization_ekf.yaml, falls back to
    its compiled defaults, loads dwb_core::DWBLocalPlanner instead of MPPI, finds no
    critics for it, and aborts the whole bringup with "No critics defined for
    FollowPath" - an error that says nothing about params files at all.

    GroupAction defaults to scoped=True, which pushes and pops the configuration scope
    around each include so each one gets its own defaults back.
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
    """A process that blocks until `checks` pass, then exits. Same helper as
    circe_bringup.launch.py; see circe_bringup/scripts/wait_for.py.

    The gate is UNCONDITIONAL and each check begins with an ENABLED flag, so a switched
    off layer drops its checks rather than its gate, the gate exits at once, and its
    handler still starts the next layer. It needs no use_sim_time: it waits for a
    message, a TF edge that can be resolved at all, or a node name, none of which
    depends on which clock is running.
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

    # Every node below Gazebo has to agree that time comes from /clock. Anything left
    # on wall clock will throw TF extrapolation errors as soon as the sim's real time
    # factor drifts from 1.0, which it will.
    sim_time = {'use_sim_time': 'true'}

    # *** PASS use_rviz EXPLICITLY, DO NOT LET IT INHERIT. ***
    # GroupAction's scoping stops a child's configurations from leaking OUT, but the
    # parent's are still visible going IN, and DeclareLaunchArgument only applies its
    # default when the configuration is not already set. This file declares use_rviz
    # (default true) for Layer 1, so slam.launch.py and navigation.launch.py -- which
    # each declare their own use_rviz defaulting to FALSE -- inherit true instead and
    # start rviz2_slam and rviz2_nav2 on top of the sim's own rviz2. Three rviz2
    # processes, three OGRE contexts, and the sim's real-time factor on the floor.
    # Layer 1 owns the only rviz here, with circe_gazebo/rviz/circe_sim.rviz.
    sim_no_rviz = dict(sim_time, use_rviz='false')

    # ---- Layer 1: simulated hardware -------------------------------------------------
    sim = _include('circe_gazebo', 'circe_sim.launch.py', arguments={
        'world': LaunchConfiguration('world'),
        'gui': LaunchConfiguration('gui'),
        'use_rviz': LaunchConfiguration('use_rviz'),
    }.items())

    # One gate for everything the simulated robot publishes, where the hardware needs two
    # (base, sensors): gz produces all of it from one spawn, so there is no separate
    # device to isolate. A fixed joint on /tf_static proves robot_state_publisher built
    # the model; the rest proves the model spawned with its plugins, /clock is running,
    # and the bridge and the sector filter are passing data.
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
    # Owns odom -> base_link. Fuses /odometry/wheels from the DiffDrive plugin and the
    # simulated /imu/data.
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
    # Depends on odom -> base_link from layer 2. Publishes no TF, so it cannot disturb
    # the frame-ownership contract.
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
    # use_lifecycle_manager:=true starts lifecycle_manager_localization alongside
    # slam_toolbox, which is what makes RViz's Nav2 panel report "Localization:
    # active" instead of a permanently grey inactive. See the note in slam.launch.py:
    # the panel queries that manager by name, and CIRCE has no AMCL to create it.
    slam = _include('circe_nav', 'slam.launch.py',
                    condition=IfCondition(use_slam),
                    arguments=dict(sim_no_rviz, use_lifecycle_manager='true').items())

    # map -> odom is also required when SLAM is off but Nav2 is on: nothing else
    # publishes it, and Nav2's global_costmap cannot activate without it. See the same
    # gate in circe_bringup.launch.py.
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
    # Outside the chain and started first, as on the robot, so it watches the bringup
    # happen and keeps running on exactly the runs where a gate fails.
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
        # Long-range mission driver, defined in circe_nav/launch/navigation.launch.py.
        # Off by default: /goal_pose still reaches Nav2 directly, and this only matters
        # for goals past the global costmap's ~30 m rolling half-width.
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

        # *** THIS IS NOT JUST THE TERRAIN LAYER. ***
        # terrain.launch.py runs scan_pitch_filter as well as terrain_node, and
        # /scan_filtered is what slam_toolbox AND both costmaps read. So false leaves
        # SLAM with no scans (the localization gate then times out and says so) and the
        # costmaps with no lidar -- it is not the lidar-only A/B it used to be. To A/B
        # only the terrain layer, remove terrain_layer from the local_costmap plugins in
        # nav2_params.yaml instead.
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

        # The chain. Layer 1 starts immediately; every layer after it is started by the
        # gate below the one before it.
        sim,
        diagnostics,
        gate_sim,
        _then(gate_sim, 'simulation', 'the EKF', ekf, gate_ekf),
        # *** PERCEPTION BEFORE SLAM. *** slam_toolbox reads /scan_filtered.
        _then(gate_ekf, 'ekf', 'perception', perception, gate_perception),
        _then(gate_perception, 'perception', 'SLAM', slam, gate_slam),
        _then(gate_slam, 'localization', 'Nav2', nav2, gate_nav2),
        _then(gate_nav2, 'navigation', 'the ready banner', ready),
    ])
