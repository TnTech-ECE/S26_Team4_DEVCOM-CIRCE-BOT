#!/usr/bin/env python3
"""CIRCE top-level bring-up. One entry point, layered by launch argument.

The layers stack bottom-up, exactly as BRINGUP_CHECKLIST.md prescribes, and each is
independently switchable, so you peel layers OFF to get down to whichever one you are
debugging:

    the whole stack                         ros2 launch circe_bringup circe_bringup.launch.py
    ... + gamepad teleop                    ... use_teleop:=true
    everything but Nav2                     ... use_nav2:=false
    ... and without SLAM                    ... use_nav2:=false use_slam:=false
    base + sensors + EKF only               ... use_nav2:=false use_slam:=false use_perception:=false
    no rover attached, lidar on the bench   ... bench_mode:=true
    ... without health monitoring           ... use_diagnostics:=false

*** THE FULL STACK IS THE DEFAULT, WHICH IT WAS NOT BEFORE. ***
use_slam and use_nav2 both defaulted false, so the bare command started base + sensors
+ EKF + perception and you added the top two layers as you worked up the checklist.
They now default true, matching circe_sim_nav.launch.py -- sim and hardware bring up
the same stack from the same bare command, which is the point of keeping one nav config
for both.

The advice that default encoded is still true and still worth following: almost every
"Nav2 doesn't work" report is really a bad TF tree or a bad /scan, and both are far
easier to see in isolation. What changed is that you no longer have to remember it,
because the gates below enforce it. Nav2 is not started until odom -> base_link,
map -> odom and /scan_filtered have each been OBSERVED live, and if one of them never
appears the bringup stops there and names it rather than starting Nav2 on top of a
broken layer. Peeling layers off with the arguments above is now a debugging tool
rather than the normal path.

Frame ownership -- exactly one publisher per edge, which is the whole point:
    map -> odom             slam_toolbox           (use_slam:=true)
    odom -> base_link       robot_localization EKF (rover_zero_4wd.yaml keeps the
                                                    driver's publish_tf false)
    base_link -> ...        robot_state_publisher, from the URDF
    camera_link -> camera_* realsense2_camera, from factory calibration


*** THE LAYERS ARE SEQUENCED, NOT MERELY ORDERED. ***

Listing actions in a LaunchDescription orders nothing: launch visits them all in a few
milliseconds and every process races to initialise at once. On hardware that race is
lost regularly, and the report is never "the launch order was wrong" -- it is one of
these:

    Failed to activate local_costmap because transform from base_link to odom did not
    become available before timeout

        Nav2's costmaps wait a fixed timeout for odom -> base_link and, missing it,
        fail to activate, whereupon lifecycle_manager_navigation aborts the ENTIRE Nav2
        bringup. The transform comes from the EKF, which cannot publish until the motor
        driver has opened /dev/rover-control and produced /odometry/wheels. On a cold
        boot the serial open plus the first odometry message is comfortably slower than
        Nav2's timeout.

    a /map that never appears

        slam_toolbox blocks on odom -> base_link, and it reads /scan_filtered rather
        than /scan (scan_topic in mapper_params_circe.yaml), so it also blocks on
        scan_pitch_filter. Started before either it does not error; it simply produces
        nothing, forever.

    a costmap with no obstacles in it

        Nav2 reads /scan_filtered, published by circe_perception's scan_pitch_filter,
        which reads /scan, published by the sector filter, which reads /scan_raw from
        the lidar driver. Any link of that chain starting into a void stays quiet.

circe_gazebo/launch/circe_sim_nav.launch.py used to paper over the same race with
TimerAction(period=12.0), and it lost it: measured, Nav2 activated before slam_toolbox
had published map -> odom. A fixed delay is a guess: too short on a cold Pi 5 and it
fails anyway, too long and every bringup pays for the worst case. So each layer here is
followed by a GATE -- circe_bringup/scripts/wait_for.py -- that blocks until the
layer's OUTPUT is observable (a message on a topic, an edge in the TF tree), and only
then is the next layer started, via RegisterEventHandler(OnProcessExit(...)). The
simulation now uses the same gates in the same order.

Waiting on the output rather than on the process is the point. launch's own
OnProcessStart fires when the OS has forked the process, which here means almost
nothing: roverrobotics_driver spawns before it opens the serial port, realsense2_camera
spawns before it resets and enumerates the D456 over USB 3, and slam_toolbox spawns
into `unconfigured` where it subscribes to nothing at all. Every one of those is a real
hardware failure that OnProcessStart reports as success.

    base            robot_state_publisher, joint_state_publisher, motor driver
      gate          base_link -> lidar_link in TF, and a message on /odometry/wheels
    sensors         lidar + sector filter, camera, IMU fusion
      gate          /scan, camera depth, /imu/data
    EKF             robot_localization
      gate          odom -> base_link                     <-- what Nav2 times out on
    perception      scan_pitch_filter + terrain_node
      gate          /scan_filtered                 <-- what SLAM AND the costmaps read
    SLAM            slam_toolbox, via nav2_lifecycle_manager
      gate          map -> odom
    Nav2            the full navigation stack
      gate          bt_navigator up and /local_costmap/costmap published

*** PERCEPTION COMES BEFORE SLAM, WHICH IS NOT WHAT THE OLD LAYER NUMBERING SAID. ***
circe_nav/config/mapper_params_circe.yaml sets `scan_topic: /scan_filtered`, not /scan.
slam_toolbox therefore consumes circe_perception's output exactly as the costmaps do,
and starting it first means it configures, activates, bonds to its lifecycle manager,
logs nothing further and never publishes map -> odom -- while scan_pitch_filter, the
node it is waiting for, has not been launched yet. That is a deadlock, and it is
invisible: `ros2 lifecycle get /slam_toolbox` says active [3], the node answers service
calls, and the only symptom is a missing transform.

It was harmless while every layer started at once, which is why the old Layer 3 / Layer
3b numbering put SLAM first. Sequencing makes it fatal, so the chain follows the data
dependency rather than the numbering. Do not reorder this pair back.

A gate whose checks are all disabled (because the layer it guards is switched off)
exits immediately, so the chain stays intact for every combination of arguments. That
is why the checks carry an enable flag instead of the gate carrying a condition: a gate
that never ran would never fire its handler, and the bringup would stop dead at
use_slam:=false.

When a gate times out it prints exactly which signal is missing and stops the chain,
leaving everything below it running so you can inspect the graph that did come up. Set
strict_order:=false to make gates warn and continue instead.
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
    """Include a launch file with its launch configurations isolated.

    *** THE GroupAction IS LOAD-BEARING. DO NOT REMOVE IT. ***

    IncludeLaunchDescription does not scope launch configurations, and
    DeclareLaunchArgument only applies its default when the configuration is not
    already set. All three circe_nav launch files declare an argument named
    `params_file`, each defaulting to a different YAML, so without scoping the first
    include to run wins and the rest silently inherit its value:

        localization.launch.py  sets params_file = localization_ekf.yaml
        slam.launch.py          inherits it, never sees mapper_params_circe.yaml
        navigation.launch.py    inherits it, never sees nav2_params.yaml

    Nav2 then finds no controller_server key in localization_ekf.yaml, falls back to
    its compiled defaults, loads dwb_core::DWBLocalPlanner instead of the configured
    MPPI, finds no critics for it, and aborts the entire bringup with "No critics
    defined for FollowPath" - an error that never mentions params files.

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
    """A process that blocks until `checks` pass, then exits. See scripts/wait_for.py.

    Each check is a list ready to splice into the command line, and every one of them
    begins with an ENABLED flag rather than being wrapped in a condition. That is
    deliberate and it is what makes the chain survive every combination of layer
    arguments: the gate itself is UNCONDITIONAL, so its OnProcessExit handler always
    fires and always starts the layer after it. Only the individual checks come and go.

    Give the gate a condition instead and use_slam:=false stops the bringup at SLAM
    forever, waiting for a process that was never launched.
    """
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
    """Start `actions` once `gate` has exited successfully.

    The returncode branch is the whole reason this is a callable rather than a plain
    entities list. A gate that timed out has told you the layer beneath is not working;
    starting the layer above it then produces a second, louder, less informative failure
    on top of the first -- Nav2 aborting its whole bringup over a missing transform is
    exactly that, and it is what sends people looking at nav2_params.yaml when the real
    fault is a serial cable.

    Everything already running is left up on purpose. The graph that DID come up is the
    evidence you need, and killing it to report a failure throws that away.
    """
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

    # True exactly when a real rover is attached, i.e. when the motor driver is running
    # and /odometry/wheels is somebody's responsibility.
    on_rover = NotSubstitution(bench_mode)

    # *** WHEN IS /imu/data ACTUALLY GOING TO BE PUBLISHED? ***
    # Not simply "when use_imu is true". With imu_source:=d456 the publisher is
    # imu_fusion.py, which republishes the camera's IMU -- so with use_camera:=false it
    # starts, subscribes to a topic nobody publishes, and stays silent. Gating on
    # use_imu alone would hang the sensors gate for its full timeout on a legitimate
    # lidar-only configuration, which is precisely the false negative that makes people
    # rip gates out again.
    imu_will_publish = AndSubstitution(
        use_imu,
        OrSubstitution(
            AndSubstitution(use_camera, EqualsSubstitution(imu_source, 'd456')),
            EqualsSubstitution(imu_source, 'bno055')))

    # scan_pitch_filter publishes /scan_filtered only in response to /scan.
    scan_filtered_live = AndSubstitution(use_perception, use_lidar)

    # ---- Layer 1: base ---------------------------------------------------------------
    # In bench_mode the motor driver is skipped (there is nothing to talk to) but
    # robot_state_publisher is still wanted, so the base launch is replaced by
    # display.launch.py -- which brings its own robot_state_publisher,
    # joint_state_publisher and RViz -- plus a static odom -> base_link.
    base = [
        _include('circe_bringup', 'rover.launch.py',
                 condition=UnlessCondition(bench_mode),
                 arguments={'strict_order': strict_order}.items()),

        GroupAction(
            condition=IfCondition(bench_mode),
            actions=[
                _include('circe_description', 'display.launch.py'),
                # *** NEVER ON THE REAL ROBOT. *** It pins the robot at the origin and
                # fights the EKF, which is exactly the bug the old circe_tf package
                # caused. bench_mode is what keeps the two mutually exclusive.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='bench_odom_to_base_link',
                    arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
                ),
            ],
        ),
    ]

    # base_link -> lidar_link is a FIXED joint, published on /tf_static the moment
    # robot_state_publisher has parsed the xacro, so it proves the URDF built and the
    # model is being published without depending on joint states. /odometry/wheels
    # proves the serial link to the motor controller is open and talking, which is the
    # single most common hardware failure on this robot.
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

    # ---- Layer 2: sensors ------------------------------------------------------------
    sensors = _include('circe_bringup', 'sensors.launch.py', arguments={
        'use_lidar': use_lidar,
        'use_camera': use_camera,
        'use_imu': use_imu,
        'imu_source': imu_source,
        'strict_order': strict_order,
    }.items())

    # /scan rather than /scan_raw: it is the topic slam_toolbox and scan_pitch_filter
    # actually read, so it checks the driver AND the sector filter behind it in one go.
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

    # ---- Layer 3: state estimation ---------------------------------------------------
    # With no /imu/data the filter has no heading source at all (imu0 is its only yaw
    # input), so fall back to the wheels' yaw rate exactly when no IMU will publish.
    ekf = _include('circe_nav', 'localization.launch.py',
                   condition=UnlessCondition(bench_mode),
                   arguments={'fuse_wheel_yaw': NotSubstitution(imu_will_publish)}.items())

    # *** THIS IS THE GATE THAT EARNS THE WHOLE MECHANISM. ***
    # odom -> base_link is what Nav2's costmaps time out waiting for, and its absence is
    # also why slam_toolbox would sit silent. In bench_mode the EKF is skipped and this
    # edge comes from the static publisher above instead, so the check is unconditional
    # either way -- the layer is "something owns odom -> base_link", not "the EKF is
    # running".
    gate_ekf = _gate('ekf', 30, hint=(
        'The EKF has /odometry/wheels but is not publishing odom -> base_link. Check '
        'that publish_tf is true in circe_nav/config/localization_ekf.yaml, that the '
        'top-level key there is still ekf_filter_node (it must match the node name in '
        'localization.launch.py), and that /odometry/wheels carries a sane frame_id.'),
        checks=[
            ['--tf', 'true', 'odom', 'base_link'],
            ['--topic', on_rover, '/odometry/filtered'],
        ])

    # ---- Layer 5: SLAM ---------------------------------------------------------------
    # use_lifecycle_manager:=true starts lifecycle_manager_localization alongside
    # slam_toolbox and lets IT drive the configure/activate transitions, instead of the
    # bespoke event pair inside slam.launch.py. Two things follow:
    #
    #   1. RViz's Nav2 panel reports "Localization: active". That panel calls
    #      /lifecycle_manager_localization/manage_nodes by name; CIRCE has no AMCL, so
    #      without this manager the service does not exist and the panel stays grey
    #      however well SLAM is running.
    #   2. SLAM is brought up the same way every other Nav2 server is, and the panel's
    #      Reset and pause controls actually reach slam_toolbox.
    #
    # Matches circe_sim_nav.launch.py, so sim and hardware activate SLAM identically -
    # which is the whole reason this workspace keeps one nav config for both.
    slam = _include('circe_nav', 'slam.launch.py',
                    condition=IfCondition(use_slam),
                    arguments={'use_lifecycle_manager': 'true'}.items())

    # map -> odom is the one signal that distinguishes "slam_toolbox is running" from
    # "slam_toolbox is working". The node-in-graph check is what tells the two apart in
    # the timeout report: node present but no transform means it is parked in
    # `unconfigured` or `inactive` and the lifecycle manager did not drive it.
    #
    # *** map -> odom IS ALSO CHECKED WHEN SLAM IS OFF BUT NAV2 IS ON. ***
    # Nav2's global_costmap runs in `map`. `use_slam:=false` alone leaves use_nav2 at
    # its default true, and this gate used to wave that through: Nav2 started, its
    # global_costmap waited out its 60 s initial_transform_timeout for a frame nothing
    # publishes, failed to activate, and lifecycle_manager_navigation aborted the whole
    # Nav2 bringup -- a minute late and naming the costmap, not the argument. Keeping the
    # check on stops the chain HERE instead, with a hint that names the cause.
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

    # ---- Layer 4: perception ---------------------------------------------------------
    # *** NAV2 DOES NOT WORK WITHOUT THIS. THIS IS NOT AN OPTIONAL EXTRA. ***
    # circe_perception runs two nodes, and Nav2's costmaps depend on BOTH:
    #
    #   scan_pitch_filter.py  publishes /scan_filtered, which is the observation source
    #                         for the voxel_layer in local_costmap AND the
    #                         obstacle_layer in global_costmap. If this node is not
    #                         running, /scan_filtered is silent and the costmaps see NO
    #                         LIDAR AT ALL - the rover drives blind. It fails safe on a
    #                         missing IMU (passes the raw scan through), but it cannot
    #                         fail safe on not being launched.
    #
    #   terrain_node.py       publishes /terrain/grid for the terrain_layer. It degrades
    #                         gracefully only because it publishes an all-unknown grid
    #                         until it has depth (no camera, camera not up yet). A
    #                         StaticLayer that has NEVER received a map does not
    #                         "contribute nothing" on Jazzy: it holds local_costmap
    #                         not-current and controller_server aborts every goal with
    #                         "Costmap timed out waiting for update". So with
    #                         use_perception:=false Nav2 plans but cannot drive until
    #                         terrain_layer is removed from nav2_params.yaml too.
    #
    # Depends on the EKF for odom -> base_link and on the camera from Layer 2. It
    # publishes no TF, so it cannot disturb the frame-ownership contract.
    #
    # use_perception:=false is an escape hatch for bringing the robot up without the
    # camera, and it is a BIGGER hammer than it looks. /scan_filtered feeds Nav2's
    # costmaps AND slam_toolbox (scan_topic in mapper_params_circe.yaml), so switching
    # perception off silences both. If you use it you MUST also point
    # circe_nav/config/nav2_params.yaml and circe_nav/config/mapper_params_circe.yaml
    # back at /scan, or you get a rover that neither maps nor sees obstacles.
    perception = _include('circe_perception', 'terrain.launch.py',
                          condition=IfCondition(use_perception))

    # /scan_filtered gets the strong check because Nav2 genuinely cannot see without it.
    # terrain_node gets only a node-in-graph check on purpose: it publishes
    # /terrain/grid only while something subscribes (see its _publish guard), and its
    # one subscriber is Nav2's local_costmap, which starts two layers LATER in this
    # chain. Waiting on the topic here would deadlock. Whether the grid then reaches the
    # costmap is exactly what the smoke test's navigation check exercises.
    gate_perception = _gate('perception', 45, hint=(
        'No /scan_filtered while /scan is flowing means scan_pitch_filter is not '
        'running or crashed on import (it needs numpy and scipy). Nav2 reads '
        '/scan_filtered, so do NOT start it until this is fixed: its costmaps would '
        'have no obstacle data at all and the rover would drive blind.'), checks=[
            ['--node', use_perception, 'scan_pitch_filter'],
            ['--node', use_perception, 'terrain_node'],
            ['--topic', scan_filtered_live, '/scan_filtered'],
        ])

    # ---- Layer 6: Nav2 ---------------------------------------------------------------
    nav2 = _include('circe_nav', 'navigation.launch.py',
                    condition=IfCondition(use_nav2),
                    arguments={
                        'use_mission': LaunchConfiguration('use_mission'),
                        'mission_leg_max': LaunchConfiguration('mission_leg_max'),
                        'mission_handoff': LaunchConfiguration('mission_handoff'),
                        'use_respawn': LaunchConfiguration('use_respawn'),
                    }.items())

    # A costmap topic is published only after that costmap has ACTIVATED, which is the
    # exact transition that fails when a transform is late -- and the two costmaps fail
    # on DIFFERENT transforms, so both are worth checking: local_costmap runs in odom
    # and needs the EKF, global_costmap runs in map and needs slam_toolbox. Reaching
    # this gate means every prerequisite was verified before Nav2 started, so a timeout
    # here is a genuine Nav2 fault rather than a race.
    #
    # This is "Nav2 is serving", not a bond-by-bond audit of all fourteen servers: the
    # ones that activate after global_costmap (behaviours, waypoint_follower, docking)
    # come up a second or two later and are not preconditions for driving.
    gate_nav2 = _gate('navigation', 120, hint=(
        'Nav2 aborts its whole bringup if any server fails to activate. Read the log '
        'for the FIRST "Failed to activate" line. "No critics defined for FollowPath" '
        'means a params file other than nav2_params.yaml reached controller_server -- '
        'see the _include docstring in this file.'), checks=[
            ['--node', use_nav2, 'bt_navigator'],
            ['--topic', use_nav2, '/local_costmap/costmap'],
            ['--topic', use_nav2, '/global_costmap/costmap'],
        ])

    # ---- Optional: teleop ------------------------------------------------------------
    # *** THE ONE THING OUTSIDE THE CHAIN, DELIBERATELY. ***
    # Teleop needs the motor driver and nothing else -- it publishes /cmd_vel, which the
    # driver consumes. Hanging it off the base gate rather than the end of the chain
    # means a gamepad works as soon as the base is verified, including on every run
    # where a LATER gate fails. That is exactly when you want to be able to drive the
    # robot: off the ramp, back to the bench, out of the way.
    teleop = _include('circe_control', 'teleop.launch.py',
                      condition=IfCondition(use_teleop))

    # ---- Health monitoring -------------------------------------------------------------
    # *** OUTSIDE THE CHAIN TOO, AND STARTED FIRST. ***
    # The gates answer "did this layer come up?" once; circe_diagnostics keeps asking for
    # as long as the rover runs, and publishes the answer as one tree on /diagnostics_agg.
    # Hanging it off a gate would leave it NOT running on exactly the runs where a layer
    # failed -- the runs you most want it for -- so it starts with Layer 1 and watches
    # the bringup as it happens. Until each layer is up its checks read "no messages",
    # which is simply true.
    #
    # The layer arguments pass straight through so a layer switched off here is not
    # reported as broken there -- bench_mode included, since it removes the motor
    # driver and with it /odometry/wheels. The EKF and Nav2 publish their own
    # diagnostics and need nothing passed. circe_gazebo's circe_sim_nav.launch.py
    # includes the same file the same way, so the tree reads identically in sim.
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
        # *** DEFAULTS FALSE FOR NOW: the D456 needs a USB 3 cable, not yet fitted. ***
        # On USB 2.1 realsense2_camera rejects depth_profile 424x240x15 and segfaults in
        # a respawn loop. Put this back to true once the camera enumerates at 5000M.
        # Lidar-only: imu_source:=d456 publishes no /imu/data (see use_imu below), so
        # the EKF fuses wheel yaw instead (fuse_wheel_yaw above), and terrain_node
        # publishes an all-unknown grid, so Nav2 drives on the lidar.
        DeclareLaunchArgument('use_camera', default_value='false'),
        # *** DEFAULTS TRUE. ***
        # This is not one of the switchable layers -- it is part of the base, and the
        # only reason it ever defaulted false was that the old source was a BNO055 that
        # is not fitted and whose driver is not installed, so use_imu:=true failed at
        # launch. With
        # imu_source:=d456 the IMU is inside the camera use_camera already starts, so
        # there is no device left to be absent.
        #
        # Costs of true when use_camera:=false: imu_fusion subscribes to a topic
        # nobody publishes and sits idle -- no warning, no output, one dormant process.
        # Costs of false when you forget it: the EKF loses its only heading reference
        # (the +39% rotational scrub goes uncorrected), scan_pitch_filter passes raw
        # scans through, and terrain_node loses its attitude correction. All three
        # degrade in silence. That asymmetry is the whole argument.
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
        # Long-range mission driver, defined in circe_nav/launch/navigation.launch.py.
        # Gated on use_nav2 there. Off by default -- it is an operator tool, and
        # /goal_pose still reaches Nav2 directly whether it runs or not.
        DeclareLaunchArgument(
            'use_mission', default_value='false',
            description='Start cable_mission.py: one far goal on /mission_goal becomes '
                        'a chain of legs Nav2 can plan, driven out on odom and home on '
                        'map. Needed for any goal past the global costmap half-width.'),
        DeclareLaunchArgument('mission_leg_max', default_value='20.0'),
        DeclareLaunchArgument('mission_handoff', default_value='3.0'),
        # Passed to nav2_bringup, which applies respawn=True, respawn_delay=2.0 to every
        # Nav2 server. A respawned server comes back UNCONFIGURED; it is
        # lifecycle_manager_navigation that makes that survivable -- it sees the bond
        # break, resets the stack, and with attempt_respawn_reconnection (Jazzy default
        # true) brings every server back up once the new process re-bonds within
        # bond_respawn_max_duration (default 10 s). Off by default to match nav2_bringup;
        # the hardware drivers and the scan/IMU pipeline respawn regardless.
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

        # The chain. Layer 1 starts immediately; every layer after it is started by the
        # gate below the one before it, so the ordering lives in the handlers rather
        # than in this list.
        *base,
        diagnostics,
        gate_base,
        _then(gate_base, 'base', 'sensors', sensors, gate_sensors, teleop),
        _then(gate_sensors, 'sensors', 'the EKF', ekf, gate_ekf),
        # *** PERCEPTION BEFORE SLAM. *** slam_toolbox reads /scan_filtered, which
        # perception publishes; the other way round deadlocks silently. See the
        # docstring.
        _then(gate_ekf, 'ekf', 'perception', perception, gate_perception),
        _then(gate_perception, 'perception', 'SLAM', slam, gate_slam),
        _then(gate_slam, 'localization', 'Nav2', nav2, gate_nav2),
        _then(gate_nav2, 'navigation', 'the ready banner', ready),
    ])
