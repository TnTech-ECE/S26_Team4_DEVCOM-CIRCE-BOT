#!/usr/bin/env python3
"""CIRCE sensor suite: RPLidar A1 + RealSense D456 (+ optional BNO055).

Replaces roverrobotics_driver/launch/accessories.launch.py. That file opened its own
YAML at launch time and branched on an `active:` key inside it, which meant "run the
lidar without the camera" required editing a config file and rebuilding. Sensors are
now plain launch arguments, so the bottom-up bring-up in the checklist is one command
each:

    ros2 launch circe_bringup sensors.launch.py use_camera:=false   # lidar alone
    ros2 launch circe_bringup sensors.launch.py use_lidar:=false    # camera alone
    ros2 launch circe_bringup sensors.launch.py                     # both

Topics and frames
-----------------
Lidar:  /scan_raw                   full 360 deg sweep, straight off the driver
        /scan                       rear half masked, in frame lidar_link -- this is
                                    what slam_toolbox and both costmaps consume

*** /scan NOW HAS A NODE BEHIND IT. ***
scan_sector_filter stands between the driver and /scan, so if it is not running /scan is
silent and BOTH slam_toolbox and the costmaps are blind -- a harder failure than the old
topology, where only /scan_filtered depended on a node. It is launched here, next to the
driver and under the same use_lidar argument, so the two cannot get separated. If /scan
is quiet, check /scan_raw first: traffic there isolates the fault to this filter rather
than the lidar.
Camera: /camera/camera/depth/image_rect_raw   in frame camera_depth_optical_frame
        /camera/camera/imu                    gyro+accel merged, ~200 Hz, NO orientation
        frames camera_link -> camera_{depth,color,accel,gyro}_* (published by the driver)

IMU:    /imu/data                             fused attitude in base_link

*** /camera/camera/imu IS NOT /imu/data, AND THE DIFFERENCE IS SILENT. ***
The camera's IMU is a 6-axis part with no magnetometer and no on-chip fusion, so every
message it publishes carries orientation (0,0,0,0) with orientation_covariance[0] = -1.
`unite_imu_method: 2` merges the gyro and accel STREAMS onto one topic; it does not
produce an attitude. Both scan_pitch_filter.py and terrain_node.py read msg.orientation
and would extract roll 0.00 / pitch 0.00 from that null quaternion on every message,
without erroring and without tripping scan_pitch_filter's imu_timeout -- messages are
arriving, they just say nothing. imu_fusion.py stands in between and supplies the
missing attitude. Choose the device with `imu_source`, never by remapping.

There is deliberately NO colour topic and no aligned-depth topic. config/realsense_d456.yaml
runs the camera depth-only at 424x240x15 for the Raspberry Pi 5, with enable_color and
align_depth.enable both false. If you are looking for /camera/camera/color/image_raw, it is
not missing -- it is switched off. The simulation still publishes it, which is an intended
sim/hardware divergence recorded in the README.

The camera runs as namespace `camera` + node name `camera`, i.e. fully-qualified
/camera/camera, which is the upstream realsense2_camera convention and gives the
/camera/camera/... topic prefix its own documentation uses. Note that the old
accessories.launch.py used name="realsense" with no namespace, so topics there were
actually /realsense/... -- BRINGUP_CHECKLIST.md Step 2 documented /camera/camera/...
and was wrong for that launch path. It is correct for this one.

Frames are unaffected by any of this: camera_link is composed by the driver from
`camera_name` in config/realsense_d456.yaml, not from the node name.

*** THE SENSORS COME UP ONE AT A TIME, EACH VERIFIED BEFORE THE NEXT. ***
Sequenced with scripts/wait_for.py, the same mechanism circe_bringup.launch.py uses
between layers. Two reasons, and the second is not about logging:

  1. Every step here is a real producer -> consumer edge -- driver -> /scan_raw ->
     sector filter -> /scan, camera -> /camera/camera/imu -> imu_fusion -> /imu/data.
     A consumer started into a void does not error, it goes quiet, and a quiet
     scan_sector_filter is indistinguishable from a quiet lidar until you go looking at
     /scan_raw. Bringing them up in order means the log's last line before the silence
     names which end of the pipe is broken.

  2. USB. The A1 and the D456 are on the same bus on a Pi 5, and the D456 is a USB 3
     device that resets itself on startup (initial_reset: true in
     realsense_d456.yaml). Enumerating both at the same instant is a known source of
     flaky camera starts. Serialising them costs a few seconds of a bring-up that
     happens once per session.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def _gate(label, timeout, checks, hint='', required='true'):
    """Block until `checks` pass, then exit. See scripts/wait_for.py for the details.

    Each check carries its own ENABLED flag rather than the gate carrying a condition,
    so a gate guarding a switched-off sensor still RUNS (and so still fires the handler
    that starts the next sensor) but has nothing to wait for. Put a condition on the
    gate instead and use_lidar:=false would stall the camera behind a process that was
    never launched.

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
            f'[sensors] {next_label} NOT STARTED: the "{label}" gate timed out.'))]

    return RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=_on_exit))


def _launch_setup(context):
    """Build the sensor chain with every launch configuration RESOLVED to a literal.

    *** WHY THIS IS AN OpaqueFunction AND NOT A PLAIN DESCRIPTION. ***
    Everything after the first gate is created by an OnProcessExit handler, i.e. LATER,
    after launch has finished visiting this file. circe_bringup.launch.py includes it
    inside a scoped GroupAction, and that scope is popped as soon as the include has
    been visited -- so a LaunchConfiguration('use_lidar') evaluated in a handler does
    not see the value passed to this include at all. It sees the INCLUDER's own
    `use_lidar`, or crashes the whole bringup if the includer has none. Reproduced:
    included scoped from a parent that declares no use_lidar, the first gate exits and
    launch dies with
        Caught exception in launch: launch configuration 'use_lidar' does not exist
    It only ever worked because circe_bringup.launch.py happens to declare every one
    of these names itself, with the value it passes. The quiet version is worse: pass
    this include a value that differs from the includer's own argument of the same
    name and the chain silently starts the wrong sensors. circe_gazebo's
    circe_sim.launch.py records the same trap biting its spawn.

    An OpaqueFunction runs while the include's scope is still live, so performing every
    substitution here -- and handing the handlers plain strings -- makes the chain mean
    what its caller passed, whoever the caller is.
    """
    def resolved(substitution):
        return substitution.perform(context)

    config = Path(get_package_share_directory('circe_bringup'), 'config')

    # Lives in circe_perception, not here, because circe_gazebo needs the SAME mask on
    # its bridged /scan_raw, and scan filtering is circe_perception's job anyway.
    sector_config = Path(get_package_share_directory('circe_perception'),
                         'config', 'scan_sector_filter.yaml')
    assert sector_config.is_file(), f'missing sector filter config: {sector_config}'

    lidar_config = config / 'lidar_a1.yaml'
    camera_config = config / 'realsense_d456.yaml'
    imu_config = config / 'bno055.yaml'
    d456_imu_config = config / 'imu_d456.yaml'
    for f in (lidar_config, camera_config, imu_config, d456_imu_config):
        assert f.is_file(), f'missing sensor config: {f}'

    # `use_imu` says WHETHER to publish /imu/data; `imu_source` says from WHAT. Two
    # arguments rather than one because every existing caller and HARDWARE_BRINGUP.md
    # already pass use_imu:=true, and because exactly one publisher may own
    # /imu/data -- the EKF would otherwise fuse two disagreeing attitudes with no
    # error anywhere. Selecting rather than enabling makes that unrepresentable.
    imu_on = LaunchConfiguration('use_imu')
    source = LaunchConfiguration('imu_source')
    use_lidar = resolved(LaunchConfiguration('use_lidar'))
    use_camera = resolved(LaunchConfiguration('use_camera'))
    strict_order = resolved(LaunchConfiguration('strict_order'))

    # The same substitution expressions as before, performed now; each is a
    # 'true'/'false' string from here on.
    d456_imu = resolved(AndSubstitution(imu_on, EqualsSubstitution(source, 'd456')))
    bno055_imu = resolved(AndSubstitution(imu_on, EqualsSubstitution(source, 'bno055')))
    camera_imu = resolved(AndSubstitution(use_camera, d456_imu))
    use_d456_imu = IfCondition(d456_imu)
    use_bno055 = IfCondition(bno055_imu)

    # sllidar_node's own defaults are wrong for an A1 (1000000 baud,
    # frame_id "laser_frame"), so every value comes from lidar_a1.yaml. That file
    # is keyed on /** precisely so renaming this node cannot silently drop them.
    #
    # *** RESPAWN, LIKE THE MOTOR DRIVER IN rover.launch.py. ***
    # sllidar_node EXITS when it cannot open or loses /dev/rplidar -- a loose USB
    # connector on a rover driving over rough ground, or a brownout on the shared 5 V
    # rail. Without respawn the lidar is gone for the rest of the session and every
    # consumer downstream (sector filter, pitch filter, SLAM, both costmaps) goes blind
    # at once. The respawned process is not a gate's target, so it restarts nothing
    # else in the chain. 2 s lets a re-plugged CP2102 finish enumerating.
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_a1',
        parameters=[lidar_config],
        output='screen',
        # The driver's raw 360 deg sweep is NOT /scan. It goes to /scan_raw and
        # reaches /scan only through the sector filter below.
        remappings=[('/scan', '/scan_raw')],
        condition=IfCondition(use_lidar),
        respawn=True,
        respawn_delay=2.0,
    )

    gate_lidar = _gate('lidar', 45, hint=(
        'The RPLidar driver is publishing nothing on /scan_raw. Check that '
        '/dev/rplidar exists (udev/60-rplidar.rules), that the motor is spinning, and '
        'that the baud rate in config/lidar_a1.yaml is 115200 for an A1 -- '
        'sllidar_node\'s own default is 1000000 and silently reads nothing.'),
        checks=[['--topic', use_lidar, '/scan_raw']], required=strict_order)

    # Masks the rear half of the sweep: the cable spool on the rear deck and the
    # trailing tether both stand through the scan plane and would otherwise be
    # reported as a wall a third of a metre behind the rover.
    #
    # *** THIS SITS BEFORE /scan ON PURPOSE. ***
    # Masking here means EVERY consumer gets the masked sweep, whatever it subscribes
    # to, and nothing downstream needed reconfiguring. Both current consumers sit
    # below this point: scan_pitch_filter reads /scan and republishes /scan_filtered,
    # and slam_toolbox reads /scan_filtered in turn (scan_topic in
    # mapper_params_circe.yaml). Mask further downstream instead and you would have to
    # get every one of those right individually.
    #
    # BOTH remappings are load-bearing. scan_to_scan_filter_chain's own topics are
    # `scan` in and `scan_filtered` out; without the second remap it would publish
    # onto /scan_filtered and collide head-on with circe_perception's
    # scan_pitch_filter, which already owns that topic.
    #
    # Gated on use_lidar with the driver: with no lidar there is nothing to filter,
    # and an idle chain node publishing nothing is just a second thing to explain.
    #
    # respawn: /scan has exactly one publisher and this is it. Stateless, so a restart
    # costs one scan.
    sector_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_sector_filter',
        parameters=[str(sector_config)],
        remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
        output='screen',
        condition=IfCondition(use_lidar),
        respawn=True,
        respawn_delay=1.0,
    )

    gate_scan = _gate('scan', 30, hint=(
        '/scan_raw is flowing but /scan is not, so the fault is in the sector filter, '
        'not the lidar. Check config/scan_sector_filter.yaml in circe_perception and '
        'that laser_filters is installed.'),
        checks=[['--topic', use_lidar, '/scan']], required=strict_order)

    # *** RESPAWN COVERS A CRASH, reconnect_timeout COVERS AN UNPLUG. ***
    # realsense_d456.yaml's reconnect_timeout lets a RUNNING driver wait out a USB
    # disconnect by itself, so an unplug does not need this. What does is the driver
    # dying -- librealsense aborting on a USB 3 error, or a segfault in the D4 frame
    # path -- which otherwise takes depth (terrain_node) and the raw IMU (imu_fusion ->
    # /imu/data -> the EKF's only heading reference) down for the rest of the session.
    # 5 s because initial_reset: true power-cycles the device on every start and it
    # has to re-enumerate before the new process can claim it.
    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        parameters=[camera_config],
        output='screen',
        condition=IfCondition(use_camera),
        respawn=True,
        respawn_delay=5.0,
    )

    # Depth is what terrain_node consumes; the raw IMU stream is what imu_fusion
    # consumes. Checking the IMU topic only when a d456-sourced /imu/data is actually
    # wanted keeps a deliberate depth-only configuration from waiting out the timeout.
    gate_camera = _gate('camera', 90, hint=(
        'The D456 did not produce data. It needs a USB 3 port -- on USB 2 the driver '
        'starts and then fails to open the depth stream. initial_reset: true in '
        'config/realsense_d456.yaml makes a cold start genuinely slow, so a timeout '
        'here is worth confirming with `rs-enumerate-devices` before changing '
        'anything.'), checks=[
            ['--topic', use_camera, '/camera/camera/depth/image_rect_raw'],
            ['--topic', camera_imu, '/camera/camera/imu'],
        ], required=strict_order)

    # *** THE D456's IMU IS RAW. SOMETHING HAS TO FUSE IT. ***
    # realsense2_camera publishes /camera/camera/imu with orientation (0,0,0,0)
    # and orientation_covariance[0] = -1: the part has no magnetometer and no
    # on-chip fusion, and unite_imu_method: 2 merges the gyro and accel STREAMS,
    # not their meaning. Remapping that topic onto /imu/data would not fail --
    # scan_pitch_filter and terrain_node would both extract roll 0.00, pitch 0.00
    # from the null quaternion and go on believing the robot is level forever,
    # which is precisely the ramp bug they were written to fix. The camera also
    # reports in camera_imu_optical_frame, where body yaw rate is -gy, not gz.
    #
    # imu_fusion.py rotates into base_link and runs a complementary filter, so
    # what reaches /imu/data has the same meaning the BNO055's messages had.
    #
    # Depends on use_camera being true -- it has nothing to subscribe to
    # otherwise. It also reads /odometry/wheels to gate gyro-bias learning, so
    # it wants the driver up; without it, it warns once and keeps running.
    #
    # respawn: the sole publisher of /imu/data. A restart re-learns the gyro bias
    # (about a second of standing still), which beats the EKF losing heading for good.
    imu_fusion = Node(
        package='circe_perception',
        executable='imu_fusion.py',
        name='imu_fusion',
        parameters=[str(d456_imu_config)],
        remappings=[('imu_in', '/camera/camera/imu'), ('imu_out', '/imu/data')],
        output='screen',
        condition=use_d456_imu,
        respawn=True,
        respawn_delay=1.0,
    )

    # Remap to /imu/data because that is what the EKF's imu0 input expects
    # (circe_nav/config/localization_ekf.yaml). Unlike the D456 the BNO055 runs
    # its own sensor fusion (operation_mode 0x0C) and publishes a real
    # quaternion, so it needs nothing in between.
    bno055 = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[imu_config],
        remappings=[('/imu', '/imu/data')],
        output='screen',
        condition=use_bno055,
    )

    return [
        lidar,
        gate_lidar,
        _then(gate_lidar, 'lidar', 'the sector filter', sector_filter, gate_scan),
        _then(gate_scan, 'scan', 'the camera', camera, gate_camera),
        _then(gate_camera, 'camera', 'IMU fusion', imu_fusion, bno055),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the RPLidar A1 (needs /dev/rplidar)'),
        DeclareLaunchArgument(
            # False, matching circe_bringup.launch.py until the D456's USB 3 cable is fitted.
            'use_camera', default_value='false',
            description='Start the RealSense D456 (needs a USB 3 port)'),
        # True, matching circe_bringup.launch.py -- an argument of the same name must
        # not mean different things in the two files. With imu_source:=d456 the IMU is
        # inside the camera, so there is no device to be absent; the only cost of true
        # with use_camera:=false is one idle node with nothing to subscribe to.
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Publish /imu/data. See imu_source for which device. Set false '
                        'to run deliberately without an IMU -- note that this silently '
                        'degrades the EKF heading, scan_pitch_filter and terrain_node.'),
        DeclareLaunchArgument(
            'imu_source', default_value='d456',
            choices=['d456', 'bno055'],
            description='Which device publishes /imu/data. d456 = the camera\'s own '
                        'IMU, fused into an attitude by circe_perception/imu_fusion.py '
                        '(requires use_camera:=true). bno055 = the external part, which '
                        'is not fitted and whose driver is not installed.'),
        DeclareLaunchArgument(
            'strict_order', default_value='true',
            description='Stop at the first sensor that fails to come up, instead of '
                        'starting the rest on top of it.'),

        OpaqueFunction(function=_launch_setup),
    ])
