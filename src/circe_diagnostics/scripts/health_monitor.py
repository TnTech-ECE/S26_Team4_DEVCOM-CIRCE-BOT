#!/usr/bin/env python3
"""Runtime health checks for CIRCE, published on /diagnostics.

    ros2 launch circe_diagnostics diagnostics.launch.py

circe_bringup's gates (circe_bringup/scripts/wait_for.py) prove each layer works ONCE,
at startup. This node keeps asking the same questions for as long as the rover runs, and
answers them as diagnostic_msgs so diagnostic_aggregator can fold them into one tree
(config/analyzers.yaml) alongside the statuses other nodes already publish themselves:

    ekf_filter_node                  robot_localization, print_diagnostics: true
    lifecycle_manager_navigation     "Nav2 Health"
    lifecycle_manager_localization   "Nav2 Health", for slam_toolbox

Nothing else on this robot reports on itself, so the rest is watched from OUTSIDE, by
its output -- the same rule wait_for.py follows, for the same reason: a driver that has
lost its device usually keeps running and simply goes quiet.

CHECKS
    rate    a topic's message rate against [min_hz, max_hz]     config: rate_checks
    SLAM    how often map -> odom is broadcast on /tf           config: slam.*
    Host    CPU temperature, under-voltage/throttling, CPU load, memory, disk

*** RATE CHECKS NEVER DESERIALISE A MESSAGE. ***
Their subscriptions use raw=True, so an arrival costs one callback and a counter
increment, not a conversion into Python objects. That matters on the Pi 5: the camera
IMU alone arrives at ~200 Hz. It is also why the camera is watched through
depth/camera_info -- published once per depth frame -- and never through the image.

*** RATES ARE MEASURED ON THIS NODE'S CLOCK, SO use_sim_time MEANS WHAT IT SAYS. ***
Under Gazebo every sensor publishes per SIM second, and that is the rate the EKF and
Nav2 consume. diagnostic_updater's own FrequencyStatus cannot measure it -- see _Rate.

*** THE SUBSCRIBER QoS MATCHES EVERY PUBLISHER. ***
BEST_EFFORT + VOLATILE is the permissive end of both QoS axes, so one profile connects
to the RealSense's BEST_EFFORT IMU, the lidar's sensor-data QoS and a RELIABLE publisher
alike. wait_for.py mirrors each publisher's profile because it must receive the latched
message on TRANSIENT_LOCAL topics; a rate check never needs that, so it takes the simple
route.

*** STATUS NAMES ARE HOW THE AGGREGATOR FILES THEM. ***
diagnostic_updater prefixes every name with this node's name, so a check labelled
"Lidar raw scan" arrives as "circe_health_monitor: Lidar raw scan", and analyzers.yaml
matches on that. Two rules follow:
  - Labels start with their group word: Lidar, Camera, IMU, Odometry, SLAM, Host. Rename
    one and it lands in the aggregator's "Other" bucket rather than vanishing -- that is
    the clue.
  - Labels contain no '/'. rqt_robot_monitor builds its tree by splitting names on '/',
    so a topic name in a label becomes spurious nesting. Topics go in the key/values.
"""

import os
import shutil
import subprocess
from collections import deque
from pathlib import Path

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR

# Permissive on both axes; see the module docstring. Depth 10 rather than 1 so a burst
# that arrives while the executor is busy is counted rather than dropped, which would
# otherwise read as a rate dip that never happened.
_ANY_PUBLISHER = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)

# /tf carries every dynamic edge in the tree -- the EKF's odom -> base_link, the wheel
# joints, slam_toolbox's map -> odom -- so it needs a deeper queue than a single topic.
_TF_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

# vcgencmd get_throttled bits. The low nibble is "right now", the same bits shifted by 16
# are "has happened since boot".
_THROTTLE_BITS = (
    (0x1, 'under-voltage', ERROR),
    (0x2, 'ARM frequency capped', WARN),
    (0x4, 'throttled', WARN),
    (0x8, 'soft temperature limit', WARN),
)

# Raspberry Pi kernels expose the same flags vcgencmd reads as a sysfs attribute on the
# firmware node. The node's name differs between boards (soc/soc:firmware on a Pi 4,
# soc@107c000000/soc@107c000000:firmware on a Pi 5), hence the patterns. Deliberately
# not a recursive glob: /sys is full of symlink cycles.
_THROTTLE_SYSFS = ('/sys/devices/platform/*/*firmware/get_throttled',
                   '/sys/devices/platform/*firmware/get_throttled')


def _level(value, warn, error):
    if value >= error:
        return ERROR
    if value >= warn:
        return WARN
    return OK


def _fixed(level, message):
    """A task that always reports the same thing -- used to put a config error IN the
    tree, where someone will see it, rather than only in a log nobody is reading."""
    def run(stat):
        stat.summary(level, message)
        return stat
    return run


class _Rate(DiagnosticTask):
    """Message rate over the last `window` update periods, timed on the NODE's clock.

    *** NOT diagnostic_updater.FrequencyStatus, AND THE CLOCK IS WHY. ***
    Jazzy's FrequencyStatus times its window with a Clock(ROS_TIME) it constructs for
    itself in clear() and run() (diagnostic_updater/_update_functions.py). That clock
    belongs to no node, never receives /clock, and so reads WALL time even under
    use_sim_time:=true -- while the Updater that calls run() fires on the node's clock,
    i.e. SIM time. Under Gazebo every rate therefore came out as nominal x real-time
    factor. Measured in circe_sim_nav at RTF 1.00: 50 depth frames over a 5.0024 s
    "window" = 9.995 Hz, WARN against a 10 Hz floor, with not one frame lost. On a host
    that runs the sim at RTF 0.6 every sensor reads 40% slow. Timing the window on the
    node's own clock is what makes use_sim_time reach the measurement.

    It also names what went quiet. The stock message for a silent source is "No events
    recorded.", which in rqt_robot_monitor gives no hint of which topic to look at.

    min_hz and max_hz are the real limits -- no hidden tolerance is taken off them.
    Callbacks and run() share this node's single-threaded executor, so no lock.
    """

    def __init__(self, label, source, min_hz, max_hz, window, clock):
        super().__init__(label)
        self.source = source
        self.min_hz = min_hz
        self.max_hz = max_hz
        self._clock = clock
        self._count = 0
        # (time, count) at each of the last `window` updates, oldest first. Empty until
        # the first update: the construction time is no baseline under use_sim_time,
        # where the clock reads zero until the first /clock arrives.
        self._history = deque(maxlen=max(int(window), 1) + 1)

    def tick(self):
        self._count += 1

    def run(self, stat):
        now = self._clock.now()
        if self._history and now < self._history[-1][0]:
            # Time went backwards: the simulation was reset. A window spanning the jump
            # measures nothing, so start a new one.
            self._history.clear()
        if now.nanoseconds > 0:
            self._history.append((now, self._count))

        stat.add('Source', self.source)
        stat.add('Events since startup', str(self._count))
        if len(self._history) < 2:
            if self._count:
                stat.summary(OK, 'receiving, measuring rate')
            else:
                stat.summary(ERROR, f'no messages on {self.source}')
            return stat

        start, start_count = self._history[0]
        window = (now - start).nanoseconds * 1e-9
        events = self._count - start_count
        hz = events / window

        if events == 0:
            stat.summary(ERROR, f'no messages on {self.source}')
        elif hz < self.min_hz:
            stat.summary(WARN, f'{hz:.1f} Hz, below {self.min_hz:g}')
        elif self.max_hz > 0.0 and hz > self.max_hz:
            stat.summary(WARN, f'{hz:.1f} Hz, above {self.max_hz:g}')
        else:
            stat.summary(OK, f'{hz:.1f} Hz')
        stat.add('Actual frequency (Hz)', f'{hz:.3f}')
        stat.add('Minimum acceptable frequency (Hz)', f'{self.min_hz:g}')
        if self.max_hz > 0.0:
            stat.add('Maximum acceptable frequency (Hz)', f'{self.max_hz:g}')
        stat.add('Events in window', str(events))
        stat.add('Duration of window (s)', f'{window:.3f}')
        return stat


class HealthMonitor(Node):

    def __init__(self):
        super().__init__('circe_health_monitor')
        self._updater = Updater(self)
        self._updater.setHardwareID('circe')

        window = self.declare_parameter('window_size', 5).value
        self._add_rate_checks(window)
        self._add_slam_check(window)
        self._add_host_checks()

        if not self._updater.tasks:
            self.get_logger().warn('every check is disabled; publishing nothing useful')

    def _num(self, name, default):
        """A numeric parameter that accepts 5 as happily as 5.0.

        A plain declare_parameter(name, 5.0) rejects an integer from the YAML with a
        type error at startup, which is a poor reward for writing `min_hz: 5`.
        """
        return float(self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value)

    def _nums(self, name, default):
        """A list of numbers, with the same tolerance as _num -- and a bare number is
        accepted as a one-element list."""
        value = self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value
        if isinstance(value, (int, float)):
            return [float(value)]
        return [float(v) for v in value]

    # ---- rate checks ----------------------------------------------------------------

    def _add_rate_checks(self, window):
        names = self.declare_parameter('rate_checks', Parameter.Type.STRING_ARRAY).value
        for name in names or []:
            label = self.declare_parameter(f'{name}.label', name).value
            enabled = self.declare_parameter(f'{name}.enabled', True).value
            topic = self.declare_parameter(f'{name}.topic', '').value
            type_name = self.declare_parameter(f'{name}.type', '').value
            min_hz = self._num(f'{name}.min_hz', 1.0)
            max_hz = self._num(f'{name}.max_hz', 0.0)

            if not enabled:
                self.get_logger().info(f'{label}: disabled')
                continue

            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError):
                problem = (f'misconfigured: {name}.type "{type_name}" is not a message '
                           'type (expected e.g. sensor_msgs/msg/LaserScan)')
                self.get_logger().error(f'{label}: {problem}')
                self._updater.add(label, _fixed(ERROR, problem))
                continue
            if not topic:
                problem = f'misconfigured: {name}.topic is not set'
                self.get_logger().error(f'{label}: {problem}')
                self._updater.add(label, _fixed(ERROR, problem))
                continue

            check = _Rate(label, topic, min_hz, max_hz, window, self.get_clock())
            self.create_subscription(msg_type, topic, lambda _raw, c=check: c.tick(),
                                     _ANY_PUBLISHER, raw=True)
            self._updater.add(check)

    # ---- SLAM -----------------------------------------------------------------------

    def _add_slam_check(self, window):
        """Count map -> odom broadcasts as they ARRIVE.

        lifecycle_manager_localization already reports on slam_toolbox, but it runs with
        bond_timeout 0.0 (circe_nav/launch/slam.launch.py), so it cannot tell when
        slam_toolbox dies -- it goes on reporting the state it last set. The transform
        is the one output that stops when SLAM stops.

        Counting arrivals rather than reading the transform's stamp out of a TF buffer
        keeps this independent of how slam_toolbox chooses to stamp it.
        """
        enabled = self.declare_parameter('slam.enabled', True).value
        parent = self.declare_parameter('slam.parent_frame', 'map').value
        child = self.declare_parameter('slam.child_frame', 'odom').value
        min_hz = self._num('slam.min_hz', 20.0)
        max_hz = self._num('slam.max_hz', 0.0)
        if not enabled:
            self.get_logger().info('SLAM: disabled')
            return

        check = _Rate(f'SLAM {parent} to {child}', f'{parent} -> {child} on /tf',
                      min_hz, max_hz, window, self.get_clock())

        def on_tf(msg):
            for tf in msg.transforms:
                if (tf.child_frame_id.lstrip('/') == child
                        and tf.header.frame_id.lstrip('/') == parent):
                    check.tick()

        self.create_subscription(TFMessage, '/tf', on_tf, _TF_QOS)
        self._updater.add(check)

    # ---- host -----------------------------------------------------------------------

    def _add_host_checks(self):
        if not self.declare_parameter('host.enabled', True).value:
            self.get_logger().info('Host: disabled')
            return

        zone_types = list(self.declare_parameter(
            'host.thermal_zone_types', ['cpu-thermal', 'x86_pkg_temp']).value)
        # One limit per zone type, in the same order -- a Pi 5's SoC and a laptop's CPU
        # package do not share a safe temperature. A single number applies to all.
        temp_warn = self._nums('host.temp_warn_c', [70.0, 90.0])
        temp_error = self._nums('host.temp_error_c', [80.0, 100.0])
        self._cpu_warn = self._num('host.cpu_warn_percent', 90.0)
        self._mem_warn = self._num('host.memory_warn_percent', 85.0)
        self._mem_error = self._num('host.memory_error_percent', 95.0)
        self._disk_path = self.declare_parameter('host.disk_path', '/').value
        self._disk_warn = self._num('host.disk_warn_percent', 85.0)
        self._disk_error = self._num('host.disk_error_percent', 95.0)

        # The Pi 5's SoC sensor is "cpu-thermal". Elsewhere (the dev laptop) fall back
        # to a named package sensor, and failing that to the hottest zone there is.
        self._zones = {}
        for zone in sorted(Path('/sys/class/thermal').glob('thermal_zone*')):
            try:
                self._zones.setdefault((zone / 'type').read_text().strip(), zone / 'temp')
            except OSError:
                continue
        # (type, path, warn, error) for the first configured type present, else None.
        self._temp_zone = next(
            ((t, self._zones[t],
              temp_warn[i if len(temp_warn) > 1 else 0],
              temp_error[i if len(temp_error) > 1 else 0])
             for i, t in enumerate(zone_types) if t in self._zones), None)

        self._throttle_source = self._find_throttle_source()

        # The first cpu_percent() call has nothing to compare against and returns 0.0.
        # Priming it here means the first published value is real.
        psutil.cpu_percent(None)

        if {len(temp_warn), len(temp_error)} - {1, len(zone_types)}:
            problem = ('misconfigured: host.temp_warn_c and host.temp_error_c need one '
                       'value, or one per entry in host.thermal_zone_types')
            self.get_logger().error(f'Host CPU temperature: {problem}')
            self._updater.add('Host CPU temperature', _fixed(ERROR, problem))
        else:
            self._updater.add('Host CPU temperature', self._host_temperature)
        self._updater.add('Host power', self._host_power)
        self._updater.add('Host CPU load', self._host_cpu)
        self._updater.add('Host memory', self._host_memory)
        self._updater.add('Host disk', self._host_disk)

    @staticmethod
    def _find_throttle_source():
        for pattern in _THROTTLE_SYSFS:
            for path in sorted(Path('/').glob(pattern.lstrip('/'))):
                return ('sysfs', str(path))
        exe = shutil.which('vcgencmd')
        if exe:
            return ('vcgencmd', exe)
        return None

    def _read_throttled(self):
        kind, where = self._throttle_source
        if kind == 'sysfs':
            return int(Path(where).read_text().strip(), 16)
        out = subprocess.run([where, 'get_throttled'], capture_output=True, text=True,
                             timeout=2.0, check=False)
        if out.returncode != 0 or '=' not in out.stdout:
            # Typically "VCHI initialization failed": the user running ROS is not in
            # the `video` group, so /dev/vcio is unreadable.
            raise OSError((out.stderr or out.stdout).strip() or f'exit {out.returncode}')
        return int(out.stdout.strip().split('=', 1)[1], 16)

    def _host_temperature(self, stat):
        """CPU temperature, held to the limits for THIS kind of computer.

        Off the Pi it degrades instead of alarming. A VM or a container has no thermal
        zones at all, which is not a fault, so it reads OK. A host with zones but none
        of the configured types (an AMD laptop, say) shows its hottest zone for
        information only: there is no honest limit for an unknown sensor, and some of
        them -- wifi cards, ACPI placeholders -- report numbers that mean nothing.
        """
        if not self._zones:
            stat.summary(OK, 'no thermal sensor on this host (VM or container?)')
            return stat

        if self._temp_zone:
            kind, path, warn, error = self._temp_zone
            try:
                temp = int(path.read_text()) / 1000.0
            except (OSError, ValueError) as exc:
                stat.summary(WARN, f'cannot read {kind}: {exc}')
                return stat
            stat.summary(_level(temp, warn, error), f'{temp:.1f} C')
            stat.add('Temperature (C)', f'{temp:.1f}')
            stat.add('Thermal zone', kind)
            stat.add('Warn / error at (C)', f'{warn:.0f} / {error:.0f}')
            return stat

        readings = {}
        for kind, path in self._zones.items():
            try:
                readings[kind] = int(path.read_text()) / 1000.0
            except (OSError, ValueError):
                continue
        if not readings:
            stat.summary(OK, 'no readable thermal zone on this host')
            return stat
        kind, temp = max(readings.items(), key=lambda kv: kv[1])
        stat.summary(OK, f'{temp:.1f} C ({kind}, no limits known for it)')
        stat.add('Temperature (C)', f'{temp:.1f}')
        stat.add('Thermal zone', kind)
        stat.add('Warn / error at (C)', 'none: not in host.thermal_zone_types')
        return stat

    def _host_power(self, stat):
        """Under-voltage and throttling, as the Pi's firmware sees them.

        Only the "right now" bits set the level. The "since boot" bits are shown but
        never raise it: one brown-out at power-on would otherwise hold the whole rover
        at WARN until the next reboot, and a status that is always yellow stops being
        read.
        """
        if self._throttle_source is None:
            stat.summary(OK, 'throttle flags unavailable on this host (not a Raspberry Pi)')
            return stat
        try:
            flags = self._read_throttled()
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            stat.summary(WARN, f'cannot read throttle flags via {self._throttle_source[0]}: '
                               f'{exc}')
            return stat

        now = [(name, lvl) for bit, name, lvl in _THROTTLE_BITS if flags & bit]
        past = [name for bit, name, _ in _THROTTLE_BITS if flags & (bit << 16)]
        if now:
            stat.summary(max(lvl for _, lvl in now), ', '.join(name for name, _ in now))
        elif past:
            stat.summary(OK, 'OK now; since boot: ' + ', '.join(past))
        else:
            stat.summary(OK, 'OK')
        stat.add('Flags', f'0x{flags:x}')
        stat.add('Since boot', ', '.join(past) or 'none')
        stat.add('Source', self._throttle_source[0])
        return stat

    def _host_cpu(self, stat):
        percent = psutil.cpu_percent(None)
        load = os.getloadavg()
        stat.summary(WARN if percent >= self._cpu_warn else OK, f'{percent:.0f}% CPU')
        stat.add('CPU usage (%)', f'{percent:.1f}')
        stat.add('Load average 1/5/15 min', ' / '.join(f'{v:.2f}' for v in load))
        stat.add('Cores', str(psutil.cpu_count()))
        return stat

    def _host_memory(self, stat):
        mem = psutil.virtual_memory()
        stat.summary(_level(mem.percent, self._mem_warn, self._mem_error),
                     f'{mem.percent:.0f}% used')
        stat.add('Used (%)', f'{mem.percent:.1f}')
        stat.add('Available (MiB)', f'{mem.available / 2**20:.0f}')
        stat.add('Total (MiB)', f'{mem.total / 2**20:.0f}')
        return stat

    def _host_disk(self, stat):
        try:
            disk = psutil.disk_usage(self._disk_path)
        except OSError as exc:
            stat.summary(WARN, f'cannot stat {self._disk_path}: {exc}')
            return stat
        stat.summary(_level(disk.percent, self._disk_warn, self._disk_error),
                     f'{disk.percent:.0f}% used')
        stat.add('Path', self._disk_path)
        stat.add('Used (%)', f'{disk.percent:.1f}')
        stat.add('Free (GiB)', f'{disk.free / 2**30:.1f}')
        return stat


def main():
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGTERM from `ros2 launch` shutting down arrives as the latter.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
