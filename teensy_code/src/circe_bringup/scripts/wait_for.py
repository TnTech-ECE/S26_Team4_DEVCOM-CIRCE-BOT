#!/usr/bin/env python3
"""Block until named ROS graph signals are live, then exit. The launch sequencer.

    ros2 run circe_bringup wait_for.py --label base --timeout 60 \
        --tf true base_link lidar_link --topic true /odometry/wheels

Exits 0 when every enabled check passes, 1 on timeout when --required is true, and 0
on timeout when it is false. circe_bringup.launch.py chains one of these between every
layer with RegisterEventHandler(OnProcessExit(...)), so a layer only starts once the
layer beneath it has been OBSERVED working -- not merely spawned.

*** WHY "PROCESS STARTED" IS NOT THE SIGNAL WE WANT ***
launch's own OnProcessStart fires when the OS has forked the process, which on this
stack means almost nothing:

    roverrobotics_driver   spawns, then opens /dev/rover-control. If the rover is off
                           or the USB has not enumerated it spawns fine and publishes
                           no odometry at all.
    realsense2_camera      spawns, then resets and enumerates the D456 over USB 3.
                           That takes seconds on a Pi 5, and initial_reset: true in
                           realsense_d456.yaml makes it take longer still.
    ekf_node               spawns instantly and publishes odom -> base_link only once
                           /odometry/wheels arrives.
    slam_toolbox           spawns, sits in `unconfigured`, and needs a lifecycle
                           transition before it subscribes to anything.

Every one of those is a real hardware failure that OnProcessStart reports as success.
So each check here waits for the OUTPUT -- a message on a topic, an edge in the TF
tree, a node in the graph -- which is the only evidence that the thing works.

CHECK KINDS
    --topic ENABLED NAME        one message received on NAME
    --tf    ENABLED PARENT CHILD  PARENT -> CHILD resolvable in the TF tree
    --node  ENABLED NAME        NAME present in the ROS graph

ENABLED is a "true"/"false" string, not a flag, so callers can pass a
LaunchConfiguration straight through and a check for a layer that is switched off
disappears instead of hanging. That is what keeps the chain intact when, say,
use_slam:=false: the gate still runs (so the handler after it still fires) but has
nothing left to wait for and exits immediately.

*** THE TOPIC CHECK MATCHES THE PUBLISHER'S QoS, IT DOES NOT ASSUME ONE. ***
This stack mixes QoS profiles: the RealSense IMU publisher is hardcoded BEST_EFFORT
(rs_node_setup.cpp, HID_QOS), the lidar and the filters use sensor-data QoS, and Nav2's
costmap topics are TRANSIENT_LOCAL latched. A fixed RELIABLE subscriber never matches
the first group, and a VOLATILE subscriber never receives the already-published message
from the last. Both failures look identical from here -- a topic that exists and never
arrives -- so the check reads the publisher's actual profile with
get_publishers_info_by_topic() and mirrors it. A gate that reports a false negative is
worse than no gate, because it stops a bringup that would have worked.
"""

import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener


def _enabled(text):
    """Interpret a launch-substituted boolean. Anything unrecognised is False."""
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


class _Check:
    """One thing to wait for. `satisfied` latches true; `detail` explains the wait."""

    def __init__(self, description):
        self.description = description
        self.satisfied = False
        self.detail = 'not seen yet'

    def poll(self, node):
        raise NotImplementedError


class _TopicCheck(_Check):
    """Wait for an actual message, not merely for a publisher to exist.

    A publisher with no data behind it is the signature of half this stack's hardware
    faults: the lidar driver advertises /scan_raw before it has talked to the motor,
    and imu_fusion advertises /imu/data whether or not the camera it reads ever
    appears. Counting publishers would pass in both cases.
    """

    def __init__(self, topic):
        super().__init__(f'message on {topic}')
        self.topic = topic
        self._sub = None
        self._seen = False

    def poll(self, node):
        if self._seen:
            self.satisfied = True
            return

        if self._sub is None:
            try:
                infos = node.get_publishers_info_by_topic(self.topic)
            except Exception as exc:                      # invalid topic name
                self.detail = f'cannot query the graph: {exc}'
                return
            if not infos:
                self.detail = 'no publisher yet'
                return

            info = infos[0]
            try:
                msg_type = get_message(info.topic_type)
            except Exception:
                # An unresolvable type means the message package is not on this
                # machine. A live publisher is then the strongest evidence available,
                # and refusing to accept it would block a bringup over a missing
                # Python import that nothing in the chain actually needs.
                node.get_logger().warn(
                    f'{self.topic}: cannot import {info.topic_type}, '
                    'accepting the publisher itself as the signal')
                self._seen = True
                self.satisfied = True
                return

            self._sub = node.create_subscription(
                msg_type, self.topic, self._on_msg, self._mirror(info.qos_profile))
            self.detail = f'publisher up ({info.topic_type}), waiting for data'
            return

        self.detail = 'subscribed, waiting for data'

    @staticmethod
    def _mirror(publisher_qos):
        """Build a subscriber QoS compatible with this publisher.

        Reliability and durability are copied exactly, which is always compatible.
        Anything the middleware reports as UNKNOWN or SYSTEM_DEFAULT falls back to
        BEST_EFFORT/VOLATILE -- the permissive end of both axes, which matches every
        publisher except a latched one, and a latched publisher always reports its
        durability honestly.
        """
        reliability = publisher_qos.reliability
        if reliability not in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
            reliability = ReliabilityPolicy.BEST_EFFORT

        durability = publisher_qos.durability
        if durability not in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            durability = DurabilityPolicy.VOLATILE

        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=reliability, durability=durability)

    def _on_msg(self, _msg):
        self._seen = True


class _TfCheck(_Check):
    """Wait for one edge of the TF tree.

    This is the check that matters most on this robot, because the frame-ownership
    contract in circe_bringup.launch.py is enforced by exactly one publisher per edge:
    odom -> base_link proves the EKF is fusing, map -> odom proves slam_toolbox got
    past its lifecycle transitions and matched a scan.
    """

    def __init__(self, parent, child, buffer):
        super().__init__(f'transform {parent} -> {child}')
        self.parent = parent
        self.child = child
        self._buffer = buffer

    def poll(self, node):
        try:
            # Time() is the zero time, which tf2 reads as "the latest available".
            self.satisfied = self._buffer.can_transform(self.parent, self.child, Time())
        except Exception as exc:
            self.detail = f'lookup failed: {exc}'
            return
        if not self.satisfied:
            self.detail = 'edge not in the tree yet'


class _NodeCheck(_Check):
    """Wait for a node to appear in the graph.

    Weaker than the other two on purpose. It is the right check only where a node's
    output is conditional on something outside the bringup -- terrain_node, for
    instance, publishes /terrain/grid only once it has depth frames AND a subscriber,
    so waiting for the topic would be waiting on the camera's view of the world rather
    than on the node having started.
    """

    def __init__(self, name):
        super().__init__(f'node {name}')
        self.name = name.lstrip('/')

    def poll(self, node):
        self.satisfied = self.name in node.get_node_names()
        if not self.satisfied:
            self.detail = 'not in the graph yet'


def _build(args, node):
    checks = []
    needs_tf = any(_enabled(spec[0]) for spec in args.tf)
    buffer = None
    if needs_tf:
        buffer = Buffer()
        # Held on the node so the listener is not garbage collected out from under
        # the buffer; without the reference /tf goes quiet after the first poll.
        node._tf_listener = TransformListener(buffer, node, spin_thread=False)

    for enabled, topic in args.topic:
        if _enabled(enabled):
            checks.append(_TopicCheck(topic))
    for enabled, parent, child in args.tf:
        if _enabled(enabled):
            checks.append(_TfCheck(parent, child, buffer))
    for enabled, name in args.node:
        if _enabled(enabled):
            checks.append(_NodeCheck(name))
    return checks


def _report(logger, label, elapsed, timeout, checks, hint, failed):
    pending = [c for c in checks if not c.satisfied]
    done = [c for c in checks if c.satisfied]

    if not failed:
        logger.info(f'[{label}] still waiting after {elapsed:.0f}/{timeout:.0f} s: '
                    + ', '.join(f'{c.description} ({c.detail})' for c in pending))
        return

    lines = ['',
             '=' * 72,
             f' BRINGUP GATE "{label}" TIMED OUT after {timeout:.0f} s',
             '',
             ' NOT READY:']
    lines += [f'   - {c.description:<40} {c.detail}' for c in pending]
    if done:
        lines += ['', ' already up:']
        lines += [f'   - {c.description}' for c in done]
    if hint:
        lines += ['', ' ' + hint]
    lines += ['=' * 72, '']
    logger.error('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='stage',
                        help='Name of the layer this gate guards, used in every log line')
    parser.add_argument('--timeout', type=float, default=45.0,
                        help='Seconds to wait before giving up')
    parser.add_argument('--required', default='true',
                        help='"true": exit non-zero on timeout, which stops the chain. '
                             '"false": warn and exit 0, letting the next layer start '
                             'anyway.')
    parser.add_argument('--poll', type=float, default=0.25,
                        help='Seconds between graph polls')
    parser.add_argument('--report-every', type=float, default=5.0,
                        help='Seconds between "still waiting" lines')
    parser.add_argument('--hint', default='',
                        help='Diagnostic printed on timeout, next to what is missing')
    parser.add_argument('--topic', nargs=2, action='append', default=[],
                        metavar=('ENABLED', 'NAME'))
    parser.add_argument('--tf', nargs=3, action='append', default=[],
                        metavar=('ENABLED', 'PARENT', 'CHILD'))
    parser.add_argument('--node', nargs=2, action='append', default=[],
                        metavar=('ENABLED', 'NAME'))

    # launch_ros appends --ros-args ... to every Node it spawns; argparse must not see
    # them, and rclpy.init must.
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init(args=sys.argv)
    node = Node('wait_for')
    logger = node.get_logger()

    checks = _build(args, node)
    if not checks:
        logger.info(f'[{args.label}] nothing to wait for (every check disabled)')
        node.destroy_node()
        rclpy.shutdown()
        return 0

    logger.info(f'[{args.label}] waiting for: '
                + ', '.join(c.description for c in checks))

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    started = time.monotonic()
    deadline = started + args.timeout
    next_report = started + args.report_every
    ready = False
    timed_out = False

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=args.poll)
            for check in checks:
                if not check.satisfied:
                    check.poll(node)

            if all(c.satisfied for c in checks):
                ready = True
                logger.info(f'[{args.label}] ready in {time.monotonic() - started:.1f} s')
                break

            now = time.monotonic()
            if now >= deadline:
                timed_out = True
                _report(logger, args.label, now - started, args.timeout,
                        checks, args.hint, failed=True)
                break
            if now >= next_report:
                _report(logger, args.label, now - started, args.timeout,
                        checks, args.hint, failed=False)
                next_report = now + args.report_every
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if ready:
        return 0

    # *** LEAVING THE LOOP IS NOT THE SAME AS PASSING. ***
    # Ctrl-C shuts the rclpy context down, `while rclpy.ok()` goes false, and the loop
    # falls out having verified nothing. Reporting that as success would make the
    # OnProcessExit handler in the launch file start the NEXT layer -- spawning fresh
    # processes into a bringup that is already tearing down. Only an explicit `ready`
    # is success.
    if not timed_out:
        logger.warn(f'[{args.label}] interrupted before its checks passed')
        return 130

    return 1 if _enabled(args.required) else 0


if __name__ == '__main__':
    sys.exit(main())
