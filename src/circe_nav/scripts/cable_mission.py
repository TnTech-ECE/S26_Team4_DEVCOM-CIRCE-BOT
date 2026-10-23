#!/usr/bin/env python3
"""Long-range mission driver: take one far goal, drive it as a chain of short legs.

    ros2 run circe_nav cable_mission.py
    # then publish one goal, e.g. from RViz's 2D Goal Pose tool retargeted to
    # /mission_goal, or:
    ros2 topic pub --once /mission_goal geometry_msgs/msg/PoseStamped \\
      '{header: {frame_id: odom}, pose: {position: {x: 100.0, y: 0.0}}}'

    # or scripted, no topic:
    ros2 run circe_nav cable_mission.py --ros-args -p auto_start:=true \\
      -p target_x:=100.0 -p target_y:=0.0

*** WHY: A 100 m GOAL IS NOT PLANNABLE, AND SCALING THE GRID IS THE WRONG FIX. ***
The global costmap is a rolling window (60 x 60 m in nav2_params.yaml), so it reaches
+/-30 m. A goal outside it is refused before the planner searches at all:

    planner_server: GridBased plugin failed to plan from (0.00, 0.00) to
    (32.00, -14.00): "Goal Coordinates of(32.000000, -14.000000) was outside bounds"

Measured: 34.93 m refused, 19.7 m fine. That is a costmap limit, not a Nav2 limit --
a 200 x 200 m window at 0.20 m is only 1.0 MB/layer and would reach 100 m. The reason
not to is that the A1 lidar sees 12 m, so a 100 m plan is 88% invention: a straight
line through unobserved ground, thrown away and recomputed every second. Chunking
plans only as far as there is evidence, which is also cheaper.

*** THE TWO FRAMES ARE THE DESIGN, NOT AN IMPLEMENTATION DETAIL. ***
Outbound legs are stamped in `odom`; the return legs in `map`:

  odom  robot->goal comes from wheels + IMU via the EKF. A map->odom correction moves
        robot and goal together, so the geometry is untouched: SLAM maps but does not
        steer. Right for the outbound leg, where terrain is new, the scan matcher is
        weakest, and there is no surveyed coordinate the goal could be wrong against.
        An incoming goal in any other frame is converted to `odom` ONCE on arrival,
        which deliberately freezes it against later SLAM corrections.

  map   anchored where SLAM believes the rover was when the mission started. Right for
        the way home, where the rover re-observes terrain it mapped on the way out --
        SLAM's best case -- and where arriving accurately actually matters.

Measured, 74.6 m out-and-back on circe_outdoor.sdf, 6 legs, 0 aborts, 0 recoveries:
    EKF error at the turnaround   0.61 m over 36.5 m driven  (1.7%)
    error on arriving home        0.02 m

*** REQUIRES SmacPlanner2D. *** With NavFn the same mission finished 1 leg of 4 (42
failures of "Failed to create a plan from potential"). See planner_server in
nav2_params.yaml.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

STATUS = {2: 'CANCELED', 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def split(x0, y0, x1, y1, leg_max):
    """Whole legs of at most leg_max metres from (x0,y0) to (x1,y1)."""
    d = math.hypot(x1 - x0, y1 - y0)
    n = max(1, math.ceil(d / leg_max))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(1, n + 1)]


class CableMission(Node):

    def __init__(self):
        super().__init__('cable_mission')
        self.declare_parameter('goal_topic', '/mission_goal')
        # Must stay well under the global costmap's half-width (30 m), with room for
        # the rover being off-course and for the planner routing around obstacles
        # rather than straight at the goal.
        self.declare_parameter('leg_max', 20.0)
        # *** THIS IS WHAT KEEPS THE ROVER MOVING. ***
        # The next leg is issued while the current one is still this far away, and
        # NavigateToPose preempts. Without it the rover must satisfy the goal checker
        # (xy_goal_tolerance 0.5 m) and then wait for this node's next tick -- up to a
        # second of standing still every leg. Set 0.0 to require full arrival.
        self.declare_parameter('handoff_distance', 3.0)
        self.declare_parameter('return_home', True)
        self.declare_parameter('outbound_frame', 'odom')
        self.declare_parameter('home_frame', 'map')
        # Safe default for cable laying: pressing past a leg you could not reach means
        # paying out cable along a route nobody chose.
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)

        g = lambda n: self.get_parameter(n).value        # noqa: E731
        self.leg_max = float(g('leg_max'))
        self.handoff = float(g('handoff_distance'))
        self.out_frame = g('outbound_frame')
        self.home_frame = g('home_frame')
        self.return_home = bool(g('return_home'))
        self.stop_on_failure = bool(g('stop_on_failure'))

        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_topic = str(g('goal_topic'))
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 1)

        self.queue = []
        self.done = []
        self.active = None        # (x, y, frame, label)
        self.goal_handle = None
        self.seq = 0              # goal generation, so preempted results are ignored
        self.phase = 'idle'
        self.home_pose = None
        self.failed = False
        self.stop = False
        self.exit_code = 0

        # *** WHETHER THIS PROCESS OUTLIVES ITS MISSION. ***
        # auto_start:=true is the one-shot CLI form documented at the top of this file:
        # the process was started to run ONE named mission, so it prints the report and
        # exits with a code the caller can test. Launched with use_mission:=true it is
        # the opposite -- an operator tool that navigation.launch.py starts alongside
        # Nav2 and that must still be there for the NEXT goal.
        #
        # Without this distinction the node called _finish -> stop = True -> process
        # exit after the first mission, in both modes. The second /mission_goal then went
        # nowhere at all: no subscriber, so no error, no warning, and no motion -- the
        # rover simply ignored the operator. Measured: mission 1 finished 4 of 4 legs and
        # logged "process has finished cleanly", after which a goal published to
        # (-14, 8) produced no response of any kind.
        #
        # It also made the 'finished' branch of _on_goal below unreachable, which is the
        # tell that re-arming was always the intent for the launched case.
        self.one_shot = bool(g('auto_start'))
        self.create_timer(0.25, self._tick)

        self.get_logger().info(
            'cable_mission ready: publish one far goal to %s. Legs <= %.1f m, '
            'handoff at %.1f m, outbound in `%s`, home in `%s`.'
            % (g('goal_topic'), self.leg_max, self.handoff,
               self.out_frame, self.home_frame))
        if bool(g('auto_start')):
            self._start(float(g('target_x')), float(g('target_y')), self.out_frame)

    # ---- geometry ------------------------------------------------------------

    def _here(self, frame):
        try:
            t = self.buf.lookup_transform(frame, 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def _to_frame(self, msg, frame):
        """Convert an incoming goal into `frame`, once, at arrival."""
        if msg.header.frame_id in ('', frame):
            return (msg.pose.position.x, msg.pose.position.y)
        try:
            t = self.buf.lookup_transform(
                frame, msg.header.frame_id, rclpy.time.Time()).transform
            q = t.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            x, y = msg.pose.position.x, msg.pose.position.y
            return (t.translation.x + x * math.cos(yaw) - y * math.sin(yaw),
                    t.translation.y + x * math.sin(yaw) + y * math.cos(yaw))
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().error(
                'cannot convert goal from `%s` to `%s`: %s'
                % (msg.header.frame_id, frame, exc))
            return None

    # ---- mission -------------------------------------------------------------

    def _on_goal(self, msg):
        if self.phase not in ('idle', 'finished'):
            self.get_logger().warn('mission already running; ignoring new goal')
            return
        xy = self._to_frame(msg, self.out_frame)
        if xy is None:
            return
        if msg.header.frame_id not in ('', self.out_frame):
            self.get_logger().info(
                'goal arrived in `%s`, frozen into `%s` at (%.1f, %.1f)'
                % (msg.header.frame_id, self.out_frame, xy[0], xy[1]))
        self._start(xy[0], xy[1], self.out_frame)

    def _start(self, tx, ty, frame):
        here = self._here(frame)
        if here is None:
            self.get_logger().error('no transform %s -> base_link yet' % frame)
            return
        # Home is where the mission STARTS, not the map origin -- the rover may have
        # been driven before this mission began.
        self.home_pose = self._here(self.home_frame)
        if self.home_pose is None and self.return_home:
            self.get_logger().error(
                'no transform %s -> base_link; cannot record home' % self.home_frame)
            return
        pts = split(here[0], here[1], tx, ty, self.leg_max)
        self.queue = [(x, y, frame, 'OUT-%d' % (i + 1)) for i, (x, y) in enumerate(pts)]
        self.phase = 'outbound'
        self.done = []
        self.failed = False
        self.get_logger().info(
            'outbound %.1f m from (%.1f, %.1f) to (%.1f, %.1f) in `%s`: %d legs'
            % (math.hypot(tx - here[0], ty - here[1]), here[0], here[1],
               tx, ty, frame, len(pts)))
        if self.return_home:
            self.get_logger().info(
                'home recorded at %s (%.2f, %.2f)'
                % (self.home_frame, self.home_pose[0], self.home_pose[1]))
        self._send()

    def _plan_home(self):
        here = self._here(self.home_frame)
        if here is None:
            self.get_logger().error('no transform %s -> base_link' % self.home_frame)
            return False
        hx, hy = self.home_pose
        pts = split(here[0], here[1], hx, hy, self.leg_max)
        self.queue = [(x, y, self.home_frame, 'HOME-%d' % (i + 1))
                      for i, (x, y) in enumerate(pts)]
        self.phase = 'home'
        self.get_logger().info(
            'return from %s (%.1f, %.1f) to (%.1f, %.1f): %d legs'
            % (self.home_frame, here[0], here[1], hx, hy, len(pts)))
        return True

    def _send(self):
        x, y, frame, label = self.queue.pop(0)
        self.active = (x, y, frame, label)
        self.seq += 1
        seq = self.seq
        if not self.ac.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('navigate_to_pose not available')
            self._finish(1)
            return
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = frame
        # Zero stamp = latest available transform. Correct for a goal that is a fixed
        # point in its frame rather than a timed observation.
        p.header.stamp = rclpy.time.Time().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.0
        goal.pose = p
        self.get_logger().info('%s -> (%.1f, %.1f) in `%s`' % (label, x, y, frame))
        self.ac.send_goal_async(goal).add_done_callback(
            lambda f, s=seq: self._accepted(f, s))

    def _accepted(self, fut, seq):
        if seq != self.seq:
            return
        try:
            h = fut.result()
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().error('send failed: %s' % exc)
            self.failed = True
            self.active = None
            return
        if not h.accepted:
            self.get_logger().error('%s REJECTED' % self.active[3])
            self.done.append((self.active[3], 'REJECTED'))
            self.failed = True
            self.active = None
            return
        self.goal_handle = h
        h.get_result_async().add_done_callback(lambda f, s=seq: self._result(f, s))

    def _result(self, fut, seq):
        # A leg we deliberately preempted reports CANCELED/ABORTED; that is not a
        # failure, and its seq is stale, so drop it.
        if seq != self.seq:
            return
        try:
            st = STATUS.get(fut.result().status, 'status %d' % fut.result().status)
        except Exception as exc:                        # noqa: BLE001
            st = 'NO RESULT (%s)' % exc
        label = self.active[3] if self.active else '?'
        self.done.append((label, st))
        try:
            log = self.get_logger().info if st == 'SUCCEEDED' else self.get_logger().error
            log('%s -> %s' % (label, st))
        except Exception:                               # noqa: BLE001
            print('%s -> %s' % (label, st), flush=True)
        if st != 'SUCCEEDED':
            self.failed = True
        self.active = None

    # ---- driving loop --------------------------------------------------------

    def _tick(self):
        if self.stop or self.phase in ('idle', 'finished'):
            return
        if self.failed and self.stop_on_failure:
            self.get_logger().error('stopping: a leg failed and stop_on_failure is set')
            self._finish(1)
            return

        # Hand off early so the rover never stops between legs.
        if self.active is not None and self.queue and self.handoff > 0.0:
            x, y, frame, label = self.active
            here = self._here(frame)
            if here is not None and math.hypot(here[0] - x, here[1] - y) <= self.handoff:
                self.get_logger().info(
                    '%s within %.1f m; handing off early' % (label, self.handoff))
                self.done.append((label, 'HANDED OFF'))
                self._send()
            return

        if self.active is not None:
            return
        if self.queue:
            self._send()
            return
        if self.phase == 'outbound':
            if not self.return_home:
                self._finish(0)
                return
            if not self._plan_home():
                self._finish(1)
                return
            self._send()
            return
        self._finish(0)

    def _finish(self, code):
        ok = sum(1 for _, st in self.done if st in ('SUCCEEDED', 'HANDED OFF'))
        print('\n===== CABLE MISSION =====', flush=True)
        for lbl, st in self.done:
            print('  %-8s %s' % (lbl, st), flush=True)
        print('  %d of %d legs ok' % (ok, len(self.done)), flush=True)
        here = self._here(self.home_frame)
        if here is not None and self.home_pose is not None and self.return_home:
            print('  final %s (%.2f, %.2f), home was (%.2f, %.2f) -> %.2f m'
                  % (self.home_frame, here[0], here[1],
                     self.home_pose[0], self.home_pose[1],
                     math.hypot(here[0] - self.home_pose[0],
                                here[1] - self.home_pose[1])), flush=True)
        self.phase = 'finished'
        self.exit_code = code

        # Exit only in one-shot mode, or when the mission actually failed -- a failure
        # should still surface as a non-zero exit so a supervisor sees it. On a clean
        # finish in launched mode, fall back to waiting for the next goal: _on_goal
        # accepts one in phase 'finished', and the rover keeps its map, its Nav2
        # servers and its place in the world instead of needing the whole navigation
        # stack restarted between deployments.
        if self.one_shot or code != 0:
            self.stop = True
        else:
            self.get_logger().info(
                'mission complete; still listening on %s for the next goal'
                % self.goal_topic)


def main():
    rclpy.init()
    node = CableMission()
    code = 0
    try:
        while rclpy.ok() and not node.stop:
            rclpy.spin_once(node, timeout_sec=0.2)
        code = node.exit_code
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
