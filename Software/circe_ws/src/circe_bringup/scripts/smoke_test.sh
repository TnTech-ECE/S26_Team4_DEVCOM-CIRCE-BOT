#!/usr/bin/env bash
# CIRCE bring-up smoke test.
#
# Launches the stack under a set of argument combinations -- simulation and hardware --
# and for each one checks, in this order:
#
#   1. READY    the launch prints its ready banner, i.e. every wait_for.py gate passed in
#               sequence. A gate that times out fails the scenario at once, naming it.
#   2. NODES    the expected nodes are up, the switched-off layers' nodes are NOT, and no
#               node name is duplicated (the signature of a double launch).
#   3. TF       tf2_tools view_frames shows ONE connected tree with the expected root, the
#               key transforms resolve, and no edge has two publishers.
#   4. CRASHES  no process died during the run (respawned or not -- a crash is a finding).
#   5. GREEN    /diagnostics_toplevel_state holds OK for N consecutive samples.
#   6. NAV      (sim_full; hardware only on request) one real NavigateToPose goal must
#               SUCCEED and the rover must end within tolerance of it.
#
# *** WHY NAV EXISTS: EVERYTHING ELSE PASSED WHILE THE ROVER COULD NOT DRIVE. ***
# An earlier version of this test reported sim_full all green -- gates, TF, costmaps
# published, diagnostics OK -- while every goal aborted after 0.4 s: terrain_node never
# published /terrain/grid, the local_costmap's terrain_layer never went "current", and
# controller_server refused to drive ("Costmap timed out waiting for update", error
# code 107). A published costmap is not a usable one. Only asking Nav2 to drive proves it.
#
# Usage, from a sourced workspace (source install/setup.bash):
#
#   smoke_test.sh                     sim scenarios + bench_dry + every hardware scenario
#                                     whose devices are plugged in
#   smoke_test.sh sim | hw | all      one group ("all" still skips absent hardware)
#   smoke_test.sh sim_full bench_dry  named scenarios, in the order given
#   smoke_test.sh --list              what each scenario launches
#
# Also installed: `ros2 run circe_bringup smoke_test.sh ...`.
#
# *** EVERY RUN IS ISOLATED, AND CLEANED UP BY MARKER, NOT BY NAME. ***
# Each scenario gets ROS_DOMAIN_ID=$CIRCE_SMOKE_DOMAIN_ID (default 87), localhost-only
# discovery, and its own GZ_PARTITION, so it cannot talk to the rover's network or to a
# Gazebo you already have open. Every process it starts inherits CIRCE_SMOKE_RUN=<id>,
# and teardown kills whatever still carries that marker -- including the children
# `ros2 launch` sometimes fails to reap, which is exactly how a stale Gazebo server
# survives to swallow the next spawn (see circe_gazebo/scripts/stop_sim.sh).
#
# *** JOB CONTROL IS ON (set -m), AND IT HAS TO BE. ***
# A non-interactive shell starts background jobs with SIGINT set to SIG_IGN, and the
# setting survives exec. `ros2 launch` then ignores the Ctrl-C this script sends it, and
# launch's own shutdown -- which begins by SIGINTing its children -- is ignored by all of
# them too. Found the hard way while writing this. With job control each launch gets its
# own process group with default signal dispositions, and the whole group can be
# signalled at once.
#
# Environment knobs:
#   CIRCE_SMOKE_DOMAIN_ID      ROS_DOMAIN_ID for the runs                  (87)
#   CIRCE_SMOKE_LOG_DIR        where logs, frames.gv/pdf and summaries go
#                              (~/.ros/circe_smoke/<timestamp>)
#   CIRCE_SMOKE_READY_TIMEOUT  seconds to wait for the ready banner        (480)
#   CIRCE_SMOKE_GREEN_TIMEOUT  seconds to wait for sustained OK            (90)
#   CIRCE_SMOKE_GREEN_SAMPLES  consecutive OK toplevel samples required    (5)
#   CIRCE_SMOKE_DISCOVERY      discovery range: LOCALHOST or SUBNET        (LOCALHOST)
#   CIRCE_SMOKE_SIM_GOAL       "x y yaw" in map for sim_full's NAV check   ("0.0 2.0 1.5708")
#                              -- clear ground in the default circe_indoor.sdf; set ""
#                              to skip, or another point if you change the world
#   CIRCE_SMOKE_HW_GOAL        the same for hw_full and hw_nocam. EMPTY BY DEFAULT BECAUSE IT DRIVES
#                              THE REAL ROVER. Set it only with the robot on clear ground
#                              and someone at the e-stop.

set -u
set -m

DOMAIN_ID=${CIRCE_SMOKE_DOMAIN_ID:-87}
READY_TIMEOUT=${CIRCE_SMOKE_READY_TIMEOUT:-480}
GREEN_TIMEOUT=${CIRCE_SMOKE_GREEN_TIMEOUT:-90}
GREEN_SAMPLES=${CIRCE_SMOKE_GREEN_SAMPLES:-5}
DISCOVERY=${CIRCE_SMOKE_DISCOVERY:-LOCALHOST}
LOG_ROOT=${CIRCE_SMOKE_LOG_DIR:-$HOME/.ros/circe_smoke/$(date +%Y%m%d_%H%M%S)}

SIM_SCENARIOS=(sim_full sim_no_nav2 sim_ekf_only)
HW_SCENARIOS=(hw_full hw_no_nav2 hw_nocam hw_bench bench_dry)

# ---- scenarios ------------------------------------------------------------------------
# Each sets:
#   KIND      sim | hw
#   LAUNCH    package and launch file
#   ARGS      launch arguments
#   BANNER    the line the launch prints once its last gate passes
#   NEEDS     devices that must be present: rover lidar camera
#   PRESENT   node names that must be in the graph
#   ABSENT    node names that must NOT be (the layers switched off)
#   RESOLVE   parent:child transforms that must resolve
#   ROOT      the one root frame of the TF tree
#   NOFRAME   frames that must not exist at all
#   ALLOWDIE  regex of process names allowed to die (headless rviz2 in bench mode)
#   HEADLESS  1 to strip DISPLAY, for launch files that open RViz unconditionally
#   GOAL      "x y yaw" in map for the NAV check; empty skips it

SIM_CORE="robot_state_publisher gz_bridge scan_sector_filter ekf_filter_node
          scan_pitch_filter terrain_node circe_health_monitor diagnostic_aggregator"
HW_CORE="robot_state_publisher joint_state_publisher roverrobotics_driver rplidar_a1
         scan_sector_filter camera imu_fusion ekf_filter_node scan_pitch_filter
         terrain_node circe_health_monitor diagnostic_aggregator"
HW_CORE_NOCAM=$(echo $HW_CORE | sed 's/ camera / /')
SLAM_NODES="slam_toolbox lifecycle_manager_localization"
NAV2_NODES="bt_navigator controller_server planner_server lifecycle_manager_navigation"

define_scenario() {
  KIND=; LAUNCH=; ARGS=; BANNER=; NEEDS=; PRESENT=; ABSENT=; RESOLVE=; ROOT=
  NOFRAME=; ALLOWDIE='^$'; HEADLESS=0; GOAL=
  case "$1" in
    sim_full)
      KIND=sim; LAUNCH="circe_gazebo circe_sim_nav.launch.py"
      ARGS="gui:=false use_rviz:=false"
      BANNER="CIRCE SIM IS UP"
      PRESENT="$SIM_CORE $SLAM_NODES $NAV2_NODES"
      RESOLVE="map:base_link odom:base_link base_link:lidar_link base_link:imu_link
               base_link:camera_depth_optical_frame"
      ROOT=map
      # A quarter turn and 2 m of driving, onto ground that is cost 0 for 0.4 m around.
      GOAL=${CIRCE_SMOKE_SIM_GOAL-0.0 2.0 1.5708} ;;
    sim_no_nav2)
      KIND=sim; LAUNCH="circe_gazebo circe_sim_nav.launch.py"
      ARGS="gui:=false use_rviz:=false use_nav2:=false"
      BANNER="CIRCE SIM IS UP"
      PRESENT="$SIM_CORE $SLAM_NODES"; ABSENT="$NAV2_NODES"
      RESOLVE="map:base_link odom:base_link base_link:lidar_link"
      ROOT=map ;;
    sim_ekf_only)
      KIND=sim; LAUNCH="circe_gazebo circe_sim_nav.launch.py"
      ARGS="gui:=false use_rviz:=false use_slam:=false use_nav2:=false"
      BANNER="CIRCE SIM IS UP"
      PRESENT="$SIM_CORE"; ABSENT="$SLAM_NODES $NAV2_NODES"
      RESOLVE="odom:base_link base_link:lidar_link base_link:imu_link"
      ROOT=odom; NOFRAME=map ;;
    # Each hardware scenario passes use_camera explicitly, so none of them changes
    # meaning if circe_bringup.launch.py's default is flipped.
    hw_full)
      KIND=hw; LAUNCH="circe_bringup circe_bringup.launch.py"; ARGS="use_camera:=true"
      BANNER="CIRCE IS UP"; NEEDS="rover lidar camera"
      PRESENT="$HW_CORE $SLAM_NODES $NAV2_NODES"
      RESOLVE="map:base_link odom:base_link base_link:lidar_link base_link:imu_link
               base_link:camera_link camera_link:camera_depth_optical_frame"
      ROOT=map
      GOAL=${CIRCE_SMOKE_HW_GOAL:-} ;;
    hw_no_nav2)
      KIND=hw; LAUNCH="circe_bringup circe_bringup.launch.py"
      ARGS="use_camera:=true use_nav2:=false"
      BANNER="CIRCE IS UP"; NEEDS="rover lidar camera"
      PRESENT="$HW_CORE $SLAM_NODES"; ABSENT="$NAV2_NODES"
      RESOLVE="map:base_link odom:base_link base_link:lidar_link base_link:camera_link"
      ROOT=map ;;
    hw_nocam)
      # The full stack on the rover with the camera off.
      # camera_link still resolves (it is in the URDF); the optical frames come only
      # from realsense2_camera, so their absence proves the camera really is off.
      # imu_fusion is present but idle, as in hw_bench.
      KIND=hw; LAUNCH="circe_bringup circe_bringup.launch.py"; ARGS="use_camera:=false"
      BANNER="CIRCE IS UP"; NEEDS="rover lidar"
      PRESENT="$HW_CORE_NOCAM $SLAM_NODES $NAV2_NODES"; ABSENT="camera"
      RESOLVE="map:base_link odom:base_link base_link:lidar_link base_link:imu_link
               base_link:camera_link"
      NOFRAME="camera_depth_optical_frame camera_color_optical_frame"
      ROOT=map
      GOAL=${CIRCE_SMOKE_HW_GOAL:-} ;;
    hw_bench)
      # The desk test: a lidar on a USB lead, no rover. Camera off so a bench without
      # the D456 still runs; SLAM and Nav2 run on the static odom -> base_link.
      KIND=hw; LAUNCH="circe_bringup circe_bringup.launch.py"
      ARGS="bench_mode:=true use_camera:=false"
      BANNER="CIRCE IS UP"; NEEDS="lidar"
      PRESENT="robot_state_publisher joint_state_publisher bench_odom_to_base_link
               rplidar_a1 scan_sector_filter scan_pitch_filter terrain_node
               circe_health_monitor diagnostic_aggregator $SLAM_NODES $NAV2_NODES"
      # imu_fusion is NOT absent: with use_imu at its default it starts even without
      # the camera and idles -- a cost circe_bringup.launch.py documents and accepts.
      ABSENT="roverrobotics_driver ekf_filter_node camera"
      RESOLVE="map:base_link odom:base_link base_link:lidar_link"
      ROOT=map; ALLOWDIE='rviz2'; HEADLESS=1 ;;
    bench_dry)
      # The HARDWARE launch path with no hardware at all: every device layer off. Runs
      # on any machine, and still exercises circe_bringup.launch.py's whole gate chain
      # (each gate with its checks disabled must exit and fire the next layer), the
      # bench_mode TF, perception, and the diagnostics tree with every sensor check off.
      KIND=hw; LAUNCH="circe_bringup circe_bringup.launch.py"
      ARGS="bench_mode:=true use_lidar:=false use_camera:=false use_slam:=false use_nav2:=false"
      BANNER="CIRCE IS UP"
      PRESENT="robot_state_publisher joint_state_publisher bench_odom_to_base_link
               scan_pitch_filter terrain_node circe_health_monitor diagnostic_aggregator"
      ABSENT="roverrobotics_driver rplidar_a1 camera ekf_filter_node
              $SLAM_NODES $NAV2_NODES"
      RESOLVE="odom:base_link base_link:lidar_link base_link:imu_link base_link:camera_link"
      ROOT=odom; NOFRAME=map; ALLOWDIE='rviz2'; HEADLESS=1 ;;
    *) return 1 ;;
  esac
  return 0
}

# ---- helpers ------------------------------------------------------------------------

say()  { printf '%s\n' "$*"; }
hr()   { printf '%.0s-' {1..78}; printf '\n'; }

have_device() {
  case "$1" in
    rover)  [ -e /dev/rover-control ] ;;
    lidar)  [ -e /dev/rplidar ] ;;
    # Intel's USB vendor id; see circe_bringup/udev/62-realsense.rules.
    camera) grep -qsx 8086 /sys/bus/usb/devices/*/idVendor ;;
    *)      return 1 ;;
  esac
}

# Driver processes started by something other than this script (a systemd bringup, a
# forgotten terminal) hold the serial ports and the camera. Launching a second set does
# not fail cleanly -- it fails as a gate timeout that looks like a hardware fault.
foreign_drivers() {
  pgrep -af '[r]overrobotics_driver|[s]llidar_node|[r]ealsense2_camera_node' || true
}

marked_pids() {
  grep -lsa "CIRCE_SMOKE_RUN=${RUN_ID:-none}" /proc/[0-9]*/environ 2>/dev/null \
    | awk -F/ '{print $3}' | grep -vx "$$" || true
}

LAUNCH_PID=
stop_run() {
  [ -n "$LAUNCH_PID" ] || return 0
  kill -INT -- "-$LAUNCH_PID" 2>/dev/null
  local i
  for i in $(seq 1 30); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
    sleep 1
  done
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null && sleep 3
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
  # Anything launch put in another process group, or failed to reap.
  local left
  left=$(marked_pids)
  if [ -n "$left" ]; then
    kill -TERM $left 2>/dev/null; sleep 3
    left=$(marked_pids)
    [ -n "$left" ] && kill -KILL $left 2>/dev/null && sleep 1
  fi
  left=$(marked_pids)
  [ -n "$left" ] && say "  !! could not stop: $(echo $left)"
  LAUNCH_PID=
}

# ---- the graph checker (runs inside the scenario's ROS domain) ------------------------

WORK=$(mktemp -d "${TMPDIR:-/tmp}/circe_smoke.XXXXXX")
trap 'stop_run; rm -rf "$WORK"; exit 130' INT TERM
trap 'stop_run; rm -rf "$WORK"' EXIT
cat > "$WORK/check_graph.py" <<'PY'
#!/usr/bin/env python3
"""Poll one live CIRCE graph: nodes, TF integrity, TF ownership, diagnostics, and
optionally one real navigation goal."""
import argparse
import math
import re
import sys
import time
from collections import Counter, defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def lv(level):
    # DiagnosticStatus.level is a `byte`, which rclpy hands over as bytes.
    return level[0] if isinstance(level, (bytes, bytearray)) else int(level)


class Checker(Node):

    def __init__(self):
        super().__init__('circe_smoke_check')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.top = []
        self.agg = {}
        # TF ownership. Jazzy's MessageInfo has no publisher GID, but it has the DDS
        # writer's publication_sequence_number, which is monotonic PER PUBLISHER: an
        # edge whose sequence ever goes backwards has at least two publishers.
        self.last_seq = {}
        self.writers = defaultdict(lambda: 1)
        self.parents = defaultdict(set)
        self.static_count = defaultdict(int)
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(DiagnosticStatus, '/diagnostics_toplevel_state',
                                 lambda m: self.top.append(lv(m.level)), reliable)
        self.create_subscription(
            DiagnosticArray, '/diagnostics_agg',
            lambda m: self.agg.update({s.name: (lv(s.level), s.message) for s in m.status}),
            reliable)
        self.create_subscription(TFMessage, '/tf', self.on_tf, QoSProfile(depth=500))
        self.create_subscription(
            TFMessage, '/tf_static', self.on_static,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_tf(self, msg, info):
        seq = info.get('publication_sequence_number')
        for t in msg.transforms:
            edge = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
            self.parents[edge[1]].add((edge[0], '/tf'))
            if seq is not None:
                if edge in self.last_seq and seq < self.last_seq[edge]:
                    self.writers[edge] += 1
                self.last_seq[edge] = seq

    def on_static(self, msg, _info):
        for t in msg.transforms:
            edge = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
            self.parents[edge[1]].add((edge[0], '/tf_static'))
            self.static_count[edge] += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--present', default='')
    ap.add_argument('--absent', default='')
    ap.add_argument('--resolve', default='')
    ap.add_argument('--noframe', default='')
    ap.add_argument('--root', default='')
    ap.add_argument('--gv', default='')
    ap.add_argument('--listen', type=float, default=6.0)
    ap.add_argument('--green-samples', type=int, default=5)
    ap.add_argument('--green-timeout', type=float, default=90.0)
    ap.add_argument('--report', default='')
    ap.add_argument('--goal', default='', help='"x y yaw" in map; empty skips NAV')
    ap.add_argument('--goal-timeout', type=float, default=120.0)
    # nav2_params.yaml's general_goal_checker xy_goal_tolerance is 0.5; a little slack
    # for the pose read a moment after SUCCEEDED.
    ap.add_argument('--goal-tolerance', type=float, default=0.6)
    a = ap.parse_args()

    rclpy.init()
    n = Checker()
    failures = 0

    def check(name, ok, detail=''):
        nonlocal failures
        failures += 0 if ok else 1
        print(f'    {"PASS" if ok else "FAIL"}  {name}' + ('' if ok or not detail else f': {detail}'))

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    spin(a.listen)

    # -- nodes
    names = [(nm, ns) for nm, ns in n.get_node_names_and_namespaces() if nm != n.get_name()]
    bare = {nm for nm, _ in names}
    dup = [k for k, v in Counter(f'{ns.rstrip("/")}/{nm}' for nm, ns in names).items() if v > 1]
    missing = [x for x in a.present.split() if x not in bare]
    stray = [x for x in a.absent.split() if x in bare]
    check('expected nodes present', not missing, f'missing {missing}')
    check('switched-off layers absent', not stray, f'still running {stray}')
    check('no duplicated node names', not dup, f'duplicated {dup}')

    # -- TF: view_frames tree shape
    if a.gv:
        try:
            text = open(a.gv).read()
            edges = re.findall(r'^"([^"]+)" -> "([^"]+)"\[', text, flags=re.M)
            parents = {p for p, _ in edges}
            children = {c for _, c in edges}
            roots = sorted(parents - children)
            check(f'view_frames: one connected tree rooted at {a.root}',
                  roots == [a.root], f'roots {roots} over {len(edges)} edges')
        except OSError as exc:
            check('view_frames produced a tree', False, str(exc))

    # -- TF: key transforms resolve, forbidden frames absent, one publisher per edge
    unresolved = []
    for pair in a.resolve.split():
        parent, child = pair.split(':')
        try:
            if not n.buffer.can_transform(parent, child, Time()):
                unresolved.append(pair)
        except Exception as exc:                             # noqa: BLE001
            unresolved.append(f'{pair} ({exc})')
    check('key transforms resolve', not unresolved, f'unresolved {unresolved}')
    frames = set(n.parents) | {p for s in n.parents.values() for p, _ in s}
    forbidden = [f for f in a.noframe.split() if f in frames]
    check('frames of switched-off layers absent', not forbidden, f'present {forbidden}')
    conflicts = []
    for child, srcs in n.parents.items():
        if len(srcs) > 1:
            conflicts.append(f'{child} has sources {sorted(srcs)}')
        for parent, topic in srcs:
            if topic == '/tf' and n.writers[(parent, child)] > 1:
                # A respawned publisher restarts its sequence too; the CRASHES check
                # below tells the two apart.
                conflicts.append(f'{parent}->{child} has several /tf publishers '
                                 '(or its publisher restarted)')
            if topic == '/tf_static' and n.static_count[(parent, child)] > 1:
                conflicts.append(f'{parent}->{child} latched by several publishers')
    check('one publisher per TF edge', not conflicts, '; '.join(conflicts))

    # -- diagnostics: sustained green
    deadline = time.monotonic() + a.green_timeout
    run, seen = 0, 0
    while time.monotonic() < deadline and run < a.green_samples:
        rclpy.spin_once(n, timeout_sec=0.1)
        for level in n.top[seen:]:
            run = run + 1 if level == 0 else 0
        seen = len(n.top)
    last = LEVEL.get(n.top[-1], n.top[-1]) if n.top else 'nothing received'
    check(f'/diagnostics_toplevel_state OK for {a.green_samples} consecutive samples',
          run >= a.green_samples, f'last level {last}')
    bad = sorted((k, v) for k, v in n.agg.items() if v[0] != 0)
    if run < a.green_samples:
        for name, (level, message) in bad:
            print(f'          {LEVEL.get(level, level):5} {name}: {message}')
    if a.report:
        with open(a.report, 'w') as f:
            for name, (level, message) in sorted(n.agg.items()):
                f.write(f'{LEVEL.get(level, level):5} {name}: {message}\n')

    # -- navigation: one real goal, end to end. Everything above can pass while the
    # controller refuses to drive; this is the only check that proves it does.
    if a.goal:
        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient

        gx, gy, gyaw = (float(v) for v in a.goal.split())
        client = ActionClient(n, NavigateToPose, 'navigate_to_pose')
        ok, detail = False, ''
        if not client.wait_for_server(timeout_sec=10.0):
            detail = 'no navigate_to_pose action server'
        else:
            goal = NavigateToPose.Goal()
            # Stamp zero means "latest" to TF, which is valid on sim and wall clocks alike.
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
            goal.pose.pose.orientation.z = math.sin(gyaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(gyaw / 2.0)
            sent = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(n, sent, timeout_sec=10.0)
            handle = sent.result()
            if handle is None or not handle.accepted:
                detail = 'goal rejected by bt_navigator'
            else:
                started = time.monotonic()
                done = handle.get_result_async()
                rclpy.spin_until_future_complete(n, done, timeout_sec=a.goal_timeout)
                if not done.done():
                    handle.cancel_goal_async()
                    spin(1.0)
                    detail = f'still driving after {a.goal_timeout:.0f} s; cancelled'
                else:
                    res = done.result()
                    spin(0.5)
                    try:
                        t = n.buffer.lookup_transform('map', 'base_link', Time())
                        err = math.hypot(t.transform.translation.x - gx,
                                         t.transform.translation.y - gy)
                    except Exception:                        # noqa: BLE001
                        err = float('nan')
                    code = getattr(res.result, 'error_code', 0)
                    ok = res.status == GoalStatus.STATUS_SUCCEEDED and err <= a.goal_tolerance
                    status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                              GoalStatus.STATUS_ABORTED: 'ABORTED',
                              GoalStatus.STATUS_CANCELED: 'CANCELED'}.get(res.status, res.status)
                    detail = (f'{status} after {time.monotonic() - started:.1f} s, '
                              f'error_code {code}, rover ended {err:.2f} m from the goal')
                    if code == 107:
                        detail += (' -- 107 is FollowPath CONTROLLER_TIMED_OUT: grep the '
                                   'log for "Costmap timed out waiting for update", i.e. a '
                                   'costmap layer that never went current (terrain_layer '
                                   'with no /terrain/grid is the known one)')
                    elif ok:
                        detail = ''
        check(f'navigation: goal ({gx:g}, {gy:g}) reached', ok, detail)
        if ok:
            print(f'          ({status}, ended {err:.2f} m from the goal)')

    n.destroy_node()
    rclpy.shutdown()
    return min(failures, 100)


if __name__ == '__main__':
    sys.exit(main())
PY

# ---- one scenario -------------------------------------------------------------------

RESULTS=()
record() { RESULTS+=("$(printf '%-14s %-5s %s' "$1" "$2" "$3")"); }

run_scenario() {
  local name=$1
  define_scenario "$name" || { say "unknown scenario: $name"; record "$name" FAIL "unknown"; return; }

  hr
  say "[$name]  ros2 launch $LAUNCH $ARGS"

  if [ "$KIND" = sim ] && ! { ros2 pkg prefix circe_gazebo >/dev/null 2>&1 && command -v gz >/dev/null; }; then
    say "  SKIP: circe_gazebo or Gazebo is not installed on this machine"
    record "$name" SKIP "no Gazebo"; return
  fi
  local dev missing_dev=""
  for dev in $NEEDS; do have_device "$dev" || missing_dev="$missing_dev $dev"; done
  if [ -n "$missing_dev" ]; then
    say "  SKIP: device(s) not present:$missing_dev"
    record "$name" SKIP "no$missing_dev"; return
  fi
  if [ "$KIND" = hw ] && [ -n "$NEEDS" ] && [ -n "$(foreign_drivers)" ]; then
    say "  SKIP: drivers already running outside this test -- they hold the devices:"
    foreign_drivers | sed 's/^/        /'
    record "$name" SKIP "devices busy"; return
  fi

  local dir="$LOG_ROOT/$name"
  mkdir -p "$dir"
  local log="$dir/launch.log"
  RUN_ID="${name}_$$_$RANDOM"
  local extra=()
  if [ "$HEADLESS" = 1 ]; then extra=(-u DISPLAY -u WAYLAND_DISPLAY QT_QPA_PLATFORM=offscreen); fi

  local t0=$SECONDS
  ( exec env "${extra[@]}" CIRCE_SMOKE_RUN="$RUN_ID" GZ_PARTITION="$RUN_ID" \
         ros2 launch $LAUNCH $ARGS ) > "$log" 2>&1 &
  LAUNCH_PID=$!

  # 1. READY
  local verdict="" deadline=$((SECONDS + READY_TIMEOUT))
  while [ $SECONDS -lt $deadline ]; do
    if grep -q "$BANNER" "$log"; then verdict=ready; break; fi
    if grep -qE 'BRINGUP GATE ".*" TIMED OUT|NOT STARTED' "$log"; then verdict=gate; break; fi
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then verdict=exited; break; fi
    sleep 2
  done
  local took=$((SECONDS - t0))
  local gates
  gates=$(grep -oE '\[[a-z_]+\] ready in [0-9.]+ s' "$log" | sed -E 's/\[([a-z_]+)\] ready in ([0-9.]+) s/\1 \2s/' | paste -sd, - | sed 's/,/, /g')
  case "$verdict" in
    ready)  say "  PASS  ready banner after ${took}s   gates: ${gates:-none}" ;;
    gate)   say "  FAIL  a gate timed out after ${took}s:"
            grep -A12 'TIMED OUT' "$log" | sed 's/^/        /' | head -24
            grep 'NOT STARTED' "$log" | sed 's/^/        /' | head -3 ;;
    exited) say "  FAIL  the launch exited after ${took}s; last lines:"
            tail -15 "$log" | sed 's/^/        /' ;;
    *)      say "  FAIL  no ready banner within ${READY_TIMEOUT}s; gates passed: ${gates:-none}" ;;
  esac
  if [ "$verdict" != ready ]; then
    stop_run; record "$name" FAIL "not ready ($verdict) -- $log"; return
  fi

  # 3a. TF tree as tf2_tools sees it (the PDF is kept for a human to look at)
  sleep 5
  ( cd "$dir" && timeout 30 ros2 run tf2_tools view_frames -t 5 -o frames ) > "$dir/view_frames.log" 2>&1

  # 2, 3b, 5. graph checks, from inside the scenario's domain
  python3 "$WORK/check_graph.py" \
      --present "$(echo $PRESENT)" --absent "$(echo $ABSENT)" \
      --resolve "$(echo $RESOLVE)" --noframe "$NOFRAME" --root "$ROOT" \
      --gv "$dir/frames.gv" --green-samples "$GREEN_SAMPLES" \
      --green-timeout "$GREEN_TIMEOUT" --report "$dir/diagnostics_agg.txt" \
      --goal "$GOAL"
  local graph_fail=$?

  # 4. CRASHES -- read after the checks so late deaths are caught too
  local died
  died=$(grep -E 'process has died' "$log" | grep -vE "$ALLOWDIE" || true)
  if [ -n "$died" ]; then
    say "    FAIL  processes died during the run:"
    echo "$died" | sed 's/^/          /' | head -10
  else
    say "    PASS  no process died"
  fi

  stop_run

  if [ "$graph_fail" -eq 0 ] && [ -z "$died" ]; then
    record "$name" PASS "ready ${took}s -- $dir"
  else
    record "$name" FAIL "see $dir"
  fi
}

# ---- main ---------------------------------------------------------------------------

if [ "${1:-}" = "--list" ]; then
  for s in "${SIM_SCENARIOS[@]}" "${HW_SCENARIOS[@]}"; do
    define_scenario "$s"
    printf '%-13s %-3s ros2 launch %s %s%s\n' "$s" "$KIND" "$LAUNCH" "$ARGS" \
      "${NEEDS:+   [needs: $NEEDS]}"
  done
  exit 0
fi

command -v ros2 >/dev/null || { say "ros2 not found: source /opt/ros/jazzy/setup.bash and install/setup.bash first"; exit 2; }
ros2 pkg prefix circe_bringup >/dev/null 2>&1 || { say "circe_bringup not found: source the workspace's install/setup.bash"; exit 2; }

SELECTED=()
if [ $# -eq 0 ]; then
  SELECTED=("${SIM_SCENARIOS[@]}" "${HW_SCENARIOS[@]}")
else
  for arg in "$@"; do
    case "$arg" in
      sim) SELECTED+=("${SIM_SCENARIOS[@]}") ;;
      hw)  SELECTED+=("${HW_SCENARIOS[@]}") ;;
      all) SELECTED+=("${SIM_SCENARIOS[@]}" "${HW_SCENARIOS[@]}") ;;
      *)   SELECTED+=("$arg") ;;
    esac
  done
fi

export ROS_DOMAIN_ID=$DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE=$DISCOVERY
export RCUTILS_COLORIZED_OUTPUT=0
mkdir -p "$LOG_ROOT"

say "CIRCE smoke test -- ROS_DOMAIN_ID=$ROS_DOMAIN_ID, discovery $DISCOVERY, logs in $LOG_ROOT"
for s in "${SELECTED[@]}"; do
  run_scenario "$s"
done

hr
say "SUMMARY"
fails=0
for r in "${RESULTS[@]}"; do
  say "  $r"
  case "$r" in *" FAIL "*) fails=$((fails + 1)) ;; esac
done
[ "$fails" -eq 0 ] && say "all run scenarios passed" || say "$fails scenario(s) failed"
exit $(( fails > 0 ))
