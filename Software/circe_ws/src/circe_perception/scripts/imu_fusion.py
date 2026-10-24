#!/usr/bin/env python3
"""Turn the D456's raw gyro + accel into a body-frame IMU with orientation.

    /camera/camera/imu  ->  imu_fusion  ->  /imu/data

Why this node has to exist
--------------------------
The D456's IMU is a 6-axis MEMS part with no magnetometer and no on-chip sensor
fusion, and realsense2_camera reflects that honestly: every message it publishes has

    orientation = (0, 0, 0, 0)
    orientation_covariance[0] = -1          # "no orientation in this message"

(base_realsense_node.cpp, ImuMessage_AddDefaultValues). `unite_imu_method: 2` does
NOT change that -- it interleaves the 200 Hz gyro and 100 Hz accel streams onto one
topic, which is a TIMING merge, not an attitude estimate.

That matters because two nodes in this workspace read /imu/data expecting the
BNO055's fused quaternion, and both do naive quaternion -> roll/pitch arithmetic:

    circe_perception/scripts/scan_pitch_filter.py   on_imu()
    circe_perception/scripts/terrain_node.py        _on_imu()

Feed either one a (0,0,0,0) quaternion and it computes roll = 0.00, pitch = 0.00 --
a perfectly level robot, on every message, forever. Neither node errors, and
scan_pitch_filter's imu_timeout backstop does NOT fire, because messages ARE
arriving; they just say nothing. The failure is silent and it reinstates exactly the
ramp bug those two nodes were written to fix. So "point the EKF at the camera IMU"
is not a config change -- something has to produce the attitude.

The second problem: frames
--------------------------
The camera publishes in camera_imu_optical_frame, which is optical convention
(X right, Y down, Z forward), related to the body by realsense's

    quaternion_optical.setRPY(-pi/2, 0, -pi/2)          # tfs.cpp:124

so that a gyro reading (gx, gy, gz) in optical becomes (gz, -gx, -gy) in the body.
The robot's YAW RATE is the optical frame's -gy, not gz. robot_localization would
handle that for itself -- it transforms every IMU input through TF -- but the two
perception nodes above do not, and cannot: they read the quaternion directly. So the
rotation into base_link has to happen here, before anything downstream sees it.

What this node does
-------------------
1. Rotates gyro and accel from the message's own frame into `target_frame` using
   TF. Looked up once and cached; it is a static transform.
2. Runs a complementary filter: integrate the gyro for the fast, quiet signal;
   pull roll and pitch toward the accelerometer's gravity vector slowly, so that
   gyro drift cannot accumulate. THIS is where "fuse gyro and accel together"
   actually happens, and its product -- gravity-referenced roll and pitch -- is the
   thing the rest of CIRCE needs.
3. Learns the gyro bias while the robot is stationary. Not optional: the EKF takes
   ONLY yaw rate from the IMU and ONLY vx/vy from the wheels
   (circe_nav/config/localization_ekf.yaml), so heading is a pure integral of
   gyro z. An uncorrected 1 deg/s bias -- ordinary for this class of MEMS part --
   is 1 deg/s of heading drift, straight into map->odom.
4. Publishes sensor_msgs/Imu in `target_frame` with orientation filled in, i.e. a
   drop-in replacement for the BNO055 on /imu/data.

Yaw is deliberately published as ZERO
-------------------------------------
With no magnetometer there is no yaw reference, so an integrated yaw would drift
without bound while LOOKING valid. Publishing a constant instead is a failure any
consumer can notice, and orientation_covariance advertises it as untrusted (1e6).
Nothing here is worse off for it:

  * the EKF takes yaw RATE (imu0_config index 11), never yaw;
  * scan_pitch_filter and terrain_node extract roll/pitch with the standard ZYX
    formulas, which are yaw-independent;
  * terrain_node._attitude_correct already rebuilds body attitude from TF yaw plus
    IMU roll/pitch -- it was written to take yaw from elsewhere.

QoS
---
Subscribes with qos_profile_sensor_data because the camera's ~/imu publisher is
hardcoded to rmw_qos_profile_sensor_data -- BEST_EFFORT -- and is not reachable by
any driver parameter (rs_node_setup.cpp:384, HID_QOS). A RELIABLE subscriber does
not match a BEST_EFFORT publisher in ROS 2 and would receive silence.

This node publishes no TF and no /cmd_vel. The one-publisher-per-edge contract is
untouched.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

GRAVITY = 9.80665


def _quat_to_rot(x, y, z, w):
    """Rotation matrix from a quaternion. Avoids a tf_transformations dependency."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _quat_from_roll_pitch(roll, pitch):
    """ZYX quaternion with yaw = 0.

    Round-trips exactly through the roll/pitch extraction used by
    scan_pitch_filter.on_imu and terrain_node._on_imu -- see the test at the bottom
    of this file's git history and the assertion in tools, if you doubt it.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    return (sr * cp, cr * sp, -sr * sp, cr * cp)   # x, y, z, w


class ImuFusion(Node):

    def __init__(self):
        super().__init__('imu_fusion')

        self.declare_parameter('target_frame', 'base_link')
        # Time constant of the complementary filter, in seconds. This is the only
        # real tuning knob. Large = trust the gyro, smooth but drifts back slowly;
        # small = trust the accelerometer, tracks gravity fast but every bump and
        # every hard acceleration shows up as attitude. 1.0 s is deliberately long:
        # the accel only has to catch slow gyro drift, and on Tennessee terrain the
        # accelerometer is the noisy input, not the gyro.
        self.declare_parameter('tau', 1.0)
        # Skip the accelerometer correction whenever |a| is this far from g. A
        # rover that is accelerating, braking or bouncing is not measuring gravity,
        # and folding those samples in tilts the estimate toward the direction of
        # travel. On rejection the filter coasts on the gyro, which is exactly what
        # a gyro is for over short intervals.
        self.declare_parameter('accel_gate', 1.5)          # m/s^2
        # Gyro bias learning. See the docstring: heading is a pure integral of
        # gyro z, so this is load-bearing, not a refinement.
        self.declare_parameter('learn_gyro_bias', True)
        # *** THIS THRESHOLD MUST BE LOOSER THAN THE BIAS IT IS LOOKING FOR. ***
        # It gates on |w - bias|, and before anything is learned that quantity IS
        # the bias. Set it at 0.02 rad/s and a part with a typical 1 deg/s
        # (0.0175 rad/s) offset sits right on the limit: sensor noise pushes it over
        # every few samples, the dwell timer resets, and the bias is never learned at
        # all -- measured, the residual stayed at the full 1.003 deg/s indefinitely.
        # 0.06 rad/s is 3.4 deg/s, which comfortably admits an uncalibrated BMI055
        # while excluding any commanded rotation (Nav2 samples wz over +/-2.0 rad/s).
        self.declare_parameter('bias_gyro_thresh', 0.06)   # rad/s, ~3.4 deg/s
        self.declare_parameter('bias_accel_thresh', 0.30)  # m/s^2 from g
        self.declare_parameter('bias_dwell', 1.0)          # s of stillness required
        self.declare_parameter('bias_tau', 60.0)           # s, learning time constant
        # Wheel odometry is the honest stationarity signal, and it is what makes the
        # loose gyro threshold above safe: a gentle Nav2 path-following turn can sit
        # under 3.4 deg/s for seconds at a time, and without this the node would
        # quietly absorb that real rotation into the bias and then subtract it. The
        # topic is the EKF's own odom0, so it is always present on hardware. If it is
        # silent the node falls back to the IMU-only test and says so once.
        self.declare_parameter('odom_topic', '/odometry/wheels')
        self.declare_parameter('bias_odom_lin', 0.01)      # m/s
        self.declare_parameter('bias_odom_ang', 0.01)      # rad/s
        self.declare_parameter('odom_timeout', 0.5)        # s
        # Integration guard. A gap longer than this means dropped frames or a
        # stalled camera; integrating across it would smear a large rotation into
        # one step, so the filter re-seeds from the accelerometer instead.
        self.declare_parameter('max_dt', 0.1)              # s
        self.declare_parameter('publish_rate_limit', 0.0)  # Hz, 0 = every message

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.target = g('target_frame')
        self.tau = float(g('tau'))
        self.accel_gate = float(g('accel_gate'))
        self.learn_bias = bool(g('learn_gyro_bias'))
        self.bias_gyro_thresh = float(g('bias_gyro_thresh'))
        self.bias_accel_thresh = float(g('bias_accel_thresh'))
        self.bias_dwell = float(g('bias_dwell'))
        self.bias_tau = float(g('bias_tau'))
        self.odom_lin = float(g('bias_odom_lin'))
        self.odom_ang = float(g('bias_odom_ang'))
        self.odom_timeout = float(g('odom_timeout'))
        self.max_dt = float(g('max_dt'))
        rate = float(g('publish_rate_limit'))
        self.min_period = 1.0 / rate if rate > 0.0 else 0.0

        self.rot = None            # target_frame <- message frame, cached once
        self.src_frame = None
        self.roll = 0.0
        self.pitch = 0.0
        self.bias = np.zeros(3)
        self.bias_n = 0
        self.last_t = None
        self.still_since = None
        self.next_pub = None       # publish_rate_limit schedule; see _publish
        self.seeded = False
        self.odom_still = None      # None = no odometry seen recently
        self.odom_stamp = None
        self._warned_tf = False
        self._warned_oriented = False
        self._warned_no_odom = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Imu, 'imu_out', qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu_in', self.on_imu, qos_profile_sensor_data)

        topic = g('odom_topic')
        if topic:
            self.create_subscription(Odometry, topic, self.on_odom, 10)

        self.get_logger().info(
            'imu_fusion: imu_in -> imu_out in %s | tau %.2f s, accel gate %.2f m/s^2, '
            'bias learning %s' % (self.target, self.tau, self.accel_gate,
                                  'on' if self.learn_bias else 'OFF'))

    # ---- frames ---------------------------------------------------------------

    def _rotation(self, frame, stamp):
        """Cache the static rotation target_frame <- frame."""
        if self.rot is not None and frame == self.src_frame:
            return self.rot
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target, frame, rclpy.time.Time())
        except Exception as exc:                        # noqa: BLE001 - TF raises many
            if not self._warned_tf:
                self._warned_tf = True
                self.get_logger().warn(
                    'imu_fusion: no transform %s -> %s yet (%s). Dropping IMU messages '
                    'until it appears -- check that robot_state_publisher is up and, on '
                    'hardware, that the RealSense driver is publishing its own camera '
                    'frames (use_nominal_extrinsics is false there by design).'
                    % (frame, self.target, exc))
            return None
        q = tf.transform.rotation
        self.rot = _quat_to_rot(q.x, q.y, q.z, q.w)
        self.src_frame = frame
        self._warned_tf = False
        self.get_logger().info(
            'imu_fusion: locked rotation %s -> %s' % (frame, self.target))
        return self.rot

    # ---- filter ---------------------------------------------------------------

    def on_imu(self, msg):
        # A device that DOES supply orientation should not be run through this node --
        # it would throw away a better estimate. Warn once and carry on, since the
        # filter output is still valid, just redundant.
        if msg.orientation_covariance[0] >= 0.0 and not self._warned_oriented:
            self._warned_oriented = True
            self.get_logger().warn(
                'imu_fusion: input already carries an orientation. This node is for '
                'raw 6-axis IMUs (the D456); for a BNO055 or a simulated IMU, publish '
                'straight to /imu/data instead.')

        rot = self._rotation(msg.header.frame_id, msg.header.stamp)
        if rot is None:
            return

        w = rot @ np.array([msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z])
        a = rot @ np.array([msg.linear_acceleration.x,
                            msg.linear_acceleration.y,
                            msg.linear_acceleration.z])

        t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t

        a_norm = float(np.linalg.norm(a))
        level = abs(a_norm - GRAVITY) < self.accel_gate and a_norm > 1e-6

        # Accelerometer attitude. Only meaningful while the specific force is
        # dominated by gravity, which `level` is the test for.
        if level:
            roll_a = math.atan2(a[1], a[2])
            pitch_a = math.atan2(-a[0], math.hypot(a[1], a[2]))
        else:
            roll_a = pitch_a = None

        # Seed from the accelerometer rather than starting at zero: the first frames
        # decide whether the rover thinks it is level while parked on a slope.
        if not self.seeded:
            if roll_a is None:
                return
            self.roll, self.pitch = roll_a, pitch_a
            self.seeded = True
            self.get_logger().info(
                'imu_fusion: seeded from gravity at roll %.2f deg, pitch %.2f deg'
                % (math.degrees(self.roll), math.degrees(self.pitch)))

        self._update_bias(w, a_norm, dt)
        wb = w - self.bias

        if 0.0 < dt <= self.max_dt:
            # Euler-angle kinematics, not a naive component-wise add: at non-zero
            # roll the body's y and z rates both contribute to pitch, and ignoring
            # that is what makes cheap filters wander on side slopes.
            tp = math.tan(self.pitch)
            sr, cr = math.sin(self.roll), math.cos(self.roll)
            self.roll += (wb[0] + wb[1] * sr * tp + wb[2] * cr * tp) * dt
            self.pitch += (wb[1] * cr - wb[2] * sr) * dt

            if roll_a is not None:
                alpha = self.tau / (self.tau + dt)
                # Wrap the residual so a correction never takes the long way round.
                self.roll += (1.0 - alpha) * _wrap(roll_a - self.roll)
                self.pitch += (1.0 - alpha) * _wrap(pitch_a - self.pitch)
        elif dt > self.max_dt and roll_a is not None:
            self.roll, self.pitch = roll_a, pitch_a

        self._publish(msg, wb, a, t)

    def on_odom(self, msg):
        v = msg.twist.twist
        self.odom_still = (abs(v.linear.x) < self.odom_lin
                           and abs(v.linear.y) < self.odom_lin
                           and abs(v.angular.z) < self.odom_ang)
        self.odom_stamp = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

    def _update_bias(self, w, a_norm, dt):
        """Learn the gyro zero-rate offset during genuine stillness."""
        if not self.learn_bias or dt <= 0.0:
            return

        now = self.last_t
        fresh = (self.odom_stamp is not None
                 and abs(now - self.odom_stamp) < self.odom_timeout)
        if not fresh:
            if not self._warned_no_odom:
                self._warned_no_odom = True
                self.get_logger().warn(
                    'imu_fusion: no wheel odometry on the bias gate. Falling back to '
                    'the IMU-only stillness test, which cannot distinguish a slow turn '
                    'from a stationary robot and may absorb real rotation into the gyro '
                    'bias. Fine on a bench; check the driver before driving.')
        elif self._warned_no_odom:
            self._warned_no_odom = False

        still = (float(np.max(np.abs(w - self.bias))) < self.bias_gyro_thresh
                 and abs(a_norm - GRAVITY) < self.bias_accel_thresh
                 and (not fresh or self.odom_still))
        if not still:
            self.still_since = None
            return
        if self.still_since is None:
            self.still_since = now
            return
        # Require a continuous dwell before believing it, so that a momentary lull
        # mid-manoeuvre cannot contribute.
        if now - self.still_since < self.bias_dwell:
            return

        # Running mean at first, leaky tracker afterwards. A flat dt/bias_tau gain
        # gives a 60 s time constant from the very first sample, so the robot would
        # have to sit still for a minute before its heading was trustworthy; 1/n
        # converges in the first second and then hands over to the slow tracker,
        # which is what lets the bias follow temperature drift over a long mission.
        self.bias_n += 1
        k = max(1.0 / self.bias_n, dt / max(self.bias_tau, 1e-6))
        self.bias += min(k, 1.0) * (w - self.bias)

    def _publish(self, src, w, a, t):
        # *** SCHEDULED FROM WHERE THE LAST SLOT WAS DUE, NOT FROM WHEN IT FIRED. ***
        # This was `skip if t - last_pub < min_period`, and at publish_rate_limit 100.0 on
        # the 200 Hz stream it delivered ~75 Hz, not 100. At epoch-scale stamps float64
        # resolves ~0.24 us, so a sample landing exactly on the 10 ms boundary is a coin
        # flip -- and every lost flip costs a whole extra input period. Advancing a fixed
        # schedule keeps the long-run rate exact whatever the jitter. After a gap longer
        # than a slot it restarts from now instead of bursting to catch up.
        if self.min_period > 0.0:
            if self.next_pub is not None and t < self.next_pub:
                return
            on_schedule = self.next_pub is not None and t - self.next_pub < self.min_period
            self.next_pub = (self.next_pub if on_schedule else t) + self.min_period

        out = Imu()
        out.header.stamp = src.header.stamp
        out.header.frame_id = self.target

        x, y, z, qw = _quat_from_roll_pitch(self.roll, self.pitch)
        out.orientation.x, out.orientation.y = x, y
        out.orientation.z, out.orientation.w = z, qw

        # Bias-corrected rates: this is what the EKF integrates into heading.
        out.angular_velocity.x = float(w[0])
        out.angular_velocity.y = float(w[1])
        out.angular_velocity.z = float(w[2])
        # Gravity is left IN, so that the EKF's imu0_remove_gravitational_acceleration
        # means what it says if linear acceleration is ever fused.
        out.linear_acceleration.x = float(a[0])
        out.linear_acceleration.y = float(a[1])
        out.linear_acceleration.z = float(a[2])

        # Roll and pitch are gravity-referenced and good to a degree or so. Yaw is
        # not estimated at all -- 1e6 is the conventional "do not use this" marker.
        out.orientation_covariance = [0.0004, 0.0, 0.0,
                                      0.0, 0.0004, 0.0,
                                      0.0, 0.0, 1e6]
        out.angular_velocity_covariance = list(src.angular_velocity_covariance)
        out.linear_acceleration_covariance = list(src.linear_acceleration_covariance)
        self.pub.publish(out)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def main():
    rclpy.init()
    node = ImuFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
