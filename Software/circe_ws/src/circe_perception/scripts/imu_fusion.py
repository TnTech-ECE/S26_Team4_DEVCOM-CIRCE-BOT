#!/usr/bin/env python3
"""Turn the D456's raw gyro + accel into a body-frame IMU with orientation.

    /camera/camera/imu  ->  imu_fusion  ->  /imu/data

The D456 IMU has no orientation (quaternion is all zeros), and its frame is optical.
This rotates gyro/accel into base_link, runs a complementary filter for roll/pitch,
learns gyro bias while stationary (the EKF's heading is a pure gyro-z integral), and
publishes a BNO055-style /imu/data. Yaw is published as 0 with covariance 1e6.
Subscribes BEST_EFFORT because the camera's IMU publisher is hardcoded to it.
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
    """ZYX quaternion with yaw = 0."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    return (sr * cp, cr * sp, -sr * sp, cr * cp)   # x, y, z, w


class ImuFusion(Node):

    def __init__(self):
        super().__init__('imu_fusion')

        self.declare_parameter('target_frame', 'base_link')
        # Complementary filter time constant (s): larger trusts the gyro more.
        self.declare_parameter('tau', 1.0)
        # Skip the accel correction when |a| is this far from g.
        self.declare_parameter('accel_gate', 1.5)          # m/s^2
        self.declare_parameter('learn_gyro_bias', True)
        # Must be looser than the bias itself, or the bias is never learned.
        self.declare_parameter('bias_gyro_thresh', 0.06)   # rad/s, ~3.4 deg/s
        self.declare_parameter('bias_accel_thresh', 0.30)  # m/s^2 from g
        self.declare_parameter('bias_dwell', 1.0)          # s of stillness required
        self.declare_parameter('bias_tau', 60.0)           # s, learning time constant
        # Wheel odometry confirms the robot is still, so slow turns aren't learned as bias.
        self.declare_parameter('odom_topic', '/odometry/wheels')
        self.declare_parameter('bias_odom_lin', 0.01)      # m/s
        self.declare_parameter('bias_odom_ang', 0.01)      # rad/s
        self.declare_parameter('odom_timeout', 0.5)        # s
        # Re-seed from the accelerometer after a gap longer than this.
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
        # A device with its own orientation doesn't need this node; warn once.
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

        # Accelerometer attitude, only while it's measuring mostly gravity.
        if level:
            roll_a = math.atan2(a[1], a[2])
            pitch_a = math.atan2(-a[0], math.hypot(a[1], a[2]))
        else:
            roll_a = pitch_a = None

        # Seed from the accelerometer so a rover parked on a slope starts correct.
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
            # Euler-angle kinematics (matters on side slopes).
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
        # Require continuous stillness.
        if now - self.still_since < self.bias_dwell:
            return

        # Running mean at first, then a slow tracker for temperature drift.
        self.bias_n += 1
        k = max(1.0 / self.bias_n, dt / max(self.bias_tau, 1e-6))
        self.bias += min(k, 1.0) * (w - self.bias)

    def _publish(self, src, w, a, t):
        # Fixed schedule so the long-run rate is exact despite stamp jitter.
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

        # Bias-corrected rates.
        out.angular_velocity.x = float(w[0])
        out.angular_velocity.y = float(w[1])
        out.angular_velocity.z = float(w[2])
        # Gravity left in.
        out.linear_acceleration.x = float(a[0])
        out.linear_acceleration.y = float(a[1])
        out.linear_acceleration.z = float(a[2])

        # Yaw not estimated: 1e6 = do not use.
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
