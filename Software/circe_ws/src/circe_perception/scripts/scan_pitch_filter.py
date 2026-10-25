#!/usr/bin/env python3
"""Drop lidar returns that are the ground, seen because the body is tilted.

The EKF runs two_d_mode, so TF says the robot is level and the costmap can't filter
ground returns by height. This uses /imu/data attitude to cut each beam at the range
where it would hit flat ground (height / dz), keeping up-slope beams.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class ScanPitchFilter(Node):
    def __init__(self):
        super().__init__('scan_pitch_filter')

        self.declare_parameter('scan_in', '/scan')
        self.declare_parameter('scan_out', '/scan_filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        # lidar_mount_z; base_link is at ground level.
        self.declare_parameter('lidar_height', 0.295)
        self.declare_parameter('range_margin', 0.85)
        self.declare_parameter('min_tilt_deg', 3.0)
        # Never cut nearer than this, so close obstacles stay visible.
        self.declare_parameter('min_keep_range', 0.35)
        self.declare_parameter('imu_timeout', 0.5)

        g = lambda n: self.get_parameter(n).value
        self.scan_out_topic = g('scan_out')
        self.h = float(g('lidar_height'))
        self.margin = float(g('range_margin'))
        self.min_tilt = math.radians(float(g('min_tilt_deg')))
        self.min_keep = float(g('min_keep_range'))
        self.imu_timeout = float(g('imu_timeout'))

        self.roll = 0.0
        self.pitch = 0.0
        self.imu_stamp = None
        self._angles = None          # cached; recomputed only if the scan geometry changes
        self._angles_key = None
        self._warned_no_imu = False
        self._filtered_scans = 0

        self.pub = self.create_publisher(LaserScan, self.scan_out_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, g('scan_in'), self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            'scan_pitch_filter: %s -> %s | lidar %.3f m, margin %.2f, deadband %.1f deg'
            % (g('scan_in'), self.scan_out_topic, self.h, self.margin, float(g('min_tilt_deg'))))

    def on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))
        self.roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                               1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.imu_stamp = self.get_clock().now()

    def _bearings(self, msg):
        key = (msg.angle_min, msg.angle_increment, len(msg.ranges))
        if key != self._angles_key:
            self._angles_key = key
            self._angles = msg.angle_min + msg.angle_increment * np.arange(len(msg.ranges))
        return self._angles

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities

        # No recent IMU: fail open and pass the scan through.
        stale = (self.imu_stamp is None or
                 (self.get_clock().now() - self.imu_stamp).nanoseconds > self.imu_timeout * 1e9)
        if stale:
            if not self._warned_no_imu:
                self._warned_no_imu = True
                self.get_logger().warn(
                    'no recent %s; passing /scan through unfiltered. Ground returns on '
                    'inclines will reach the costmap.' % self.get_parameter('imu_topic').value)
            out.ranges = msg.ranges
            self.pub.publish(out)
            return
        self._warned_no_imu = False

        tilt = math.hypot(self.pitch, self.roll)
        if tilt < self.min_tilt:
            out.ranges = msg.ranges
            self.pub.publish(out)
            return

        phi = self._bearings(msg)
        # Downward component of each beam under the body's true attitude.
        dz = (math.sin(self.pitch) * np.cos(phi)
              - math.cos(self.pitch) * math.sin(self.roll) * np.sin(phi))

        cut = np.full(dz.shape, np.inf)
        down = dz > 1e-6                       # beams angled below horizontal
        cut[down] = self.h / dz[down] * self.margin
        np.maximum(cut, self.min_keep, out=cut)

        ranges = np.asarray(msg.ranges, dtype=np.float32)
        killed = np.isfinite(ranges) & (ranges >= cut)
        ranges = np.where(killed, np.inf, ranges)
        out.ranges = ranges.tolist()
        self.pub.publish(out)

        n = int(killed.sum())
        if n:
            self._filtered_scans += 1
            if self._filtered_scans % 25 == 1:
                self.get_logger().info(
                    'tilt %.1f deg: dropped %d of %d ground returns (nearest cut %.2f m)'
                    % (math.degrees(tilt), n, len(ranges), float(np.min(cut[down]))
                       if down.any() else float('nan')))


def main():
    rclpy.init()
    node = ScanPitchFilter()
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
