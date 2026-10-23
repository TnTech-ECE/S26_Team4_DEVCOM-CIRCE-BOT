#!/usr/bin/env python3
"""Drop lidar returns that are the ground, seen because the body is tilted.

*** THE PROBLEM THIS EXISTS TO SOLVE ***

The RPLidar sweeps a plane fixed to the chassis, 0.295 m above base_link. On level
ground that plane is horizontal and never sees the floor. On a ramp the whole plane
tilts with the body, and the down-slope half of the sweep drives into the ground:

    ground strike range = lidar height / sin(tilt)

      5 deg -> 3.38 m      20 deg -> 0.86 m
      9 deg -> 1.89 m      25 deg -> 0.70 m
     15 deg -> 1.14 m      31 deg -> 0.57 m

Measured in sim on circe_indoor.sdf, driving onto the two test ramps:

    9 deg ramp   IMU pitch  9.11 deg   116 of 720 beams under 1.5 m
    31 deg ramp  IMU pitch 83.92 deg   367 of 430 beams under 1.5 m  (85%)

Those returns are real - the beam genuinely hit something - so no range or intensity
filter can distinguish them. They are simply not obstacles.

*** WHY THE COSTMAP CANNOT FILTER THEM BY HEIGHT ***

nav2_costmap_2d's VoxelLayer already knows how to reject points by height: that is what
min_obstacle_height is for. It cannot help here, because the EKF runs with

    two_d_mode: true            (circe_nav/config/localization_ekf.yaml)

which zeroes roll, pitch and z out of odom -> base_link. Confirmed on both ramps above:
true IMU pitch 9.11 deg and 83.92 deg, TF pitch 0.00 deg, TF z 0.000 m in every sample.
So the costmap transforms each ground return through a frame that insists the robot is
flat, places it at the lidar's own height, and cannot tell it from a wall. Every point
lands at z = 0.295 whatever the body is really doing.

Turning two_d_mode off would fix that at the source, but it feeds pitch into the frame
slam_toolbox scan-matches in - a 2D SLAM being handed an out-of-plane scan - so it is a
much larger change with its own failure mode. This node is the surgical version.

*** WHAT IT DOES ***

Reads the true attitude from /imu/data (which is never flattened), computes for EACH
beam the range at which it would strike flat ground, and marks everything at or beyond
that range as "no return". Beams pointing up, and every beam when the robot is level,
are passed through untouched.

For a beam at bearing phi in the lidar frame, with body roll r and pitch p, the downward
component of the beam direction is

    dz = sin(p) cos(phi) - cos(p) sin(r) sin(phi)

and the beam meets flat ground at height/dz where dz > 0. Doing it per beam rather than
per scan matters: on a ramp the up-slope half of the sweep is still looking at real
obstacles and must not be thrown away.

The result is that on an incline the rover falls back on circe_perception's terrain grid
- the depth camera - for the ground ahead, which is the sensor that can actually tell a
slope from a wall. The lidar keeps its up-slope beams and resumes fully once level.

Publishes a filtered LaserScan; publishes no TF. Point the costmap's observation source
at the output and leave the raw scan for anything that wants it.
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
        # Height of the scan plane above the ground the robot is standing on. This is
        # lidar_mount_z from circe_description/urdf/accessories/rplidar_a1.urdf.xacro,
        # and it is a height above ground rather than above base_link only because
        # base_link itself sits at ground level on this robot.
        self.declare_parameter('lidar_height', 0.295)
        # Cut slightly short of the geometric strike range. Depth of field, beam
        # divergence and a ground that is not perfectly flat all put returns a little
        # nearer than the ideal intersection.
        self.declare_parameter('range_margin', 0.85)
        # Below this tilt the strike range is past the lidar's useful range anyway
        # (3.4 m at 5 deg), so filtering would only cost coverage.
        self.declare_parameter('min_tilt_deg', 3.0)
        # Never cut nearer than this, so a genuine obstacle right in front of a tilted
        # robot is still seen. At 31 deg the strike range is 0.57 m and the margin would
        # otherwise pull the cut to 0.48 m.
        self.declare_parameter('min_keep_range', 0.35)
        # An IMU that has stopped updating must not silently disable the filter, nor
        # freeze it on a stale attitude.
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

        # No usable attitude: pass the scan through unchanged. Failing open keeps the
        # robot navigating on level ground, which is where it spends most of its life,
        # and the alternative - blanking the lidar - is far more dangerous.
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
