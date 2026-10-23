#!/usr/bin/env python3
"""Terrain traversability for CIRCE, from the D456 depth image.

Reads the depth image, judges each 5 cm patch of ground as drivable or not, and publishes
a graded nav_msgs/OccupancyGrid that Nav2 consumes as a StaticLayer in both costmaps.

    ros2 launch circe_perception terrain.launch.py
    ros2 launch circe_perception terrain.launch.py use_sim_time:=true

Consumes:  /camera/camera/depth/image_rect_raw   16UC1 (hardware, mm) or 32FC1 (gz, m)
           /camera/camera/depth/camera_info      intrinsics
           TF camera_depth_optical_frame -> base_link (static) and -> odom (per frame)
Publishes: /terrain/grid                          OccupancyGrid, 0-100, -1 unknown, in odom

Publishes NO TF and NO /cmd_vel. This is a pure topic transform, so the workspace's
one-publisher-per-edge contract is untouched. It also does not talk to the costmaps --
whether Nav2 listens is decided in circe_nav/config/nav2_params.yaml.

*** WHY THE GRID IS PERSISTENT ***
The camera is mounted at pitch 0, so ground is not visible closer than ~0.62 m. A
per-frame snapshot would therefore forget a hazard exactly as the rover reached it. The
grid instead accumulates in odom and scrolls with the robot: a patch classified at 4 m is
still known while driving over it blind. The same accumulation beats depth noise, which is
~2% of range (+/-8 cm at 4 m, the same order as the step threshold) -- each cell is
re-observed at shrinking range, so its estimate sharpens as it starts to matter.

*** WHY z IS HEIGHT ABOVE GROUND, WITH NO OFFSET ***
base_to_chassis puts chassis_link +0.125 above base_link and the axle sits 0.125 above the
ground, so base_link is at ground level. The EKF runs two_d_mode: true, which pins z, roll
and pitch to zero, so odom z equals base_link z. Every threshold in config/terrain.yaml is
therefore a raw height, not a difference from some datum.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy import ndimage
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_ros import Buffer, TransformListener


def _quat_to_rot(x, y, z, w):
    """Rotation matrix from a quaternion. Avoids a tf_transformations dependency."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _tf_to_rt(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    return _quat_to_rot(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])


def _grade(value, warn, lethal):
    """Linear ramp from 0 at `warn` to 1 at `lethal`, clamped. Vectorised."""
    if lethal <= warn:
        return (value >= lethal).astype(np.float32)
    return np.clip((value - warn) / (lethal - warn), 0.0, 1.0).astype(np.float32)


class TerrainNode(Node):

    def __init__(self):
        super().__init__('terrain_node')

        p = self.declare_parameters('', [
            ('global_frame', 'odom'),
            ('robot_frame', 'base_link'),
            ('roi_min_range', 0.6),
            ('roi_max_range', 4.0),
            ('roi_half_width', 2.0),
            ('depth_min', 0.4),
            ('depth_max', 4.5),
            ('pixel_stride', 4),
            ('resolution', 0.05),
            ('grid_size_m', 12.0),
            ('decay', 0.98),
            ('min_points_per_cell', 3.0),
            ('full_confidence_points', 12.0),
            ('min_points_for_lethal', 6.0),
            ('step_warn', 0.03),
            ('step_lethal', 0.06),
            ('slope_warn', 0.17),
            ('slope_lethal', 0.44),
            ('rough_warn', 0.010),
            ('rough_max', 0.040),
            ('rough_max_cost', 60),
            ('drop_warn', 0.04),
            ('drop_lethal', 0.08),
            ('step_window', 3),
            ('slope_window', 9),
            ('min_slope_support', 0.25),
            ('publish_rate', 5.0),
            ('lethal_cost', 100),
            ('publish_debug_cloud', False),
            ('use_imu_attitude', True),
            ('imu_topic', '/imu/data'),
            ('clear_driven', True),
            ('driven_radius', 0.4),
        ])
        self.cfg = {d.name: d.value for d in p}

        self.res = float(self.cfg['resolution'])
        self.n_cells = int(round(float(self.cfg['grid_size_m']) / self.res))

        # Running sums per cell, all decayed together each frame. Keeping sums rather than
        # a list of heights is what makes this O(points) with no allocation per frame.
        shape = (self.n_cells, self.n_cells)
        self.acc_n = np.zeros(shape, dtype=np.float32)
        self.acc_z = np.zeros(shape, dtype=np.float32)
        self.acc_z2 = np.zeros(shape, dtype=np.float32)
        # Cells the robot's own footprint has occupied. See _classify: whatever the
        # terrain maths says about ground the robot is demonstrably standing on, it is
        # drivable, because the robot drove there.
        self.driven = np.zeros(shape, dtype=bool)
        # World coords (in global_frame) of cell [0, 0]. None until the first robot pose.
        self.origin = None
        self.imu_rpy = None      # (roll, pitch) from the IMU; None until the first message

        self.k = None            # (fx, fy, cx, cy)
        self.row_band = None     # (v_lo, v_hi) computed once from camera_info + static TF

        # *** THE LISTENER GETS ITS OWN NODE AND THREAD. BOTH ARGUMENTS ARE LOAD-BEARING. ***
        # Both lookups at a message stamp below wait up to 0.05 s for TF to catch up, and
        # they always have to: the EKF publishes odom -> base_link only on its 30 Hz tick,
        # stamped with its latest measurement, so a depth frame routinely arrives ~20 ms
        # BEFORE the transform for its own stamp. With TransformListener(buffer, self)
        # the /tf subscription ran on this node's single-threaded executor -- the one the
        # depth callback was blocking while it waited. No transform could arrive during
        # the wait, so the timeout was dead code and every frame newer than the last
        # processed TF was dropped.
        #
        # spin_thread=True alone does NOT fix it while `self` is passed: the listener
        # adds the node it is given to its own executor, rclpy.spin() in main() takes the
        # node straight back (a node belongs to one executor at a time), and every
        # callback lands on the blocked thread again -- measured, 109 of 110 frames still
        # dropped, each callback running the full 50 ms timeout. node=None makes the
        # listener create a private node that its thread alone spins.
        #
        # Measured in circe_sim_nav before the fix: 0 of 77 frames covered on arrival,
        # the covering TF 20 ms later, 100 of 100 frames dropped. No /terrain/grid ever,
        # so the local_costmap's terrain_layer never went current and controller_server
        # aborted every goal with "Costmap timed out waiting for update". Whether it
        # worked at all depended on the EKF timer's phase against the camera's -- i.e.
        # on start-up timing -- so it failed per RUN, not per frame. The tf2 buffer core
        # is thread-safe.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)
        self._unknown = None     # cached all-unknown grid data; see _publish_unknown

        # Depth is published best-effort by both the RealSense driver (depth_qos:
        # SENSOR_DATA) and the gz bridge. A RELIABLE subscription would simply never
        # connect to either.
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            CameraInfo, 'depth/camera_info', self._on_camera_info, 10)
        self.create_subscription(
            Image, 'depth/image_rect_raw', self._on_depth, sensor_qos)

        if self.cfg['use_imu_attitude']:
            self.create_subscription(
                Imu, self.cfg['imu_topic'], self._on_imu, sensor_qos)

        self.grid_pub = self.create_publisher(OccupancyGrid, 'grid', 1)
        self.create_timer(1.0 / float(self.cfg['publish_rate']), self._publish)

        self._warned = set()
        self.get_logger().info(
            f'terrain_node up: {self.n_cells}x{self.n_cells} cells @ {self.res} m, '
            f'ROI {self.cfg["roi_min_range"]}-{self.cfg["roi_max_range"]} m')

    # -- helpers ---------------------------------------------------------------------

    def _warn_once(self, key, msg):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(msg)

    def _on_camera_info(self, msg):
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _compute_row_band(self, height):
        """Rows that can contain ground within the ROI, from the real intrinsics and TF.

        Deliberately not hardcoded: it depends on the camera resolution, on the mount
        height, and on camera_mount_pitch, all of which change. Casting a ray per row and
        intersecting z=0 in base_link is exact and costs nothing -- it runs once.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.cfg['robot_frame'], self.camera_frame, rclpy.time.Time())
        except Exception:
            return None

        rot, trans = _tf_to_rt(tf)
        _, fy, _, cy = self.k
        rmin, rmax = self.cfg['roi_min_range'], self.cfg['roi_max_range']

        rows = []
        for v in range(height):
            # Ray through the centre column of row v, in the optical frame (z forward).
            d = rot @ np.array([0.0, (v - cy) / fy, 1.0])
            if d[2] >= -1e-6:          # not pointing down; never meets the ground
                continue
            s = -trans[2] / d[2]
            if s <= 0:
                continue
            hit = trans + s * d
            if rmin <= math.hypot(hit[0], hit[1]) <= rmax:
                rows.append(v)

        if not rows:
            return None
        return min(rows), max(rows)

    def _decode(self, msg):
        """Depth image to metres, without cv_bridge.

        Hardware publishes 16UC1 in millimetres; the Gazebo rgbd_camera publishes 32FC1 in
        metres. Both have to work or the sim-first development path breaks.
        """
        if msg.encoding == '16UC1':
            buf = np.frombuffer(msg.data, dtype=np.uint16)
            img = buf.reshape(msg.height, msg.step // 2)[:, :msg.width]
            return img.astype(np.float32) * 0.001
        if msg.encoding in ('32FC1', '32FC1;'):
            buf = np.frombuffer(msg.data, dtype=np.float32)
            return buf.reshape(msg.height, msg.step // 4)[:, :msg.width]
        self._warn_once('enc', f'unsupported depth encoding {msg.encoding!r}')
        return None

    def _on_imu(self, msg):
        q = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self.imu_rpy = (roll, pitch)

    def _attitude_correct(self, rot_odom_cam, stamp):
        """Put the body's real roll and pitch back into the camera's rotation.

        *** WHY THIS IS NECESSARY, AND WHY IT IS THE RAMP BUG ***

        The EKF runs with two_d_mode: true (circe_nav/config/localization_ekf.yaml), so
        odom -> base_link carries yaw only - roll, pitch and z are forced to zero.
        Measured on the 9 deg test ramp: true IMU pitch 8.64 deg, TF pitch 0.00 deg.

        Everything downstream of that TF therefore believes the robot is level. For this
        node the consequence is severe, because it deprojects depth into odom THROUGH
        that transform. Nose-down 9 deg coming off a ramp, the flat floor ahead really
        does rise at 9 deg in base_link - that is what a level floor looks like from a
        tilted robot - and a level-assuming rotation leaves it rising in odom too. So
        the node writes a 9 deg upslope where there is flat concrete.

        Worse, the grid ACCUMULATES. Cells seen while climbing (body pitched up), while
        cresting (level) and while descending (pitched down) are averaged together at
        heights that disagree by tens of centimetres. The step and roughness tests then
        see a discontinuity that exists only in the frame maths, mark the cells lethal,
        and the terrain_layer hands the local costmap a wall exactly where the rover is
        about to drive. That is the "stuck half on the ramp" failure: the robot is not
        physically trapped, it is standing next to an obstacle it invented.

        The fix keeps the camera's static mount rotation from TF - which is correct,
        being a fixed URDF joint - and rebuilds only the body attitude, taking yaw from
        TF (the EKF does estimate that) and roll and pitch from the IMU (which is never
        flattened).

        The residual error is z: TF still reports 0 on a ramp, so heights observed from
        different elevations are still offset. That is much the smaller term - a 9 deg
        rotation error costs 0.6 m of height at the 4 m lookahead, an elevation offset
        costs 0.1-0.2 m - and removing it needs 3D odometry, not a rotation fix.
        """
        if self.imu_rpy is None:
            return rot_odom_cam
        try:
            tfb = self.tf_buffer.lookup_transform(
                self.cfg['global_frame'], self.cfg['robot_frame'],
                stamp, rclpy.duration.Duration(seconds=0.05))
        except Exception:
            return rot_odom_cam

        r_ob, _ = _tf_to_rt(tfb)              # odom <- base_link, yaw only
        r_bc = r_ob.T @ rot_odom_cam          # base_link <- camera, the static mount
        yaw = math.atan2(r_ob[1, 0], r_ob[0, 0])
        roll, pitch = self.imu_rpy

        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        return (rz @ ry @ rx) @ r_bc

    def _mark_driven(self, rx, ry):
        """Record the ground the robot is standing on as proven traversable.

        *** THIS IS THE ANTI-TRAP GUARANTEE. ***

        A terrain classifier can always be wrong, and when it is wrong underneath the
        robot the robot cannot move at all: every MPPI rollout starts inside a lethal
        cell, every trajectory scores as a collision, and the controller commands zero.
        Nothing in the recovery behaviours helps, because spin and backup have to cross
        the same cells.

        The one piece of evidence that outranks any threshold is that the robot is
        physically there. Cells its footprint has occupied are marked permanently
        drivable, so the path it climbed is guaranteed to still be open when it turns
        around and comes back down. This is what stops "stuck half on the ramp".

        The mask scrolls and is discarded with the grid in _recenter, so it does not
        accumulate stale claims about ground the robot left long ago.
        """
        if self.origin is None:
            return
        r = float(self.cfg['driven_radius'])
        cx = int((rx - self.origin[0]) / self.res)
        cy = int((ry - self.origin[1]) / self.res)
        rad = max(1, int(round(r / self.res)))
        x0, x1 = max(0, cx - rad), min(self.n_cells, cx + rad + 1)
        y0, y1 = max(0, cy - rad), min(self.n_cells, cy + rad + 1)
        if x0 >= x1 or y0 >= y1:
            return
        yy, xx = np.ogrid[y0:y1, x0:x1]
        self.driven[y0:y1, x0:x1] |= ((xx - cx) ** 2 + (yy - cy) ** 2) <= rad * rad

    def _recenter(self, rx, ry):
        """Scroll the grid so the robot stays near the middle, preserving overlap."""
        half = self.n_cells * self.res * 0.5
        # Snap to the cell lattice, otherwise every scroll resamples and blurs the map.
        ox = math.floor((rx - half) / self.res) * self.res
        oy = math.floor((ry - half) / self.res) * self.res

        if self.origin is None:
            self.origin = [ox, oy]
            return

        dx = int(round((ox - self.origin[0]) / self.res))
        dy = int(round((oy - self.origin[1]) / self.res))
        if dx == 0 and dy == 0:
            return

        for arr in (self.acc_n, self.acc_z, self.acc_z2, self.driven):
            arr[...] = np.roll(arr, (-dy, -dx), axis=(0, 1))
            # Zero whatever wrapped around, so old terrain does not reappear on the
            # opposite side of the map.
            if dx > 0:
                arr[:, -dx:] = 0
            elif dx < 0:
                arr[:, :-dx] = 0
            if dy > 0:
                arr[-dy:, :] = 0
            elif dy < 0:
                arr[:-dy, :] = 0

        self.origin = [ox, oy]

    # -- main path -------------------------------------------------------------------

    def _on_depth(self, msg):
        if self.k is None:
            self._warn_once('noinfo', 'waiting for depth/camera_info')
            return
        self.camera_frame = msg.header.frame_id

        if self.row_band is None:
            self.row_band = self._compute_row_band(msg.height)
            if self.row_band is None:
                self._warn_once(
                    'noband',
                    'no image rows see ground inside the ROI. Either TF to '
                    f'{self.cfg["robot_frame"]} is not available yet, or the camera '
                    'pitch/height put the ground outside roi_min_range..roi_max_range.')
                return
            self.get_logger().info(
                f'ground occupies rows {self.row_band[0]}-{self.row_band[1]} '
                f'of {msg.height}; ignoring the rest')

        depth = self._decode(msg)
        if depth is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.cfg['global_frame'], self.camera_frame,
                msg.header.stamp, rclpy.duration.Duration(seconds=0.05))
        except Exception as exc:
            self._warn_once('notf', f'no transform to {self.cfg["global_frame"]}: {exc}')
            return
        rot, trans = _tf_to_rt(tf)
        if self.cfg['use_imu_attitude']:
            rot = self._attitude_correct(rot, msg.header.stamp)

        stride = int(self.cfg['pixel_stride'])
        v_lo, v_hi = self.row_band
        # The row-band crop is the real "filter before the CPU": the top ~58% of the frame
        # cannot contain ground in the ROI, so it is never read.
        sub = depth[v_lo:v_hi + 1:stride, ::stride]
        vs = np.arange(v_lo, v_hi + 1, stride, dtype=np.float32)
        us = np.arange(0, depth.shape[1], stride, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        valid = np.isfinite(sub) & (sub > self.cfg['depth_min']) & (sub < self.cfg['depth_max'])
        if not valid.any():
            return

        z = sub[valid]
        fx, fy, cx, cy = self.k
        pts = np.stack([
            (uu[valid] - cx) * z / fx,
            (vv[valid] - cy) * z / fy,
            z,
        ])
        world = rot @ pts + trans[:, None]

        self._recenter(trans[0], trans[1])
        if self.cfg['clear_driven']:
            try:
                rb = self.tf_buffer.lookup_transform(
                    self.cfg['global_frame'], self.cfg['robot_frame'], rclpy.time.Time())
                self._mark_driven(rb.transform.translation.x, rb.transform.translation.y)
            except Exception:
                pass

        # Decay before adding this frame's evidence, so old observations fade.
        d = float(self.cfg['decay'])
        self.acc_n *= d
        self.acc_z *= d
        self.acc_z2 *= d

        gx = ((world[0] - self.origin[0]) / self.res).astype(np.int32)
        gy = ((world[1] - self.origin[1]) / self.res).astype(np.int32)
        inside = (gx >= 0) & (gx < self.n_cells) & (gy >= 0) & (gy < self.n_cells)
        if not inside.any():
            return
        gx, gy, wz = gx[inside], gy[inside], world[2][inside].astype(np.float32)

        # Flat index + bincount is the whole accumulation: one pass, no Python loop.
        flat = gy * self.n_cells + gx
        size = self.n_cells * self.n_cells
        self.acc_n += np.bincount(flat, minlength=size).reshape(self.acc_n.shape)
        self.acc_z += np.bincount(flat, weights=wz, minlength=size).reshape(self.acc_z.shape)
        self.acc_z2 += np.bincount(flat, weights=wz * wz, minlength=size).reshape(self.acc_z2.shape)

    # -- classification --------------------------------------------------------------

    def _classify(self):
        n = self.acc_n
        observed = n >= float(self.cfg['min_points_per_cell'])
        if not observed.any():
            return None

        with np.errstate(invalid='ignore', divide='ignore'):
            mean = np.where(observed, self.acc_z / np.maximum(n, 1e-6), np.nan)
            var = np.where(observed, self.acc_z2 / np.maximum(n, 1e-6) - mean * mean, 0.0)
        rough = np.sqrt(np.clip(var, 0.0, None))

        # *** NEVER LET AN UNOBSERVED CELL ACT AS A HEIGHT REFERENCE ***
        # An earlier version filled unobserved cells with the global median height and ran
        # the filters over that. On flat ground it looked fine; on a ramp it was a
        # disaster. Observation is patchy at range, so an elevated ramp cell would find
        # filled neighbours sitting at floor height and report the ramp's full elevation as
        # a STEP -- a drivable 9 deg ramp classified 59% lethal, with cost rising along its
        # length exactly as its height above the fake floor grew.
        #
        # The fix is to make unobserved cells invisible to each filter rather than
        # substituting a value: +BIG for the minimum filter, -BIG for the maximum, and
        # count-normalised smoothing for the slope. Every observed cell contains itself in
        # its own window, so low <= mean <= high always holds and no validity mask is
        # needed on the result.
        BIG = np.float32(1e6)
        obsf = observed.astype(np.float32)
        meanf = np.where(observed, mean, 0.0).astype(np.float32)

        # *** STEP AND SLOPE MUST BE MEASURED AT DIFFERENT SCALES ***
        # A step is a DISCONTINUITY; a slope is a GRADIENT. Measuring both over the same
        # window conflates them, and the failure is not subtle: with a single 9-cell
        # (0.45 m) window, a perfectly drivable 9 deg ramp rises 0.071 m across it and
        # trips a 0.06 m step threshold, so every ramp reads lethal.
        #
        # So: relief over a SMALL window (the scale at which a wheel actually meets an
        # edge), and slope from the gradient of a SMOOTHED field. Smoothing first also
        # stops a sharp kerb producing a spurious near-vertical slope reading.
        #
        # Sanity check of the two scales, at the shipped defaults:
        #                     step (3 cells, 0.10 m span)   slope (smoothed)
        #   9 deg ramp        0.158 x 0.10 = 0.016  free    0.157 rad  below warn
        #   31 deg ramp       0.600 x 0.10 = 0.060          0.541 rad  LETHAL
        #   0.08 m kerb       0.080         LETHAL          smoothed away
        sw = int(self.cfg['step_window'])
        low = ndimage.minimum_filter(
            np.where(observed, meanf, BIG), size=sw, mode='nearest')
        high = ndimage.maximum_filter(
            np.where(observed, meanf, -BIG), size=sw, mode='nearest')

        step_up = np.where(observed, meanf - low, 0.0)      # positive discontinuity
        step_down = np.where(observed, high - meanf, 0.0)   # drop-off below local ground

        # Count-normalised box smoothing: the mean of the OBSERVED cells in the window,
        # not of the window. den is the observed fraction, so it also tells us where the
        # gradient is trustworthy.
        lw = int(self.cfg['slope_window'])
        num = ndimage.uniform_filter(meanf, size=lw, mode='nearest')
        den = ndimage.uniform_filter(obsf, size=lw, mode='nearest')
        smooth = np.where(den > 1e-6, num / np.maximum(den, 1e-6), 0.0)
        gy, gx = np.gradient(smooth, self.res)
        slope = np.arctan(np.hypot(gx, gy))
        # A gradient computed across mostly-empty neighbourhood is noise, not terrain.
        slope = np.where(den >= float(self.cfg['min_slope_support']), slope, 0.0)

        cost = np.zeros_like(meanf, dtype=np.float32)
        np.maximum(cost, _grade(step_up, self.cfg['step_warn'], self.cfg['step_lethal']), cost)
        np.maximum(cost, _grade(slope, self.cfg['slope_warn'], self.cfg['slope_lethal']), cost)
        np.maximum(cost, _grade(step_down, self.cfg['drop_warn'], self.cfg['drop_lethal']), cost)
        # Roughness is graded only -- it distinguishes gravel from tarmac and should
        # produce a preference, never a barrier.
        rough_cost = _grade(rough, self.cfg['rough_warn'], self.cfg['rough_max'])
        rough_cost = rough_cost * (float(self.cfg['rough_max_cost']) / 100.0)
        np.maximum(cost, rough_cost, cost)

        lethal = (
            (step_up >= self.cfg['step_lethal'])
            | (slope >= self.cfg['slope_lethal'])
            | (step_down >= self.cfg['drop_lethal'])
        ) & (n >= float(self.cfg['min_points_for_lethal']))

        # Confidence: a distant, thinly-observed cell nudges the planner; it does not stop
        # it. Lethal cells bypass this, having already passed min_points_for_lethal.
        conf = np.clip(n / float(self.cfg['full_confidence_points']), 0.0, 1.0)
        out = np.clip(cost * conf * 100.0, 0, 99).astype(np.int8)
        out[lethal] = int(self.cfg['lethal_cost'])
        out[~observed] = -1

        # Ground the robot has stood on outranks every threshold above. See _mark_driven:
        # without this, one bad classification under the wheels stops the rover dead with
        # no recovery, because spin and backup have to cross the same cells.
        if self.cfg['clear_driven']:
            out[self.driven] = 0
        return out

    def _publish(self):
        if self.grid_pub.get_subscription_count() == 0:
            return
        if self.origin is None:
            self._publish_unknown()
            return
        data = self._classify()
        if data is None:
            return

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['global_frame']
        msg.info.resolution = self.res
        msg.info.width = self.n_cells
        msg.info.height = self.n_cells
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()
        self.grid_pub.publish(msg)

    def _publish_unknown(self):
        """An all-unknown grid around the robot, until the first depth frame lands.

        *** WITHOUT THIS, NO DEPTH MEANS NO DRIVING. ***
        The local_costmap's terrain_layer is a StaticLayer, and on Jazzy a StaticLayer
        that has never received a map holds the whole costmap "not current":
        controller_server then aborts every goal after 0.3 s with "Costmap timed out
        waiting for update". Measured in circe_sim_nav -- the planner found the path,
        and the rover never moved. So this node only "degraded gracefully" once it had
        published at least once; use_camera:=false (the documented lidar-only setup), a
        D456 that never starts, or a TF fault meant Nav2 could plan but never drive.

        Unknown (-1) cells contribute nothing: the layer reads them as free or as no
        information depending on track_unknown_space, and with use_maximum it can only
        ever RAISE a cell's cost. This changes no cost anywhere; it lets the layer go
        current.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.cfg['global_frame'], self.cfg['robot_frame'], rclpy.time.Time())
        except Exception:
            return      # no odom yet; the costmap cannot be current without it anyway
        if self._unknown is None:
            self._unknown = [-1] * (self.n_cells * self.n_cells)
        half = self.n_cells * self.res * 0.5
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg['global_frame']
        msg.info.resolution = self.res
        msg.info.width = self.n_cells
        msg.info.height = self.n_cells
        msg.info.origin.position.x = tf.transform.translation.x - half
        msg.info.origin.position.y = tf.transform.translation.y - half
        msg.info.origin.orientation.w = 1.0
        msg.data = self._unknown
        self.grid_pub.publish(msg)


def main():
    rclpy.init()
    node = TerrainNode()
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
