"""Obstacle + cliff detection from a downward-tilted VL53L5CX.

Every valid zone is turned into a 3D point in the robot base frame (floor = z 0):
  point higher than obstacle_min_height and closer than obstacle_max_distance -> OBSTACLE
  point lower than -cliff_min_drop (ray went past an edge)                     -> CLIFF
  zone should see floor within cliff_max_floor_range but returns nothing        -> CLIFF
                                                                    (if invalid_is_cliff)
Subscribes  tof/distance_mm (Int16MultiArray, -1 = invalid)
Publishes   tof/obstacle_detected (Bool)   tof/cliff_detected (Bool)
            tof/nearest_obstacle  (Float32, m ahead of base frame origin, inf = none)
            tof/zone_class (Int8MultiArray: 0 clear, 1 obstacle, 2 cliff, -1 unknown)
            tof/points  (PointCloud2, all valid points, base frame)
            tof/hazard_points (PointCloud2, obstacles + cliff markers, for a Nav2 costmap)
Service     tof/calibrate_floor (std_srvs/Trigger): with the robot on flat, clear floor,
            measures the real mounting height + tilt and saves them.
"""
import os
import threading
import time
import warnings
from collections import deque

import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, Header, Int16MultiArray, Int8MultiArray, \
    MultiArrayDimension
from std_srvs.srv import Trigger

from .geometry import TofGeometry, fit_floor

CLEAR, OBSTACLE, CLIFF, UNKNOWN = 0, 1, 2, -1


class Debounce:
    """Turns on after `frames` consecutive positives, off after `frames` consecutive negatives."""

    def __init__(self, frames):
        self.frames, self.count, self.state = max(1, frames), 0, False

    def update(self, raw):
        self.count = self.count + 1 if raw != self.state else 0
        if self.count >= self.frames:
            self.state, self.count = raw, 0
        return self.state


class HazardNode(Node):
    def __init__(self):
        super().__init__('tof_hazard_node')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.base_frame = p('base_frame', 'base_link')
        self.height = p('mount_height_m', 0.10)
        self.tilt_deg = p('tilt_down_deg', 20.0)
        self.model = p('range_model', 'radial')
        self.fov = p('fov_deg', 45.0)
        self.x_off = p('mount_x_m', 0.0)
        self.y_off = p('mount_y_m', 0.0)
        self.obst_min_h = p('obstacle_min_height', 0.03)
        self.obst_max_h = p('obstacle_max_height', 0.50)
        self.obst_max_d = p('obstacle_max_distance', 0.60)
        self.cliff_drop = p('cliff_min_drop', 0.04)
        self.cliff_max_floor = p('cliff_max_floor_range', 0.80)
        self.invalid_is_cliff = p('invalid_is_cliff', True)
        self.min_zones = p('min_zones', 2)
        # obstacles need this many zones to agree (cliffs keep min_zones)
        self.obst_min_zones = p('obstacle_min_zones', self.min_zones)
        # per-zone median over the last N frames (1 = off): one bad reading can't trigger
        self.smooth_n = max(1, int(p('smoothing_frames', 3)))
        self._hist = deque(maxlen=self.smooth_n)
        self.debounce_n = p('debounce_frames', 2)
        self.cliff_marker_z = p('cliff_marker_height', 0.05)
        self.calib_file = os.path.expanduser(p('calibration_file', '~/.ros/vl53l5cx_floor.yaml'))
        self.calib_frames = p('calibration_frames', 30)
        if p('use_calibration', True):
            self._load_calibration()

        self.geo = None
        self.obst_db = Debounce(self.debounce_n)
        self.cliff_db = Debounce(self.debounce_n)
        self._collect = None
        self._lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self.create_subscription(Int16MultiArray, 'tof/distance_mm', self._cb, 10,
                                 callback_group=cb)
        self.pub_obst = self.create_publisher(Bool, 'tof/obstacle_detected', 10)
        self.pub_cliff = self.create_publisher(Bool, 'tof/cliff_detected', 10)
        self.pub_near = self.create_publisher(Float32, 'tof/nearest_obstacle', 10)
        self.pub_class = self.create_publisher(Int8MultiArray, 'tof/zone_class', 10)
        self.pub_pts = self.create_publisher(PointCloud2, 'tof/points', 10)
        self.pub_haz = self.create_publisher(PointCloud2, 'tof/hazard_points', 10)
        self.create_service(Trigger, 'tof/calibrate_floor', self._calibrate, callback_group=cb)

    # ---------------------------------------------------------------- calibration
    def _load_calibration(self):
        if not os.path.exists(self.calib_file):
            self.get_logger().info(f'No floor calibration at {self.calib_file}; using parameters. '
                                   'Run: ros2 service call /tof/calibrate_floor std_srvs/srv/Trigger')
            return
        with open(self.calib_file) as f:
            c = yaml.safe_load(f) or {}
        self.height = float(c.get('mount_height_m', self.height))
        self.tilt_deg = float(c.get('tilt_down_deg', self.tilt_deg))
        self.model = c.get('range_model', self.model)
        self.get_logger().info(f'Loaded floor calibration from {self.calib_file}')

    def _calibrate(self, req, resp):
        with self._lock:
            self._collect = []
        deadline = time.time() + 10.0 + self.calib_frames / 5.0
        while time.time() < deadline:
            with self._lock:
                if len(self._collect) >= self.calib_frames:
                    break
            time.sleep(0.05)
        with self._lock:
            frames, self._collect = self._collect, None
        if len(frames) < 5:
            resp.success, resp.message = False, 'No data on tof/distance_mm - is the driver running?'
            return resp

        stack = np.stack(frames)  # (k, n, n) metres, NaN invalid
        valid_frac = np.mean(np.isfinite(stack), axis=0)
        with np.errstate(all='ignore'):
            med = np.nanmedian(stack, axis=0)
        med[valid_frac < 0.8] = np.nan
        n = med.shape[0]
        fits = fit_floor(med, n, self.fov)
        flipped = fit_floor(med[::-1], n, self.fov)
        if not fits:
            resp.success = False
            resp.message = 'Too few valid floor zones - point the sensor more at the floor.'
            return resp
        res, h, t, model = fits[0]
        if res > 0.05:
            msg = (f'Poor floor fit (residual {res * 100:.1f}%). Make sure only flat floor is in '
                   'view (clear ~1 m ahead, no walls).')
            if flipped and flipped[0][0] < res * 0.5:
                msg += ' The rows look upside-down: set flip_vertical: true in vl53l5cx.yaml.'
            resp.success, resp.message = False, msg
            return resp

        self.height, self.tilt_deg, self.model = h, t, model
        self.geo = None  # rebuild with new numbers
        os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
        with open(self.calib_file, 'w') as f:
            yaml.safe_dump({'mount_height_m': round(h, 4), 'tilt_down_deg': round(t, 2),
                            'range_model': model, 'fit_residual': round(res, 4),
                            'calibrated': time.strftime('%Y-%m-%d %H:%M:%S')}, f)
        others = ', '.join(f'{m}: {r * 100:.1f}%' for r, _, _, m in fits)
        resp.success = True
        resp.message = (f'Measured height {h * 100:.1f} cm, tilt {t:.1f} deg below horizontal, '
                        f'{model} ranges (fit residual {others}). Saved to {self.calib_file}')
        self.get_logger().info(resp.message)
        return resp

    # ---------------------------------------------------------------- detection
    def _cb(self, msg):
        n = msg.layout.dim[0].size if msg.layout.dim else 8
        raw = np.array(msg.data, dtype=np.float32).reshape(n, -1)
        r = np.where(raw > 0, raw / 1000.0, np.nan)

        with self._lock:
            if self._collect is not None:
                self._collect.append(r)
                return  # don't trigger hazards while calibrating

        if self.smooth_n > 1:
            if self._hist and self._hist[-1].shape != r.shape:
                self._hist.clear()
            self._hist.append(r)
            stack = np.stack(self._hist)
            valid_n = np.isfinite(stack).sum(axis=0)
            with warnings.catch_warnings():   # all-NaN zones -> NaN, without a warning
                warnings.simplefilter('ignore', RuntimeWarning)
                med = np.nanmedian(stack, axis=0)
            # a zone counts as "no return" only if most recent frames had no return
            r = np.where(valid_n * 2 > len(self._hist), med, np.nan)

        if self.geo is None or self.geo.n != n:
            self.geo = TofGeometry(n, self.height, self.tilt_deg, self.model, self.fov,
                                   self.x_off, self.y_off)
            self.get_logger().info('Zone coverage:\n' + self.geo.describe())
        g = self.geo

        pts = g.points(r)
        valid = np.isfinite(r)
        z, fwd = pts[..., 2], pts[..., 0]
        floor_expected = g.floor_range < self.cliff_max_floor

        cls = np.full((n, n), UNKNOWN, dtype=np.int8)
        cls[valid] = CLEAR
        obst = valid & (z > self.obst_min_h) & (z < self.obst_max_h) & (fwd < self.obst_max_d)
        cliff = valid & (z < -self.cliff_drop) & floor_expected
        if self.invalid_is_cliff:
            cliff |= ~valid & floor_expected
        cls[cliff] = CLIFF
        cls[obst] = OBSTACLE

        obstacle = self.obst_db.update(int(obst.sum()) >= self.obst_min_zones)
        cliff_on = self.cliff_db.update(int(cliff.sum()) >= self.min_zones)
        nearest = float(fwd[obst].min()) if obst.any() else float('inf')

        self.pub_obst.publish(Bool(data=bool(obstacle)))
        self.pub_cliff.publish(Bool(data=bool(cliff_on)))
        self.pub_near.publish(Float32(data=nearest))

        c = Int8MultiArray()
        c.layout.dim = [MultiArrayDimension(label='rows', size=n, stride=n * n),
                        MultiArrayDimension(label='cols', size=n, stride=n)]
        c.data = cls.flatten().tolist()
        self.pub_class.publish(c)

        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.base_frame)
        self.pub_pts.publish(point_cloud2.create_cloud_xyz32(header, pts[valid].tolist()))
        haz = list(pts[obst])
        if cliff.any():
            # Mark where the floor *should* have been, raised so a costmap keeps it.
            cp = g.floor_points()[cliff]
            cp[:, 2] = self.cliff_marker_z
            haz += list(cp)
        self.pub_haz.publish(point_cloud2.create_cloud_xyz32(
            header, np.asarray(haz, dtype=np.float32).reshape(-1, 3).tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = HazardNode()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
