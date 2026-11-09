"""Global localization: find the robot's pose in a saved map with NO initial guess.

Exhaustive correlative scan matching (coarse -> fine) of the current lidar scan against
the map's likelihood field, over every free position and every heading. Then:

  mode 'initialpose'  -> publish /initialpose (slam_toolbox localization mode or AMCL)
  mode 'deserialize'  -> call /slam_toolbox/deserialize_map at that pose, so a NEW mapping
                         session continues the OLD map from wherever the robot was placed

It also monitors localization quality (how well the live scan fits the map at the current
map->base_link pose) on /localization/score and can re-localize automatically when lost.

Services
  /localization/global_localize  std_srvs/Trigger
Topics
  /localization/score   std_msgs/Float32   0..1 (fraction of scan explained by the map)
  /localization/status  std_msgs/String    JSON with the last result
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .grid_utils import GridMap, scan_to_points, transform_points, wrap, yaw_from_quat


def _score_positions(F, grid, pts_rot, rows, cols):
    """Mean likelihood of rotated points (metres, base frame) placed at many cell positions."""
    dr = pts_rot[:, 1] / grid.res
    dc = pts_rot[:, 0] / grid.res
    rr = np.floor(rows[:, None] + 0.5 + dr[None, :]).astype(np.int32)
    cc = np.floor(cols[:, None] + 0.5 + dc[None, :]).astype(np.int32)
    ok = (rr >= 0) & (rr < grid.h) & (cc >= 0) & (cc < grid.w)
    v = F[np.clip(rr, 0, grid.h - 1), np.clip(cc, 0, grid.w - 1)]
    return np.where(ok, v, 0.0).mean(axis=1)


def _rot(pts, th):
    c, s = math.cos(th), math.sin(th)
    return np.stack([c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1]], axis=1)


def _subsample(pts, n):
    if len(pts) <= n:
        return pts
    return pts[np.linspace(0, len(pts) - 1, n).astype(int)]


def global_search(grid, pts, robot_radius=0.12, coarse_step=0.2, coarse_angle_deg=4.0,
                  n_coarse=120, n_fine=360, keep=15):
    """Returns list of dicts {x, y, yaw, score} best first."""
    F_coarse = grid.likelihood_field(max(coarse_step * 0.75, 0.1))
    F_fine = grid.likelihood_field(0.05)
    clear = grid.dist_to_occupied()
    cand = grid.free & (clear > robot_radius)
    stride = max(1, int(round(coarse_step / grid.res)))
    sub = np.zeros_like(cand)
    sub[stride // 2::stride, stride // 2::stride] = True
    rows, cols = np.nonzero(cand & sub)
    if len(rows) == 0:
        return []
    pc, pf = _subsample(pts, n_coarse), _subsample(pts, n_fine)

    # --- coarse: every candidate cell x every heading
    angles = np.arange(-math.pi, math.pi, math.radians(coarse_angle_deg))
    S = np.stack([_score_positions(F_coarse, grid, _rot(pc, a), rows, cols) for a in angles])
    order = np.argsort(S, axis=None)[::-1]
    seeds = []
    for k in order[:4000]:
        ai, ci = np.unravel_index(k, S.shape)
        r, c, a = rows[ci], cols[ci], angles[ai]
        if all(math.hypot(r - s[0], c - s[1]) * grid.res > 0.4 or
               abs(wrap(a - s[2])) > math.radians(15) for s in seeds):
            seeds.append((r, c, a))
        if len(seeds) >= keep:
            break

    # --- refine each seed on the sharp field
    def refine(r0, c0, a0, span_cells, ang_span, ang_step):
        best = (-1.0, r0, c0, a0)
        dr, dc = np.meshgrid(np.arange(-span_cells, span_cells + 1),
                             np.arange(-span_cells, span_cells + 1), indexing='ij')
        rr, cc = (r0 + dr).ravel(), (c0 + dc).ravel()
        okc = (rr >= 0) & (rr < grid.h) & (cc >= 0) & (cc < grid.w)
        rr, cc = rr[okc], cc[okc]
        for a in a0 + np.arange(-ang_span, ang_span + 1e-9, ang_step):
            s = _score_positions(F_fine, grid, _rot(pf, a), rr, cc)
            i = int(np.argmax(s))
            if s[i] > best[0]:
                best = (float(s[i]), int(rr[i]), int(cc[i]), wrap(a))
        return best

    out = []
    for r, c, a in seeds:
        s, r, c, a = refine(r, c, a, stride, math.radians(coarse_angle_deg), math.radians(1.0))
        s, r, c, a = refine(r, c, a, 2, math.radians(1.0), math.radians(0.25))
        x, y = grid.to_world(r, c)
        out.append({'x': float(x), 'y': float(y), 'yaw': float(a), 'score': s})
    out.sort(key=lambda d: -d['score'])
    return out


class GlobalLocalizer(Node):
    def __init__(self):
        super().__init__('global_localizer')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.mode = p('mode', 'initialpose')            # initialpose | deserialize
        self.map_yaml = p('map_yaml', '')               # '' = use the /map topic
        self.posegraph = p('posegraph_file', '')        # for mode deserialize
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        self.auto_start = p('auto_start', True)
        self.auto_delay = p('auto_start_delay', 3.0)
        self.max_range = p('max_range', 8.0)
        self.robot_radius = p('robot_radius', 0.12)
        self.coarse_step = p('coarse_step', 0.2)
        self.coarse_angle = p('coarse_angle_deg', 4.0)
        self.min_score = p('min_score', 0.45)
        self.min_misfit_ratio = p('min_misfit_ratio', 1.5)
        self.monitor = p('monitor', True)
        self.lost_score = p('lost_score', 0.35)
        self.lost_time = p('lost_time', 5.0)
        self.auto_recover = p('auto_recover', True)

        self.grid = None
        self.F_monitor = None
        self.scan = None
        self.laser_tf = None
        self.busy = threading.Lock()
        self.last = {'state': 'waiting for map and scan'}
        self.low_since = None
        self.started = time.time()
        self.done_auto = not self.auto_start

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.map_yaml:
            self._set_grid(GridMap.from_yaml(self.map_yaml))
            self.get_logger().info(f'Loaded map {self.map_yaml}')
        else:
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, 'map', lambda m: self._set_grid(GridMap.from_msg(m)),
                                     qos, callback_group=cb)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data, callback_group=cb)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.pub_score = self.create_publisher(Float32, 'localization/score', 10)
        self.pub_status = self.create_publisher(String, 'localization/status', 10)
        self.create_service(Trigger, 'localization/global_localize', self._srv, callback_group=cb)
        if self.mode == 'deserialize':
            from slam_toolbox.srv import DeserializePoseGraph
            self.DPG = DeserializePoseGraph
            self.deser = self.create_client(DeserializePoseGraph, 'slam_toolbox/deserialize_map',
                                            callback_group=cb)
        self.create_timer(1.0, self._tick, callback_group=cb)

    def _set_grid(self, g):
        self.grid = g
        self.F_monitor = g.likelihood_field(0.08)

    # ------------------------------------------------------------------ helpers
    def _scan_points_base(self):
        s = self.scan
        if s is None:
            return None
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, s.header.frame_id, Time())
                self.laser_tf = (t.transform.translation.x, t.transform.translation.y,
                                 yaw_from_quat(t.transform.rotation))
            except Exception:
                return None
        return transform_points(scan_to_points(s, self.max_range, 0.1), *self.laser_tf)

    def _current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quat(t.transform.rotation))

    def _status(self, **kw):
        self.last = kw
        self.pub_status.publish(String(data=json.dumps(kw)))

    # ------------------------------------------------------------------ main work
    def localize(self):
        if not self.busy.acquire(blocking=False):
            return False, 'already running'
        try:
            if self.grid is None:
                return False, 'no map yet'
            pts = self._scan_points_base()
            if pts is None or len(pts) < 50:
                return False, 'no usable scan yet (or missing base_link->laser TF)'
            t0 = time.time()
            self._status(state='searching')
            res = global_search(self.grid, pts, self.robot_radius, self.coarse_step,
                                self.coarse_angle)
            if not res:
                return False, 'no free space in map'
            best = res[0]
            second = next((r for r in res[1:]
                           if math.hypot(r['x'] - best['x'], r['y'] - best['y']) > 0.5
                           or abs(wrap(r['yaw'] - best['yaw'])) > math.radians(20)), None)
            # misfit = fraction of the scan the map does NOT explain. In a rectangular room a
            # wrong pose can still match most wall points, so compare misfits, not scores.
            misfit = max(1.0 - best['score'], 0.01)
            ratio = (1.0 - second['score']) / misfit if second else 99.0
            dt = time.time() - t0
            msg = (f"x={best['x']:.2f} y={best['y']:.2f} yaw={math.degrees(best['yaw']):.0f}deg "
                   f"score={best['score']:.2f} (next best fits {ratio:.1f}x worse, {dt:.1f}s)")
            if best['score'] < self.min_score:
                self._status(state='failed', reason='low score', **best)
                return False, ('Scan does not match the map well: ' + msg +
                               '. Is this the right map? Move somewhere with more structure.')
            if ratio < self.min_misfit_ratio:
                self._status(state='ambiguous', best=best, second=second)
                return False, ('Ambiguous (symmetric area?): ' + msg + f". Alternative: x={second['x']:.2f} "
                               f"y={second['y']:.2f}. Rotate/move the robot a little and retry.")
            self._apply(best)
            self._status(state='localized', ratio=ratio, seconds=dt, **best)
            self.low_since = None
            return True, 'Localized: ' + msg
        finally:
            self.busy.release()

    def _apply(self, pose):
        if self.mode == 'deserialize':
            if not self.deser.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('slam_toolbox/deserialize_map not available')
            req = self.DPG.Request()
            req.filename = self.posegraph
            req.match_type = self.DPG.Request.START_AT_GIVEN_POSE
            req.initial_pose.x, req.initial_pose.y = pose['x'], pose['y']
            req.initial_pose.theta = pose['yaw']
            self.deser.call_async(req)
            self.get_logger().info(f'Continuing map {self.posegraph} from the found pose')
            self.mode = 'initialpose'   # later re-localizations only correct the pose
            return
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = pose['x'], pose['y']
        m.pose.pose.orientation.z = math.sin(pose['yaw'] / 2)
        m.pose.pose.orientation.w = math.cos(pose['yaw'] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(2) ** 2
        self.pub_init.publish(m)

    def _srv(self, req, resp):
        try:
            resp.success, resp.message = self.localize()
        except Exception as e:  # noqa: BLE001
            resp.success, resp.message = False, f'error: {e}'
        self.get_logger().info(resp.message)
        return resp

    def _tick(self):
        if not self.done_auto and time.time() - self.started > self.auto_delay \
                and self.grid is not None and self.scan is not None:
            self.done_auto = True
            ok, msg = self.localize()
            self.get_logger().info(('' if ok else 'Auto-localize failed: ') + msg)
            if not ok:
                self.done_auto = False
                self.started = time.time() + 2.0  # retry a bit later
        if not self.monitor or self.grid is None or self.mode == 'deserialize':
            return
        pose, pts = self._current_pose(), self._scan_points_base()
        if pose is None or pts is None or len(pts) < 30:
            return
        w = transform_points(pts, *pose)
        r, c = self.grid.to_cell(w[:, 0], w[:, 1])
        ok = self.grid.inside(r, c)
        score = float(np.where(ok, self.F_monitor[np.clip(r, 0, self.grid.h - 1),
                                                  np.clip(c, 0, self.grid.w - 1)], 0).mean())
        self.pub_score.publish(Float32(data=score))
        if score < self.lost_score:
            self.low_since = self.low_since or time.time()
            if self.auto_recover and time.time() - self.low_since > self.lost_time:
                self.get_logger().warn(f'Localization looks lost (score {score:.2f}) - re-localizing')
                ok, msg = self.localize()
                self.get_logger().info(msg)
                self.low_since = None if ok else time.time()
        else:
            self.low_since = None


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizer()
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
