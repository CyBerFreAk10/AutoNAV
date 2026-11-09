"""Boundary-first exploration.

Phase 1  OUTER   find the nearest wall and follow it (wall on the right by default) until
                 the robot is back where it started (closed loop). The loop's winding
                 direction tells whether it was the outer boundary (CCW for a right-hand wall)
                 or a free-standing object (CW); objects are recorded and the robot heads
                 off into open space to find the real boundary.
Phase 2  INTERIOR  every obstacle cluster INSIDE the boundary loop that has not been
                 circled yet is an inner structure: plan a path to a spot beside it, turn so
                 it is on the wall side, and follow it until that loop closes too.
                 Unseen gaps (frontiers) inside the boundary are visited afterwards.
DONE             stop and save the map + pose graph to maps_dir.

Inputs : /scan, /map (slam_toolbox), TF map->base_link, optional /tof/cliff_detected,
         /tof/obstacle_detected
Outputs: /cmd_vel, /explorer/state (JSON), /explorer/path, /explorer/boundary
Services: /explorer/start, /explorer/stop (std_srvs/Trigger)

discrete_drive: the rover only knows forward / back / spin left / spin right, so every
command is either pure translation or pure rotation; gentle turns are produced by mixing
forward and turn ticks in the right proportion (sigma-delta).
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy import ndimage
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .grid_utils import GridMap, Planner, scan_to_points, transform_points, wrap, yaw_from_quat


def signed_area(xy):
    x, y = xy[:, 0], xy[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def points_in_polygon(pts, poly):
    """Even-odd rule, vectorised over points."""
    x, y = pts[:, 0][:, None], pts[:, 1][:, None]
    x1, y1 = poly[:, 0][None, :], poly[:, 1][None, :]
    x2, y2 = np.roll(poly[:, 0], -1)[None, :], np.roll(poly[:, 1], -1)[None, :]
    cond = (y1 > y) != (y2 > y)
    xint = (x2 - x1) * (y - y1) / np.where(y2 - y1 == 0, 1e-12, y2 - y1) + x1
    return (np.sum(cond & (x < xint), axis=1) % 2) == 1


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        self.robot_radius = p('robot_radius', 0.15)
        self.side = -1 if p('wall_side', 'right') == 'right' else 1
        self.wall_dist = p('wall_dist', 0.35)
        self.front_stop = p('front_stop', 0.40)
        self.lin_speed = p('lin_speed', 0.15)
        self.ang_speed = p('ang_speed', 0.6)
        self.kp = p('wall_kp', 3.0)
        self.kd = p('wall_kd', 1.0)
        self.discrete = p('discrete_drive', True)
        self.hz = p('control_hz', 10.0)
        self.loop_min = p('loop_min_length', 2.5)
        self.loop_radius = p('loop_close_radius', 0.35)
        self.max_follow = p('max_follow_length', 80.0)
        self.island_min_area = p('island_min_area', 0.02)
        self.frontier_min = p('frontier_min_cells', 8)
        self.goal_tol = p('goal_tolerance', 0.2)
        self.use_tof = p('use_tof', True)
        self.maps_dir = os.path.expanduser(p('maps_dir', '~/maps'))
        self.map_name = p('map_name', '')
        self.autostart = p('autostart', False)
        self.add_on_set_parameters_callback(self._on_params)

        self.scan = None
        self.laser_tf = None
        self.grid = None
        self.cliff = self.tof_obstacle = False
        self.sd_acc = 0.0
        self.reset()

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data, callback_group=cb)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map_msg', m),
                                 qos, callback_group=cb)
        self.map_msg = None
        if self.use_tof:
            self.create_subscription(Bool, 'tof/cliff_detected',
                                     lambda m: setattr(self, 'cliff', m.data), 10, callback_group=cb)
            self.create_subscription(Bool, 'tof/obstacle_detected',
                                     lambda m: setattr(self, 'tof_obstacle', m.data), 10,
                                     callback_group=cb)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_state = self.create_publisher(String, 'explorer/state', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_path = self.create_publisher(Path, 'explorer/path', latched)
        self.pub_boundary = self.create_publisher(Path, 'explorer/boundary', latched)
        self.create_service(Trigger, 'explorer/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, 'explorer/stop', self._srv_stop, callback_group=cb)
        self.create_timer(1.0 / self.hz, self._tick)
        if self.autostart:
            self.state = 'FIND_WALL'
        self.get_logger().info(f'Explorer ready ({"autostart" if self.autostart else "call /explorer/start"}),'
                               f' wall on the {"right" if self.side < 0 else "left"}, '
                               f'{"discrete" if self.discrete else "continuous"} drive')

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == 'map_name':
                self.map_name = prm.value or ''
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ bookkeeping
    def reset(self):
        self.state = 'IDLE'
        self.phase = 'outer'
        self.follow_target = 'outer'     # 'outer' or dict(island)
        self.follow_path = []
        self.follow_len = 0.0
        self.follow_start = None
        self.outer_poly = None
        self.outer_attempts = 0
        self.done_pts = np.zeros((0, 2))
        self.islands_done = 0
        self.blacklist = []
        self.goto = None
        self.turn_yaw = None
        self.recover = []
        self.stuck_ref = None
        self.prev_err = None
        self.info = ''
        self.last_plan = {}

    def _set(self, state, info=''):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}' + (f' ({info})' if info else ''))
        self.state, self.info = state, info

    def _srv_start(self, req, resp):
        self.reset()
        self._set('FIND_WALL', 'started')
        resp.success, resp.message = True, 'exploration started'
        return resp

    def _srv_stop(self, req, resp):
        self._set('STOPPED', 'stop requested')
        self.drive(0, 0)
        resp.success, resp.message = True, 'stopped'
        return resp

    # ------------------------------------------------------------------ sensing
    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quat(t.transform.rotation))

    def scan_polar(self):
        s = self.scan
        if s is None:
            return None
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, s.header.frame_id, Time())
            except Exception:
                return None
            self.laser_tf = (t.transform.translation.x, t.transform.translation.y,
                             yaw_from_quat(t.transform.rotation))
        pts = transform_points(scan_to_points(s, 8.0, 0.05), *self.laser_tf)
        return np.degrees(np.arctan2(pts[:, 1], pts[:, 0])), np.hypot(pts[:, 0], pts[:, 1])

    @staticmethod
    def sector(polar, a0, a1, default=10.0):
        ang, rng = polar
        lo, hi = min(a0, a1), max(a0, a1)
        m = (ang >= lo) & (ang <= hi)
        return float(rng[m].min()) if m.any() else default

    # ------------------------------------------------------------------ actuation
    def drive(self, lin, ang):
        t = Twist()
        if self.discrete:
            if abs(lin) < 1e-3 and abs(ang) < 1e-3:
                self.sd_acc = 0.0
            elif abs(lin) < 1e-3:
                t.angular.z = float(math.copysign(self.ang_speed, ang))
            elif lin < 0:
                t.linear.x = -self.lin_speed
            else:
                # sigma-delta: fraction of ticks spent turning ~ |ang| / ang_speed
                self.sd_acc += max(-1.0, min(1.0, ang / self.ang_speed))
                if self.sd_acc > 0.5:
                    t.angular.z, self.sd_acc = self.ang_speed, self.sd_acc - 1.0
                elif self.sd_acc < -0.5:
                    t.angular.z, self.sd_acc = -self.ang_speed, self.sd_acc + 1.0
                else:
                    t.linear.x = self.lin_speed
        else:
            t.linear.x = float(max(-self.lin_speed, min(self.lin_speed, lin)))
            t.angular.z = float(max(-self.ang_speed, min(self.ang_speed, ang)))
        self.pub_cmd.publish(t)

    # ------------------------------------------------------------------ main loop
    def _tick(self):
        st = self.state
        if st in ('IDLE', 'DONE', 'STOPPED'):
            if st != 'IDLE':
                self.drive(0, 0)
            return self._publish_state()
        pose, polar = self.pose(), self.scan_polar()
        if pose is None or polar is None:
            self.drive(0, 0)
            self.info = 'waiting for map->base_link TF and /scan'
            return self._publish_state()

        front = self.sector(polar, -20, 20)
        if self.tof_obstacle:
            front = min(front, self.robot_radius + 0.05)

        if self.cliff and not self.recover:
            self.get_logger().warn('Cliff! backing up')
            self._start_recovery(away=True)
        if self.recover:
            cmd, until = self.recover[0]
            if time.time() > until:
                self.recover.pop(0)
            self.drive(*cmd)
            return self._publish_state()
        if self._stuck(pose):
            self.get_logger().warn('Stuck - recovering')
            self._start_recovery(away=False)
            if self.state == 'GOTO' and self.goto:
                self.goto['stuck'] = self.goto.get('stuck', 0) + 1
            return

        handler = {'FIND_WALL': self._find_wall, 'FOLLOW': self._follow, 'ESCAPE': self._escape,
                   'PLAN': self._plan, 'GOTO': self._goto, 'TURN_TO': self._turn_to}[st]
        handler(pose, polar, front)
        self._publish_state()

    def _start_recovery(self, away):
        now = time.time()
        turn = -self.side * self.ang_speed if away else self.side * self.ang_speed
        self.recover = [((-self.lin_speed, 0.0), now + 1.0), ((0.0, turn), now + 2.2)]
        self.stuck_ref = None

    def _stuck(self, pose):
        if self.state not in ('FOLLOW', 'GOTO', 'FIND_WALL', 'ESCAPE'):
            self.stuck_ref = None
            return False
        now = time.time()
        if self.stuck_ref is None:
            self.stuck_ref = (pose, now)
            return False
        p0, t0 = self.stuck_ref
        moved = math.hypot(pose[0] - p0[0], pose[1] - p0[1]) > 0.05 or abs(wrap(pose[2] - p0[2])) > 0.2
        if moved:
            self.stuck_ref = (pose, now)
            return False
        return now - t0 > 8.0

    # ------------------------------------------------------------------ states
    def _find_wall(self, pose, polar, front):
        if front < self.wall_dist + 0.15:
            return self._begin_follow('outer', pose)
        ang, rng = polar
        near = rng < 3.0
        if not near.any():
            return self.drive(self.lin_speed, 0.0)
        target = float(ang[near][np.argmin(rng[near])])
        err = math.radians(target)
        if abs(err) > math.radians(20):
            self.drive(0.0, math.copysign(self.ang_speed, err))
        else:
            self.drive(self.lin_speed, 1.5 * err)
        self.info = f'heading to nearest wall {rng[near].min():.2f} m away'

    def _begin_follow(self, target, pose):
        self.follow_target = target
        self.follow_path = [pose[:2]]
        self.follow_len = 0.0
        self.follow_start = pose
        self.prev_err = None
        name = 'outer boundary' if target == 'outer' else f'structure #{self.islands_done + 1}'
        self._set('FOLLOW', name)

    def _wall_control(self, polar, front):
        s = self.side
        fside = self.sector(polar, s * 25, s * 60)
        side = self.sector(polar, s * 60, s * 120)
        if front < self.front_stop or fside < self.wall_dist * 0.75:
            self.prev_err = None
            return 0.0, -s * self.ang_speed           # inner corner / too close: turn away
        if side > self.wall_dist * 1.8 and fside > self.wall_dist * 1.8:
            self.prev_err = None
            return self.lin_speed, s * 0.6 * self.ang_speed   # wall ended: arc around corner
        err = min(side, fside * 0.7) - self.wall_dist     # + = too far from the wall
        derr = 0.0 if self.prev_err is None else (err - self.prev_err) * self.hz
        self.prev_err = err
        return self.lin_speed, s * (self.kp * err + self.kd * derr)

    def _follow(self, pose, polar, front):
        lin, ang = self._wall_control(polar, front)
        self.drive(lin, ang)
        last = self.follow_path[-1]
        step = math.hypot(pose[0] - last[0], pose[1] - last[1])
        if step > 0.05:
            self.follow_path.append(pose[:2])
            self.follow_len += step
        d0 = math.hypot(pose[0] - self.follow_start[0], pose[1] - self.follow_start[1])
        self.info = f'followed {self.follow_len:.1f} m, {d0:.2f} m from loop start'
        if self.follow_len > self.loop_min and d0 < self.loop_radius:
            self._loop_closed(forced=False)
        elif self.follow_len > self.max_follow:
            self._loop_closed(forced=True)

    def _loop_closed(self, forced):
        self.drive(0, 0)
        path = np.array(self.follow_path)
        area = signed_area(path)
        outer_like = area * (-self.side) > 0       # wall on right -> outer boundary is CCW
        self._publish_path(self.pub_boundary, path)
        if self.follow_target == 'outer':
            if outer_like or forced or self.outer_attempts >= 3:
                self.outer_poly = path if (outer_like and not forced) else None
                self.phase = 'interior'
                self.get_logger().info(
                    f'Outer boundary closed: {self.follow_len:.1f} m, area {abs(area):.1f} m^2'
                    + (' (forced)' if forced else ''))
                self._set('PLAN', 'outer boundary done')
            else:
                self._mark_done_inside(path)
                self.outer_attempts += 1
                self.get_logger().info('That loop went around an object, not the boundary - '
                                       'heading into open space to find the outer wall')
                self._set('ESCAPE', 'looking for the outer wall')
        else:
            self._mark_done(self.follow_target['pts'])
            self.islands_done += 1
            self._set('PLAN', f'structure #{self.islands_done} done')

    def _mark_done(self, pts):
        self.done_pts = np.vstack([self.done_pts, pts])

    def _mark_done_inside(self, poly):
        g = self._grid()
        if g is None:
            return
        r, c = np.nonzero(g.occupied)
        pts = np.stack(g.to_world(r, c), axis=1)
        self._mark_done(pts[points_in_polygon(pts, poly)])

    def _escape(self, pose, polar, front):
        ang, rng = polar
        if self.turn_yaw is None:   # face the most open direction (away from the object)
            self.turn_yaw = wrap(pose[2] + math.radians(float(ang[np.argmax(rng)])))
        err = wrap(self.turn_yaw - pose[2])
        if abs(err) > math.radians(12):
            return self.drive(0.0, math.copysign(self.ang_speed, err))
        if front < self.wall_dist + 0.15:
            self.turn_yaw = None
            return self._begin_follow('outer', pose)
        self.drive(self.lin_speed, 0.0)

    # ------------------------------------------------------------------ planning
    def _grid(self):
        if self.map_msg is None:
            return None
        if self.grid is None or self.grid[0] is not self.map_msg:
            self.grid = (self.map_msg, GridMap.from_msg(self.map_msg))
        return self.grid[1]

    def _plan(self, pose, polar, front):
        self.drive(0, 0)
        g = self._grid()
        if g is None:
            self.info = 'waiting for /map'
            return
        t0 = time.time()
        planner = Planner(g, inflate=self.robot_radius + 0.05)
        start = planner.nearest_free(*planner.cell(pose[0], pose[1]))
        if start is None:
            self.get_logger().warn('Robot is inside inflated obstacles - backing off')
            return self._start_recovery(away=True)
        dist, parent = planner.search(start)

        islands = self._find_islands(g, planner, dist)
        frontiers = [] if islands else self._find_frontiers(g, planner, dist)
        self.last_plan = {'islands_left': len(islands), 'frontiers': len(frontiers),
                          'plan_s': round(time.time() - t0, 2)}
        targets = islands or frontiers
        if not targets:
            return self._finish()
        tgt = min(targets, key=lambda t: t['cost'])
        path = planner.path_to(parent, start, tgt['cell'])
        if path is None:
            self.blacklist.append(tgt['xy'])
            return
        self.goto = {'target': tgt, 'path': path, 'idx': 0, 't0': time.time(),
                     'timeout': 30.0 + 15.0 * tgt['cost'] * planner.res, 'replan': time.time()}
        self._publish_path(self.pub_path, np.array(path))
        kind = 'structure' if tgt['kind'] == 'island' else 'unexplored gap'
        self._set('GOTO', f'{kind} at ({tgt["xy"][0]:.1f}, {tgt["xy"][1]:.1f}), '
                          f'{len(islands)} structures / {len(frontiers)} gaps left')

    def _blacklisted(self, xy):
        return any(math.hypot(xy[0] - b[0], xy[1] - b[1]) < 0.3 for b in self.blacklist)

    def _find_islands(self, g, planner, dist):
        occ = g.occupied
        lab, n = ndimage.label(ndimage.binary_dilation(occ), structure=np.ones((3, 3)))
        done_tree = cKDTree(self.done_pts) if len(self.done_pts) else None
        out = []
        for k in range(1, n + 1):
            r, c = np.nonzero((lab == k) & occ)
            if len(r) * g.res ** 2 < self.island_min_area:
                continue
            pts = np.stack(g.to_world(r, c), axis=1)
            if self.outer_poly is not None:
                if points_in_polygon(pts, self.outer_poly).mean() < 0.8:
                    continue                      # part of / outside the outer boundary
            elif (r.min() <= 1 or c.min() <= 1 or r.max() >= g.h - 2 or c.max() >= g.w - 2):
                continue
            if done_tree is not None:
                d, _ = done_tree.query(pts, distance_upper_bound=0.2)
                if np.isfinite(d).mean() > 0.5:
                    continue
            centroid = pts.mean(axis=0)
            if self._blacklisted(centroid):
                continue
            # ring of cells at wall_dist from this island = where following can start
            mask = np.zeros_like(occ)
            mask[r, c] = True
            dmap = ndimage.distance_transform_edt(~mask) * g.res
            ring = (np.abs(dmap - self.wall_dist) < 0.08) & g.free
            rr, cc = np.nonzero(ring)
            if len(rr) == 0:
                continue
            pr, pc = rr // planner.f, cc // planner.f
            okp = (pr < planner.H) & (pc < planner.W)
            pr, pc, rr, cc = pr[okp], pc[okp], rr[okp], cc[okp]
            costs = dist[pr, pc]
            if not np.isfinite(costs).any():
                continue
            i = int(np.argmin(costs))
            ax, ay = g.to_world(rr[i], cc[i])
            # heading that puts the island on the wall side
            j = int(np.argmin((pts[:, 0] - ax) ** 2 + (pts[:, 1] - ay) ** 2))
            to_isl = math.atan2(pts[j, 1] - ay, pts[j, 0] - ax)
            out.append({'kind': 'island', 'cell': (int(pr[i]), int(pc[i])), 'cost': float(costs[i]),
                        'xy': (float(ax), float(ay)), 'yaw': wrap(to_isl - self.side * math.pi / 2),
                        'pts': pts, 'centroid': centroid})
        return out

    def _find_frontiers(self, g, planner, dist):
        front = g.free & ndimage.binary_dilation(g.unknown)
        lab, n = ndimage.label(front, structure=np.ones((3, 3)))
        reach = np.isfinite(dist)
        out = []
        for k in range(1, n + 1):
            r, c = np.nonzero(lab == k)
            if len(r) < self.frontier_min:
                continue
            pts = np.stack(g.to_world(r, c), axis=1)
            if self.outer_poly is not None and points_in_polygon(pts, self.outer_poly).mean() < 0.5:
                continue
            centroid = pts.mean(axis=0)
            if self._blacklisted(centroid):
                continue
            # nearest reachable planning cell within 0.6 m of the frontier
            m = np.zeros((planner.H, planner.W), bool)
            pr, pc = r // planner.f, c // planner.f
            okp = (pr < planner.H) & (pc < planner.W)
            m[pr[okp], pc[okp]] = True
            near = (ndimage.distance_transform_edt(~m) * planner.res < 0.6) & reach
            if not near.any():
                continue
            cand = np.where(near, dist, np.inf)
            i = np.unravel_index(int(np.argmin(cand)), cand.shape)
            out.append({'kind': 'frontier', 'cell': (int(i[0]), int(i[1])), 'cost': float(cand[i]),
                        'xy': (float(centroid[0]), float(centroid[1])), 'size': int(len(r))})
        return out

    # ------------------------------------------------------------------ path following
    def _goto(self, pose, polar, front):
        gt = self.goto
        tgt = gt['target']
        path = gt['path']
        goal = path[-1]
        if math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < self.goal_tol:
            self.drive(0, 0)
            if tgt['kind'] == 'island':
                self.turn_yaw = tgt['yaw']
                self._set('TURN_TO', 'lining up with the structure')
            else:
                self._set('PLAN', 'reached gap')
            return
        if time.time() - gt['t0'] > gt['timeout'] or gt.get('stuck', 0) >= 2:
            self.get_logger().warn('Could not reach target - skipping it')
            self.blacklist.append(tgt['xy'] if tgt['kind'] == 'frontier' else tgt['centroid'])
            return self._set('PLAN', 'target skipped')
        if time.time() - gt['replan'] > 4.0 or front < self.robot_radius + 0.08:
            gt['replan'] = time.time()
            g = self._grid()
            if g is not None:
                new = Planner(g, inflate=self.robot_radius + 0.05).plan(pose[0], pose[1], *goal)
                if new:
                    gt['path'], gt['idx'] = new, 0
                    path = new
                    self._publish_path(self.pub_path, np.array(path))
            if front < self.robot_radius + 0.08:
                self.drive(-self.lin_speed, 0.0)
                return
        # advance along the path, aim at a point ~0.3 m ahead
        d = [math.hypot(px - pose[0], py - pose[1]) for px, py in path]
        gt['idx'] = max(gt['idx'], int(np.argmin(d[gt['idx']:])) + gt['idx'])
        la = next((i for i in range(gt['idx'], len(path)) if d[i] > 0.3), len(path) - 1)
        tx, ty = path[la]
        err = wrap(math.atan2(ty - pose[1], tx - pose[0]) - pose[2])
        if abs(err) > math.radians(30):
            self.drive(0.0, math.copysign(self.ang_speed, err))
        else:
            self.drive(self.lin_speed, 2.0 * err)
        self.info = f'{d[-1]:.2f} m to target'

    def _turn_to(self, pose, polar, front):
        err = wrap(self.turn_yaw - pose[2])
        if abs(err) > math.radians(10):
            return self.drive(0.0, math.copysign(self.ang_speed, err))
        self.drive(0, 0)
        self.turn_yaw = None
        self._begin_follow(self.goto['target'], pose)

    # ------------------------------------------------------------------ finish
    def _finish(self):
        self.drive(0, 0)
        name = self.map_name or time.strftime('explored_%Y%m%d_%H%M%S')
        base = os.path.join(self.maps_dir, name)
        os.makedirs(self.maps_dir, exist_ok=True)
        try:
            from slam_toolbox.srv import SaveMap, SerializePoseGraph
            c1 = self.create_client(SaveMap, 'slam_toolbox/save_map')
            c2 = self.create_client(SerializePoseGraph, 'slam_toolbox/serialize_map')
            if c1.wait_for_service(timeout_sec=2.0):
                r = SaveMap.Request()
                r.name.data = base
                c1.call_async(r)
            if c2.wait_for_service(timeout_sec=2.0):
                r = SerializePoseGraph.Request()
                r.filename = base
                c2.call_async(r)
            saved = f'map saved to {base}.yaml/.pgm (+ .posegraph)'
        except Exception as e:  # noqa: BLE001
            saved = f'could not save map: {e}'
        self.get_logger().info(f'Exploration complete: {self.islands_done} structures mapped; {saved}')
        self._set('DONE', saved)

    # ------------------------------------------------------------------ publishing
    def _publish_path(self, pub, xy):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in xy:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        pub.publish(msg)

    def _publish_state(self):
        self.pub_state.publish(String(data=json.dumps({
            'state': self.state, 'phase': self.phase, 'info': self.info,
            'structures_done': self.islands_done, 'outer_closed': self.outer_poly is not None
            or self.phase == 'interior', **self.last_plan})))


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0, 0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
