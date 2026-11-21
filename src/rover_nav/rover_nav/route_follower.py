"""Drive a dashboard route (start -> waypoints -> end) around walls, without Nav2.

The dashboard's Go publishes the route on /route_follower/route (nav_msgs/Path: start,
waypoints..., end) when Nav2 is not running. Each leg is planned on /map with route_planning (walls kept at robot_radius + margin,
middle of gaps preferred), re-planned every few seconds as the map fills in, and driven
with the same discrete w/a/s/d logic as the explorer. At the end the rover turns to the
end point's heading.

Inputs : /route_follower/route, /map, /scan, TF map->base_link, optional /tof/cliff_detected,
         /tof/obstacle_detected
Outputs: /cmd_vel, /route_follower/state (JSON), /route_follower/path (planned path)
Service: /route_follower/cancel (std_srvs/Trigger); an empty route also cancels.

Speeds and robot_radius come from explorer.yaml (the launch files pass them through), so
field_test.sh --motion --apply tunes both nodes.
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .grid_utils import GridMap, scan_to_points, transform_points, wrap, yaw_from_quat
from .route_planning import make_planner, plan_leg


class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.map_frame = p('map_frame', 'map')
        self.base_frame = p('base_frame', 'base_link')
        self.robot_radius = p('robot_radius', 0.15)
        self.lin_speed = p('lin_speed', 0.15)
        self.ang_speed = p('ang_speed', 0.6)
        self.discrete = p('discrete_drive', True)
        self.hz = p('control_hz', 10.0)
        self.goal_tol = p('goal_tolerance', 0.2)
        self.final_heading = p('final_heading', True)     # turn to the end point's arrow
        self.heading_tol = math.radians(p('heading_tolerance_deg', 12.0))
        self.replan_s = p('replan_s', 3.0)
        self.use_tof = p('use_tof', True)
        self.start_skip = p('start_skip_dist', 0.5)       # closer than this to Start: skip it

        self.scan = self.map_msg = None
        self.laser_tf = None
        self.cliff = self.tof_obstacle = False
        self.sd_acc = 0.0
        self.t_start = self.get_clock().now()
        self._idle('')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, 'route_follower/route', self._on_route, latched)
        self.create_subscription(OccupancyGrid, 'map', lambda m: setattr(self, 'map_msg', m), latched)
        self.create_subscription(LaserScan, 'scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        if self.use_tof:
            self.create_subscription(Bool, 'tof/cliff_detected',
                                     lambda m: setattr(self, 'cliff', m.data), 10)
            self.create_subscription(Bool, 'tof/obstacle_detected',
                                     lambda m: setattr(self, 'tof_obstacle', m.data), 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_state = self.create_publisher(String, 'route_follower/state', 10)
        self.pub_path = self.create_publisher(
            Path, 'route_follower/path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, 'route_follower/cancel', self._srv_cancel)
        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info('Route follower ready: press Go on the dashboard')

    # ------------------------------------------------------------------ state
    def _idle(self, info, state='IDLE'):
        self.state, self.info = state, info
        self.targets = []          # [(x, y, yaw)]
        self.idx = 0
        self.path = []
        self.goal_xy = None        # current target as planned (moved off walls if needed)
        self.leg_t0 = self.replan_t = 0.0
        self.recover = []
        self.stuck_ref = None
        self.blocked_since = None
        self.attempts = 0
        self.halt_ticks = 0

    def _on_route(self, msg):
        # The topic is latched: ignore a route left over from before this node started.
        if Time.from_msg(msg.header.stamp) < self.t_start and msg.header.stamp.sec:
            return
        if not msg.poses:
            self._stop('route cleared')
            return
        pts = [(ps.pose.position.x, ps.pose.position.y, yaw_from_quat(ps.pose.orientation))
               for ps in msg.poses]
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f'route frame {msg.header.frame_id} != {self.map_frame}')
        self._idle('')
        self.start_pt, self.targets = pts[0], pts[1:]
        self.use_start = True          # decided on the first tick, once the pose is known
        self.state = 'START'
        self.get_logger().info(f'New route: {len(self.targets)} points')

    def _srv_cancel(self, req, resp):
        was = self.state
        self._stop('cancelled')
        resp.success, resp.message = True, f'route cancelled (was {was})'
        return resp

    def _stop(self, why, state='STOPPED'):
        self._idle(why, state)
        self.drive(0, 0)
        self._publish_path([])
        self.get_logger().info(f'Route {state.lower()}: {why}')

    # ------------------------------------------------------------------ helpers
    def pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quat(t.transform.rotation))

    def front_range(self):
        s = self.scan
        if s is None:
            return 10.0
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, s.header.frame_id, Time())
            except Exception:
                return 10.0
            self.laser_tf = (t.transform.translation.x, t.transform.translation.y,
                             yaw_from_quat(t.transform.rotation))
        pts = transform_points(scan_to_points(s, 8.0, 0.05), *self.laser_tf)
        ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
        m = np.abs(ang) <= 25
        front = float(np.hypot(pts[m, 0], pts[m, 1]).min()) if m.any() else 10.0
        if self.tof_obstacle:
            front = min(front, self.robot_radius + 0.05)
        return front

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

    def _plan(self, pose):
        """Plan current pose -> current target, then the remaining legs (for display)."""
        if self.map_msg is None:
            return False
        g = GridMap.from_msg(self.map_msg)
        pl = make_planner(g, self.robot_radius)
        clear = self.robot_radius + 0.10
        pts = [pose[:2]] + [t[:2] for t in self.targets[self.idx:]]
        legs = [plan_leg(pl, a, b, clear) for a, b in zip(pts, pts[1:])]
        if not legs or not legs[0]['ok']:
            self.info = legs[0]['note'] if legs else 'nothing to do'
            return False
        self.path = [tuple(p) for p in legs[0]['path']]
        self.goal_xy = self.path[-1]
        full = [p for lg in legs for p in lg['path']]
        self._publish_path(full)
        self.replan_t = time.time()
        return True

    # ------------------------------------------------------------------ main loop
    def _tick(self):
        if self.state in ('IDLE', 'DONE', 'STOPPED', 'FAILED'):
            if self.state != 'IDLE' and self.halt_ticks < 3:
                self.drive(0, 0)
                self.halt_ticks += 1
            return self._publish_state()
        pose = self.pose()
        if pose is None:
            self.drive(0, 0)
            self.info = 'waiting for map->base_link TF (is localization / SLAM running?)'
            return self._publish_state()

        if self.state == 'START':
            if math.hypot(self.start_pt[0] - pose[0], self.start_pt[1] - pose[1]) > self.start_skip:
                self.targets.insert(0, self.start_pt)    # drive to Start first
            if not self._plan(pose):
                return self._stop(f'cannot plan: {self.info}', 'FAILED')
            self.state, self.leg_t0 = 'DRIVE', time.time()

        if self.cliff and not self.recover:
            self.get_logger().warn('Cliff! backing up')
            self.recover = [((-self.lin_speed, 0.0), time.time() + 1.0)]
            self.replan_t = 0.0
        if self.recover:
            cmd, until = self.recover[0]
            if time.time() > until:
                self.recover.pop(0)
            self.drive(*cmd)
            return self._publish_state()

        if self.state == 'TURN':
            err = wrap(self.targets[-1][2] - pose[2])
            if abs(err) <= self.heading_tol:
                return self._stop('reached the end', 'DONE')
            self.drive(0.0, math.copysign(self.ang_speed, err))
            self.info = f'turning to the end heading ({math.degrees(err):+.0f} deg)'
            return self._publish_state()

        tx, ty, _ = self.targets[self.idx]
        gx, gy = self.goal_xy or (tx, ty)
        to_goal = min(math.hypot(tx - pose[0], ty - pose[1]), math.hypot(gx - pose[0], gy - pose[1]))
        front = self.front_range()
        blocked = front < self.robot_radius + 0.08
        # A point next to a wall can't be reached exactly: close enough and blocked = arrived
        if to_goal < self.goal_tol or (blocked and to_goal < 2.5 * self.goal_tol):
            self.idx += 1
            self.attempts = 0
            if self.idx >= len(self.targets):
                self.drive(0, 0)
                if self.final_heading:
                    self.state = 'TURN'
                    return self._publish_state()
                return self._stop('reached the end', 'DONE')
            self.leg_t0 = time.time()
            self._plan(pose)
            return self._publish_state()

        # leg timeout: 30 s + 15 s per metre of straight-line distance
        if time.time() - self.leg_t0 > 30.0 + 15.0 * math.hypot(tx - pose[0], ty - pose[1]) * 2:
            return self._stop(f'point {self.idx + 1} not reached in time', 'FAILED')

        if blocked:
            self.drive(0, 0)
            self.blocked_since = self.blocked_since or time.time()
            if time.time() - self.blocked_since > 3.0:
                self.attempts += 1
                if self.attempts > 3:
                    return self._stop('path blocked', 'FAILED')
                self.get_logger().warn('Blocked ahead - backing up and re-planning')
                self.recover = [((-self.lin_speed, 0.0), time.time() + 1.0)]
                self.blocked_since, self.replan_t = None, 0.0
            self.info = f'something {front:.2f} m ahead - waiting'
            return self._publish_state()
        self.blocked_since = None

        if self._stuck(pose):
            self.attempts += 1
            if self.attempts > 3:
                return self._stop('stuck', 'FAILED')
            self.get_logger().warn('Not moving - backing up and re-planning')
            self.recover = [((-self.lin_speed, 0.0), time.time() + 1.0)]
            self.replan_t = 0.0
            return self._publish_state()

        if time.time() - self.replan_t > self.replan_s:
            if not self._plan(pose):
                self.drive(0, 0)
                self.attempts += 1
                if self.attempts > 10:
                    return self._stop(f'cannot plan: {self.info}', 'FAILED')
                return self._publish_state()

        # pure-pursuit-ish: aim at the path point ~0.3 m ahead
        path = self.path
        d = [math.hypot(px - pose[0], py - pose[1]) for px, py in path]
        i0 = int(np.argmin(d))
        la = next((i for i in range(i0, len(path)) if d[i] > 0.3), len(path) - 1)
        ax, ay = path[la]
        err = wrap(math.atan2(ay - pose[1], ax - pose[0]) - pose[2])
        if abs(err) > math.radians(30):
            self.drive(0.0, math.copysign(self.ang_speed, err))
        else:
            self.drive(self.lin_speed, 2.0 * err)
        remaining = sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1)
                        in zip([pose[:2]] + path[la:], path[la:]))
        self.info = f'point {self.idx + 1}/{len(self.targets)}, {remaining:.1f} m to it'
        self._publish_state()

    def _stuck(self, pose):
        now = time.time()
        if self.stuck_ref is None:
            self.stuck_ref = (pose, now)
            return False
        p0, t0 = self.stuck_ref
        if math.hypot(pose[0] - p0[0], pose[1] - p0[1]) > 0.05 or abs(wrap(pose[2] - p0[2])) > 0.2:
            self.stuck_ref = (pose, now)
            return False
        if now - t0 > 8.0:
            self.stuck_ref = None
            return True
        return False

    # ------------------------------------------------------------------ publishing
    def _publish_path(self, xy):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in xy:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def _publish_state(self):
        self.pub_state.publish(String(data=json.dumps({
            'state': self.state, 'info': self.info, 'point': self.idx + 1,
            'points': len(self.targets)})))


def main(args=None):
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
