"""Tiny 2D simulator for testing mapping / localization / exploration without the rover.

Drives a virtual robot from /cmd_vel inside a ground-truth occupancy map and publishes a
noisy 360-beam /scan (frame laser_frame) at 10 Hz. Odometry comes from laser_odometry
like on the real robot (set publish_odom_tf:=true to publish perfect odom instead).

  world_yaml: map_server yaml to use as the world ('' = built-in test room:
              8 x 6 m with an alcove, a box, a pillar and an L-shaped block)
Publishes /sim/ground_truth (PoseStamped, frame 'world') so you can check accuracy.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from .grid_utils import GridMap


def default_world(res=0.02):
    w, h = int(8.0 / res), int(6.0 / res)
    occ = np.zeros((h + 40, w + 40), bool)          # 0.4 m margin of empty space outside

    def box(x0, y0, x1, y1):
        occ[int((y0 + 0.4) / res):int((y1 + 0.4) / res), int((x0 + 0.4) / res):int((x1 + 0.4) / res)] = True

    t = 0.1
    box(0, 0, 8, t); box(0, 6 - t, 8, 6); box(0, 0, t, 6); box(8 - t, 0, 8, 6)  # outer walls
    box(5.0, 4.2, 5.0 + t, 6)                  # alcove partition from the top wall
    box(2.0, 2.0, 3.0, 2.6)                    # box
    yy, xx = np.mgrid[0:occ.shape[0], 0:occ.shape[1]]
    occ |= (xx * res - 0.4 - 5.5) ** 2 + (yy * res - 0.4 - 2.0) ** 2 < 0.25 ** 2   # pillar
    box(2.5, 4.0, 4.0, 4.3); box(3.7, 3.2, 4.0, 4.3)                  # L-shaped block
    data = np.where(occ, 100, 0).astype(np.int16)
    return GridMap(data, res, (-0.4, -0.4))


class Sim(Node):
    def __init__(self):
        super().__init__('rover_sim')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        wy = p('world_yaml', '')
        self.world = GridMap.from_yaml(wy) if wy else default_world()
        self.x, self.y, self.th = p('start_x', 1.0), p('start_y', 1.0), p('start_yaw', 0.3)
        self.pub_odom_tf = p('publish_odom_tf', False)
        self.n = p('beams', 360)
        self.max_range = p('max_range', 8.0)
        self.noise = p('range_noise', 0.01)
        self.radius = p('robot_radius', 0.15)
        self.motion_noise = p('motion_noise', 0.05)
        self.cmd, self.cmd_t = (0.0, 0.0), 0.0
        self.occ = self.world.occupied
        self.rng = np.random.default_rng(1)
        self.angles = -math.pi + np.arange(self.n) * 2 * math.pi / self.n
        self.steps = np.arange(0.05, self.max_range, self.world.res * 0.5)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.pub_scan = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_gt = self.create_publisher(PoseStamped, 'sim/ground_truth', 10)
        self.tf = TransformBroadcaster(self)
        self.last = time.time()
        self.create_timer(0.02, self._step)
        self.create_timer(0.1, self._scan)
        self.get_logger().info(f'Sim world {self.world.w}x{self.world.h} cells, start '
                               f'({self.x:.2f}, {self.y:.2f}, {math.degrees(self.th):.0f} deg)')

    def _on_cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.angular.z), time.time()

    def _blocked(self, x, y):
        r, c = self.world.to_cell(x, y)
        k = int(math.ceil(self.radius / self.world.res))
        sub = self.occ[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1]
        if sub.size == 0:
            return True
        rr, cc = np.nonzero(sub)
        return bool(np.any((rr - min(k, r)) ** 2 + (cc - min(k, c)) ** 2 <= k * k))

    def _step(self):
        now = time.time()
        dt, self.last = now - self.last, now
        v, w = self.cmd if now - self.cmd_t < 0.5 else (0.0, 0.0)
        v *= 1 + self.motion_noise * self.rng.normal()
        w *= 1 + self.motion_noise * self.rng.normal()
        th = self.th + w * dt
        x, y = self.x + v * dt * math.cos(th), self.y + v * dt * math.sin(th)
        self.th = math.atan2(math.sin(th), math.cos(th))
        if not self._blocked(x, y):
            self.x, self.y = x, y
        stamp = self.get_clock().now().to_msg()
        gt = PoseStamped()
        gt.header.stamp, gt.header.frame_id = stamp, 'world'
        gt.pose.position.x, gt.pose.position.y = self.x, self.y
        gt.pose.orientation.z, gt.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        self.pub_gt.publish(gt)
        if self.pub_odom_tf:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'odom', 'base_link'
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation = gt.pose.orientation
            self.tf.sendTransform(t)

    def _scan(self):
        a = self.th + self.angles
        xs = self.x + np.cos(a)[:, None] * self.steps[None, :]
        ys = self.y + np.sin(a)[:, None] * self.steps[None, :]
        r, c = self.world.to_cell(xs, ys)
        inside = self.world.inside(r, c)
        hit = np.zeros_like(inside)
        hit[inside] = self.occ[r[inside], c[inside]]
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
        rng = np.where(first >= 0, self.steps[np.maximum(first, 0)], np.inf)
        rng = rng + self.noise * self.rng.normal(size=self.n)
        s = LaserScan()
        s.header.stamp = self.get_clock().now().to_msg()
        s.header.frame_id = 'laser_frame'
        s.angle_min, s.angle_max = float(self.angles[0]), float(self.angles[-1])
        s.angle_increment = float(2 * math.pi / self.n)
        s.scan_time, s.time_increment = 0.1, 0.0
        s.range_min, s.range_max = 0.1, float(self.max_range)
        s.ranges = [float(v) for v in rng]
        self.pub_scan.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = Sim()
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
