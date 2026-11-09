"""Lidar-only odometry (no wheel encoders needed).

Point-to-line ICP of every scan against a keyframe scan. Publishes the odom -> base_link
TF and nav_msgs/Odometry on /odom, which slam_toolbox needs as its motion prior.

- constant-velocity prediction as the ICP initial guess
- new keyframe every keyframe_dist metres / keyframe_angle radians (limits drift)
- degeneracy handling: in feature-poor directions (long bare corridor) the update is
  suppressed instead of sliding along the wall
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .grid_utils import scan_to_points, transform_points, voxel_downsample, wrap, yaw_from_quat


def compose(a, b):
    """Pose a ∘ b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class Keyframe:
    def __init__(self, pts, k=6):
        self.pts = pts
        self.tree = cKDTree(pts)
        _, idx = self.tree.query(pts, k=min(k, len(pts)))
        nb = pts[idx]                                   # (M, k, 2)
        nb = nb - nb.mean(axis=1, keepdims=True)
        cov = np.einsum('mki,mkj->mij', nb, nb)
        w, v = np.linalg.eigh(cov)
        self.normals = v[:, :, 0]                       # smallest eigenvalue -> line normal
        # how line-like each neighbourhood is (0 = blob, 1 = perfect line)
        self.lineness = 1.0 - w[:, 0] / np.maximum(w[:, 1], 1e-9)


def icp(src, kf, guess, iters=20, thr0=0.5, thr1=0.12, degenerate_ratio=0.02):
    x, y, th = guess
    info = {'ok': False, 'inliers': 0.0, 'rmse': float('nan'), 'degenerate': False}
    for i in range(iters):
        thr = thr0 + (thr1 - thr0) * min(1.0, i / (iters * 0.6))
        p = transform_points(src, x, y, th)
        d, j = kf.tree.query(p, distance_upper_bound=thr)
        m = np.isfinite(d)
        if m.sum() < 30:
            return guess, info
        pm, q, n = p[m], kf.pts[j[m]], kf.normals[j[m]]
        e = np.einsum('ij,ij->i', n, pm - q)
        dpdth = np.stack([-(pm[:, 1] - y), pm[:, 0] - x], axis=1)
        J = np.column_stack([n[:, 0], n[:, 1], np.einsum('ij,ij->i', n, dpdth)])
        wgt = (0.3 + 0.7 * kf.lineness[j[m]]) / np.maximum(1.0, np.abs(e) / 0.05)  # Huber-ish
        H = J.T @ (J * wgt[:, None])
        g = J.T @ (wgt * e)
        ev, evec = np.linalg.eigh(H)
        keep = ev > degenerate_ratio * ev.max()
        info['degenerate'] = not keep.all()
        # solve only in well-constrained directions
        delta = -evec[:, keep] @ ((evec[:, keep].T @ g) / ev[keep])
        x, y, th = x + delta[0], y + delta[1], wrap(th + delta[2])
        if abs(delta[0]) + abs(delta[1]) < 1e-4 and abs(delta[2]) < 1e-4:
            break
    p = transform_points(src, x, y, th)
    d, _ = kf.tree.query(p, distance_upper_bound=thr1)
    ok = np.isfinite(d)
    info['inliers'] = float(ok.mean())
    info['rmse'] = float(np.sqrt(np.mean(d[ok] ** 2))) if ok.any() else float('nan')
    info['ok'] = info['inliers'] > 0.35
    return (x, y, th), info


class LaserOdometry(Node):
    def __init__(self):
        super().__init__('laser_odometry')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.odom_frame = p('odom_frame', 'odom')
        self.base_frame = p('base_frame', 'base_link')
        self.publish_tf = p('publish_tf', True)
        self.max_range = p('max_range', 8.0)
        self.min_range = p('min_range', 0.12)
        self.voxel = p('voxel_size', 0.04)
        self.kf_dist = p('keyframe_dist', 0.15)
        self.kf_angle = p('keyframe_angle', 0.15)

        self.laser_tf = None  # (x, y, yaw) of laser in base frame
        self.kf = None
        self.kf_pose = (0.0, 0.0, 0.0)  # keyframe pose in odom
        self.rel = (0.0, 0.0, 0.0)      # current pose relative to keyframe
        self.vel = (0.0, 0.0, 0.0)      # last per-scan motion (base frame)
        self.last_t = None
        self.bad = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, 'odom', 20)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.get_logger().info('Laser odometry ready (waiting for /scan)')

    def _laser_in_base(self, frame):
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except Exception:
                self.get_logger().warn(f'Waiting for TF {self.base_frame} -> {frame}',
                                       throttle_duration_sec=5.0)
                return None
            self.laser_tf = (t.transform.translation.x, t.transform.translation.y,
                             yaw_from_quat(t.transform.rotation))
        return self.laser_tf

    def _on_scan(self, scan):
        ltf = self._laser_in_base(scan.header.frame_id)
        if ltf is None:
            return
        pts = scan_to_points(scan, self.max_range, self.min_range)
        pts = voxel_downsample(transform_points(pts, *ltf), self.voxel)
        if len(pts) < 40:
            return
        t = Time.from_msg(scan.header.stamp).nanoseconds * 1e-9
        dt = (t - self.last_t) if self.last_t else 0.1
        self.last_t = t

        if self.kf is None:
            self.kf = Keyframe(pts)
            self._publish(scan.header.stamp, self.kf_pose, (0.0, 0.0, 0.0), dt)
            return

        guess = compose(self.rel, self.vel)
        new_rel, info = icp(pts, self.kf, guess)
        if not info['ok']:
            self.bad += 1
            self.get_logger().warn(f'Scan match poor (inliers {info["inliers"]:.0%}) - '
                                   'using motion prediction', throttle_duration_sec=2.0)
            new_rel = guess
            if self.bad >= 3:   # re-anchor on the current scan
                self._new_keyframe(pts, compose(self.kf_pose, new_rel))
                self.vel = (0.0, 0.0, 0.0)
                self._publish(scan.header.stamp, self.kf_pose, self.vel, dt)
                return
        else:
            self.bad = 0
        self.vel = compose(inverse(self.rel), new_rel)
        self.rel = new_rel
        pose = compose(self.kf_pose, self.rel)
        if math.hypot(self.rel[0], self.rel[1]) > self.kf_dist or abs(self.rel[2]) > self.kf_angle:
            self._new_keyframe(pts, pose)
        self._publish(scan.header.stamp, pose, self.vel, dt)

    def _new_keyframe(self, pts, pose):
        self.kf = Keyframe(pts)
        self.kf_pose = pose
        self.rel = (0.0, 0.0, 0.0)

    def _publish(self, stamp, pose, vel, dt):
        qz, qw = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y = pose[0], pose[1]
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf_pub.sendTransform(tf)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = pose[0], pose[1]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.pose.covariance[0] = o.pose.covariance[7] = 0.01
        o.pose.covariance[35] = 0.01
        dt = max(dt, 1e-3)
        o.twist.twist.linear.x, o.twist.twist.linear.y = vel[0] / dt, vel[1] / dt
        o.twist.twist.angular.z = vel[2] / dt
        self.pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = LaserOdometry()
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
