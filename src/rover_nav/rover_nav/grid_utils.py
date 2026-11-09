"""Occupancy-grid helpers shared by the localizer and the explorer."""
import heapq
import math
import os

import numpy as np
import yaml
from scipy import ndimage

UNKNOWN, FREE_MAX, OCC_MIN = -1, 25, 50


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GridMap:
    """Occupancy grid. data[row, col]: row 0 = lowest y (same layout as nav_msgs/OccupancyGrid)."""

    def __init__(self, data, resolution, origin_xy):
        self.data = data.astype(np.int16)
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.h, self.w = data.shape

    @classmethod
    def from_msg(cls, msg):
        d = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin.position
        return cls(d, msg.info.resolution, (o.x, o.y))

    @classmethod
    def from_yaml(cls, path):
        """Load a map_server .yaml/.pgm pair."""
        from PIL import Image
        with open(path) as f:
            meta = yaml.safe_load(f)
        img = meta['image']
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(path), img)
        a = np.asarray(Image.open(img).convert('L'), dtype=np.float32)
        p = a / 255.0 if meta.get('negate', 0) else (255.0 - a) / 255.0
        d = np.full(a.shape, UNKNOWN, np.int16)
        d[p > float(meta.get('occupied_thresh', 0.65))] = 100
        d[p < float(meta.get('free_thresh', 0.25))] = 0
        return cls(np.flipud(d), meta['resolution'], meta['origin'][:2])

    # --- masks
    @property
    def occupied(self):
        return self.data >= OCC_MIN

    @property
    def free(self):
        return (self.data >= 0) & (self.data <= FREE_MAX)

    @property
    def unknown(self):
        return self.data < 0

    # --- coordinates (col = x, row = y)
    def to_cell(self, x, y):
        return (np.floor((np.asarray(y) - self.oy) / self.res).astype(int),
                np.floor((np.asarray(x) - self.ox) / self.res).astype(int))

    def to_world(self, r, c):
        return (self.ox + (np.asarray(c) + 0.5) * self.res,
                self.oy + (np.asarray(r) + 0.5) * self.res)

    def inside(self, r, c):
        return (r >= 0) & (r < self.h) & (c >= 0) & (c < self.w)

    def dist_to_occupied(self):
        """Metres from every cell to the nearest occupied cell."""
        occ = self.occupied
        if not occ.any():
            return np.full(occ.shape, 1e3, np.float32)
        return (ndimage.distance_transform_edt(~occ) * self.res).astype(np.float32)

    def likelihood_field(self, sigma_m):
        d = self.dist_to_occupied()
        return np.exp(-0.5 * (d / sigma_m) ** 2).astype(np.float32)


def scan_to_points(scan, max_range=None, min_range=None):
    """LaserScan -> (N, 2) points in the scan frame, invalid rays dropped."""
    r = np.asarray(scan.ranges, dtype=np.float32)
    a = scan.angle_min + np.arange(len(r), dtype=np.float32) * scan.angle_increment
    lo = max(scan.range_min, min_range or 0.0)
    hi = min(scan.range_max, max_range or scan.range_max)
    ok = np.isfinite(r) & (r > lo) & (r < hi)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def transform_points(pts, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def voxel_downsample(pts, cell):
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / cell).astype(np.int64)
    _, idx = np.unique(keys[:, 0] * 1_000_003 + keys[:, 1], return_index=True)
    return pts[np.sort(idx)]


# ---------------------------------------------------------------- planning
class Planner:
    """Grid planner on a coarsened copy of the map.

    Cells closer than `inflate` to an obstacle (or unknown) are blocked; cells near
    obstacles cost extra so paths stay in the middle of corridors.
    """
    NB = [(-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414), (0, -1, 1.0),
          (0, 1, 1.0), (1, -1, 1.414), (1, 0, 1.0), (1, 1, 1.414)]

    def __init__(self, grid, inflate, plan_res=0.1, wall_cost_dist=0.4, unknown_ok=False):
        self.grid = grid
        f = max(1, int(round(plan_res / grid.res)))
        self.f = f
        self.res = grid.res * f
        obstacle = grid.occupied | (grid.unknown if not unknown_ok else np.zeros_like(grid.occupied))
        d = (ndimage.distance_transform_edt(~obstacle) * grid.res) if obstacle.any() \
            else np.full(obstacle.shape, 1e3)
        h, w = (grid.h // f) * f, (grid.w // f) * f
        dmin = d[:h, :w].reshape(h // f, f, w // f, f).min(axis=(1, 3))
        self.dist = dmin
        self.blocked = dmin < inflate
        self.cost = 1.0 + 4.0 * np.clip((wall_cost_dist - dmin) / wall_cost_dist, 0, 1)
        self.H, self.W = self.blocked.shape

    def cell(self, x, y):
        r, c = self.grid.to_cell(x, y)
        return int(r) // self.f, int(c) // self.f

    def world(self, r, c):
        return (self.grid.ox + (c + 0.5) * self.res, self.grid.oy + (r + 0.5) * self.res)

    def ok(self, r, c):
        return 0 <= r < self.H and 0 <= c < self.W and not self.blocked[r, c]

    def nearest_free(self, r, c, max_cells=6):
        if self.ok(r, c):
            return r, c
        best = None
        for rr in range(r - max_cells, r + max_cells + 1):
            for cc in range(c - max_cells, c + max_cells + 1):
                if self.ok(rr, cc):
                    d = (rr - r) ** 2 + (cc - c) ** 2
                    if best is None or d < best[0]:
                        best = (d, rr, cc)
        return (best[1], best[2]) if best else None

    def search(self, start, goal=None):
        """Dijkstra / A*. Returns (dist array, parent dict). Stops early when goal reached."""
        dist = np.full((self.H, self.W), np.inf)
        parent = {}
        sr, sc = start
        dist[sr, sc] = 0.0
        h = (lambda r, c: math.hypot(r - goal[0], c - goal[1])) if goal else (lambda r, c: 0.0)
        pq = [(h(sr, sc), 0.0, sr, sc)]
        while pq:
            _, g, r, c = heapq.heappop(pq)
            if g > dist[r, c]:
                continue
            if goal and (r, c) == goal:
                break
            for dr, dc, step in self.NB:
                rr, cc = r + dr, c + dc
                if not self.ok(rr, cc):
                    continue
                ng = g + step * self.cost[rr, cc]
                if ng < dist[rr, cc]:
                    dist[rr, cc] = ng
                    parent[(rr, cc)] = (r, c)
                    heapq.heappush(pq, (ng + h(rr, cc), ng, rr, cc))
        return dist, parent

    def path_to(self, parent, start, goal):
        if goal != start and goal not in parent:
            return None
        cells = [goal]
        while cells[-1] != start:
            cells.append(parent[cells[-1]])
        cells.reverse()
        return [self.world(r, c) for r, c in cells]

    def plan(self, sx, sy, gx, gy):
        s = self.nearest_free(*self.cell(sx, sy))
        g = self.nearest_free(*self.cell(gx, gy))
        if s is None or g is None:
            return None
        _, parent = self.search(s, g)
        return self.path_to(parent, s, g)
