"""Plan a route (start -> waypoints -> end) around walls on an occupancy grid.

Shared by the dashboard (preview while editing) and route_follower (driving).
Each leg is planned with the grid planner: cells closer than robot_radius + margin to an
obstacle are blocked, cells near walls cost extra (paths keep to the middle of gaps), and
unknown cells are allowed but discouraged (SLAM maps are speckled with unknown cells).
The grid path is then shortened with line-of-sight shortcuts so the rover drives straight
lines between corners instead of a staircase.
"""
import math

import numpy as np
from scipy import ndimage

from .grid_utils import GridMap, Planner

SNAP_M = 0.6   # a point inside a wall / too close to one is moved to free space within this


def fill_speckle(grid, window_m=0.25, free_frac=0.3):
    """Unknown cells inside scanned floor (gaps between lidar rays) -> free. Unknown cells
    that are mostly surrounded by unknown (unexplored space) stay unknown."""
    k = max(3, int(round(window_m / grid.res)) | 1)
    frac = ndimage.uniform_filter(grid.free.astype(np.float32), size=k, mode='constant')
    d = grid.data.copy()
    d[grid.unknown & (frac >= free_frac)] = 0
    return GridMap(d, grid.res, (grid.ox, grid.oy))


def make_planner(grid, robot_radius, margin=0.05):
    grid = fill_speckle(grid)
    return Planner(grid, inflate=robot_radius + margin, plan_res=max(grid.res, 0.05),
                   unknown_ok=True, unknown_cost=2.0)


def _line_ok(pl, a, b, clearance):
    """Straight line between coarse cells a and b keeps `clearance` from obstacles and
    does not cut across unknown cells."""
    (r0, c0), (r1, c1) = a, b
    n = int(max(abs(r1 - r0), abs(c1 - c0)) * 2) + 1
    for t in np.linspace(0.0, 1.0, n):
        r = int(round(r0 + (r1 - r0) * t))
        c = int(round(c0 + (c1 - c0) * t))
        if not pl.ok(r, c) or pl.dist[r, c] < clearance or pl.unknown[r, c]:
            return False
    return True


def _shortcut(pl, cells, clearance):
    if len(cells) <= 2:
        return cells
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = len(cells) - 1
        while j > i + 1 and not _line_ok(pl, cells[i], cells[j], clearance):
            j -= 1
        out.append(cells[j])
        i = j
    return out


def plan_leg(pl, a, b, clearance):
    """a, b: (x, y). Returns dict(ok, path=[(x, y)...], length, note)."""
    snap = int(math.ceil(SNAP_M / pl.res))
    for pt, what in ((a, 'start of this leg'), (b, 'this point')):
        r, c = pl.cell(*pt)
        if not (-snap <= r < pl.H + snap and -snap <= c < pl.W + snap):
            return {'ok': False, 'path': [list(a), list(b)], 'length': 0.0,
                    'note': f'{what} is outside the map'}
    s = pl.nearest_free(*pl.cell(*a), max_cells=snap)
    g = pl.nearest_free(*pl.cell(*b), max_cells=snap)
    if s is None or g is None:
        bad = 'start of this leg' if s is None else 'this point'
        return {'ok': False, 'path': [list(a), list(b)], 'length': 0.0,
                'note': f'{bad} is inside or right next to a wall'}
    _, parent = pl.search(s, g)
    if g != s and g not in parent:
        return {'ok': False, 'path': [list(a), list(b)], 'length': 0.0,
                'note': 'no way through - walled off, or the gap is narrower than the rover'}
    cells = [g]
    while cells[-1] != s:
        cells.append(parent[cells[-1]])
    cells.reverse()
    cells = _shortcut(pl, cells, clearance)
    pts = [pl.world(r, c) for r, c in cells]
    # keep the exact requested end point when it is free, else the snapped one
    pts[0] = tuple(a) if pl.cell(*a) == s else pts[0]
    pts[-1] = tuple(b) if pl.cell(*b) == g else pts[-1]
    note = '' if pl.cell(*b) == g else 'point is too close to a wall - moved to free space'
    length = float(sum(math.hypot(x1 - x0, y1 - y0) for (x0, y0), (x1, y1) in zip(pts, pts[1:])))
    return {'ok': True, 'path': [[round(x, 3), round(y, 3)] for x, y in pts],
            'length': round(length, 2), 'note': note}


def plan_route(grid, points, robot_radius, planner=None):
    """points: [(x, y), ...] in order. Returns {'ok', 'legs': [...], 'length'}."""
    pl = planner or make_planner(grid, robot_radius)
    clearance = robot_radius + 0.10
    legs = [plan_leg(pl, a, b, clearance) for a, b in zip(points, points[1:])]
    return {'ok': all(lg['ok'] for lg in legs), 'legs': legs,
            'length': round(sum(lg['length'] for lg in legs), 2)}
