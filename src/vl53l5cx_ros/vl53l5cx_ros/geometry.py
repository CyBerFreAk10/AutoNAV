"""Mounting geometry for a downward-tilted VL53L5CX.

Frames: base frame x forward, y left, z up, z = 0 on the floor.
Zone (r, c): row 0 = TOP of the scene, col 0 = LEFT (after the driver's flip settings).
tilt_down_deg: angle of the sensor's optical axis BELOW horizontal
               (0 = looking straight ahead, 90 = looking straight down).
range_model:   'radial'        -> reported distance is along each zone's ray
               'perpendicular' -> reported distance is along the optical axis
"""
import math

import numpy as np


def zone_rays(n, fov_deg=45.0):
    """Per-zone ray directions in the (untilted) sensor frame, shape (n, n, 3).
    Returns (unit, pinhole) where pinhole has x == 1."""
    step = math.radians(fov_deg) / n
    idx = (n - 1) / 2.0 - np.arange(n)
    elev = idx[:, None] * step * np.ones((1, n))   # + up (row 0 top)
    azim = np.ones((n, 1)) * idx[None, :] * step   # + left (col 0 left)
    pin = np.stack([np.ones((n, n)), np.tan(azim), np.tan(elev)], axis=-1)
    unit = pin / np.linalg.norm(pin, axis=-1, keepdims=True)
    return unit, pin


def tilt(v, tilt_down_rad):
    """Rotate sensor-frame vectors so the optical axis points tilt_down below horizontal."""
    c, s = math.cos(tilt_down_rad), math.sin(tilt_down_rad)
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    return np.stack([x * c + z * s, y, -x * s + z * c], axis=-1)


class TofGeometry:
    def __init__(self, n, height_m, tilt_down_deg, range_model='radial',
                 fov_deg=45.0, x_offset_m=0.0, y_offset_m=0.0):
        self.n = n
        self.h = height_m
        self.tilt_deg = tilt_down_deg
        self.model = range_model
        self.offset = np.array([x_offset_m, y_offset_m, height_m])
        unit, pin = zone_rays(n, fov_deg)
        base = unit if range_model == 'radial' else pin
        # Point (base frame) = offset + range * ray
        self.ray = tilt(base, math.radians(tilt_down_deg))
        self.unit_ray = tilt(unit, math.radians(tilt_down_deg))
        down = -self.ray[..., 2]
        with np.errstate(divide='ignore'):
            # Range at which each zone would hit a flat floor (inf if it looks above horizon).
            self.floor_range = np.where(down > 1e-3, height_m / np.maximum(down, 1e-9), np.inf)

    def points(self, ranges_m):
        """ranges_m: (n, n) metres (NaN = invalid) -> (n, n, 3) base-frame points."""
        return self.offset + ranges_m[..., None] * self.ray

    def floor_points(self):
        r = np.where(np.isfinite(self.floor_range), self.floor_range, np.nan)
        return self.points(r)

    def describe(self, max_range_m=3.0):
        fp = self.floor_points()
        lines = [f'height {self.h:.3f} m, tilt {self.tilt_deg:.1f} deg below horizontal, '
                 f'{self.model} ranges']
        for r in range(self.n):
            fr = self.floor_range[r, self.n // 2]
            if not np.isfinite(fr) or fr > max_range_m:
                lines.append(f'  row {r}: looks above the floor / beyond {max_range_m} m '
                             f'-> obstacle zone only')
            else:
                xs = fp[r, :, 0]
                ys = fp[r, :, 1]
                lines.append(f'  row {r}: hits floor {xs.min():.2f}-{xs.max():.2f} m ahead, '
                             f'width {ys.max() - ys.min():.2f} m (range {fr:.2f} m)')
        return '\n'.join(lines)


def fit_floor(ranges_m, n, fov_deg=45.0):
    """Fit mounting height + tilt from one floor-only frame (n x n, NaN invalid).
    Tries both range models; returns list of (residual, height, tilt_deg, model) best first."""
    unit, pin = zone_rays(n, fov_deg)
    ok = np.isfinite(ranges_m)
    results = []
    for model, base in (('radial', unit), ('perpendicular', pin)):
        best = None
        for t in np.arange(0.0, 90.01, 0.25):
            down = -tilt(base, math.radians(t))[..., 2]
            m = ok & (down > 0.05)
            if m.sum() < 12:
                continue
            h_i = ranges_m[m] * down[m]
            h = float(np.median(h_i))
            if h <= 0.005:
                continue
            res = float(np.median(np.abs(h_i - h)) / h)
            if best is None or res < best[0]:
                best = (res, h, float(t), model)
        if best:
            results.append(best)
    return sorted(results)
