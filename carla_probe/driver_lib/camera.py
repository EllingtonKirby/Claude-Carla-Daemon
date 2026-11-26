"""Camera geometry: pixels <-> the ego frame, and where a pixel meets a flat road.

Ego frame: fwd (m, forward), left (m, left), up (m), origin on the ground under the car's centre.
Image sizes: "full" is <CAM>_full.jpg, "half" is <CAM>.jpg; each has its own intrinsics.

    from lib import camera
    cams = camera.load(frame_dir)                 # {"CAM_FRONT": Camera, ...}
    cam = cams["CAM_FRONT"]
    cam.ground_point(u, v)                        # (fwd, left) where pixel (u, v) meets the road, or None
    cam.to_pixel(fwd, left, up=0.0)               # (u, v), or None when behind the camera
    cam.point_at_depth(u, v, z)                   # (fwd, left, up) for a depth z along the optical axis
"""
from __future__ import annotations

import json
import math
import os
from typing import Dict, Optional, Tuple

import numpy as np


class Camera:
    def __init__(self, name: str, spec: dict, size: str = "full") -> None:
        self.name = name
        m = spec["mount"]
        self.mount = (float(m["x"]), -float(m["y"]), float(m["z"]))   # fwd, left, up
        self.yaw = math.radians(float(m.get("yaw", 0.0)))              # + = turned right (CARLA)
        self.pitch = math.radians(float(m.get("pitch", 0.0)))          # + = nose up
        key = "full_jpg" if size == "full" else "jpg"
        k = spec["intrinsics"][key]
        self.fx, self.fy, self.cx, self.cy = float(k["fx"]), float(k["fy"]), float(k["cx"]), float(k["cy"])
        self.width, self.height = int(k["width"]), int(k["height"])
        self.size = size
        # Camera axes in the ego frame (fwd, left, up).
        cp, sp, cy, sy = math.cos(self.pitch), math.sin(self.pitch), math.cos(self.yaw), math.sin(self.yaw)
        fwd = np.array([cp * cy, -cp * sy, sp])          # optical axis
        right = np.array([-sy, -cy, 0.0])                # image x
        up = np.array([-sp * cy, sp * sy, cp])
        self.R = np.stack([right, -up, fwd], axis=1)     # columns: image x, image y (down), optical axis
        self.C = np.array(self.mount)

    def rays(self, u, v) -> np.ndarray:
        """Unit-z rays (optical-axis depth 1) for pixels, in the ego frame. Shape (..., 3)."""
        u = np.asarray(u, float); v = np.asarray(v, float)
        d = np.stack([(u - self.cx) / self.fx, (v - self.cy) / self.fy, np.ones_like(u)], axis=-1)
        return d @ self.R.T

    def point_at_depth(self, u, v, z) -> np.ndarray:
        """Ego (fwd, left, up) of pixel (u, v) at depth z (metres along the optical axis)."""
        return self.C + self.rays(u, v) * np.asarray(z, float)[..., None]

    def ground_points(self, u, v, max_range: float = 80.0) -> np.ndarray:
        """Ego (fwd, left) where each pixel's ray meets the flat road (up = 0). NaN above the horizon
        or beyond max_range. Assumes a flat road at the car's own level: on a slope or a crest the
        error grows with distance."""
        r = self.rays(u, v)
        t = np.where(r[..., 2] < -1e-6, -self.C[2] / np.minimum(r[..., 2], -1e-6), np.nan)
        p = self.C[:2] + r[..., :2] * t[..., None]
        far = np.hypot(p[..., 0] - self.C[0], p[..., 1] - self.C[1]) > max_range
        p[far] = np.nan
        return p

    def ground_point(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        p = self.ground_points(np.array([u]), np.array([v]))[0]
        return None if np.isnan(p).any() else (float(p[0]), float(p[1]))

    def to_pixels(self, pts: np.ndarray) -> np.ndarray:
        """Ego (fwd, left, up) points -> (u, v, depth). Depth <= 0 means behind the camera."""
        q = (np.asarray(pts, float) - self.C) @ self.R          # image x, image y, depth
        z = q[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.cx + self.fx * q[..., 0] / z
            v = self.cy + self.fy * q[..., 1] / z
        return np.stack([u, v, z], axis=-1)

    def to_pixel(self, fwd: float, left: float, up: float = 0.0) -> Optional[Tuple[float, float]]:
        u, v, z = self.to_pixels(np.array([[fwd, left, up]]))[0]
        if z <= 0.1 or not (0 <= u < self.width and 0 <= v < self.height):
            return None
        return float(u), float(v)

    def image_path(self, frame_dir: str) -> str:
        return os.path.join(frame_dir, f"{self.name}_full.jpg" if self.size == "full" else f"{self.name}.jpg")


def load(frame_dir: str, size: str = "full") -> Dict[str, Camera]:
    """Cameras of a frame directory (reads <frame_dir>/../cameras.json)."""
    path = os.path.join(frame_dir, "..", "cameras.json")
    with open(path) as f:
        spec = json.load(f)["cameras"]
    return {name: Camera(name, s, size) for name, s in spec.items()}
