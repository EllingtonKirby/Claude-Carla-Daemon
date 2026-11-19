"""Where will the ego be if it runs this plan? A measured model of the daemon's controller.

The driver chooses a path; the plan controller (pid.py) and CARLA's physics decide how the car
actually moves along it. On 2026-11-18 the privileged driver lost points it could have kept because
it did not know that motion: it left 0.5 m of clearance where the car strays ~0.4 m off a planned
line, and committed a left turn on a launch slower than it assumed.

This predicts the ego's motion for a plan from tools/controller_model.json, which
tools/characterize_controller.py measured by driving the real car through the real controller:

  speed     follows the measured launch curve toward the target (entered at the car's current
            speed) or the measured braking curve when the controller would brake
  position  advances along the planned path, from the ego origin, at that speed; past the last
            waypoint it continues straight
  band      the measured cross-track error (95th percentile) for this speed and the path's
            curvature: the prediction says "here, give or take band_m sideways"

It is a fit to measurements, not a simulation: it does not model wheel slip, a second car in the
way, or a path the controller cannot physically follow (it reports a curvature beyond any measured
one as `beyond_measured`). Pure numpy, no simulator: unit-tested offline.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

DT = 0.05


class MotionModel:
    def __init__(self, table: Dict[str, Any], config: str, brake_speed: float, brake_ratio: float) -> None:
        assert config in table["configs"], f"controller_model.json has no config {config!r}"
        c = table["configs"][config]
        self.config = config
        self.brake_speed, self.brake_ratio = brake_speed, brake_ratio
        self.launch = {float(k): np.asarray(v, float) for k, v in c["launch"].items()}
        self.brake = {float(k): np.asarray(v, float) for k, v in c["brake"].items()}
        rows = c.get("offset", []) + c.get("arc", [])
        self._err = np.array([[r["speed"], r.get("curvature", r.get("peak_curvature", 0.0)), r["p95"]] for r in rows], float) \
            if rows else np.zeros((0, 3))
        self.max_curvature = float(self._err[:, 1].max()) if len(self._err) else 0.0

    @classmethod
    def load(cls, path: Path, config: str, brake_speed: float, brake_ratio: float) -> "MotionModel":
        return cls(json.loads(Path(path).read_text()), config, brake_speed, brake_ratio)

    # ------------------------------------------------------------------ longitudinal
    def _follow(self, curve: np.ndarray, v: float, rising: bool) -> int:
        """Index on `curve` where the car is now: first sample at/above v (rising) or at/below v."""
        idx = np.nonzero(curve >= v - 1e-6)[0] if rising else np.nonzero(curve <= v + 1e-6)[0]
        return int(idx[0]) if len(idx) else len(curve) - 1

    def speed_profile(self, v0: float, target: float, ticks: int) -> np.ndarray:
        braking = target < self.brake_speed or (target > 0 and v0 / target > self.brake_ratio)
        out = np.empty(ticks)
        if braking:
            v_from = min((k for k in self.brake if k >= v0), default=max(self.brake))
            curve = self.brake[v_from] * (v0 / v_from if v_from > 0 else 1.0) if v0 > v_from else self.brake[v_from]
            i = self._follow(curve, v0, rising=False)
            for t in range(ticks):
                v = float(curve[min(i + t + 1, len(curve) - 1)])
                out[t] = max(v, target if target >= self.brake_speed else 0.0)
            return out
        # launch / cruise: the launch curve for the target (interpolated between measured targets)
        ks = sorted(self.launch)
        hi = min((k for k in ks if k >= target), default=ks[-1])
        lo = max((k for k in ks if k <= target), default=ks[0])
        w = 0.0 if hi == lo else (target - lo) / (hi - lo)
        n = min(len(self.launch[lo]), len(self.launch[hi]))
        curve = (1 - w) * self.launch[lo][:n] + w * self.launch[hi][:n]
        if target > ks[-1]:
            curve = curve * (target / ks[-1])
        if v0 >= 0.85 * target:
            # Already near the target: a cruising car settles onto it. The launch curves' overshoot
            # belongs to a launch from standstill, not to a car already at speed.
            for t in range(ticks):
                out[t] = target + (v0 - target) * math.exp(-(t + 1) * DT / 0.4)
            return out
        i = self._follow(curve, v0, rising=True)
        for t in range(ticks):
            out[t] = float(curve[min(i + t + 1, n - 1)])
        return out

    # ------------------------------------------------------------------ lateral
    def band(self, speed: float, curvature: float) -> float:
        """Measured p95 cross-track error, interpolated by inverse distance in (speed, curvature)."""
        if not len(self._err):
            return 0.3
        d = np.hypot((self._err[:, 0] - speed) / 3.0, (self._err[:, 1] - abs(curvature)) / 0.03)
        w = 1.0 / (d + 0.05) ** 2
        return float(max(0.05, np.sum(w * self._err[:, 2]) / np.sum(w)))

    # ------------------------------------------------------------------ the prediction
    def predict(self, plan_ego: np.ndarray, speed: float, target_speed: Optional[float], plan_interval_s: float,
                ticks: int) -> Dict[str, Any]:
        wps = np.asarray(plan_ego, float).reshape(-1, 2)
        assert len(wps) >= 2, "plan needs >= 2 waypoints"
        if target_speed is None:
            spacing = float(np.linalg.norm(np.diff(wps, axis=0), axis=1).sum())
            target_speed = spacing / (plan_interval_s * (len(wps) - 1))
        pts = np.vstack([[0.0, 0.0], wps])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        headings = np.arctan2(np.diff(pts[:, 1]), np.diff(pts[:, 0]))
        # curvature per segment: heading change over the mean length of the two segments either side
        curv = np.zeros(len(seg))
        for j in range(1, len(seg)):
            dh = math.atan2(math.sin(headings[j] - headings[j - 1]), math.cos(headings[j] - headings[j - 1]))
            curv[j] = dh / max((seg[j] + seg[j - 1]) / 2, 1e-3)
        vs = self.speed_profile(speed, float(target_speed), ticks)
        s, poses, beyond = 0.0, [], False
        for t in range(ticks):
            s += float(vs[t]) * DT
            if s <= cum[-1]:
                j = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
                frac = (s - cum[j]) / max(seg[j], 1e-9)
                p = pts[j] + frac * (pts[j + 1] - pts[j])
                h, k = headings[j], curv[j]
            else:
                h, k = headings[-1], 0.0
                p = pts[-1] + (s - cum[-1]) * np.array([math.cos(h), math.sin(h)])
            beyond = bool(beyond or (self.max_curvature > 0 and abs(float(k)) > self.max_curvature * 1.25))
            poses.append({"t": round((t + 1) * DT, 2), "fwd": round(float(p[0]), 2), "left": round(float(p[1]), 2),
                          "yaw_rel_deg": round(math.degrees(h), 1), "speed": round(float(vs[t]), 2),
                          "band_m": round(self.band(float(vs[t]), k), 2)})
        return {"config": self.config, "target_speed": round(float(target_speed), 2), "poses": poses,
                "end_speed": poses[-1]["speed"] if poses else speed, "distance_m": round(s, 2),
                "plan_length_m": round(float(cum[-1]), 2), "reaches_plan_end": bool(s >= cum[-1]),
                "max_band_m": max((p["band_m"] for p in poses), default=0.0), "beyond_measured": beyond}


def box_corners(fwd: float, left: float, yaw_deg: float, half_len: float, half_wid: float, rear: Optional[float] = None) -> np.ndarray:
    """Corners of an oriented box in the ego frame ([fwd, left], yaw + = turned left). `rear` lets
    the ego's own body be asymmetric about its origin (front_m != rear_m)."""
    rear = half_len if rear is None else rear
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    local = np.array([[half_len, half_wid], [half_len, -half_wid], [-rear, -half_wid], [-rear, half_wid]])
    return np.stack([fwd + local[:, 0] * c - local[:, 1] * s, left + local[:, 0] * s + local[:, 1] * c], axis=1)


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def poly_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Distance between two convex polygons; 0 when they touch or overlap (separating-axis test)."""
    for poly in (a, b):
        for i in range(len(poly)):
            edge = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                break
        else:
            continue
        break
    else:
        return 0.0
    best = math.inf
    for p, q in ((a, b), (b, a)):
        for pt in p:
            for i in range(len(q)):
                best = min(best, _seg_dist(pt, q[i], q[(i + 1) % len(q)]))
    return best
