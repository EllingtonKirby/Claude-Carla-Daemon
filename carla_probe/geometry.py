"""Frame conventions and route geometry. Pure numpy: no carla import, unit-tested off the simulator.

TWO frames, and only two:

* **CARLA world** (what the simulator reports): LEFT-HANDED. x forward-ish, y to the RIGHT of +x,
  yaw in DEGREES, positive = clockwise seen from above. A transform's forward vector is
  ``(cos yaw, sin yaw)`` and its right vector is ``(-sin yaw, cos yaw)``.
* **Ego frame** (what clients reason and command in): x = FORWARD, y = LEFT, metres, origin at the
  ego's CARLA location. ``get_state`` ego-frame fields, ``render_bev`` and ``apply_plan`` waypoints
  all use it. A point with ``left > 0`` is on the ego's left regardless of world heading.

With privileged access the ego pose is the hero actor's CARLA transform. With sensors-only access
it is estimated from GNSS + compass (functions at the bottom), expressed in the same CARLA world
frame, and the route is projected through the same GNSS conversion so both share one frame.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np


def world_to_ego(ego_xy: Sequence[float], ego_yaw_deg: float, pts_world: np.ndarray) -> np.ndarray:
    """CARLA world points (N,2) -> ego frame (N,2) as [forward, left]."""
    yaw = math.radians(ego_yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    d = np.asarray(pts_world, dtype=np.float64).reshape(-1, 2) - np.asarray(ego_xy, dtype=np.float64)[:2]
    fwd = d[:, 0] * c + d[:, 1] * s
    left = d[:, 0] * s - d[:, 1] * c  # = -(right), right = -dx*s + dy*c
    return np.stack([fwd, left], axis=-1)


def ego_to_world(ego_xy: Sequence[float], ego_yaw_deg: float, pts_ego: np.ndarray) -> np.ndarray:
    """Ego-frame points (N,2) [forward, left] -> CARLA world (N,2). Inverse of :func:`world_to_ego`."""
    yaw = math.radians(ego_yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(pts_ego, dtype=np.float64).reshape(-1, 2)
    fwd, left = p[:, 0], p[:, 1]
    x = fwd * c + left * s
    y = fwd * s - left * c
    return np.stack([x, y], axis=-1) + np.asarray(ego_xy, dtype=np.float64)[:2]


def vec_world_to_ego(ego_yaw_deg: float, vec_world: Sequence[float]) -> Tuple[float, float]:
    """A world-frame vector (e.g. velocity) -> ego frame (forward, left). No translation."""
    fwd_left = world_to_ego((0.0, 0.0), ego_yaw_deg, np.asarray(vec_world, dtype=np.float64)[:2])
    return float(fwd_left[0, 0]), float(fwd_left[0, 1])


def wrap_deg(a: float) -> float:
    """Wrap an angle to (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def order_footprint(pts: np.ndarray) -> np.ndarray:
    """Collapse a box's (8 or 4) corner points to its unique footprint corners, ordered around the
    centroid, so a filled polygon draws the rectangle rather than a bowtie."""
    p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    _uniq, idx = np.unique(np.round(p, 3), axis=0, return_index=True)
    corners = p[np.sort(idx)]
    c = corners.mean(axis=0)
    order = np.argsort(np.arctan2(corners[:, 1] - c[1], corners[:, 0] - c[0]))
    return corners[order]


def cumulative_arclength(route_xy: np.ndarray) -> np.ndarray:
    seg = np.linalg.norm(np.diff(np.asarray(route_xy, dtype=np.float64), axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def project_on_route(route_xy: np.ndarray, point_xy: Sequence[float], hint_idx: int = 0,
                     window: int = -1) -> Tuple[int, float, float]:
    """Project a point onto a dense route polyline.

    Searches segments ``[hint_idx - 10, hint_idx + window]`` (``window < 0`` = to the end) so a
    route that loops back near itself does not snap the ego to a later/earlier pass.

    Returns ``(segment_index, arclength_s, lateral_offset_m)``; lateral is signed in the CARLA
    left-handed sense converted to "left of the route direction is positive".
    """
    r = np.asarray(route_xy, dtype=np.float64)
    assert r.ndim == 2 and len(r) >= 2, "route needs >= 2 points"
    p = np.asarray(point_xy, dtype=np.float64)[:2]
    lo = max(0, hint_idx - 10)
    hi = len(r) - 1 if window < 0 else min(len(r) - 1, hint_idx + window)
    a, b = r[lo:hi], r[lo + 1:hi + 1]
    ab = b - a
    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    t = np.clip(np.sum((p - a) * ab, axis=1) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    d2 = np.sum((proj - p) ** 2, axis=1)
    k = int(np.argmin(d2))
    seg = lo + k
    s = float(cumulative_arclength(r)[seg] + t[k] * math.sqrt(denom[k]))
    # CARLA left-handed: right of direction (dx,dy) is (-dy,dx); left is (dy,-dx).
    dirv = ab[k] / math.sqrt(denom[k])
    off = p - proj[k]
    lateral_left = float(off[0] * dirv[1] - off[1] * dirv[0])
    return seg, s, lateral_left


def point_at_arclength(route_xy: np.ndarray, s: float) -> np.ndarray:
    """Point at arclength ``s`` along the route; extrapolates along the last segment past the end."""
    r = np.asarray(route_xy, dtype=np.float64)
    cum = cumulative_arclength(r)
    if s <= 0.0:
        return r[0].copy()
    if s >= cum[-1]:
        tangent = r[-1] - r[-2]
        n = float(np.linalg.norm(tangent))
        return r[-1] + (s - cum[-1]) * (tangent / n if n > 1e-9 else 0.0)
    j = int(np.searchsorted(cum, s, side="right")) - 1
    frac = (s - cum[j]) / max(cum[j + 1] - cum[j], 1e-12)
    return r[j] + frac * (r[j + 1] - r[j])


def route_points_ahead(route_xy: np.ndarray, s0: float, distances: Sequence[float]) -> np.ndarray:
    """World points at arclength ``s0 + d`` for each ``d`` in ``distances`` (N,2)."""
    return np.stack([point_at_arclength(route_xy, s0 + float(d)) for d in distances], axis=0)


# ------------------------------------------------------------------------------------------------
# GNSS / compass (sensors-only access). CARLA's GNSS sensor maps world <-> lat/lon with a Mercator
# projection about the town's geo reference (carla/geom/GeoLocation.cpp); these are exact inverses.

EARTH_RADIUS_EQUA = 6378137.0


def _merc_lat(lat_deg: float) -> float:
    return math.log(math.tan((90.0 + lat_deg) * math.pi / 360.0))


def world_to_gps(x: float, y: float, lat_ref: float, lon_ref: float) -> Tuple[float, float]:
    """CARLA world (x, y) -> (lat, lon) degrees, as the GNSS sensor computes it (noise-free)."""
    scale = math.cos(math.radians(lat_ref))
    mx = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0 + x
    my = scale * EARTH_RADIUS_EQUA * _merc_lat(lat_ref) - y
    lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    lat = 360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0
    return lat, lon


def gps_to_world(lat: float, lon: float, lat_ref: float, lon_ref: float) -> Tuple[float, float]:
    """(lat, lon) degrees -> CARLA world (x, y). Inverse of :func:`world_to_gps`."""
    scale = math.cos(math.radians(lat_ref))
    x = scale * EARTH_RADIUS_EQUA * math.pi * (lon - lon_ref) / 180.0
    y = scale * EARTH_RADIUS_EQUA * (_merc_lat(lat_ref) - _merc_lat(lat))
    return x, y


def solve_geo_reference(x: float, y: float, lat: float, lon: float) -> Tuple[float, float]:
    """Recover the town's (lat_ref, lon_ref) from ONE (world, gps) correspondence, e.g. the first
    point of the leaderboard's global plan, which is handed to every agent in both forms.

    The y equation involves lat_ref alone and is monotone in it over (-85, 85) deg, so bisect it;
    lon_ref then follows in closed form."""
    def f(r: float) -> float:
        return math.cos(math.radians(r)) * EARTH_RADIUS_EQUA * (_merc_lat(r) - _merc_lat(lat)) - y

    lo, hi, f_lo = -85.0, 85.0, f(-85.0)
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        f_mid = f(mid)
        if (f_mid < 0.0) == (f_lo < 0.0):
            lo, f_lo = mid, f_mid
        else:
            hi = mid
    lat_ref = 0.5 * (lo + hi)
    lon_ref = lon - x * 180.0 / (math.pi * EARTH_RADIUS_EQUA * math.cos(math.radians(lat_ref)))
    return lat_ref, lon_ref


def compass_to_yaw_deg(compass_rad: float) -> float:
    """IMU compass (radians clockwise from north; north is world -y) -> CARLA world yaw (degrees)."""
    return wrap_deg(math.degrees(compass_rad) - 90.0)


def predict_blend(prev_xy: Optional[Sequence[float]], speed: float, yaw_deg: float, dt: float,
                  meas_xy: Sequence[float], gain: float) -> Tuple[float, float]:
    """One step of a complementary position filter: dead-reckon the previous estimate with speed and
    heading, then pull it toward the GNSS measurement by ``gain`` (1.0 = raw GNSS)."""
    if prev_xy is None or gain >= 1.0:
        return float(meas_xy[0]), float(meas_xy[1])
    yaw = math.radians(yaw_deg)
    px = prev_xy[0] + speed * dt * math.cos(yaw)
    py = prev_xy[1] + speed * dt * math.sin(yaw)
    return px + gain * (meas_xy[0] - px), py + gain * (meas_xy[1] - py)
