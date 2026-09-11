"""Top-down bird's-eye-view renderer. Pure numpy + cv2 (no carla): the agent converts the scene into
ego-frame primitives and this module only draws them, so it is testable off the simulator.

Image convention: ego FORWARD is UP, ego LEFT is LEFT, 10 m grid. Colours are cv2 BGR.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

Poly = np.ndarray  # (K, 2) ego-frame [forward, left]

_BG = (245, 245, 245)
_LANE = (208, 208, 208)
_JUNCTION = (190, 196, 204)
_GRID = (228, 228, 228)
_ROUTE = (255, 200, 130)
_ACTOR = {
    "vehicle": (40, 140, 230),
    "walker": (210, 60, 240),
    "static": (140, 140, 140),
    "parked": (60, 100, 160),
}
_TL = {"Red": (40, 40, 220), "Yellow": (0, 200, 230), "Green": (60, 200, 60)}
_STOP = (0, 0, 200)
_ROUTE_PT = (255, 0, 255)
_PLAN = (200, 170, 0)
_EGO = (0, 0, 235)


def render_bev(
    out_path: Path,
    *,
    forward_m: float,
    back_m: float,
    side_m: float,
    px_per_m: float,
    lanes: Sequence[Tuple[bool, Poly]],
    route: Optional[np.ndarray],
    actors: Sequence[Tuple[str, Poly]],
    traffic_lights: Sequence[Tuple[str, float, float]],
    stop_signs: Sequence[Tuple[float, float]],
    route_points: Optional[np.ndarray],
    plan: Optional[np.ndarray],
    ego_poly: Poly,
    header: str,
) -> str:
    """Draw the scene and write a PNG. ``lanes`` are ``(is_junction, polygon)``; ``actors`` are
    ``(kind, footprint)`` with kind in vehicle|walker|static|parked; ``traffic_lights`` are
    ``(state, fwd, left)``. Returns the absolute output path."""
    ppm = px_per_m
    width = int(round(2 * side_m * ppm))
    height = int(round((forward_m + back_m) * ppm))

    def to_px(pts: np.ndarray) -> np.ndarray:
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        col = (side_m - p[:, 1]) * ppm
        row = (forward_m - p[:, 0]) * ppm
        return np.round(np.stack([col, row], axis=-1)).astype(np.int32).reshape(-1, 1, 2)

    def pt(fwd: float, left: float) -> Tuple[int, int]:
        q = to_px(np.array([[fwd, left]]))[0, 0]
        return int(q[0]), int(q[1])

    canvas = np.full((height, width, 3), _BG, dtype=np.uint8)

    for is_junction, poly in lanes:
        cv2.fillPoly(canvas, [to_px(poly)], _JUNCTION if is_junction else _LANE)

    for g in range(-int(back_m // 10) * 10, int(forward_m) + 1, 10):
        r = int(round((forward_m - g) * ppm))
        cv2.line(canvas, (0, r), (width, r), _GRID, 1)
    for g in range(-int(side_m // 10) * 10, int(side_m) + 1, 10):
        c = int(round((side_m - g) * ppm))
        cv2.line(canvas, (c, 0), (c, height), _GRID, 1)

    if route is not None and len(route) >= 2:
        cv2.polylines(canvas, [to_px(route)], False, _ROUTE, max(2, int(0.8 * ppm)))

    for kind, poly in actors:
        pp = to_px(poly)
        cv2.fillPoly(canvas, [pp], _ACTOR.get(kind, _ACTOR["static"]))
        cv2.polylines(canvas, [pp], True, (30, 30, 30), 1)

    radius = max(4, int(0.6 * ppm))
    for state, fwd, left in traffic_lights:
        c = pt(fwd, left)
        cv2.circle(canvas, c, radius, _TL.get(state, (120, 120, 120)), -1)
        cv2.circle(canvas, c, radius, (0, 0, 0), 1)
    for fwd, left in stop_signs:
        cv2.drawMarker(canvas, pt(fwd, left), _STOP, cv2.MARKER_TRIANGLE_DOWN, max(10, int(1.0 * ppm)), 2)

    if route_points is not None:
        for fwd, left in np.asarray(route_points).reshape(-1, 2):
            cv2.drawMarker(canvas, pt(fwd, left), _ROUTE_PT, cv2.MARKER_TILTED_CROSS, max(8, int(0.7 * ppm)), 2)

    if plan is not None and len(plan) >= 1:
        path = np.vstack([np.zeros((1, 2)), np.asarray(plan).reshape(-1, 2)])
        cv2.polylines(canvas, [to_px(path)], False, _PLAN, 2)
        for fwd, left in path[1:]:
            cv2.circle(canvas, pt(fwd, left), 3, _PLAN, -1)

    ep = to_px(ego_poly)
    cv2.fillPoly(canvas, [ep], _EGO)
    cv2.polylines(canvas, [ep], True, (0, 0, 0), 1)

    cv2.rectangle(canvas, (0, 0), (width, 24), (255, 255, 255), -1)
    cv2.putText(canvas, header, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    legend = "up=fwd grid=10m | lane=grey route=blue veh=orange walker=pink parked=brown TL=dot stop=tri tgt=X plan=teal ego=red"
    cv2.rectangle(canvas, (0, height - 20), (width, height), (255, 255, 255), -1)
    cv2.putText(canvas, legend, (6, height - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (70, 70, 70), 1, cv2.LINE_AA)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), canvas)
    return str(out_path.resolve())


def box_polygon(center_fwd: float, center_left: float, yaw_rel_rad: float, half_len: float,
                half_width: float) -> Poly:
    """Rectangle footprint in the ego frame; ``yaw_rel_rad`` is the box heading, CCW-positive
    (toward +left) in the ego frame."""
    c, s = np.cos(yaw_rel_rad), np.sin(yaw_rel_rad)
    local = np.array([[half_len, half_width], [half_len, -half_width], [-half_len, -half_width], [-half_len, half_width]])
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.array([center_fwd, center_left])


def lanes_to_polys(centers: np.ndarray, yaws_rel_rad: np.ndarray, widths: np.ndarray, step_m: float,
                   junction: np.ndarray) -> List[Tuple[bool, Poly]]:
    """One rectangle per lane-centre sample (slightly overlong so consecutive samples overlap)."""
    half = 0.5 * step_m + 0.25
    return [
        (bool(j), box_polygon(float(c[0]), float(c[1]), float(y), half, 0.5 * float(w)))
        for c, y, w, j in zip(np.asarray(centers).reshape(-1, 2), yaws_rel_rad, widths, junction)
    ]
