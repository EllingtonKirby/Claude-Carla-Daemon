"""Road users from the perception service, as footprints on the road.

    from lib import objects
    objs = objects.scan(frame_dir)                       # all four cameras, default prompts
    objs = objects.scan(frame_dir, cams=["CAM_FRONT"], prompts=["car", "traffic cone"])
    print(objects.table(objs))

Each object is a dict:
    label, score, cam          what SAM 3 called it, its confidence, which camera saw it
    near_fwd, near_left        the point of its footprint nearest to your car (m, ego frame)
    fwd_min, fwd_max           the footprint's extent along your heading
    left_min, left_max         ... and across it (left positive)
    depth_fwd, depth_left      the mask's median Depth Pro 2 point (a second, independent estimate)
    agree_m                    distance between the footprint centre and the depth point
    truncated                  the mask touches the image edge: the footprint is only partly seen
    mask_px                    mask size in pixels (small = far or occluded)

How the footprint is made: for every image column the mask covers, its lowest pixel is taken as the
object's underside and intersected with a flat road raised by BODY_BOTTOM_M (0.2 m for vehicles,
whose lowest visible edge is a bumper or sill; 0 for pedestrians, bicycles and cones). This is
what most drivers ended up writing by hand. It measures the near side and the width well; it cannot
see the far side, so fwd_max/left extents away from you are lower bounds. On slopes, kerbs and for
objects whose bottom is hidden (by a parked car, by the image edge) it is wrong: compare with the
depth point, and look (lib.zoom) when agree_m is large.

Needs the perception service (the prompt says when one is configured). ~0.7 s per camera.
"""
from __future__ import annotations

import os
from typing import Dict, Iterable, List, Optional

import numpy as np

from . import camera as _camera

DEFAULT_PROMPTS = ("car", "truck", "bus", "pedestrian", "bicycle", "motorcycle")
DEFAULT_CAMS = ("CAM_FRONT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK")
EGO_HALF_LEN, EGO_HALF_WID = 2.5, 1.0        # the ego body box, for dropping its own bonnet / boot

_pc = None


def _client():
    global _pc
    if _pc is None:
        from carla_probe.perception_client import PerceptionClient
        _pc = PerceptionClient()
    return _pc


# The lowest visible pixel of a vehicle is mostly the bottom of its bumper or sill, not the tyre
# contact, so it is intersected with a plane this far above the road (measured on CARLA ground-truth
# masks: 0.2 m gave the smallest nearest-point error, 1.0 m median against 1.5 m at road level).
BODY_BOTTOM_M = {"pedestrian": 0.0, "person": 0.0, "rider": 0.0, "bicycle": 0.0, "traffic cone": 0.0}
DEFAULT_BOTTOM_M = 0.2


def _footprint(mask: np.ndarray, cam: "_camera.Camera", bottom_m: float = DEFAULT_BOTTOM_M) -> Optional[dict]:
    ys, xs = np.nonzero(mask)
    if xs.size < 12:
        return None
    cols = np.unique(xs)
    if cols.size > 160:                                   # subsample wide masks
        cols = cols[:: cols.size // 160 + 1]
    bottom = np.array([ys[xs == c].max() for c in cols])
    r = cam.rays(cols.astype(float), bottom.astype(float) + 0.5)
    t = np.where(r[:, 2] < -1e-6, (bottom_m - cam.C[2]) / np.minimum(r[:, 2], -1e-6), np.nan)
    pts = cam.C[:2] + r[:, :2] * t[:, None]
    pts[np.hypot(pts[:, 0], pts[:, 1]) > 80] = np.nan
    ok = ~np.isnan(pts).any(axis=1)
    if ok.sum() < 3:
        return None
    pts = pts[ok]
    d = np.hypot(pts[:, 0], pts[:, 1])
    i = int(np.argmin(d))
    h, w = mask.shape
    truncated = bool(xs.min() <= 1 or xs.max() >= w - 2 or ys.max() >= h - 2)
    return {"near_fwd": round(float(pts[i, 0]), 2), "near_left": round(float(pts[i, 1]), 2),
            "fwd_min": round(float(pts[:, 0].min()), 2), "fwd_max": round(float(pts[:, 0].max()), 2),
            "left_min": round(float(pts[:, 1].min()), 2), "left_max": round(float(pts[:, 1].max()), 2),
            "centre": pts.mean(axis=0), "truncated": truncated}


def _is_ego(fp: dict) -> bool:
    return abs(fp["near_fwd"]) < EGO_HALF_LEN + 0.3 and abs(fp["near_left"]) < EGO_HALF_WID + 0.2 and \
        abs(fp["fwd_min"]) < EGO_HALF_LEN + 0.6 and abs(fp["fwd_max"]) < EGO_HALF_LEN + 0.6


def scan_camera(frame_dir: str, cam_name: str, prompts: Iterable[str] = DEFAULT_PROMPTS,
                with_depth: bool = True) -> List[dict]:
    cam = _camera.load(frame_dir)[cam_name]
    path = cam.image_path(frame_dir)
    pc = _client()
    labels, segs = pc.segment(path, prompts=list(prompts))
    depth = pc.depth(path, f_px=cam.fx) if with_depth else None
    out = []
    for s in segs:
        m = labels == s["id"]
        fp = _footprint(m, cam, BODY_BOTTOM_M.get(s["label"].lower(), DEFAULT_BOTTOM_M))
        if fp is None or _is_ego(fp):
            continue
        o = {"label": s["label"], "score": s["score"], "cam": cam_name, "mask_px": int(m.sum()), **fp}
        if depth is not None:
            v, u = np.nonzero(m)
            z = depth[v, u]
            good = np.isfinite(z) & (z > 0.5) & (z < 150)
            if good.sum() >= 6:
                p = np.median(cam.point_at_depth(u[good], v[good], z[good]), axis=0)
                o["depth_fwd"], o["depth_left"] = round(float(p[0]), 2), round(float(p[1]), 2)
                o["agree_m"] = round(float(np.hypot(*(p[:2] - fp["centre"]))), 2)
        o["bbox"] = s.get("bbox")
        out.append(o)
    return out


def scan(frame_dir: str, cams: Iterable[str] = DEFAULT_CAMS, prompts: Iterable[str] = DEFAULT_PROMPTS,
         with_depth: bool = True, merge_m: float = 1.5) -> List[dict]:
    """Objects from several cameras, merged where two cameras see the same one (same label, near
    points within merge_m), sorted by distance of the nearest point."""
    objs: List[dict] = []
    for c in cams:
        if os.path.exists(os.path.join(frame_dir, f"{c}_full.jpg")):
            objs += scan_camera(frame_dir, c, prompts, with_depth)
    objs.sort(key=lambda o: -o["mask_px"])
    kept: List[dict] = []
    for o in objs:
        if not any(k["label"] == o["label"] and
                   np.hypot(k["near_fwd"] - o["near_fwd"], k["near_left"] - o["near_left"]) < merge_m for k in kept):
            kept.append(o)
    for o in kept:
        o.pop("centre", None)
    kept.sort(key=lambda o: np.hypot(o["near_fwd"], o["near_left"]))
    return kept


def table(objs: List[dict]) -> str:
    """One line per object, for printing."""
    rows = ["label        cam              near fwd/left    fwd span      left span     depth fwd/left  agree  trunc"]
    for o in objs:
        rows.append(f"{o['label'][:12]:12} {o['cam']:16} {o['near_fwd']:6.1f} {o['near_left']:+6.1f}   "
                    f"{o['fwd_min']:5.1f}..{o['fwd_max']:<5.1f}  {o['left_min']:+5.1f}..{o['left_max']:<+5.1f}  "
                    f"{o.get('depth_fwd', float('nan')):6.1f} {o.get('depth_left', float('nan')):+6.1f}  "
                    f"{o.get('agree_m', float('nan')):5.1f}  {'yes' if o['truncated'] else ''}")
    return "\n".join(rows)
