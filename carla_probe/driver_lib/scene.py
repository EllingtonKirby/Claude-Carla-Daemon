"""The scene as TransFuser v6 sees it: road users as boxes, and a bird's-eye map of road, lane
markings and traffic-light states. Available only when the session runs the TransFuser service
(the prompt says so; otherwise every call raises with that message).

    from lib import scene
    s = scene.read(frame_dir)                 # ~1.5 s
    print(scene.describe(s))
    s["objects"]      # boxes, nearest first
    s["lanes"]        # same format as lanes.markings(): lines, centre_left, width, heading_deg
    s["lights"]       # traffic-light cells ahead: state, nearest fwd/left
    scene.is_road(s, fwd, left)               # True / False from the map
    png = scene.render(s)                     # a picture of the map and the boxes, to look at

TransFuser v6 is a driving network trained on CARLA; only its perception outputs are used here. It
looks through its own ring of six cameras (TFV6_1 ... TFV6_6, also yours to view). The map covers
32 m behind to 64 m ahead and 40 m to each side, at 0.25 m.

Each object: class (vehicle, walker, biker, parking = a parked vehicle, special = an emergency
vehicle, obstacle = cones / barriers / debris, traffic_light, stop_sign), fwd / left (box centre),
yaw_rel_deg (+ = turned left, about 180 = oncoming), length_m / width_m, near_fwd / near_left (the
box corner or edge point nearest you), speed_mps (rough), braking, score.
"""
from __future__ import annotations

import base64
import io
import json
import math
import os
import urllib.request
from typing import Dict, List

import cv2
import numpy as np

from . import lanes as _lanes

MIN_FWD, MAX_FWD, MIN_RIGHT, MAX_RIGHT, PX_PER_M = -32.0, 64.0, -40.0, 40.0, 4.0
LIGHT_STATES = {"traffic_green": "green", "traffic_red_normal": "red", "traffic_red_not_normal": "red"}


def _post(frame_dir: str, score_min: float) -> Dict:
    addr = os.environ.get("CARLA_PROBE_PERCEPTION_ADDR", "")
    if not addr:
        raise RuntimeError("no perception service in this session")
    path = os.path.join(frame_dir, "TFV6_rgb.jpg")
    if not os.path.exists(path):
        raise RuntimeError("this session has no TransFuser camera ring (no TFV6_rgb.jpg in the frame)")
    with open(path, "rb") as f:
        body = json.dumps({"image_b64": base64.b64encode(f.read()).decode(), "score_min": score_min}).encode()
    req = urllib.request.Request(f"http://{addr}/scene", data=body, headers={"Content-Type": "application/json"})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(req, timeout=120) as r:
            out = json.loads(r.read())
    except urllib.error.HTTPError as e:
        out = json.loads(e.read())
    if not out.get("ok"):
        raise RuntimeError(f"scene service: {out.get('error')}")
    return out


def _cell(fwd, left):
    """Map cell (row, col) of ego points. Rows run to the right, columns forward."""
    r = ((-np.asarray(left, float)) - MIN_RIGHT) * PX_PER_M
    c = (np.asarray(fwd, float) - MIN_FWD) * PX_PER_M
    return r.astype(int), c.astype(int)


def _nearest_point(o: Dict):
    c, s = math.cos(math.radians(o["yaw_rel_deg"])), math.sin(math.radians(o["yaw_rel_deg"]))
    rx, ry = -o["fwd"], -o["left"]
    bx, by = c * rx + s * ry, -s * rx + c * ry
    bx = max(-o["length_m"] / 2, min(o["length_m"] / 2, bx)); by = max(-o["width_m"] / 2, min(o["width_m"] / 2, by))
    return round(o["fwd"] + c * bx - s * by, 2), round(o["left"] + s * bx + c * by, 2)


def read(frame_dir: str, score_min: float = 0.3) -> Dict:
    out = _post(frame_dir, score_min)
    objs = []
    for o in out["boxes"]:
        if o["length_m"] < 0.2 or o["width_m"] < 0.2:          # degenerate placeholders
            continue
        o["near_fwd"], o["near_left"] = _nearest_point(o)
        objs.append(o)
    objs.sort(key=lambda o: math.hypot(o["near_fwd"], o["near_left"]))
    lab = np.load(io.BytesIO(base64.b64decode(out["bev"]["labels_npy_b64"])), allow_pickle=False)
    names = out["bev"]["classes"]
    # lanes: the lane-marker class resampled onto lanes.py's grid, then the same line fit
    F, L = np.meshgrid(_lanes.FWD, _lanes.LEFT, indexing="ij")
    rr, cc = _cell(F, L)
    ok = (rr >= 0) & (rr < lab.shape[0]) & (cc >= 0) & (cc < lab.shape[1])
    paint = np.zeros(F.shape, bool)
    paint[ok] = lab[rr[ok], cc[ok]] == names.index("lane_markers")
    ln = _lanes.lines_from_paint(paint, np.zeros_like(paint), "TransFuser map")
    lights = []
    for name, state in LIGHT_STATES.items():
        cells = np.argwhere(lab == names.index(name))
        if len(cells):
            fwd = cells[:, 1] / PX_PER_M + MIN_FWD
            left = -(cells[:, 0] / PX_PER_M + MIN_RIGHT)
            ahead = fwd > 0
            if ahead.any():
                i = int(np.argmin(np.where(ahead, fwd, np.inf)))
                lights.append({"state": state, "fwd": round(float(fwd[i]), 1), "left": round(float(left[i]), 1),
                               "cells": int(len(cells))})
    lights.sort(key=lambda d: d["fwd"])
    return {"objects": objs, "lanes": ln, "lights": lights, "labels": lab, "classes": names}


def is_road(s: Dict, fwd: float, left: float) -> bool:
    r, c = _cell(fwd, left)
    lab = s["labels"]
    if not (0 <= r < lab.shape[0] and 0 <= c < lab.shape[1]):
        return False
    return s["classes"][int(lab[r, c])] in ("road", "lane_markers")


def describe(s: Dict, n: int = 12) -> str:
    rows = [f"{len(s['objects'])} objects (nearest first):"]
    for o in s["objects"][:n]:
        rows.append(f"  {o['class']:13} near {o['near_fwd']:6.1f} {o['near_left']:+6.1f}  centre {o['fwd']:6.1f} {o['left']:+6.1f}  "
                    f"yaw {o['yaw_rel_deg']:+6.1f}  {o['length_m']:.1f}x{o['width_m']:.1f} m  {o['speed_mps']:.1f} m/s"
                    f"{'  braking' if o['braking'] else ''}  score {o['score']:.2f}")
    if s["lights"]:
        rows.append("traffic lights ahead: " + ", ".join(f"{d['state']} at {d['fwd']} m (left {d['left']:+})" for d in s["lights"]))
    rows.append(_lanes.describe(s["lanes"]))
    return "\n".join(rows)


def render(s: Dict, out: str = None, back_m: float = 20.0, ahead_m: float = 50.0, side_m: float = 20.0) -> str:
    """A bird's-eye picture (forward is up, your car at the bottom centre) of the map and boxes."""
    colours = {"road": (80, 80, 80), "lane_markers": (255, 255, 255), "stop_signs": (0, 0, 200),
               "traffic_green": (0, 200, 0), "traffic_red_normal": (0, 0, 255), "traffic_red_not_normal": (0, 0, 255)}
    lab = s["labels"]
    img = np.zeros(lab.shape + (3,), np.uint8)
    for k, name in enumerate(s["classes"]):
        if name in colours:
            img[lab == k] = colours[name]
    for o in s["objects"]:
        c, sn = math.cos(math.radians(o["yaw_rel_deg"])), math.sin(math.radians(o["yaw_rel_deg"]))
        corners = []
        for dx, dy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            f = o["fwd"] + c * dx * o["length_m"] / 2 - sn * dy * o["width_m"] / 2
            l = o["left"] + sn * dx * o["length_m"] / 2 + c * dy * o["width_m"] / 2
            r, cc = _cell(f, l)
            corners.append([int(cc), int(r)])
        col = {"walker": (0, 140, 255), "special": (255, 0, 255), "obstacle": (0, 200, 255)}.get(o["class"], (255, 180, 0))
        cv2.polylines(img, [np.array(corners)], True, col, 2)
    r0, c0 = _cell(0, 0)
    cv2.rectangle(img, (int(c0 - 2.4 * PX_PER_M), int(r0 - 0.9 * PX_PER_M)), (int(c0 + 2.4 * PX_PER_M), int(r0 + 0.9 * PX_PER_M)), (0, 255, 255), -1)
    # crop and turn so that forward is up and left is left
    rb, cb = _cell(np.array([-back_m, ahead_m]), np.array([side_m, -side_m]))
    crop = img[max(0, rb[0]):rb[1], max(0, cb[0]):cb[1]]
    crop = cv2.rotate(crop, cv2.ROTATE_90_COUNTERCLOCKWISE)
    crop = cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    path = out or os.path.join(os.environ.get("CARLA_PROBE_CODE_DIR") or os.getcwd(), "scene_bev.png")
    cv2.imwrite(path, crop)
    return path
