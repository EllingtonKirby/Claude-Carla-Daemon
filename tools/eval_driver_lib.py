#!/usr/bin/env python3
"""Score the driver perception library (carla_probe/driver_lib) against CARLA ground truth.

    python tools/eval_driver_lib.py SCENES_ROOT [--objects] [--out results.json]

SCENES_ROOT holds capture_route_*/route_scenes/scene_*.json from a dense capture (frames carry
<cam>_inst.png and <cam>_depth.npy). No Claude usage.

lanes     lanes.markings() on CAM_FRONT against the same line fit run on the ground-truth paint
          (semantic tag 24, road_line, from <cam>_inst.png): lane-centre offset, width and heading
          errors, and how often a lane was found. Then LaneTracker over each route's scene sequence
          (scenes are 40 ticks apart): error against the ground-truth centre on the frames where the
          raw scan found a lane, the tracker's error on frames where it did not (it coasts), and
          jitter (median change between consecutive estimates) for both.
scene     (--scene; needs the TransFuser service at CARLA_PROBE_PERCEPTION_ADDR and frames with
          TFV6_rgb.jpg) scene.read() objects and lanes, scored the same way.
objects   (--objects; needs CARLA_PROBE_PERCEPTION_ADDR) objects.scan() footprints against
          actors_truth boxes: recall within 30 m and the error of the nearest point, for the
          footprint and for the mask's median-depth point.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_probe.driver_lib import camera, lanes  # noqa: E402


def frame_dir_of(sc: dict) -> str:
    return str(Path(list(sc["images"].values())[0]).parent)


def gt_markings(fd: str):
    cam = camera.load(fd, size="half")["CAM_FRONT"]
    inst = cv2.imread(os.path.join(fd, "CAM_FRONT_inst.png"), cv2.IMREAD_UNCHANGED)
    if inst is None:
        return None
    tag = inst[..., 2]
    F, L = np.meshgrid(lanes.FWD, lanes.LEFT, indexing="ij")
    uvz = cam.to_pixels(np.stack([F, L, np.zeros_like(F)], axis=-1))
    u = np.round(uvz[..., 0]).astype(int); v = np.round(uvz[..., 1]).astype(int)
    ok = (uvz[..., 2] > 0.1) & (u >= 0) & (u < tag.shape[1]) & (v >= 0) & (v < tag.shape[0])
    paint = np.zeros(F.shape, bool)
    paint[ok] = tag[v[ok], u[ok]] == 24
    return lanes.lines_from_paint(paint, np.zeros_like(paint), "CAM_FRONT")


def truth_nearest(t: dict):
    c, s = math.cos(math.radians(t["yaw_rel_deg"])), math.sin(math.radians(t["yaw_rel_deg"]))
    # point in the box frame closest to the ego origin
    rx, ry = -t["fwd"], -t["left"]
    bx, by = c * rx + s * ry, -s * rx + c * ry
    bx = max(-t["length_m"] / 2, min(t["length_m"] / 2, bx)); by = max(-t["width_m"] / 2, min(t["width_m"] / 2, by))
    return t["fwd"] + c * bx - s * by, t["left"] + s * bx + c * by


def score_objects(fd: str, sc: dict, obj: dict) -> None:
    from carla_probe.driver_lib import objects
    score_dets(objects.scan(fd), sc, obj)


def score_dets(dets: list, sc: dict, obj: dict) -> None:
    for t in sc["actors_truth"]:
        tn = truth_nearest(t)
        if math.hypot(*tn) > 30:
            continue
        obj["truth"] += 1
        best = min(dets, key=lambda o: math.hypot(o["near_fwd"] - tn[0], o["near_left"] - tn[1]), default=None)
        if best is None or math.hypot(best["near_fwd"] - tn[0], best["near_left"] - tn[1]) > 4:
            continue
        obj["found"] += 1
        obj["near_err"].append(math.hypot(best["near_fwd"] - tn[0], best["near_left"] - tn[1]))
        obj["left_err"].append(abs(best["near_left"] - tn[1]))
        if "depth_fwd" in best:
            obj["depth_err"].append(math.hypot(best["depth_fwd"] - t["fwd"], best["depth_left"] - t["left"]))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes_root", type=Path)
    ap.add_argument("--objects", action="store_true")
    ap.add_argument("--scene", action="store_true", help="also score scene.read() (TransFuser v6): objects and lanes")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    routes = sorted(glob.glob(str(a.scenes_root / "*/route_scenes")))
    res = {"lanes": {}, "tracker": {}}
    errs = {"centre": [], "width": [], "heading": []}; found = n = 0
    tr_raw, tr_trk, tr_fill, jit_raw, jit_trk = [], [], [], [], []
    obj = {"truth": 0, "found": 0, "near_err": [], "depth_err": [], "left_err": []}
    sobj = {"truth": 0, "found": 0, "near_err": [], "depth_err": [], "left_err": []}
    serr = {"centre": [], "width": [], "heading": []}; sfound = sn = 0
    for rd in routes:
        scenes = [json.load(open(p)) for p in sorted(glob.glob(rd + "/scene_*.json"))]
        trk_path = "/tmp/_eval_lane_tracker.json"
        if os.path.exists(trk_path):
            os.remove(trk_path)
        trk = lanes.LaneTracker(path=trk_path)
        prev_raw = prev_trk = None
        for sc in scenes:
            fd = frame_dir_of(sc)
            if not os.path.exists(os.path.join(fd, "CAM_FRONT_inst.png")):
                continue
            g = gt_markings(fd)
            m = lanes.markings(fd)
            est = trk.update(m, int(sc["step"]), float(sc["speed_mps"]))
            if a.objects and sc.get("actors_truth") is not None:
                score_objects(fd, sc, obj)
            if a.scene and os.path.exists(os.path.join(fd, "TFV6_rgb.jpg")):
                from carla_probe.driver_lib import scene
                sres = scene.read(fd)
                if sc.get("actors_truth") is not None:
                    score_dets(sres["objects"], sc, sobj)
                if g is not None and g.get("centre_left") is not None:
                    sn += 1
                    sl = sres["lanes"]
                    if sl.get("centre_left") is not None:
                        sfound += 1
                        serr["centre"].append(sl["centre_left"] - g["centre_left"])
                        serr["width"].append(sl["width"] - g["width"])
                        serr["heading"].append(sl["heading_deg"] - g["heading_deg"])
            if g is None or g.get("centre_left") is None:
                prev_raw = prev_trk = None
                continue
            n += 1
            if m.get("centre_left") is not None:
                found += 1
                errs["centre"].append(m["centre_left"] - g["centre_left"])
                errs["width"].append(m["width"] - g["width"])
                errs["heading"].append(m["heading_deg"] - g["heading_deg"])
                tr_raw.append(abs(m["centre_left"] - g["centre_left"]))
                if prev_raw is not None:
                    jit_raw.append(abs(m["centre_left"] - prev_raw))
                prev_raw = m["centre_left"]
            if est.get("centre_left") is not None:
                (tr_trk if m.get("centre_left") is not None else tr_fill).append(abs(est["centre_left"] - g["centre_left"]))
                if prev_trk is not None:
                    jit_trk.append(abs(est["centre_left"] - prev_trk))
                prev_trk = est["centre_left"]
    med = lambda v: round(float(np.median(np.abs(v))), 3) if len(v) else None
    res["lanes"] = {"frames_with_gt_lane": n, "found": round(found / max(1, n), 3),
                    "centre_err_m": med(errs["centre"]), "centre_bias_m": round(float(np.median(errs["centre"])), 3) if errs["centre"] else None,
                    "width_err_m": med(errs["width"]), "heading_err_deg": med(errs["heading"])}
    res["tracker"] = {"raw_err_m": med(tr_raw), "tracked_err_m": med(tr_trk),
                      "tracked_err_when_raw_missing_m": med(tr_fill), "frames_raw_missing_tracker_answered": len(tr_fill),
                      "raw_jitter_m": med(jit_raw), "tracked_jitter_m": med(jit_trk),
                      "raw_err_p90_m": round(float(np.percentile(tr_raw, 90)), 3) if tr_raw else None,
                      "tracked_err_p90_m": round(float(np.percentile(tr_trk, 90)), 3) if tr_trk else None}
    if a.scene:
        res["scene_objects"] = {"truth_within_30m": sobj["truth"], "recall": round(sobj["found"] / max(1, sobj["truth"]), 3),
                                "nearest_point_err_m": med(sobj["near_err"]), "nearest_left_err_m": med(sobj["left_err"])}
        res["scene_lanes"] = {"frames_with_gt_lane": sn, "found": round(sfound / max(1, sn), 3),
                              "centre_err_m": med(serr["centre"]), "width_err_m": med(serr["width"]),
                              "heading_err_deg": med(serr["heading"])}
    if a.objects:
        res["objects"] = {"truth_within_30m": obj["truth"], "recall": round(obj["found"] / max(1, obj["truth"]), 3),
                          "nearest_point_err_m": med(obj["near_err"]), "nearest_left_err_m": med(obj["left_err"]),
                          "depth_centre_err_m": med(obj["depth_err"])}
    print(json.dumps(res, indent=1))
    if a.out:
        a.out.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
