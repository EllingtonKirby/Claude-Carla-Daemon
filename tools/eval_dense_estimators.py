#!/usr/bin/env python3
"""Score perception backends on depth and on WHERE THE ROAD USERS ARE, against CARLA ground truth.

    <venv>/bin/python tools/eval_dense_estimators.py SCENES_ROOT --backend {gt,legacy,strong}
                       [--limit N] [--cams CAM_FRONT,...] [--out results.json]

SCENES_ROOT holds capture_route_*/route_scenes/scene_*.json (tools/capture_route_scenes.py with the
dense track), whose frames carry <cam>_full.jpg, the ground-truth <cam>_depth.npy and <cam>_inst.png.
No Claude usage: this runs the perception models only.

Two measures, per backend:
  depth      per-pixel |d - gt| / gt and the signed median, by true-depth band, all cameras.
  actors     the pipeline a driver's script would run: instance masks -> median depth over the
             mask -> back-project -> ego frame [fwd, left]; detections from all cameras merged
             (within 2 m), matched greedily to `actors_truth` within 6 m. Reports recall, position
             error and signed bias by range band, and extras (detections within 34 m that match
             neither an actor nor a parked car). Truth includes occluded actors, so recall < 1 even
             for the gt backend. The scenes hold few pedestrians (12), so ped_recall is anecdotal.
             `--backend gt` runs the same pipeline on ground-truth depth and masks: the ceiling of
             the method, so the gap to it is the models' and not the pipeline's.

Backends: gt; legacy = Depth Anything V2 + Mask2Former (venv_perception); strong = Depth Pro 2 +
SAM 3 (venv_perception2). Both model backends come from carla_probe/perception_server.py.
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

ROAD_USER_TAGS = {12: "pedestrian", 13: "rider", 14: "car", 15: "truck", 16: "bus", 18: "motorcycle", 19: "bicycle"}
PED_LABELS = {"pedestrian", "person", "rider"}
DEPTH_BANDS = [(0, 10), (10, 20), (20, 40), (40, 80)]
RANGE_BANDS = [(0, 10), (10, 20), (20, 40)]
MATCH_M, MERGE_M, MIN_PX = 6.0, 2.0, 30


def cam_to_ego(pts_cam: np.ndarray, mount: dict) -> np.ndarray:
    """Camera frame (X right, Y down, Z forward) -> ego [fwd, left]. Mount yaw + = to the right."""
    f, r = pts_cam[:, 2], pts_cam[:, 0]
    yaw = math.radians(mount["yaw"])
    x = f * math.cos(yaw) - r * math.sin(yaw) + mount["x"]
    y_right = f * math.sin(yaw) + r * math.cos(yaw) + mount["y"]
    return np.stack([x, -y_right], axis=1)


def instance_position(mask: np.ndarray, depth: np.ndarray, K: dict) -> np.ndarray | None:
    v, u = np.nonzero(mask)
    z = depth[v, u]
    ok = np.isfinite(z) & (z > 0.5) & (z < 150)
    if ok.sum() < MIN_PX // 2:
        return None
    v, u, z = v[ok], u[ok], z[ok]
    X = (u - K["cx"]) * z / K["fx"]
    Y = (v - K["cy"]) * z / K["fy"]
    return np.median(np.stack([X, Y, z], axis=1), axis=0)


def gt_instances(inst_bgr: np.ndarray):
    tag = inst_bgr[..., 2]
    iid = inst_bgr[..., 1].astype(np.int32) * 256 + inst_bgr[..., 0]
    out = []
    for t, name in ROAD_USER_TAGS.items():
        sel = tag == t
        for i in np.unique(iid[sel]):
            m = sel & (iid == i)
            if m.sum() >= MIN_PX:
                out.append((name, m))
    return out


def model_instances(models, rgb: np.ndarray, size):
    labels, segs = models.segment(rgb, [])
    labels = cv2.resize(labels.astype(np.int32), size, interpolation=cv2.INTER_NEAREST)
    out = []
    for s in segs:
        m = labels == s["id"]
        if m.sum() >= MIN_PX:
            out.append((s["label"], m))
    return out


def merge(dets):
    kept = []
    for d in sorted(dets, key=lambda d: -d["px"]):
        if all(np.hypot(d["fwd"] - k["fwd"], d["left"] - k["left"]) > MERGE_M for k in kept):
            kept.append(d)
    return kept


def band(r, bands):
    for lo, hi in bands:
        if lo <= r < hi:
            return f"{lo}-{hi}"
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scenes_root", type=Path)
    ap.add_argument("--backend", required=True, choices=["gt", "legacy", "strong"])
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--cams", default="CAM_FRONT,CAM_FRONT_LEFT,CAM_FRONT_RIGHT,CAM_BACK")
    ap.add_argument("--depth-preset", default="depthpro2-h")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    cams = a.cams.split(",")

    models = None
    if a.backend == "legacy":
        from carla_probe.perception_server import Models
        models = Models("cuda:0", "panoptic")
    elif a.backend == "strong":
        from carla_probe.perception_server import StrongModels
        models = StrongModels("cuda:0", a.depth_preset)

    scenes = sorted(glob.glob(str(a.scenes_root / "*/route_scenes/scene_*.json")))
    scenes = [s for s in scenes if json.load(open(s)).get("actors_truth") is not None]
    if a.limit:
        scenes = scenes[:: max(1, len(scenes) // a.limit)][: a.limit]

    rel = {f"{lo}-{hi}": [] for lo, hi in DEPTH_BANDS}
    act = {f"{lo}-{hi}": {"truth": 0, "found": 0, "err": [], "dfwd": [], "dleft": [],
                          "ped_truth": 0, "ped_found": 0} for lo, hi in RANGE_BANDS}
    extras, n_det, t_model = 0, 0, 0.0
    for si, sp in enumerate(scenes):
        sc = json.load(open(sp))
        frame_dir = Path(sc["camera_model"]).parent / Path(list(sc["images"].values())[0]).parent.name
        cm = json.load(open(sc["camera_model"]))["cameras"]
        dets = []
        for cam in cams:
            full = frame_dir / f"{cam}_full.jpg"
            gt_d = frame_dir / f"{cam}_depth.npy"
            if not (full.exists() and gt_d.exists()):
                continue
            gtd = np.load(gt_d).astype(np.float32)
            h, w = gtd.shape
            Kd = cm[cam]["intrinsics"]["depth_npy"]
            if models is None:
                depth = gtd
                inst = gt_instances(cv2.imread(str(frame_dir / f"{cam}_inst.png"), cv2.IMREAD_UNCHANGED))
            else:
                rgb = cv2.imread(str(full))[..., ::-1].copy()
                t0 = time.time()
                d_full = models.depth(rgb, f_px=cm[cam]["intrinsics"]["full_jpg"]["fx"])
                depth = cv2.resize(d_full, (w, h), interpolation=cv2.INTER_AREA)
                inst = model_instances(models, rgb, (w, h))
                t_model += time.time() - t0
                ok = (gtd > 0.5) & (gtd < 80)
                r = (depth[ok] - gtd[ok]) / gtd[ok]
                for lo, hi in DEPTH_BANDS:
                    sel = (gtd[ok] >= lo) & (gtd[ok] < hi)
                    if sel.any():
                        rel[f"{lo}-{hi}"].append(r[sel][:: max(1, sel.sum() // 400)])
            for label, m in inst:
                p = instance_position(m, depth, Kd)
                if p is None:
                    continue
                fl = cam_to_ego(p[None], cm[cam]["mount"])[0]
                dets.append({"fwd": float(fl[0]), "left": float(fl[1]), "label": label, "px": int(m.sum())})
        dets = merge(dets)
        n_det += len(dets)
        truth = [t for t in sc["actors_truth"] if band(math.hypot(t["fwd"], t["left"]), RANGE_BANDS)]
        pairs = sorted(((math.hypot(d["fwd"] - t["fwd"], d["left"] - t["left"]), i, j)
                        for i, d in enumerate(dets) for j, t in enumerate(truth)))
        used_d, used_t = set(), set()
        for dist, i, j in pairs:
            if dist > MATCH_M or i in used_d or j in used_t:
                continue
            used_d.add(i); used_t.add(j)
            t, d = truth[j], dets[i]
            b = act[band(math.hypot(t["fwd"], t["left"]), RANGE_BANDS)]
            b["found"] += 1; b["err"].append(dist)
            b["dfwd"].append(d["fwd"] - t["fwd"]); b["dleft"].append(d["left"] - t["left"])
            if "walker" in t["type"]:
                b["ped_found"] += 1
        for t in truth:
            b = act[band(math.hypot(t["fwd"], t["left"]), RANGE_BANDS)]
            b["truth"] += 1
            if "walker" in t["type"]:
                b["ped_truth"] += 1
        # Parked cars are real objects but not in actors_truth: a detection on one is not an extra.
        parked = sc.get("parked_truth") or []
        extras += sum(1 for i, d in enumerate(dets) if i not in used_d
                      and math.hypot(d["fwd"], d["left"]) < RANGE_BANDS[-1][1] - MATCH_M
                      and not any(math.hypot(d["fwd"] - q["fwd"], d["left"] - q["left"]) < MATCH_M for q in parked))
        if (si + 1) % 20 == 0:
            print(f"  {si + 1}/{len(scenes)} scenes", file=sys.stderr, flush=True)

    res = {"backend": a.backend, "scenes": len(scenes), "cams": cams,
           "model_s_per_image": round(t_model / max(1, len(scenes) * len(cams)), 3) if models else None,
           "depth": {}, "actors": {}, "extras": extras, "detections": n_det}
    if models:
        res["depth_model"], res["segment_model"] = models.depth_model_name, models.segment_model_name
        for k, v in rel.items():
            r = np.concatenate(v) if v else np.array([])
            res["depth"][k] = {"abs_rel_median": round(float(np.median(np.abs(r))), 3) if r.size else None,
                               "signed_median": round(float(np.median(r)), 3) if r.size else None}
    for k, b in act.items():
        res["actors"][k] = {"truth": b["truth"], "recall": round(b["found"] / max(1, b["truth"]), 3),
                            "err_median_m": round(float(np.median(b["err"])), 2) if b["err"] else None,
                            "bias_fwd_m": round(float(np.median(b["dfwd"])), 2) if b["dfwd"] else None,
                            "bias_left_m": round(float(np.median(b["dleft"])), 2) if b["dleft"] else None,
                            "ped_recall": round(b["ped_found"] / b["ped_truth"], 3) if b["ped_truth"] else None}
    print(json.dumps(res, indent=1))
    if a.out:
        a.out.write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
