#!/usr/bin/env python3
"""TransFuser v6 (LEAD) as a PERCEPTION service: 3D boxes and a bird's-eye semantic map from cameras.

    env -u LD_LIBRARY_PATH ~/workspace/venv_lead/bin/python carla_probe/tfv6_server.py \\
        --checkpoint ~/workspace/lead/outputs/checkpoints/visiononly_resnet34 --port 8785

The study is how an agentic model drives and which tools it needs; TransFuser here is a perception
tool only. Its planning outputs (waypoints, target speed) are computed by the network but never
returned: the planning inputs it needs are fixed dummies, so they could not carry route information
anyway. Runs in LEAD's own interpreter (torch 2.5, Python 3.10), separate from the daemon's venv.

Input: TFV6_rgb.jpg, the 6 TransFuser cameras stitched left to right (carla_probe/agent.py with
TFV6_CAMERAS=1). Preprocessing follows lead/inference/sensor_agent.py: BGR -> RGB, horizontal FOV
crop and camera selection per the checkpoint config, (C, H, W) float.

POST /scene {"image_b64": <jpeg>, "score_min": 0.3}
  -> {"ok", "boxes": [{"class", "fwd", "left", "yaw_rel_deg", "length_m", "width_m", "speed_mps",
                       "braking", "score"}],
      "bev": {"classes": [...], "fwd_min": -32, "fwd_max": 64, "left_min": -40, "left_max": 40,
              "m_per_cell": 1.0, "labels_npy_b64": <int8 H x W, row 0 = fwd_max, col 0 = left_max>},
      "ms"}
GET /health -> {"ok", "checkpoint", "classes", "bev_classes"}

Ego frame on output: fwd metres ahead, left metres to the LEFT (TransFuser's vehicle system is
x forward, y right; converted here). length_m / width_m are full extents.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import torch

from lead.common import common_utils
from lead.common.constants import TransfuserBEVSemanticClass, TransfuserBoundingBoxClass
from lead.inference.config_open_loop import OpenLoopConfig
from lead.inference.open_loop_inference import OpenLoopInference
from lead.training.config_training import TrainingConfig

BOX_NAMES = {c.value: c.name.lower() for c in TransfuserBoundingBoxClass}
BEV_NAMES = [c.name.lower() for c in sorted(TransfuserBEVSemanticClass, key=lambda c: c.value)]


def _npy_b64(a: np.ndarray) -> str:
    buf = io.BytesIO()
    np.save(buf, a, allow_pickle=False)
    return base64.b64encode(buf.getvalue()).decode()


class TFv6:
    def __init__(self, checkpoint: str, device: str = "cuda:0") -> None:
        with open(os.path.join(checkpoint, "config.json")) as f:
            self.cfg = TrainingConfig(json.load(f))
        self.device = torch.device(device)
        self.inf = OpenLoopInference(self.cfg, OpenLoopConfig(raise_error_on_missing_key=False), checkpoint,
                                     self.device, prefix="model")
        self.checkpoint = checkpoint
        self.lock = threading.Lock()

    def _rgb(self, bgr: np.ndarray) -> torch.Tensor:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        chw = np.transpose(rgb, (2, 0, 1))
        if self.cfg.horizontal_fov_reduction > 0:
            chw = common_utils.fov_crop(chw, self.cfg.horizontal_fov_reduction, chw=True)
        if self.cfg.num_used_cameras != self.cfg.num_available_cameras:
            w = chw.shape[2] // self.cfg.num_available_cameras
            chw = np.concatenate([chw[:, :, i * w:(i + 1) * w] for i, use in enumerate(self.cfg.used_cameras) if use], axis=2)
        return torch.tensor(chw, dtype=torch.float32, device=self.device)[None]

    @torch.inference_mode()
    def scene(self, bgr: np.ndarray, score_min: float = 0.3) -> dict:
        one_hot = torch.zeros((1, 6), device=self.device); one_hot[0, 3] = 1.0      # "lane follow"
        data = {
            "rgb": self._rgb(bgr),
            "rasterized_lidar": torch.zeros((1, 1, 1, 1), device=self.device),     # unused in LTF mode
            "target_point_previous": torch.tensor([[0.0, 0.0]], device=self.device),
            "target_point": torch.tensor([[20.0, 0.0]], device=self.device),
            "target_point_next": torch.tensor([[40.0, 0.0]], device=self.device),
            "speed": torch.tensor([0.0], device=self.device),
            "command": one_hot, "next_command": one_hot.clone(),
            "town": np.array(["Town00"]),
            # The checkpoint keeps an auxiliary radar-detection head that its planner does not use
            # (use_radar_detection=False); it still expects the tensor, so it gets an empty one.
            "radar": torch.zeros((1, self.cfg.num_radar_sensors * self.cfg.num_radar_points_per_sensor, 5),
                                 device=self.device),
        }
        with self.lock:
            pred = self.inf.forward(data)
        boxes = []
        for b in pred.pred_bounding_box_vehicle_system or []:
            if b.score < score_min:
                continue
            boxes.append({"class": BOX_NAMES.get(int(b.clazz), str(b.clazz)),
                          "fwd": round(float(b.x), 2), "left": round(-float(b.y), 2),
                          "yaw_rel_deg": round(-math.degrees(float(b.yaw)), 1),
                          "length_m": round(2 * float(b.w), 2), "width_m": round(2 * float(b.h), 2),
                          "speed_mps": round(float(b.velocity), 2), "braking": bool(b.brake > 0.5),
                          "score": round(float(b.score), 3)})
        boxes.sort(key=lambda o: math.hypot(o["fwd"], o["left"]))
        bev = None
        if pred.pred_bev_semantic is not None:
            lab = pred.pred_bev_semantic[0].argmax(0).to(torch.int8).cpu().numpy()
            bev = {"classes": BEV_NAMES, "shape": list(lab.shape), "labels_npy_b64": _npy_b64(lab)}
        return {"boxes": boxes, "bev": bev}


def make_handler(m: TFv6):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body):
            d = json.dumps(body).encode()
            self.send_response(code); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(d))); self.end_headers(); self.wfile.write(d)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"ok": True, "checkpoint": os.path.basename(m.checkpoint.rstrip("/")),
                                 "classes": sorted(BOX_NAMES.values()), "bev_classes": BEV_NAMES})
            else:
                self._send(404, {"ok": False, "error": "unknown path"})

        def do_POST(self):
            t0 = time.time()
            try:
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if self.path != "/scene":
                    return self._send(404, {"ok": False, "error": "unknown path"})
                arr = np.frombuffer(base64.b64decode(body["image_b64"]), np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                out = m.scene(bgr, float(body.get("score_min", 0.3)))
                out.update({"ok": True, "ms": round((time.time() - t0) * 1000, 1)})
                self._send(200, out)
            except Exception as e:
                self._send(500, {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc(limit=3)})
    return H


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8785)
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    m = TFv6(a.checkpoint, a.device)
    print(f"[tfv6] serving {a.checkpoint} on {a.host}:{a.port}", flush=True)
    ThreadingHTTPServer((a.host, a.port), make_handler(m)).serve_forever()


if __name__ == "__main__":
    main()
