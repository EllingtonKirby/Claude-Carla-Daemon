#!/usr/bin/env python3
"""Learned perception as a service, for the driver's scripts: metric depth and instance masks.

    ~/workspace/venv_perception/bin/python carla_probe/perception_server.py --port 8795 --device cuda:0

This is experiment 2 of the privileged ladder: give the driver the same KINDS of arrays the dense
track gives it (carla_probe/gt_dense.py), but estimated from its own cameras by off-the-shelf
models, so a run stays Bench2Drive-eligible. The gap between the two is the price of imperfect
pixels.

Models (both via transformers, loaded once):
  depth    Depth Anything V2, metric, outdoor (trained on Virtual KITTI 2: synthetic driving, which
           is close to CARLA). Output resized to the input image: metres along the optical axis.
  segment  SAM 3 when it can be loaded (gated on the Hugging Face hub: needs an approved token in
           HF_TOKEN) -- open-vocabulary, honours free-text prompts. Otherwise Mask2Former panoptic
           on Cityscapes -- a fixed label set (car, truck, bus, person, rider, bicycle, motorcycle,
           ...), and `prompts` only filters it. /health says which one is live.

Runs in its OWN interpreter (torch + recent transformers): the daemon's venv must not grow torch,
and the vla venv is Python 3.8. HTTP + JSON, bodies carry base64 .npy, so the client stays stdlib
+ numpy inside the code tool's confinement. It never reads a path: the client sends image bytes.

POST /depth    {"image_b64"}                -> {"ok", "depth_npy_b64", "shape", "ms"}
POST /segment  {"image_b64", "prompts": []} -> {"ok", "labels_npy_b64", "segments": [...], "ms"}
GET  /health                                -> {"ok", "depth_model", "segment_model", "open_vocabulary"}
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import torch

DEPTH_MODEL = "depth-anything/Depth-Anything-V2-Metric-Outdoor-Large-hf"
SAM3_MODEL = "facebook/sam3"
PANOPTIC_MODEL = "facebook/mask2former-swin-large-cityscapes-panoptic"
# Cityscapes says "person"; drivers say "pedestrian".
_ALIASES = {"pedestrian": "person", "people": "person", "cyclist": "rider", "bike": "bicycle",
            "motorbike": "motorcycle", "vehicle": "car"}


def _npy_b64(a: np.ndarray) -> str:
    buf = io.BytesIO()
    np.save(buf, a, allow_pickle=False)
    return base64.b64encode(buf.getvalue()).decode()


def _decode_image(b64: str) -> np.ndarray:
    arr = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("could not decode the image")
    return img[..., ::-1].copy()   # RGB


VEHICLE_PROMPTS = ["car", "truck", "bus", "pedestrian", "bicycle", "motorcycle"]


class Models:
    """Legacy backend: Depth Anything V2 (metric outdoor) + SAM 3 via transformers or Mask2Former."""

    def __init__(self, device: str, segmenter: str) -> None:
        self.depth_model_name = DEPTH_MODEL
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.device = device
        self.lock = threading.Lock()
        self.depth_proc = AutoImageProcessor.from_pretrained(DEPTH_MODEL)
        self.depth_model = AutoModelForDepthEstimation.from_pretrained(DEPTH_MODEL).to(device).eval()
        self.segment_model_name = None
        self.open_vocab = False
        if segmenter in ("auto", "sam3"):
            try:
                from transformers import Sam3Model, Sam3Processor
                self.sam_proc = Sam3Processor.from_pretrained(SAM3_MODEL)
                self.sam = Sam3Model.from_pretrained(SAM3_MODEL).to(device).eval()
                self.segment_model_name, self.open_vocab = SAM3_MODEL, True
            except Exception as e:   # gated / not installed: say so, and fall back
                if segmenter == "sam3":
                    raise
                print(f"[perception] SAM 3 unavailable ({type(e).__name__}: {str(e)[:160]}); "
                      f"falling back to {PANOPTIC_MODEL}", flush=True)
        if self.segment_model_name is None:
            from transformers import AutoImageProcessor as P, Mask2FormerForUniversalSegmentation
            self.pan_proc = P.from_pretrained(PANOPTIC_MODEL)
            self.pan = Mask2FormerForUniversalSegmentation.from_pretrained(PANOPTIC_MODEL).to(device).eval()
            self.segment_model_name = PANOPTIC_MODEL

    @torch.inference_mode()
    def depth(self, rgb: np.ndarray, f_px: Optional[float] = None) -> np.ndarray:
        with self.lock:
            inputs = self.depth_proc(images=rgb, return_tensors="pt").to(self.device)
            pred = self.depth_model(**inputs).predicted_depth          # 1 x h x w, metres
            pred = torch.nn.functional.interpolate(pred[:, None], size=rgb.shape[:2], mode="bicubic",
                                                   align_corners=False)[0, 0]
        return pred.clamp(min=0).float().cpu().numpy()

    @torch.inference_mode()
    def segment(self, rgb: np.ndarray, prompts: List[str]) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        h, w = rgb.shape[:2]
        labels = np.zeros((h, w), dtype=np.int32)
        segs: List[Dict[str, Any]] = []
        with self.lock:
            if self.open_vocab:
                for prompt in prompts or VEHICLE_PROMPTS:
                    inputs = self.sam_proc(images=rgb, text=prompt, return_tensors="pt").to(self.device)
                    out = self.sam(**inputs)
                    res = self.sam_proc.post_process_instance_segmentation(
                        out, threshold=0.5, mask_threshold=0.5, target_sizes=[(h, w)])[0]
                    for m, sc in zip(res["masks"], res["scores"]):
                        m = m.cpu().numpy().astype(bool)
                        if not m.any():
                            continue
                        sid = len(segs) + 1
                        labels[m & (labels == 0)] = sid
                        segs.append(self._seg(sid, prompt, float(sc), m))
            else:
                inputs = self.pan_proc(images=rgb, return_tensors="pt").to(self.device)
                out = self.pan(**inputs)
                res = self.pan_proc.post_process_panoptic_segmentation(out, target_sizes=[(h, w)],
                                                                       label_ids_to_fuse=set())[0]
                seg_map = res["segmentation"].cpu().numpy()
                wanted = {_ALIASES.get(p.lower(), p.lower()) for p in prompts}
                id2label = self.pan.config.id2label
                for info in res["segments_info"]:
                    name = id2label[info["label_id"]]
                    if wanted and name not in wanted:
                        continue
                    m = seg_map == info["id"]
                    sid = len(segs) + 1
                    labels[m] = sid
                    segs.append(self._seg(sid, name, float(info.get("score", 1.0)), m))
        return labels, segs

    @staticmethod
    def _seg(sid: int, label: str, score: float, m: np.ndarray) -> Dict[str, Any]:
        ys, xs = np.nonzero(m)
        return {"id": sid, "label": label, "score": round(score, 3), "area": int(m.sum()),
                "bbox": [int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())]}


class StrongModels:
    """Strong backend: Apple Depth Pro 2 (metric depth, sharp edges) + Meta SAM 3 (open-vocabulary
    instance masks), each from its own package (third_party/ml-depth-pro-2, third_party/sam3; see
    scripts/setup_perception2.sh). Needs Python >= 3.12, so it runs from ~/workspace/venv_perception2.
    SAM 3 weights are gated on the Hugging Face hub: the token in ~/.cache/huggingface/token must have
    access to facebook/sam3. Depth Pro 2 weights are for research use only (its LICENSE_MODEL).

    Depth takes the camera's focal length in pixels when the caller knows it (every CARLA camera
    does: frames/cameras.json); otherwise Depth Pro 2 estimates it from the image.
    """

    def __init__(self, device: str, depth_preset: str = "depthpro2-h", sam_threshold: float = 0.5) -> None:
        from depth_pro_2 import create_model_and_transforms
        from sam3.model.sam3_image_processor import Sam3Processor
        from sam3.model_builder import build_sam3_image_model
        self.device = device
        self.lock = threading.Lock()
        self.dp, self.dp_transform = create_model_and_transforms(depth_preset, device=device)
        self.dp.eval()
        self.sam = build_sam3_image_model(device="cpu").to(device).eval()   # its builder moves only for device == "cuda"
        self.sam_proc = Sam3Processor(self.sam, device=device, confidence_threshold=sam_threshold)
        self.depth_model_name = f"apple/{depth_preset}"
        self.segment_model_name = "facebook/sam3"
        self.open_vocab = True

    @torch.inference_mode()
    def depth(self, rgb: np.ndarray, f_px: Optional[float] = None) -> np.ndarray:
        with self.lock:
            pred = self.dp.infer(self.dp_transform(rgb), f_px=f_px)
            d = pred["depth"].float()
        return torch.nan_to_num(d, posinf=1000.0).clamp(0, 1000.0).cpu().numpy()

    @torch.inference_mode()
    def segment(self, rgb: np.ndarray, prompts: List[str]) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        from PIL import Image
        h, w = rgb.shape[:2]
        labels = np.zeros((h, w), dtype=np.int32)
        segs: List[Dict[str, Any]] = []
        with self.lock, torch.autocast("cuda", dtype=torch.bfloat16):
            state = self.sam_proc.set_image(Image.fromarray(rgb))   # encode once, prompt many times
            for prompt in prompts or VEHICLE_PROMPTS:
                self.sam_proc.reset_all_prompts(state)
                out = self.sam_proc.set_text_prompt(prompt=prompt, state=state)
                order = torch.argsort(out["scores"], descending=True)
                for i in order.tolist():
                    m = out["masks"][i, 0].cpu().numpy().astype(bool)
                    m &= labels == 0                       # higher-scoring masks keep shared pixels
                    if m.sum() < 8:
                        continue
                    sid = len(segs) + 1
                    labels[m] = sid
                    segs.append(Models._seg(sid, prompt, float(out["scores"][i]), m))
        return labels, segs


def make_handler(models):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: Dict[str, Any]) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/health":
                self._send(200, {"ok": True, "depth_model": models.depth_model_name, "segment_model": models.segment_model_name,
                                 "open_vocabulary": models.open_vocab})
            else:
                self._send(404, {"ok": False, "error": "unknown path"})

        def do_POST(self):
            t0 = time.time()
            try:
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                rgb = _decode_image(body["image_b64"])
                if self.path == "/depth":
                    d = models.depth(rgb, f_px=body.get("f_px"))
                    self._send(200, {"ok": True, "depth_npy_b64": _npy_b64(d.astype(np.float16)), "shape": list(d.shape),
                                     "ms": round((time.time() - t0) * 1000, 1)})
                elif self.path == "/segment":
                    labels, segs = models.segment(rgb, list(body.get("prompts") or []))
                    self._send(200, {"ok": True, "labels_npy_b64": _npy_b64(labels.astype(np.int16)), "segments": segs,
                                     "ms": round((time.time() - t0) * 1000, 1)})
                else:
                    self._send(404, {"ok": False, "error": "unknown path"})
            except Exception as e:
                self._send(500, {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc(limit=3)})
    return Handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8795)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--backend", default="legacy", choices=["legacy", "strong"],
                    help="legacy = Depth Anything V2 + Mask2Former/SAM 3 (transformers); strong = Depth Pro 2 + SAM 3")
    ap.add_argument("--segmenter", default="auto", choices=["auto", "sam3", "panoptic"], help="legacy backend only")
    ap.add_argument("--depth-preset", default="depthpro2-h", help="strong backend only: depthpro2-{s,b,l,h}")
    a = ap.parse_args()
    models = StrongModels(a.device, a.depth_preset) if a.backend == "strong" else Models(a.device, a.segmenter)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(models))
    print(f"[perception] serving on {a.host}:{a.port} depth={models.depth_model_name} segment={models.segment_model_name}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
