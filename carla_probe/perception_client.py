"""Client for carla_probe/perception_server.py, for use from the driver's scripts (code tool).

Stdlib + numpy only: it runs inside carla_probe/code_runner.py, whose confinement allows exactly
one network address, CARLA_PROBE_PERCEPTION_ADDR. The client reads the image itself (a frame the
driver may read) and sends the bytes, so the service never opens a path on anyone's behalf.

    from carla_probe.perception_client import PerceptionClient
    pc = PerceptionClient()
    depth = pc.depth("/.../frames/frame_00120/CAM_FRONT_full.jpg")       # HxW float32 metres
    labels, segments = pc.segment(path, prompts=["car", "pedestrian"])   # HxW int32, [dict]
"""
from __future__ import annotations

import base64
import io
import json
import os
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


def _decode_npy(b64: str) -> np.ndarray:
    return np.load(io.BytesIO(base64.b64decode(b64)), allow_pickle=False)


class PerceptionClient:
    def __init__(self, addr: Optional[str] = None, timeout_s: float = 60.0) -> None:
        addr = addr or os.environ.get("CARLA_PROBE_PERCEPTION_ADDR", "")
        if not addr:
            raise RuntimeError("no perception service in this session (CARLA_PROBE_PERCEPTION_ADDR is unset)")
        self.base = f"http://{addr}"
        self.timeout_s = timeout_s
        # Direct, never via an environment proxy: the code tool allows exactly this one address.
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with self._opener.open(req, timeout=self.timeout_s) as r:
            out = json.loads(r.read())
        if not out.get("ok", False):
            raise RuntimeError(f"perception service: {out.get('error')}")
        return out

    def info(self) -> Dict[str, Any]:
        with self._opener.open(self.base + "/health", timeout=self.timeout_s) as r:
            return json.loads(r.read())

    @staticmethod
    def _image(path: str) -> str:
        with open(path, "rb") as f:
            return base64.b64encode(f.read()).decode()

    def depth(self, path: str, f_px: Optional[float] = None) -> np.ndarray:
        """Metric depth in metres along the optical axis, at the image's own resolution.

        f_px: the camera's focal length in pixels at this image's size (frames/cameras.json). The
        strong backend uses it instead of estimating the field of view; the legacy one ignores it."""
        body: Dict[str, Any] = {"image_b64": self._image(path)}
        if f_px is not None:
            body["f_px"] = float(f_px)
        out = self._post("/depth", body)
        return _decode_npy(out["depth_npy_b64"]).astype(np.float32)

    def segment(self, path: str, prompts: Optional[List[str]] = None) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
        """Instance masks. Returns (labels HxW int32 with 0 = background, segments)."""
        out = self._post("/segment", {"image_b64": self._image(path), "prompts": prompts or []})
        return _decode_npy(out["labels_npy_b64"]).astype(np.int32), out["segments"]
