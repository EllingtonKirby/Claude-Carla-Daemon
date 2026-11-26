"""Zoom into part of a camera image, optionally brightened (fog, dusk), and save it for viewing.

    from lib import zoom
    path = zoom.crop(frame_dir, "CAM_FRONT", (600, 200, 760, 290), scale=4, brighten=2.5)
    path = zoom.around(frame_dir, "CAM_FRONT", fwd=25, left=-3.5, size_m=6)   # a box around a ground point

Then read the PNG it returns. Upscaling adds no detail; it makes what is there easier to see.
"""
from __future__ import annotations

import os
from typing import Optional, Tuple

import cv2
import numpy as np

from . import camera as _camera


def _out(name: str) -> str:
    d = os.environ.get("CARLA_PROBE_CODE_DIR") or os.getcwd()
    return os.path.join(d, name)


def crop(frame_dir: str, cam: str, box: Tuple[int, int, int, int], scale: float = 4.0, brighten: float = 1.0,
         out: Optional[str] = None) -> str:
    """box = (x0, y0, x1, y1) in pixels of <cam>_full.jpg. brighten > 1 stretches contrast (CLAHE) and
    gain, for fog. Returns the PNG path."""
    img = cv2.imread(os.path.join(frame_dir, f"{cam}_full.jpg"))
    h, w = img.shape[:2]
    x0, y0, x1, y1 = max(0, int(box[0])), max(0, int(box[1])), min(w, int(box[2])), min(h, int(box[3]))
    c = img[y0:y1, x0:x1]
    if brighten and brighten != 1.0:
        lab = cv2.cvtColor(c, cv2.COLOR_BGR2LAB)
        lab[..., 0] = cv2.createCLAHE(clipLimit=1.5 * brighten, tileGridSize=(4, 4)).apply(lab[..., 0])
        c = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    c = cv2.resize(c, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    path = out or _out(f"zoom_{cam}.png")
    cv2.imwrite(path, c)
    return path


def around(frame_dir: str, cam: str, fwd: float, left: float, size_m: float = 6.0, height_m: float = 3.0,
           **kw) -> Optional[str]:
    """Crop a box size_m wide and height_m tall standing on the ground at (fwd, left) in the ego frame.
    None when that point is not in this camera's view."""
    c = _camera.load(frame_dir)[cam]
    corners = []
    for dl in (-size_m / 2, size_m / 2):
        for up in (0.0, height_m):
            p = c.to_pixels(np.array([[fwd, left + dl, up]]))[0]
            if p[2] > 0.1:
                corners.append(p[:2])
    if len(corners) < 2:
        return None
    pts = np.array(corners)
    x0, y0 = pts.min(axis=0); x1, y1 = pts.max(axis=0)
    if x1 < 0 or y1 < 0 or x0 > c.width or y0 > c.height:
        return None
    return crop(frame_dir, cam, (x0 - 8, y0 - 8, x1 + 8, y1 + 8), **kw)
