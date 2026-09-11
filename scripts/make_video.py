#!/usr/bin/env python3
"""Build an mp4 from a session's recorded camera frames.

    python scripts/make_video.py SESSION_DIR [--fps 10] [--out SESSION_DIR/drive.mp4]

Frames come from ``SESSION_DIR/camera_log/step_*.jpg``, written while recording is on (see
``record`` in docs/OPERATOR_GUIDE.md and ``record`` in carla_probe/config.yaml). Uses cv2's mp4v
writer, so it needs no ffmpeg/pyav.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import cv2


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    frames = sorted(
        (args.session / "camera_log").glob("step_*.jpg"),
        key=lambda p: int(re.findall(r"\d+", p.stem)[-1]),
    )
    if not frames:
        print(f"no frames under {args.session / 'camera_log'} (was recording on?)", file=sys.stderr)
        return 1
    first = cv2.imread(str(frames[0]))
    h, w = first.shape[:2]
    out = args.out or (args.session / "drive.mp4")
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    for f in frames:
        img = cv2.imread(str(f))
        if img is not None and img.shape[:2] == (h, w):
            writer.write(img)
    writer.release()
    print(f"wrote {len(frames)} frames -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
