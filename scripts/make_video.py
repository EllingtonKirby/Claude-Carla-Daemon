#!/usr/bin/env python3
"""Build an mp4 from a session's recorded camera frames.

    python scripts/make_video.py SESSION_DIR [--fps 10] [--out SESSION_DIR/drive.mp4]

Frames come from ``SESSION_DIR/camera_log/step_*.jpg``, written while recording is on (see
``record`` in docs/OPERATOR_GUIDE.md and ``record`` in carla_probe/config.yaml). Encodes H.264 with
ffmpeg (must be on PATH); otherwise stdlib only.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    frames = sorted((args.session / "camera_log").glob("step_*.jpg"))  # zero-padded: name order is step order
    if not frames:
        print(f"no frames under {args.session / 'camera_log'} (was recording on?)", file=sys.stderr)
        return 1
    if shutil.which("ffmpeg") is None:
        print("ffmpeg not found on PATH", file=sys.stderr)
        return 1
    out = args.out or (args.session / "drive.mp4")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-framerate", f"{args.fps:g}",
                    "-pattern_type", "glob", "-i", str(args.session / "camera_log" / "step_*.jpg"),
                    "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "20", "-movflags", "+faststart", str(out)], check=True)
    print(f"wrote {len(frames)} frames -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
