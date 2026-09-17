#!/usr/bin/env python3
"""Build a video of what the driver actually saw, one panel per percept.

    python tools/percept_video.py SESSION_DIR [SESSION_DIR ...] [--fps 2] [--out-dir DIR]

A benchmark run records no continuous video: ``record.camera_every_n_ticks`` is 0 by default,
because every recorded frame costs simulation time. What a session does keep is
``frames/frame_<step>/`` -- written once per ``percept`` -- holding the camera JPGs the driver was
looking at when it made that decision. This stitches them into one mp4 per session: four views per
panel, one panel per decision.

The result is the drive as the *driver* perceived it, not as a spectator would see it, which is the
view that matters when a route is lost to something that was on screen and not acted on. Two
consequences worth keeping in mind while watching:

* **Panels are not evenly spaced in simulation time.** A panel is a decision; the gap to the next
  one is however many ticks that decision advanced. A held-up junction and a fast open straight
  look the same here.
* **A camera the percept did not write is absent**, so a frame directory missing any view is
  skipped rather than padded -- otherwise the four streams would drift out of step with each other
  and every later panel would show four different moments.

For a spectator view instead, replay the session's CARLA recording (``RouteScenario_*.log``).

Needs ffmpeg on PATH; stdlib otherwise.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path
from typing import List, Sequence

CAMS = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK")
# Per-view size in the panel. Even numbers: libx264 with yuv420p cannot encode odd dimensions, and
# the cameras are configured at different aspect ratios, so every view is scaled to this one.
VIEW_W, VIEW_H = 400, 224


def panel_dirs(session: Path, cams: Sequence[str] = CAMS) -> List[Path]:
    """The frame directories that hold every camera, in step order.

    ``frame_<step>`` is zero-padded, so lexicographic order is step order.
    """
    return [d for d in sorted((session / "frames").glob("frame_*"))
            if all((d / f"{c}.jpg").exists() for c in cams)]


def filter_chain(w: int = VIEW_W, h: int = VIEW_H) -> str:
    """Three forward views across the top, the rear view centred below them."""
    scale = ",".join(f"[{i}:v]scale={w}:{h},setsar=1[v{i}]" for i in range(len(CAMS)))
    return (f"{scale};"
            f"[v0][v1][v2]hstack=inputs=3[top];"
            f"[v3]pad={w * 3}:ih:{w}:0:black[bot];"
            f"[top][bot]vstack=inputs=2[out]")


def ffmpeg_cmd(session: Path, out: Path, fps: float) -> List[str]:
    cmd = ["ffmpeg", "-y", "-loglevel", "error"]
    for cam in CAMS:
        cmd += ["-framerate", str(fps), "-pattern_type", "glob",
                "-i", str(session / "frames" / f"frame_*/{cam}.jpg")]
    return cmd + ["-filter_complex", filter_chain(), "-map", "[out]", "-shortest",
                  "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", str(out)]


def build(session: Path, out: Path, fps: float) -> bool:
    panels = panel_dirs(session)
    if not panels:
        print(f"{session.name}: no frame directory holds all of {', '.join(CAMS)}", file=sys.stderr)
        return False
    skipped = len(sorted((session / "frames").glob("frame_*"))) - len(panels)
    if skipped:
        print(f"{session.name}: {skipped} frame dir(s) missing a camera, skipped")

    result = subprocess.run(ffmpeg_cmd(session, out, fps), capture_output=True, text=True)
    if result.returncode != 0:
        print(f"{session.name}: ffmpeg failed\n{result.stderr.strip()[:800]}", file=sys.stderr)
        return False
    print(f"{session.name}: {len(panels)} panels -> {out} ({out.stat().st_size // 1024} KiB)")
    return True


def main(argv: Sequence[str] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--fps", type=float, default=2.0, help="panels per second (default 2)")
    ap.add_argument("--out-dir", type=Path, default=None,
                    help="write <session name>.mp4 here instead of SESSION_DIR/percepts.mp4")
    args = ap.parse_args(argv)

    if shutil.which("ffmpeg") is None:
        print("ffmpeg not found on PATH", file=sys.stderr)
        return 1
    if args.out_dir:
        args.out_dir.mkdir(parents=True, exist_ok=True)

    built = sum(build(s, (args.out_dir / f"{s.name}.mp4") if args.out_dir else (s / "percepts.mp4"),
                      args.fps)
                for s in args.sessions)
    return 0 if built == len(args.sessions) else 1


if __name__ == "__main__":
    raise SystemExit(main())
