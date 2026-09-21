#!/usr/bin/env python3
"""Build a route-perception eval set from a running PRIVILEGED session.

    ACCESS=privileged bash scripts/run_session.sh <ROUTE_ID> <SESSION_DIR> &
    python tools/capture_route_scenes.py SESSION_DIR [--every 40] [--max 200] [--out DIR]

Drives the session forward itself -- no model is involved, so a scene set costs nothing but
wall-clock -- and at each sample writes one labelled scene:

    {"step": 480, "speed_mps": 6.2,
     "images": {"CAM_FRONT": "<path>", ...},          # what a driver would have to look at
     "nav_sparse": [[5.0,-0.2], [10.0,-0.4], ...],    # the 4 points a driver actually reasons from
     "truth": [[2.0,-0.1], [4.0,-0.2], ..., [40.0,-6.1]]}   # the DENSE route, the answer

The question it exists to answer: can a model say where the road goes from the images alone? If it
cannot at 20 m, then the waypoints it feeds the PID are wrong at the source and no amount of
controller tuning fixes that; if it can, the control problem is downstream. `route_truth` is
privileged, which is exactly why the labels come from a privileged session while the images are the
ones a sensors driver would see.

Scenes are plain JSON with absolute image paths, so scoring is offline and repeatable: one capture,
any number of prompts or models scored against it.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_probe.client import ProbeClient  # noqa: E402

WHY = "capturing a route-perception scene; no driving decision is being made"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--addr", default="127.0.0.1:8790")
    ap.add_argument("--every", type=int, default=40, help="ticks between scenes (default %(default)s)")
    ap.add_argument("--max", type=int, default=200, help="stop after this many scenes")
    ap.add_argument("--speed", type=float, default=5.0, help="target speed while capturing")
    ap.add_argument("--out", type=Path, default=None, help="default: SESSION_DIR/route_scenes/")
    ap.add_argument("--wait", type=float, default=600.0)
    args = ap.parse_args()

    out = args.out or args.session / "route_scenes"
    out.mkdir(parents=True, exist_ok=True)
    probe = ProbeClient(args.addr, wait_s=args.wait)

    status = probe.call("status")
    assert status.get("ok"), status
    access = status["status"]["access"]
    assert access == "privileged", f"need a privileged session for the labels, got access={access}"

    scenes: List[Dict[str, Any]] = []
    while len(scenes) < args.max:
        percept = probe.call("percept")
        state = probe.call("get_state")["state"]
        truth = state.get("route_truth")
        if not truth:
            print("no route_truth in state: the route is not built yet or this is not privileged")
            break
        frames_dir = Path(percept["frames_dir"])
        scene = {
            "step": percept["status"]["step"],
            "speed_mps": percept["status"]["speed_mps"],
            "images": {Path(n).stem: str(frames_dir / n) for n in percept["files"]},
            "nav_sparse": (percept["observe"].get("nav") or {}).get("route_points_ego"),
            "nav_sparse_at_m": (percept["observe"].get("nav") or {}).get("route_points_at_m"),
            "truth": truth,
        }
        scenes.append(scene)
        (out / f"scene_{scene['step']:05d}.json").write_text(json.dumps(scene, indent=1))
        print(f"scene {len(scenes):>3} at step {scene['step']:>5}  v={scene['speed_mps']:.1f} m/s  "
              f"truth to {truth[-1][0]:.0f} m", flush=True)

        # Drive on along the route itself: the point is to sample varied geometry, not to test
        # driving, so follow the sparse nav points rather than inventing a plan.
        nav = scene["nav_sparse"]
        if not nav or len(nav) < 2:
            print("no route points: at the route end")
            break
        reply = probe.call("apply_plan", waypoints=nav, ticks=args.every,
                           target_speed=args.speed, why=WHY)
        reason = (reply.get("result") or {}).get("reason")
        if reason in ("route_end", "shutdown") or not reply.get("ok", False):
            print(f"stopping: {reason or reply.get('error')}")
            break

    index = out / "scenes.json"
    index.write_text(json.dumps({"session": str(args.session), "count": len(scenes),
                                 "scenes": [s["step"] for s in scenes]}, indent=1))
    print(f"\nwrote {len(scenes)} scene(s) to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
