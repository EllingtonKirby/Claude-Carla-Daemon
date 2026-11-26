#!/usr/bin/env python3
"""How well did the ego hold its lane? Distance from the LANE CENTRE, per tick.

    python tools/lane_keeping.py SESSION_DIR... [--every 5] [--until STEP] [--port 20200] [--gpu 0] [--json]

Needs `recorder_truth.json` (run tools/recorder_truth.py first) and a CARLA server, which it uses
only to load the town's map and project each true ego position onto its lane.

Why this exists rather than reading `lateral_offset_m`: that field is the offset from the ROUTE, a
sparse GPS polyline the daemon interpolates, and it reads reassuringly small even when the car is
sitting on a lane line. Lane position is a different quantity and needs the map. On a run whose
route offset had a median of 0.17 m, the distance from the lane centre had a median of 1.47 m and
was beyond 1 m for 72% of the drive -- which is what a viewer sees in the video and what the route
offset cannot show.

Reported per session:
  off-centre      median / p90 / max distance from the lane centre, over MOVING ticks only
  time off        share of moving ticks beyond 0.5 m and beyond 1.0 m (a lane is ~3.5 m wide)
  heading error   |ego yaw - lane yaw|; inside a junction this is unreliable, because the map
                  projects the ego onto whichever crossing lane is nearest, so it is reported but
                  should not be read as a tracking error there
  wander rate     how fast the offset changes (m/s), a proxy for smoothness: a car tracking badly
                  and a car tracking a curve can share a median, and this separates them
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

try:
    import carla
except ImportError:  # keeps --help usable outside the venv
    carla = None  # type: ignore


def connect(port: int, gpu: int):
    """Reuse a server on `port`, else start one and return it so the caller can stop it."""
    client = carla.Client("127.0.0.1", port)
    client.set_timeout(5.0)
    try:
        client.get_server_version()
        client.set_timeout(300.0)   # load_world on a big town takes minutes
        return client, None
    except Exception:
        pass
    root = os.environ.get("CARLA_ROOT")
    assert root, "CARLA_ROOT is not set and no server answered on the port"
    proc = subprocess.Popen(
        [f"{root}/CarlaUE4.sh", "-RenderOffScreen", f"-carla-rpc-port={port}", f"-graphicsadapter={gpu}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(90):
        time.sleep(2.0)
        try:
            client = carla.Client("127.0.0.1", port)
            client.set_timeout(10.0)
            client.get_server_version()
            client.set_timeout(300.0)
            return client, proc
        except Exception:
            continue
    proc.terminate()
    raise RuntimeError(f"no CARLA server on port {port}")


def measure(client, session: Path, every: int, until: Optional[int] = None) -> Optional[Dict[str, Any]]:
    truth_path = session / "recorder_truth.json"
    if not truth_path.is_file():
        print(f"{session.name}: no recorder_truth.json (run tools/recorder_truth.py first)")
        return None
    truth = json.loads(truth_path.read_text())
    hero, offset = str(truth["hero_id"]), truth["frame_offset"]
    # result.json's record carries the town. session.json does not, and silently measuring against
    # whatever map happens to be loaded would give confident nonsense.
    record = json.loads((session / "result.json").read_text())["_checkpoint"]["records"][0]
    town = record.get("town_name")
    assert town, f"{session.name}: no town_name in result.json"
    world = client.get_world()
    if town not in world.get_map().name:
        client.load_world(town)
        time.sleep(3.0)                 # the map is not queryable the instant load_world returns
        world = client.get_world()
    cmap = world.get_map()
    # Projecting one town's coordinates onto another's map yields confident nonsense (hundreds of
    # metres "off centre") rather than an error, so make the mismatch fatal.
    assert town in cmap.name, f"{session.name}: wanted {town}, world has {cmap.name}"

    offs: List[float] = []
    heads: List[float] = []
    for row in [json.loads(l) for l in (session / "trajectory.jsonl").read_text().splitlines()][::every]:
        if row["speed"] <= 0.5:      # a parked car holds its lane perfectly; that is not tracking
            continue
        if until is not None and row["step"] > until:   # e.g. stop before a collision wrecks the pose
            break
        frame = truth["frames"].get(str(row["frame"] - offset))
        if not frame:
            continue
        pose = frame["actors"].get(hero)
        if not pose:
            continue
        wp = cmap.get_waypoint(carla.Location(x=pose[0], y=pose[1], z=0.0),
                               project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            continue
        centre = wp.transform.location
        offs.append(math.hypot(centre.x - pose[0], centre.y - pose[1]))
        heads.append(abs((wp.transform.rotation.yaw - pose[2] + 180.0) % 360.0 - 180.0))
    if len(offs) < 10:
        print(f"{session.name}: only {len(offs)} moving samples, not enough to report")
        return None
    o = np.asarray(offs)
    wander = np.abs(np.diff(o)) * (20.0 / every)  # m/s
    return {
        "session": session.name, "samples": len(offs),
        "median": float(np.median(o)), "p90": float(np.percentile(o, 90)), "max": float(o.max()),
        "beyond_0_5": float(np.mean(o > 0.5)), "beyond_1_0": float(np.mean(o > 1.0)),
        "heading_median": float(np.median(heads)), "heading_p90": float(np.percentile(heads, 90)),
        "wander_p90": float(np.percentile(wander, 90)),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--every", type=int, default=5, help="sample every Nth tick (default %(default)s)")
    ap.add_argument("--until", type=int, default=None, help="ignore ticks after this step (e.g. a collision)")
    ap.add_argument("--port", type=int, default=20200)
    ap.add_argument("--gpu", type=int, default=0)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    assert carla is not None, "carla is not importable: run inside the venv (source scripts/env.sh)"
    client, proc = connect(args.port, args.gpu)
    try:
        rows = [r for r in (measure(client, s, args.every, args.until) for s in args.sessions) if r]
    finally:
        if proc is not None:
            proc.terminate()
    if args.json:
        print(json.dumps(rows, indent=2))
        return 0
    head = ["session", "n", "off-centre med", "p90", "max", ">0.5m", ">1.0m", "wander p90", "head med"]
    print("| " + " | ".join(head) + " |")
    print("|" + "|".join("---" for _ in head) + "|")
    for r in rows:
        print("| " + " | ".join([
            r["session"], str(r["samples"]), f"{r['median']:.2f} m", f"{r['p90']:.2f} m", f"{r['max']:.2f} m",
            f"{100*r['beyond_0_5']:.0f}%", f"{100*r['beyond_1_0']:.0f}%",
            f"{r['wander_p90']:.2f} m/s", f"{r['heading_median']:.1f}°",
        ]) + " |")
    print("\nA lane is ~3.5 m wide, so beyond ~1.75 m the car is out of it. Heading error inside a "
          "junction is unreliable (the map projects onto the nearest crossing lane).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
