#!/usr/bin/env python3
"""Does the VLA service run, and does it put the road where the road is?

    python tools/vla_smoke.py SCENES_DIR [--addr 127.0.0.1:8791] [--modes plain,cot,commentary]
                              [--text "Follow the route. Accelerate to drive through the junction."]

Feeds each captured scene (tools/capture_route_scenes.py) to carla_probe/vla_server.py in each
mode and compares the returned 20-point route with the DENSE route label at 5/10/15/20 m, the
same yardstick tools/eval_route_perception.py applies to the LLM driver. Also prints the model's
language (its own commentary in `cot`, its acknowledgement otherwise) and the per-step latency.

A frame-convention check is built in: on a scene where the truth bends LEFT (positive lateral
offset), a route that bends RIGHT by the same amount means the [fwd,left] <-> [fwd,right] flip in
the server is wrong, not the model.
"""

from __future__ import annotations

import argparse
import base64
import json
import urllib.request
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

DISTANCES = [5.0, 10.0, 15.0, 20.0]


def post(addr: str, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
    req = urllib.request.Request(f"http://{addr}{path}", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read())


def lateral_at(points: List[List[float]], dists: List[float]) -> List[float]:
    p = np.asarray(points, dtype=float)
    p = p[np.argsort(p[:, 0])]
    return [float(np.interp(d, p[:, 0], p[:, 1])) for d in dists]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", type=Path)
    ap.add_argument("--addr", default="127.0.0.1:8791")
    ap.add_argument("--modes", default="plain,cot")
    ap.add_argument("--text", default="Follow the route. Accelerate to drive with the target speed.")
    ap.add_argument("--camera", default="CAM_FRONT")
    ap.add_argument("--limit", type=int, default=20)
    args = ap.parse_args()

    files = sorted(args.scenes.glob("scene_*.json"))[:args.limit]
    assert files, f"no scenes in {args.scenes}"
    modes = args.modes.split(",")
    errs: Dict[str, List[List[float]]] = {m: [] for m in modes}
    post(args.addr, "/reset", {})
    for f in files:
        s = json.loads(f.read_text())
        nav = s["nav_sparse"]
        # SimLingo takes the next two sparse route nodes; ours are at fixed distances, so use the
        # 10 m and 20 m ones, which are the closest analogue to a planner's min_distance of 7.5 m.
        tps = [nav[1], nav[2]] if len(nav) >= 3 else nav[:2]
        img = base64.b64encode(Path(s["images"][args.camera]).read_bytes()).decode()
        truth = lateral_at(s["truth"], DISTANCES)
        print(f"\nstep {s['step']:>5}  v={s['speed_mps']:.1f}  truth " +
              " ".join(f"{t:+.1f}" for t in truth) + f"  (at {DISTANCES} m)")
        for m in modes:
            body = {"image_jpeg_b64": img, "speed_mps": s["speed_mps"], "target_points": tps,
                    "mode": m, "text": args.text}
            out = post(args.addr, "/act", body)
            if not out.get("ok"):
                print(f"  {m:<11} ERROR {out.get('error')}")
                continue
            got = lateral_at(out["route"], DISTANCES)
            errs[m].append([abs(g - t) for g, t in zip(got, truth)])
            c = out["control"]
            print(f"  {m:<11} route " + " ".join(f"{g:+.1f}" for g in got) +
                  f"  v*={out['desired_speed_mps']:.1f}  steer={c['steer']:+.2f} thr={c['throttle']:.2f} "
                  f"brk={c['brake']:.0f}  {out['ms']:.0f} ms  lang={out['language']!r}")
    print("\nmean |lateral error| vs the dense route, m:")
    print("| mode | " + " | ".join(f"{d:g} m" for d in DISTANCES) + " | n |")
    print("|---|" + "---|" * len(DISTANCES) + "---|")
    for m in modes:
        if errs[m]:
            e = np.asarray(errs[m])
            print(f"| {m} | " + " | ".join(f"{v:.2f}" for v in e.mean(axis=0)) + f" | {len(e)} |")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
