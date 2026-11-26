#!/usr/bin/env python3
"""Sweep the plan controller's steering settings on the test pad and rank them by tracking error.

    python tools/tune_controller.py --port 20090 [--out tmp/tune.json]

Same rig and manoeuvres as tools/characterize_controller.py (the Bench2Drive car, the daemon's own
controller class, 20 Hz synchronous), restricted to the ones that cost points: lane changes and
arcs at 3-9 m/s, plus a straight run to catch a setting that weaves. Longitudinal: the launch boost.
Score: mean p95 cross-track error over the manoeuvres; straight-line steer std is reported beside
it because a short lookahead buys corner accuracy with weave.
"""
import argparse
import copy
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import carla  # noqa: E402
from characterize_controller import DT, Rig, arc_plan, cross_track, run_up, straight_plan  # noqa: E402

from carla_probe.pid import PIDController  # noqa: E402

MANOEUVRES = [("offset", 6.0, 3.5, 15.0), ("offset", 9.0, 3.5, 15.0), ("offset", 9.0, 1.5, 15.0),
              ("arc", 3.0, 12.0, None), ("arc", 6.0, 12.0, None), ("arc", 6.0, 20.0, None), ("arc", 9.0, 35.0, None),
              ("straight", 9.0, None, None)]


def candidates(base):
    out = [("adaptive kp2.0 la=0.6v+1.5", {"turn_kp": 2.0, "speed_scale": 0.6, "speed_offset": 1.5})]
    for kp, ki, scale, off in ((2.5, 0.5, 0.6, 1.5), (2.0, 0.8, 0.6, 1.5), (2.0, 0.5, 0.5, 1.2), (2.5, 0.5, 0.5, 1.2),
                               (2.0, 0.5, 0.45, 1.0), (2.0, 0.5, 0.6, 1.0), (3.0, 0.5, 0.6, 1.5)):
        out.append((f"adaptive kp{kp} ki{ki} la={scale}v+{off}", {"turn_kp": kp, "turn_ki": ki, "speed_scale": scale,
                                                               "speed_offset": off, "lookahead_min": min(2.5, off + 1.0)}))
    return out


def run(rig, cfg, m):
    kind, v, a, b = m
    ctl = PIDController(cfg)
    run_up(rig, ctl, v)
    x, y, yaw, _ = rig.state()
    if kind == "offset":
        off = lambda s, lat=a, dist=b: lat * (0.5 - 0.5 * math.cos(math.pi * min(s, dist) / dist))
        plan = straight_plan(x, y, yaw, length=b + 60, step=1.0, offset=off)
        ticks = int((b + 15) / v / DT)
    elif kind == "arc":
        plan = arc_plan(x, y, yaw, a, length=v * 3.0 + 20)
        ticks = int(3.0 / DT)
    else:
        plan = straight_plan(x, y, yaw, length=120, step=2.0)
        ticks = int(4.0 / DT)
    rows = rig.drive(ctl, plan, v, ticks, 0.5)
    e = cross_track(rows, plan, (x, y))
    steer = np.array([r[6] for r in rows])
    return float(np.percentile(e, 95)), float(np.std(np.diff(steer)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=20090)
    ap.add_argument("--out", type=Path, default=Path("tmp/tune.json"))
    a = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    base = yaml.safe_load((repo / "carla_probe" / "config.yaml").read_text())["pid"]
    base.update({"long_mode": "pid", "max_throttle": 0.75, "launch_throttle": 1.0})
    client = carla.Client("localhost", a.port)
    client.set_timeout(300)
    rig = Rig(client, "Town04")
    pc = rig.car.get_physics_control()
    print("steering_curve (speed km/h -> steer scale):", [(round(p.x, 1), round(p.y, 2)) for p in pc.steering_curve], flush=True)
    results = []
    try:
        for name, over in candidates(base):
            cfg = copy.deepcopy(base)
            cfg.update(over)
            p95s, jitter = [], 0.0
            for m in MANOEUVRES:
                p95, jit = run(rig, cfg, m)
                if m[0] == "straight":
                    jitter = jit
                else:
                    p95s.append(p95)
            r = {"name": name, "override": over, "mean_p95": round(float(np.mean(p95s)), 3),
                 "worst_p95": round(float(np.max(p95s)), 3), "straight_steer_jitter": round(jitter, 4),
                 "per_manoeuvre": [round(x, 3) for x in p95s]}
            results.append(r)
            print(f"{name:32s} mean p95 {r['mean_p95']:.2f}  worst {r['worst_p95']:.2f}  jitter {jitter:.4f}  {r['per_manoeuvre']}", flush=True)
            a.out.parent.mkdir(parents=True, exist_ok=True)
            a.out.write_text(json.dumps(results, indent=1))
    finally:
        rig.car.destroy()
        s = rig.world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = False, None
        rig.world.apply_settings(s)


if __name__ == "__main__":
    main()
