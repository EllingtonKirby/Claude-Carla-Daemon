#!/usr/bin/env python3
"""How well did `predict` foresee the ego's motion? Compares predictions.jsonl (one predicted path per
apply_plan, written by privileged-extras sessions) with trajectory.jsonl (where the car went).

    python tools/prediction_error.py SESSION_DIR [SESSION_DIR ...]

Per plan and pooled: position error (m) at 0.5 / 1.0 / 1.5 s, split along / across the car's heading,
and how often the actual sideways deviation stayed inside the predicted band_m. A band that holds
~95% of the time is honest; much less means the model under-states the controller's error.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_probe import geometry as geo  # noqa: E402


def session(d: Path):
    pred = [json.loads(l) for l in (d / "predictions.jsonl").read_text().splitlines() if l.strip()]
    traj = {json.loads(l)["step"]: json.loads(l) for l in (d / "trajectory.jsonl").read_text().splitlines() if l.strip()}
    rows = []
    for p in pred:
        x0, y0, yaw0 = p["ego_world"]
        for i, pose in enumerate(p["poses"]):
            actual = traj.get(p["step"] + i + 1)
            if actual is None:
                break
            a = geo.world_to_ego((x0, y0), yaw0, np.array([[actual["x"], actual["y"]]]))[0]
            h = math.radians(pose["yaw_rel_deg"])
            d_f, d_l = a[0] - pose["fwd"], a[1] - pose["left"]
            along = d_f * math.cos(h) + d_l * math.sin(h)
            across = -d_f * math.sin(h) + d_l * math.cos(h)
            rows.append((pose["t"], along, across, pose["band_m"], actual["speed"] - pose["speed"]))
    return rows


def main():
    rows = []
    for arg in sys.argv[1:]:
        r = session(Path(arg))
        rows += r
        print(f"{arg}: {len(r)} predicted ticks")
    if not rows:
        return 1
    R = np.array(rows)
    for t in (0.5, 1.0, 1.5):
        m = np.isclose(R[:, 0], t)
        if m.any():
            print(f"t={t:.1f}s: |along| median {np.median(abs(R[m, 1])):.2f} m  |across| median {np.median(abs(R[m, 2])):.2f} m"
                  f"  p95 {np.percentile(abs(R[m, 2]), 95):.2f} m  speed error median {np.median(R[m, 4]):+.2f} m/s  (n={m.sum()})")
    inside = np.mean(np.abs(R[:, 2]) <= R[:, 3])
    print(f"sideways deviation inside the predicted band: {100 * inside:.0f}% of ticks (target ~95%)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
