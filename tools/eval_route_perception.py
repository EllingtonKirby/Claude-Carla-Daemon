#!/usr/bin/env python3
"""Score a model on where it thinks the road goes, against the dense route.

    python tools/eval_route_perception.py SCENES_DIR [--model claude-opus-5] [--limit 40]
                                          [--out DIR] [--json]

Each scene (from tools/capture_route_scenes.py) holds the camera images a driver would see, the
sparse `nav.route_points_ego` it actually reasons from, and the DENSE route as the label. The model
is shown the images and the sparse points -- the driver's real situation -- and asked for the
route's lateral offset at fixed distances ahead. Its answer is scored against the label.

The question: when a driver hands `apply_plan` a path, is that path where the road actually goes?
If the error grows past a lane width by 20 m, the waypoints feeding the PID are wrong at the source
and controller tuning cannot fix it. If the error stays small, the control problem is downstream.

Runs the model through `claude -p`, so it bills the same subscription a benchmark run does; there is
no API key here. One call per scene, each short. Scenes are scored independently, so this is
resumable and parallel-safe: re-running skips scenes whose answer file already exists.

Reports mean |error| per distance -- the error-versus-distance curve is the whole point, since a
model can be perfect at 5 m and useless at 30 m -- plus the share of scenes where the answer lands
within half a lane (0.875 m) at each distance.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

# Distances scored. Kept inside the captured truth's reach; each is a separate question, because a
# single aggregate hides exactly the near-good/far-bad shape we are looking for.
DISTANCES = [5.0, 10.0, 15.0, 20.0, 30.0, 40.0]
HALF_LANE = 0.875  # a lane is ~3.5 m: beyond this the car is closer to the line than the centre

PROMPT = """You are looking at the camera views from a car driving in the CARLA simulator, on a \
route it must follow. The ego frame is: +forward is straight ahead, +left is to your LEFT, in metres.

You are also given the route waypoints the navigation system provides, which are sparse and \
approximate: {nav}
(these are [forward, left] in metres, at {at_m} m ahead respectively)

From the images and those points, estimate where the ROAD YOU SHOULD DRIVE actually goes. Report \
the lateral offset of the centre of your driving path at each of these distances ahead: {dists} m.

Answer with one line of JSON and nothing else:
{{"offsets": [<offset at {first} m>, ..., <offset at {last} m>]}}
Positive means the path lies to your LEFT, negative to your RIGHT. Use metres."""


def ask(scene: Dict[str, Any], model: str, cwd: Path) -> Optional[List[float]]:
    nav = scene.get("nav_sparse") or []
    prompt = PROMPT.format(
        nav=json.dumps(nav), at_m=json.dumps(scene.get("nav_sparse_at_m") or []),
        dists=", ".join(f"{d:g}" for d in DISTANCES), first=f"{DISTANCES[0]:g}", last=f"{DISTANCES[-1]:g}")
    # The images go in as Read calls the model makes itself, which is how a driver sees them.
    listing = "\n".join(f"  {name}: {path}" for name, path in sorted(scene["images"].items()))
    prompt = f"{prompt}\n\nRead these images first:\n{listing}"
    allow = ",".join(f"Read({p})" for p in sorted(scene["images"].values()))
    out = subprocess.run(
        ["claude", "-p", prompt, "--model", model, "--setting-sources", "", "--tools", "Read",
         "--permission-mode", "dontAsk", "--allowedTools", allow],
        cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=300)
    m = re.search(r'\{[^{}]*"offsets"\s*:\s*\[[^\]]*\][^{}]*\}', out.stdout)
    if not m:
        return None
    try:
        vals = json.loads(m.group(0))["offsets"]
    except (json.JSONDecodeError, KeyError):
        return None
    return [float(v) for v in vals] if len(vals) == len(DISTANCES) else None


def truth_at(scene: Dict[str, Any]) -> List[float]:
    """The dense route's lateral offset at each scored distance, interpolated along forward."""
    t = np.asarray(scene["truth"], dtype=float)
    return [float(np.interp(d, t[:, 0], t[:, 1])) for d in DISTANCES]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", type=Path)
    ap.add_argument("--model", default="claude-opus-5")
    ap.add_argument("--limit", type=int, default=40)
    ap.add_argument("--out", type=Path, default=None, help="default: SCENES_DIR/answers_<model>/")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    out = args.out or args.scenes / f"answers_{args.model}"
    out.mkdir(parents=True, exist_ok=True)
    files = sorted(args.scenes.glob("scene_*.json"))[:args.limit]
    assert files, f"no scene_*.json in {args.scenes}"

    rows: List[Dict[str, Any]] = []
    for i, f in enumerate(files, 1):
        scene = json.loads(f.read_text())
        cached = out / f.name
        if cached.is_file():
            answer = json.loads(cached.read_text())["offsets"]
        else:
            answer = ask(scene, args.model, repo)
            cached.write_text(json.dumps({"step": scene["step"], "offsets": answer}))
        truth = truth_at(scene)
        ok = answer is not None
        print(f"[{i}/{len(files)}] step {scene['step']:>5} " +
              ("  ".join(f"{d:g}m {a:+.1f}/{t:+.1f}" for d, a, t in zip(DISTANCES, answer, truth))
               if ok else "  no parsable answer"), flush=True)
        rows.append({"step": scene["step"], "answer": answer, "truth": truth, "ok": ok})

    scored = [r for r in rows if r["ok"]]
    if not scored:
        print("\nno scenes produced a parsable answer")
        return 1
    err = np.abs(np.array([r["answer"] for r in scored]) - np.array([r["truth"] for r in scored]))
    if args.json:
        print(json.dumps({"model": args.model, "n": len(scored), "distances": DISTANCES,
                          "mean_abs_error": err.mean(axis=0).tolist(),
                          "within_half_lane": (err < HALF_LANE).mean(axis=0).tolist(),
                          "rows": rows}, indent=2))
        return 0
    print(f"\n{args.model}: {len(scored)}/{len(rows)} scenes answered\n")
    print("| distance | mean |error| | p90 |error| | within half a lane |")
    print("|---|---|---|---|")
    for j, d in enumerate(DISTANCES):
        print(f"| {d:g} m | {err[:, j].mean():.2f} m | {np.percentile(err[:, j], 90):.2f} m | "
              f"{100*(err[:, j] < HALF_LANE).mean():.0f}% |")
    print(f"\nA lane is ~3.5 m wide, so an error past {HALF_LANE} m puts the planned path nearer the "
          f"lane line than the centre.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
