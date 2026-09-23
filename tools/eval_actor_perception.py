#!/usr/bin/env python3
"""Score a model on WHERE THE OTHER CARS ARE, against the simulator's ground truth.

    python tools/eval_actor_perception.py SCENES_DIR --model claude-haiku-4-5-20251001 [--limit 40]
                                          [--range 40] [--out DIR] [--json]

Each scene (tools/capture_route_scenes.py, privileged) holds the camera images a driver sees and
`actors_truth`, every vehicle and pedestrian within range in the ego frame. The model is shown the
images and asked to list every road user it can see as [fwd, left] metres. Predictions are matched
to truth greedily by distance (a match must be within 6 m); the report gives, per range band:

  recall       share of true actors the model reported at all
  error        distance between the reported and the true position of matched actors
  fwd / left   the SIGNED mean error along and across -- a consistent sign is a bias a rule can
               correct ("you under-estimate lateral separation by 1 m"); noise is not
  extras       reported actors that match nothing (hallucinations, or actors outside the range)

Why: every collision we have is a commitment made a few metres from a car whose position the
driver had just estimated. This measures that estimate, by range, offline, for any model, for a
few cents a scene. Runs the model through `claude -p` (the subscription); answers are cached per
scene so re-runs are free and scenes can be added.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

BANDS = [(0.0, 10.0), (10.0, 20.0), (20.0, 40.0)]
MATCH_M = 6.0

PROMPT = """You are looking at the camera views from a car in the CARLA simulator. The ego frame is: \
fwd = metres straight ahead of you (negative = behind), left = metres to your LEFT (negative = right). \
The cameras: CAM_FRONT looks ahead ({fov_front} deg wide), CAM_FRONT_LEFT and CAM_FRONT_RIGHT look 60 deg \
to each side, CAM_BACK looks straight back. Your own car is about 4.9 m long and 1.8 m wide; the \
cameras sit near its centre.

List EVERY vehicle and pedestrian you can see within {range_m:.0f} m, each as its centre position \
[fwd, left] in metres. Include cars beside and behind you. Do not list anything you cannot see.

Answer with one line of JSON and nothing else:
{{"actors": [[fwd, left], [fwd, left], ...]}}
(an empty list if there is none)"""


def ask(scene: Dict[str, Any], model: str, cwd: Path, range_m: float, fov_front: int) -> Optional[List[List[float]]]:
    prompt = PROMPT.format(range_m=range_m, fov_front=fov_front)
    listing = "\n".join(f"  {name}: {path}" for name, path in sorted(scene["images"].items()))
    prompt = f"{prompt}\n\nRead these images first:\n{listing}"
    allow = ",".join(f"Read({p})" for p in sorted(scene["images"].values()))
    out = subprocess.run(
        ["claude", "-p", prompt, "--model", model, "--setting-sources", "", "--tools", "Read",
         "--permission-mode", "dontAsk", "--allowedTools", allow],
        cwd=str(cwd), stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
        timeout=300, stdin=subprocess.DEVNULL)
    m = re.search(r'\{\s*"actors"\s*:\s*\[.*?\]\s*\}', out.stdout, re.S)
    if not m:
        return None
    try:
        vals = json.loads(m.group(0))["actors"]
        return [[float(v[0]), float(v[1])] for v in vals]
    except (json.JSONDecodeError, KeyError, TypeError, ValueError, IndexError):
        return None


def match(pred: List[List[float]], truth: List[List[float]]):
    """Greedy nearest-pair matching within MATCH_M. Returns [(truth_i, pred_j, error)], unmatched preds."""
    pairs = []
    if pred and truth:
        P, T = np.asarray(pred, float), np.asarray(truth, float)
        d = np.linalg.norm(T[:, None, :] - P[None, :, :], axis=2)
        used_t, used_p = set(), set()
        for flat in np.argsort(d, axis=None):
            i, j = divmod(int(flat), d.shape[1])
            if d[i, j] > MATCH_M:
                break
            if i in used_t or j in used_p:
                continue
            used_t.add(i); used_p.add(j)
            pairs.append((i, j, float(d[i, j])))
    extras = [j for j in range(len(pred)) if j not in {p[1] for p in pairs}]
    return pairs, extras


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenes", type=Path)
    ap.add_argument("--model", required=True)
    ap.add_argument("--limit", type=int, default=60)
    ap.add_argument("--range", type=float, default=40.0, help="score actors within this range (m)")
    ap.add_argument("--fov-front", type=int, default=110)
    ap.add_argument("--out", type=Path, default=None, help="default: SCENES_DIR/actors_<model>/")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parents[1]
    out = args.out or args.scenes / f"actors_{args.model}"
    out.mkdir(parents=True, exist_ok=True)
    files = sorted(args.scenes.glob("scene_*.json"))[:args.limit]
    assert files, f"no scene_*.json in {args.scenes}"

    per_band = {b: {"n": 0, "found": 0, "err": [], "fwd": [], "left": []} for b in BANDS}
    extras_total, answered, n_pred = 0, 0, 0
    for i, f in enumerate(files, 1):
        scene = json.loads(f.read_text())
        truth_all = [a for a in scene.get("actors_truth", []) + scene.get("parked_truth", [])
                     if (a["fwd"] ** 2 + a["left"] ** 2) ** 0.5 <= args.range]
        truth = [[a["fwd"], a["left"]] for a in truth_all]
        cached = out / f.name
        if cached.is_file():
            pred = json.loads(cached.read_text())["actors"]
        else:
            pred = ask(scene, args.model, repo, args.range, args.fov_front)
            cached.write_text(json.dumps({"step": scene["step"], "actors": pred}))
        if pred is None:
            print(f"[{i}/{len(files)}] step {scene['step']:>5}  no parsable answer  (truth: {len(truth)} actors)")
            continue
        answered += 1
        n_pred += len(pred)
        pairs, extras = match(pred, truth)
        extras_total += len(extras)
        matched = {p[0] for p in pairs}
        for ti, t in enumerate(truth):
            r = (t[0] ** 2 + t[1] ** 2) ** 0.5
            for b in BANDS:
                if b[0] <= r < b[1]:
                    per_band[b]["n"] += 1
                    if ti in matched:
                        j, e = next((p[1], p[2]) for p in pairs if p[0] == ti)
                        per_band[b]["found"] += 1
                        per_band[b]["err"].append(e)
                        per_band[b]["fwd"].append(pred[j][0] - t[0])
                        per_band[b]["left"].append(pred[j][1] - t[1])
        print(f"[{i}/{len(files)}] step {scene['step']:>5}  truth {len(truth):>2}  reported {len(pred):>2}  "
              f"matched {len(pairs):>2}  extras {len(extras)}  " +
              ("  ".join(f"({truth[p[0]][0]:.0f},{truth[p[0]][1]:+.0f})->err {p[2]:.1f}" for p in pairs[:4])), flush=True)

    if not answered:
        print("\nno scenes produced a parsable answer")
        return 1
    if args.json:
        print(json.dumps({"model": args.model, "scenes": answered, "bands": [
            {"band": b, **{k: (v if k in ("n", "found") else float(np.mean(v)) if v else None)
                            for k, v in per_band[b].items()}} for b in BANDS],
            "extras": extras_total, "reported": n_pred}, indent=1))
        return 0
    print(f"\n{args.model}: {answered}/{len(files)} scenes answered, {n_pred} actors reported, "
          f"{extras_total} matched nothing within {MATCH_M:.0f} m\n")
    print("| range | true actors | recall | mean error | signed fwd | signed left |")
    print("|---|---|---|---|---|---|")
    for b in BANDS:
        d = per_band[b]
        if d["n"] == 0:
            continue
        rec = d["found"] / d["n"]
        print(f"| {b[0]:.0f}-{b[1]:.0f} m | {d['n']} | {100 * rec:.0f}% | "
              + (f"{np.mean(d['err']):.1f} m | {np.mean(d['fwd']):+.1f} m | {np.mean(d['left']):+.1f} m |"
                 if d["err"] else "- | - | - |"))
    print("\nsigned = reported minus true; a consistent sign is a bias, noise is not.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
