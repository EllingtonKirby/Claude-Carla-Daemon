#!/usr/bin/env python3
"""How close the car actually came to anything, once per plan.

    python tools/clearance_lint.py BEFORE=dev10,base2,base3 AFTER=guide2,new2,new3
    python tools/clearance_lint.py RUN=dev10 --jsonl tmp/clearance.jsonl --near 0.5

Offline, but it needs ``recorder_truth.json`` in each session (``tools/recorder_truth.py``) and
``tools/actor_extents.json`` (``tools/actor_extents.py --rebuild``).

Why this exists: a collision is the only outcome the scorer records, and there are 49 of them in a
60-session corpus -- too few to tell whether a change helped. A near miss is the same behaviour
with a better draw, and there are hundreds per corpus. Measuring the clearance the driver LEAVES,
once per plan, turns a change that would need ~60 sessions to show up in DS into one that shows up
in one offline pass over sessions already on disk (docs/JOURNAL.md §1.2).

What it measures, precisely: for every ``apply_plan``, the smallest gap between the ego's body and
any other road user's body at any tick of the burst, from the recording. Bodies, not centres --
the distinction is the entire point, since two cars touch when their centres are still 4.8 m apart.

Four things it deliberately does not do:

* **It does not use the driver's estimate of where anything was.** This is ground truth, so it
  measures the situation the driver created, not the one it believed in. (``clearance`` in the
  daemon is the other half of that pair, and answers from the driver's own estimates.)
* **It ignores map geometry.** Kerbs, poles and walls come back from the recorder as thousands of
  ``static.prop.mesh`` actors with no usable extent, so clearance here is to vehicles, walkers and
  scenario props only. Contacts with layout (8 of the first corpus's 49) are invisible to it.
* **It does not judge fault.** A vehicle that drives into a correctly stopped ego produces a small
  clearance and should: the number measures exposure, not blame.
* **It is not a collision count.** Zero clearance means the bounding boxes overlapped at a sampled
  tick, which is close to but not the same as the scorer's collision event. Checked against the
  scorer over the first corpus: 52 of 60 sessions agree on whether anything was hit, 7 sessions
  show an overlap the scorer did not score, and 1 scored a contact with map layout, which this
  tool excludes by design. At plan level, 185 plans overlap and 17 of those are in sessions that
  scored nothing -- a ~9% over-count, 10 of the 17 against bicycles and pedestrians, whose
  blueprint boxes are generous next to what the physics actually collides with. So read the
  overlap column as an upper bound and the near-miss rate, which does not depend on the last few
  centimetres, as the measurement.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

import actor_extents  # noqa: E402
from collisions import (ALIGN_TOLERANCE_M, DEFAULT_GLOB, alignment_error, issued,  # noqa: E402
                        pose, sessions, trajectory, truth)
from pool_report import read_report_only, repo_commit  # noqa: E402

NEAR_M = 0.5
"""At or below this, the plan came close enough that the outcome was luck. Half a metre is about
the tracking error the controller admits to plus a margin; it is a reporting threshold, not a
physical one, and the distributions are printed so it can be argued with."""

STRIDE = 2
"""Sample every Nth tick of a burst. At 20 Hz and road speeds a tick is under half a metre, so
every second tick cannot hide an approach; halving the work matters over 60 sessions."""

CONSIDER_M = 25.0
"""Centre-to-centre pre-filter. Nothing outside this can be the closest body."""

ROAD_USER_PREFIXES = ("vehicle.", "walker.")
SCENARIO_PROP_PREFIXES = ("static.prop.constructioncone", "static.prop.trafficwarning",
                          "static.prop.warningconstruction", "static.prop.dirtdebris")
"""``static.prop.mesh`` is map furniture and is excluded: there are thousands per route and the
recorder gives no extent for them. The named props are scenario obstacles, which are placed to be
driven around and are exactly what a clearance metric should see."""


# --------------------------------------------------------------------------- polygon geometry

def corners(x: float, y: float, yaw_deg: float, ext: Dict[str, float]) -> List[Tuple[float, float]]:
    """Body corners in the CARLA world frame.

    Built from the transform's own basis vectors -- forward ``(cos, sin)``, right ``(-sin, cos)``
    -- so the world's left-handedness is carried rather than assumed away (carla_probe/geometry.py).
    """
    a = math.radians(yaw_deg)
    fx, fy = math.cos(a), math.sin(a)
    rx, ry = -math.sin(a), math.cos(a)
    f, r, w = ext["front_m"], -ext["rear_m"], ext["half_width_m"]
    return [(x + fx * lf + rx * lw, y + fy * lf + ry * lw)
            for lf, lw in ((f, w), (f, -w), (r, -w), (r, w))]


def _seg_point(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    den = dx * dx + dy * dy
    t = 0.0 if den == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / den))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _separated(poly_a, poly_b) -> bool:
    """Separating-axis test over both rectangles' edge normals."""
    for poly in (poly_a, poly_b):
        for i in range(len(poly)):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % len(poly)]
            nx, ny = -(by - ay), bx - ax
            pa = [nx * px + ny * py for px, py in poly_a]
            pb = [nx * px + ny * py for px, py in poly_b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return True
    return False


def polygon_distance(poly_a, poly_b) -> float:
    """Metres between two convex polygons; 0.0 when they overlap."""
    if not _separated(poly_a, poly_b):
        return 0.0
    best = float("inf")
    for p, q in ((poly_a, poly_b), (poly_b, poly_a)):
        for px, py in p:
            for i in range(len(q)):
                ax, ay = q[i]
                bx, by = q[(i + 1) % len(q)]
                best = min(best, _seg_point(px, py, ax, ay, bx, by))
    return best


# --------------------------------------------------------------------------- per-session pass

def _is_obstacle(type_id: str) -> bool:
    return (type_id.startswith(ROAD_USER_PREFIXES)
            or type_id.startswith(SCENARIO_PROP_PREFIXES))


def bursts(cmds: Sequence[dict], last_step: int) -> List[dict]:
    """Every ``apply_plan`` with the step range its burst actually occupied.

    A burst ends at its own ``ticks`` or when the next command supersedes it, whichever is first:
    the driver can cut a plan short, and counting the requested ticks would then charge a plan for
    a stretch it was not driving.
    """
    plans = [c for c in cmds if c.get("cmd") == "apply_plan"]
    steps = sorted({c.get("step") for c in cmds if c.get("step") is not None})
    out = []
    for c in plans:
        s0 = c.get("step")
        if s0 is None:
            continue
        nxt = next((s for s in steps if s > s0), None)
        s1 = s0 + int(c.get("args", {}).get("ticks") or 0)
        out.append({"i": c.get("i"), "step0": s0, "step1": min(x for x in (s1, nxt, last_step)
                                                               if x is not None),
                    "why": c.get("why"), "args": c.get("args") or {}})
    return out


def session_clearances(session_dir: str, table: Dict[str, Dict[str, float]], *,
                       stride: int = STRIDE, unknown: Optional[set] = None) -> Optional[List[dict]]:
    """One record per plan, or None when the session has no usable ground truth."""
    tr = truth(session_dir)
    traj = trajectory(session_dir)
    if not tr or not traj:
        return None
    align = alignment_error(tr, traj)
    if align is None or align > ALIGN_TOLERANCE_M:
        return None
    hero_id = str(tr["hero_id"])
    hero_type = (tr["actors"].get(hero_id) or {}).get("type", "")
    hero_ext = actor_extents.extent_of(table, hero_type, unknown)
    kinds = {aid: a.get("type", "") for aid, a in tr["actors"].items()
             if aid != hero_id and _is_obstacle(a.get("type", ""))}
    if not kinds:
        return []
    exts = {aid: actor_extents.extent_of(table, t, unknown) for aid, t in kinds.items()}

    out = []
    steps = sorted(traj)
    for b in bursts(issued(session_dir), steps[-1] if steps else 0):
        best, best_at, best_id = float("inf"), None, None
        for step in range(b["step0"], b["step1"] + 1, max(1, stride)):
            row = traj.get(step)
            if row is None or "frame" not in row:
                continue
            hero = pose(tr, row["frame"], hero_id)
            if hero is None:
                continue
            hero_poly = corners(hero[0], hero[1], hero[2], hero_ext)
            for aid, ext in exts.items():
                other = pose(tr, row["frame"], aid)
                if other is None:
                    continue
                if math.hypot(other[0] - hero[0], other[1] - hero[1]) > CONSIDER_M:
                    continue
                d = polygon_distance(hero_poly, corners(other[0], other[1], other[2], ext))
                if d < best:
                    best, best_at, best_id = d, step, aid
                    if d == 0.0:
                        break
            if best == 0.0:
                break
        if best == float("inf"):
            continue
        # How far the ego actually went during the burst. A per-PLAN rate is confounded if a
        # change also changes plan length -- shorter plans meet fewer hazards each, and the rate
        # falls without the driving being any safer. Exposure lets the rate be normalised.
        travelled = 0.0
        prev = None
        for step in range(b["step0"], b["step1"] + 1):
            row = traj.get(step)
            if row is None:
                continue
            if prev is not None:
                travelled += math.hypot(row["x"] - prev["x"], row["y"] - prev["y"])
            prev = row
        out.append({
            "dist_m": round(travelled, 2), "ticks": b["step1"] - b["step0"],
            "session": os.path.basename(session_dir), "commit": repo_commit(session_dir),
            "plan_i": b["i"], "step0": b["step0"], "step1": b["step1"],
            "min_clearance_m": round(best, 3), "at_step": best_at,
            "nearest_type": kinds.get(best_id), "n_wps": len(b["args"].get("waypoints") or []),
            "target_speed": b["args"].get("target_speed"), "why": b["why"],
        })
    return out


# --------------------------------------------------------------------------- reporting

def gather(specs: Sequence[str], pattern: str = DEFAULT_GLOB, stride: int = STRIDE,
           table: Optional[Dict] = None) -> Tuple[List[dict], Dict[str, List[str]], set]:
    table = actor_extents.load() if table is None else table
    unknown: set = set()
    recs, skipped = [], {"no_truth": []}
    for arm, route, d in sessions(specs, pattern):
        got = session_clearances(d, table, stride=stride, unknown=unknown)
        if got is None:
            skipped["no_truth"].append(os.path.basename(d))
            continue
        for r in got:
            r["arm"], r["route"] = arm, route
        recs.extend(got)
    return recs, skipped, unknown


def _quantiles(values: Sequence[float]) -> Dict[str, float]:
    v = sorted(values)
    if not v:
        return {}
    def q(p):
        return v[min(len(v) - 1, int(p * len(v)))]
    return {"p05": q(0.05), "p10": q(0.10), "median": statistics.median(v), "p90": q(0.90)}


def arm_order(recs: Sequence[dict], specs: Optional[Sequence[str]] = None) -> List[str]:
    """Arms in the order the caller named them, NOT sorted.

    Sorting them put AFTER before BEFORE and printed the delta as if the change had been undone.
    The direction of a comparison is the one thing a reader takes straight from the output.
    """
    if specs:
        named = [sp.partition("=")[0] for sp in specs]
        present = {r["arm"] for r in recs}
        return [n for n in named if n in present]
    seen: List[str] = []
    for r in recs:
        if r["arm"] not in seen:
            seen.append(r["arm"])
    return seen


def near_rate(recs: Sequence[dict], near_m: float) -> float:
    return sum(1 for r in recs if r["min_clearance_m"] <= near_m) / len(recs)


def _paired(recs: Sequence[dict], arms: Sequence[str], near_m: float) -> None:
    """Per-route near-miss rates, paired across the two arms.

    Pairing matters for the same reason it does in tools/pool_report.py: routes differ enormously
    in how much room there is to leave, and an unpaired corpus rate mixes that in with the effect.
    The report-only route is printed and excluded, as everywhere else.
    """
    a, b = arms
    report_only = read_report_only()
    routes = sorted({r["route"] for r in recs if r["route"]})
    print("\nnear-miss rate (clearance <= %.2f m) per route" % near_m)
    print("%-10s %8s %8s %8s %8s %9s" % ("route", f"{a} n", a, f"{b} n", b, "delta"))
    deltas = []
    for route in routes:
        ra = [r for r in recs if r["route"] == route and r["arm"] == a]
        rb = [r for r in recs if r["route"] == route and r["arm"] == b]
        if not ra or not rb:
            continue
        pa, pb = near_rate(ra, near_m), near_rate(rb, near_m)
        flag = "  (report-only)" if route in report_only else ""
        print("%-10s %8d %7.1f%% %8d %7.1f%% %+8.1f%s"
              % (route, len(ra), 100 * pa, len(rb), 100 * pb, 100 * (pb - pa), flag))
        if route not in report_only:
            deltas.append(100 * (pb - pa))
    if len(deltas) < 2:
        return
    mean = statistics.mean(deltas)
    sd = statistics.stdev(deltas)
    se = sd / math.sqrt(len(deltas))
    print("\npaired over %d routes: mean %+.1f points, sd %.1f, SE %.1f, t = %.2f (df %d); "
          "%d of %d improved" % (len(deltas), mean, sd, se, mean / se if se else float("nan"),
                                 len(deltas) - 1, sum(1 for d in deltas if d < 0), len(deltas)))
    print("Negative is better: fewer plans left under %.2f m." % near_m)


def report(recs: Sequence[dict], near_m: float = NEAR_M, skipped=None, unknown=None,
           specs: Optional[Sequence[str]] = None) -> None:
    if not recs:
        print("no plans with ground truth")
        return
    if skipped and skipped.get("no_truth"):
        print("skipped (no usable recorder truth): %d session(s): %s\n"
              % (len(skipped["no_truth"]), " ".join(sorted(skipped["no_truth"])[:6])))
    if unknown:
        print("WARNING: no measured extents for %s -- a mid-size car was assumed, so every "
              "clearance involving one is wrong. Rebuild tools/actor_extents.json.\n"
              % ", ".join(sorted(unknown)))

    arms = arm_order(recs, specs)
    print("%-8s %6s %8s %8s %8s %8s %9s %9s" % ("arm", "plans", "p05", "p10", "median", "p90",
                                                f"<{near_m:.2f}m", "overlap"))
    print("-" * 70)
    for arm in arms:
        sub = [r["min_clearance_m"] for r in recs if r["arm"] == arm]
        qs = _quantiles(sub)
        near = sum(1 for x in sub if x <= near_m)
        touch = sum(1 for x in sub if x <= 0.0)
        print("%-8s %6d %8.2f %8.2f %8.2f %8.2f %6d %3.0f%% %5d %3.0f%%"
              % (arm, len(sub), qs["p05"], qs["p10"], qs["median"], qs["p90"],
                 near, 100.0 * near / len(sub), touch, 100.0 * touch / len(sub)))

    if len(arms) == 2:
        _paired(recs, arms, near_m)

    print("\nexposure: is the rate falling, or are the plans just shorter?")
    for arm in arms:
        sub = [r for r in recs if r["arm"] == arm]
        dist = sum(r.get("dist_m") or 0.0 for r in sub)
        near = sum(1 for r in sub if r["min_clearance_m"] <= near_m)
        print("  %-8s %5d plans  %7.0f m driven  %5.1f m/plan  %5.2f near misses per 100 m"
              % (arm, len(sub), dist, dist / len(sub), 100.0 * near / dist if dist else 0.0))

    print("\nnearest body, by kind (all arms)")
    kinds: Dict[str, List[float]] = {}
    for r in recs:
        kinds.setdefault((r["nearest_type"] or "?").split(".")[0], []).append(r["min_clearance_m"])
    for kind, vals in sorted(kinds.items(), key=lambda kv: -len(kv[1])):
        print("  %-10s %5d plans   median %.2f m   under %.2f m: %d"
              % (kind, len(vals), statistics.median(vals), near_m,
                 sum(1 for x in vals if x <= near_m)))


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arms", nargs="+", metavar="NAME=prefix[,prefix...]")
    ap.add_argument("--glob", default=DEFAULT_GLOB)
    ap.add_argument("--near", type=float, default=NEAR_M, metavar="M",
                    help="clearance at or below this counts as a near miss (default: %(default)s)")
    ap.add_argument("--stride", type=int, default=STRIDE, metavar="N",
                    help="sample every Nth tick of a burst (default: %(default)s)")
    ap.add_argument("--jsonl", metavar="PATH", help="write one JSON record per plan")
    args = ap.parse_args(argv)

    recs, skipped, unknown = gather(args.arms, args.glob, args.stride)
    if args.jsonl:
        with open(args.jsonl, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r) + "\n")
        print("wrote %d records to %s\n" % (len(recs), args.jsonl))
    report(recs, args.near, skipped, unknown, args.arms)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
