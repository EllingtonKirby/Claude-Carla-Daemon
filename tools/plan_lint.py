#!/usr/bin/env python3
"""Lint the plans a driver actually sent, against what the controller can do with them.

    python tools/plan_lint.py SESSION_DIR [SESSION_DIR ...] [--json] [--plans]

Offline: reads ``commands.jsonl`` (what the driver asked for), ``trajectory.jsonl`` (where the ego
then went) and the session's ``config.yaml`` snapshot (the tuning that run actually used). No
simulator, no recording.

Why this exists: a route score says the drive was bad, not that the *plans* were bad, and the two
come apart. A driver can pick every manoeuvre correctly and still weave, because the waypoints it
drew were unusable by the controller that has to track them. This reports the geometry directly, so
a change to the driver's guidance can be measured on plans without re-running a whole benchmark.

What it checks, all of it derived from carla_probe/pid.py and agent.py rather than taste:

``reach_short``      ``_compute_control`` sets ``plan_done`` (and BRAKES for the rest of the burst)
                     once the last waypoint is behind the ego or within 1.5 m. Surviving a burst
                     therefore needs reach > ``speed * ticks * 0.05 + 1.5``. Reported with the
                     margin, negative meaning the burst outran the plan.
``stalled``          the same rule replayed against the ego's actual poses: the tick the plan ran
                     out, and how many ticks of the burst were spent braking afterwards. This is
                     ``reach_short`` confirmed rather than predicted.
``no_target_speed``  without ``target_speed`` the desired speed is the plan's mean waypoint spacing
                     over ``plan_interval_s``, so geometry and speed are coupled: drawing finer
                     waypoints silently commands a crawl. The implied speed is reported.
``aim_gap``          steering never reads the whole plan. It aims at ONE interpolated point at
                     ``clip(speed_scale*v + speed_offset, lookahead_min, lookahead_max)`` metres
                     (``adaptive``), so what matters is the resolution of the segment bracketing
                     that point -- measured from the EGO, so a distant first waypoint is itself a
                     long segment. A long bracket means the aim point is interpolated across a
                     chord and corners get cut. Meaningless under ``steer_mode: fixed``, which
                     snaps the aim to a waypoint instead; the check is skipped there.
``straight_ray``     every waypoint on ``left = 0``. The plan is pinned to the world when issued,
                     so this does not mean "stay in the lane": it means "hold my current offset and
                     heading", and any lateral error the ego had at that instant is frozen in.
``two_points``       the minimum the daemon accepts, which through a bend is a chord.

``track_rms`` is not a violation but the outcome: how far the ego was from the path it was given,
over the ticks before the plan ran out. The path starts at the ego pose the plan was pinned to, not
at the first waypoint, because that is the polyline ``pid.py`` interpolates along. A plan can be legal on every check above and still be
untrackable (too tight for the steering to follow); this is what catches that. Under ``access:
sensors`` ``trajectory.jsonl`` holds the driver's GNSS *estimate*, which is the same frame the plan
was pinned in, so this measures tracking rather than absolute position -- correctly, since that is
the loop the controller closes. Where the true path matters, use tools/recorder_truth.py.

Note ``track_rms`` and ``stalled`` are exactly the two signals an advance reply could hand the
driver at runtime. Measuring them here first says whether that feedback would be worth the
command-surface change (benchmark/PROTOCOL.md rule 2) before anything is built.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from carla_probe import geometry as geo  # noqa: E402

_TICK_S = 0.05
_PLAN_DONE_M = 1.5   # agent.py _compute_control: last waypoint nearer than this ends the plan


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue   # a session killed mid-write leaves a torn last line
    return rows


def _lookahead(pid: Dict[str, Any], speed: float) -> Optional[float]:
    """Arclength the steering aims at, or None when the mode does not have one."""
    mode = str(pid["steer_mode"]).strip().lower()
    if mode == "adaptive":
        return float(np.clip(pid["speed_scale"] * speed + pid["speed_offset"],
                             pid["lookahead_min"], pid["lookahead_max"]))
    if mode == "lead":
        # The curvature term adds up to +2 m on top; ignored, so this is the floor of the lookahead.
        return float(np.clip(pid["speed_scale"] * (speed * 3.6) + pid["speed_offset"], 24.0, 105.0)) / 10.0 - 2.0
    return None   # fixed: the aim snaps to a waypoint, so there is no bracketing segment to measure


def _aim_segment_len(wps: np.ndarray, lookahead: float) -> Tuple[float, float]:
    """Length of the plan segment bracketing ``lookahead``, and the plan's total arclength.

    Arclength runs from the EGO (pid.py _interp_aim prepends the origin), so the gap from the ego to
    the first waypoint is itself a segment the aim point can land in.
    """
    pts = np.vstack([np.zeros((1, 2)), wps])
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.cumsum(seg)
    if cum[-1] <= lookahead:
        return float(seg[-1]), float(cum[-1])
    return float(seg[int(np.searchsorted(cum, lookahead))]), float(cum[-1])


def _dist_to_polyline(poly: np.ndarray, p: Sequence[float]) -> float:
    """True Euclidean distance from a point to a polyline (segment ends included)."""
    a, b = poly[:-1], poly[1:]
    ab = b - a
    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    t = np.clip(np.sum((np.asarray(p, dtype=np.float64)[:2] - a) * ab, axis=1) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return float(np.sqrt(np.min(np.sum((proj - np.asarray(p, dtype=np.float64)[:2]) ** 2, axis=1))))


def _plan_done_at(plan_world: np.ndarray, poses: List[Dict[str, Any]]) -> Optional[int]:
    """Replay agent.py's plan_done test over the burst; the index of the first tick it trips."""
    last = plan_world[-1]
    for i, pose in enumerate(poses):
        ego = geo.world_to_ego((pose["x"], pose["y"]), pose["yaw"], last.reshape(1, 2))[0]
        if ego[0] < 0.0 or float(np.hypot(ego[0], ego[1])) < _PLAN_DONE_M:
            return i
    return None


def analyse_session(session_dir: Path, aim_gap_m: float) -> Dict[str, Any]:
    cfg_path = session_dir / "config.yaml"
    assert cfg_path.exists(), f"{session_dir}: no config.yaml snapshot (a partial copy of a session, or not a session dir)"
    cfg = yaml.safe_load(cfg_path.read_text())
    pid, plan_interval_s = cfg["pid"], float(cfg["plan_interval_s"])

    traj = {int(r["step"]): r for r in _load_jsonl(session_dir / "trajectory.jsonl")}
    cmds = _load_jsonl(session_dir / "commands.jsonl")
    plans: List[Dict[str, Any]] = []

    for row in cmds:
        if row.get("cmd") != "apply_plan" or row.get("ok") is False:
            continue
        step = int(row["step"])
        pose = traj.get(step)
        if pose is None:
            continue   # the session ended inside this burst; nothing to measure it against
        wps = np.asarray(row["waypoints"], dtype=np.float64).reshape(-1, 2)
        ticks = int(row["ticks"])
        speed = float(pose["speed"])
        target_speed = row.get("target_speed")

        need = speed * ticks * _TICK_S + _PLAN_DONE_M
        reach = float(wps[-1][0])
        spacing = float(np.linalg.norm(np.diff(wps, axis=0), axis=1).sum())
        implied_speed = spacing / (plan_interval_s * max(len(wps) - 1, 1))

        lookahead = _lookahead(pid, speed)
        gap, arclen = _aim_segment_len(wps, lookahead if lookahead is not None else 0.0)

        plan_world = geo.ego_to_world((pose["x"], pose["y"]), pose["yaw"], wps)
        # The path the controller interpolates starts at the EGO, not at the first waypoint
        # (pid.py _interp_aim prepends the origin), so tracking is measured against that.
        path_world = np.vstack([[[pose["x"], pose["y"]]], plan_world])
        poses = [traj[s] for s in range(step, step + ticks) if s in traj]
        done_at = _plan_done_at(plan_world, poses)
        tracked = poses if done_at is None else poses[:done_at]
        dists = [_dist_to_polyline(path_world, (p["x"], p["y"])) for p in tracked]

        plans.append({
            "step": step, "n_wps": len(wps), "ticks": ticks, "speed": round(speed, 2),
            "target_speed": target_speed, "reach_m": round(reach, 1), "need_m": round(need, 1),
            "margin_m": round(reach - need, 1), "arclen_m": round(arclen, 1),
            "aim_m": None if lookahead is None else round(lookahead, 1), "aim_gap_m": round(gap, 1),
            "implied_speed": round(implied_speed, 2),
            "max_left_m": round(float(np.max(np.abs(wps[:, 1]))), 2),
            "done_at_tick": done_at,
            "braking_ticks": 0 if done_at is None else len(poses) - done_at,
            "track_rms_m": round(float(np.sqrt(np.mean(np.square(dists)))), 3) if dists else None,
            "track_max_m": round(float(np.max(dists)), 3) if dists else None,
            "flags": [],
        })
        p = plans[-1]
        if p["margin_m"] < 0.0:
            p["flags"].append("reach_short")
        if target_speed is None:
            p["flags"].append("no_target_speed")
        if lookahead is not None and gap > aim_gap_m:
            p["flags"].append("aim_gap")
        if p["max_left_m"] <= 0.1:
            p["flags"].append("straight_ray")
        if len(wps) <= 2:
            p["flags"].append("two_points")
        if done_at is not None:
            p["flags"].append("stalled")

    meta = {}
    for name in ("session.json", "driver.json"):
        path = session_dir / name
        if path.exists():
            meta.update(json.loads(path.read_text()))
    return {
        "session": str(session_dir),
        "benchmark": meta.get("benchmark"), "route": meta.get("route"),
        "track": meta.get("track", cfg.get("access")), "model": meta.get("model"),
        "steer_mode": pid["steer_mode"], "n_plans": len(plans), "plans": plans,
    }


def _pct(n: int, total: int) -> str:
    return f"{n:4d} ({100.0 * n / total:4.0f}%)" if total else "   0"


def _median(xs: Sequence[float]) -> Optional[float]:
    return float(np.median(xs)) if len(xs) else None


def _fmt(x: Optional[float], unit: str = " m", nd: int = 2) -> str:
    return "  --" if x is None else f"{x:.{nd}f}{unit}"


def report(res: Dict[str, Any]) -> List[str]:
    plans, n = res["plans"], res["n_plans"]
    head = f"{res['session']}  [{res.get('benchmark') or '?'} route {res.get('route') or '?'}, " \
           f"{res.get('track') or '?'}, steer_mode={res['steer_mode']}]"
    if not n:
        return [head, "  no apply_plan commands"]
    flag_of = lambda f: [p for p in plans if f in p["flags"]]  # noqa: E731
    burst_ticks = sum(p["ticks"] for p in plans)
    braking = sum(p["braking_ticks"] for p in plans)
    rms = [p["track_rms_m"] for p in plans if p["track_rms_m"] is not None]
    lines = [head, f"  plans {n}   burst ticks {burst_ticks}   waypoints/plan "
                   f"{_fmt(_median([p['n_wps'] for p in plans]), '', 1)}"]
    short = flag_of("reach_short")
    lines.append(f"  reach_short     {_pct(len(short), n)}   median margin "
                 f"{_fmt(_median([p['margin_m'] for p in short]), ' m', 1)}")
    stalled = flag_of("stalled")
    lines.append(f"  stalled         {_pct(len(stalled), n)}   {braking} ticks braking after "
                 f"plan_done ({100.0 * braking / max(burst_ticks, 1):.0f}% of burst ticks)")
    nots = flag_of("no_target_speed")
    lines.append(f"  no_target_speed {_pct(len(nots), n)}   median implied speed "
                 f"{_fmt(_median([p['implied_speed'] for p in nots]), ' m/s', 1)}")
    if res["steer_mode"] != "fixed":
        gaps = flag_of("aim_gap")
        lines.append(f"  aim_gap         {_pct(len(gaps), n)}   median bracket "
                     f"{_fmt(_median([p['aim_gap_m'] for p in plans]), ' m', 1)} vs aim "
                     f"{_fmt(_median([p['aim_m'] for p in plans if p['aim_m'] is not None]), ' m', 1)}")
    lines.append(f"  straight_ray    {_pct(len(flag_of('straight_ray')), n)}")
    lines.append(f"  two_points      {_pct(len(flag_of('two_points')), n)}")
    lines.append(f"  track_rms       median {_fmt(_median(rms))}   worst "
                 f"{_fmt(max((p['track_max_m'] for p in plans if p['track_max_m'] is not None), default=None))}")
    return lines


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--aim-gap", type=float, default=4.0,
                    help="flag plans whose aim point is bracketed by a segment longer than this (m)")
    ap.add_argument("--plans", action="store_true", help="one line per plan, not just the summary")
    ap.add_argument("--json", action="store_true", help="machine-readable, everything included")
    args = ap.parse_args(argv)

    results = [analyse_session(s, args.aim_gap) for s in args.sessions]
    if args.json:
        print(json.dumps(results, indent=1))
        return 0
    for res in results:
        print("\n".join(report(res)))
        if args.plans:
            for p in res["plans"]:
                print(f"    step {p['step']:5d}  v {p['speed']:5.2f}  ticks {p['ticks']:3d}  "
                      f"wps {p['n_wps']:2d}  reach {p['reach_m']:5.1f} (need {p['need_m']:5.1f})  "
                      f"rms {_fmt(p['track_rms_m'])}  {','.join(p['flags']) or '-'}")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
