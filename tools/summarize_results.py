#!/usr/bin/env python3
"""Summarize probe sessions into a benchmark table.

    python tools/summarize_results.py sessions/*            # markdown table
    python tools/summarize_results.py sessions/* --json     # machine-readable

Per session directory it reads:
  result.json     leaderboard scoring: DS (score_composed), RC (score_route), IS (score_penalty),
                  status and infractions. Written by the evaluator when the route ends.
  session.json    launch metadata, including the benchmark (bench2drive | fail2drive) and the
                  access level (privileged | sensors).
  commands.jsonl  the daemon's command log: how many decisions the driver made and wall-clock time.

Sessions without a scored record (killed or crashed before scoring) are reported with DS 0 and
status "no result", so they count against the mean instead of silently disappearing. Means are
reported per benchmark and access level, never pooled across either: privileged and sensors-only
are different benchmarks, and so are Bench2Drive and Fail2Drive (different routes, route budgets
and success rules).

A Fail2Drive run is scored in Base/Generalization pairs, and the mean DS over a mixed bag of its
routes is not its headline number -- use tools/f2d_score.py for that. Stdlib only.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

# Infractions that change the driving score. min_speed_infractions are reported separately: they
# carry a percentage penalty but are logged many times per route.
_SCORED = (
    "collisions_pedestrian",
    "collisions_vehicle",
    "collisions_layout",
    "red_light",
    "stop_infraction",
    "outside_route_lanes",
    "yield_emergency_vehicle_infractions",
    "scenario_timeouts",
    "route_dev",
    "vehicle_blocked",
    "route_timeout",
)


def _command_stats(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {"commands": None, "advancing_commands": None, "wall_s": None}
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    cmds = [r for r in rows if "cmd" in r]
    t = [r["t_wall"] for r in rows if "t_wall" in r]
    return {
        "commands": len(cmds),
        "advancing_commands": sum(1 for r in cmds if r.get("advancing")),
        "wall_s": round(max(t) - min(t), 1) if len(t) >= 2 else 0.0,
    }


def _meta(session: Path) -> Dict[str, str]:
    meta = session / "session.json"
    if meta.is_file():
        try:
            d = json.loads(meta.read_text())
            return {"benchmark": str(d.get("benchmark") or "bench2drive"),
                    "access": str(d.get("access") or "unknown")}
        except json.JSONDecodeError:
            pass
    # Sessions launched before the benchmark selector existed are all Bench2Drive.
    return {"benchmark": "bench2drive", "access": "unknown"}


def summarize(session: Path) -> List[Dict[str, Any]]:
    base = {"session": session.name, **_meta(session), **_command_stats(session / "commands.jsonl")}
    result = session / "result.json"
    records = []
    if result.is_file():
        records = json.loads(result.read_text()).get("_checkpoint", {}).get("records", [])
    if not records:
        return [{**base, "route_id": None, "town": None, "scenario": None, "status": "no result",
                 "DS": 0.0, "RC": 0.0, "IS": None, "infractions": {}, "min_speed_infractions": 0,
                 "duration_game_s": None}]
    rows = []
    for rec in records:
        inf = rec.get("infractions", {})
        rows.append({
            **base,
            "route_id": rec.get("route_id"),
            "town": rec.get("town_name"),
            "scenario": rec.get("scenario_name"),
            "status": rec.get("status"),
            "DS": float(rec["scores"]["score_composed"]),
            "RC": float(rec["scores"]["score_route"]),
            "IS": float(rec["scores"]["score_penalty"]),
            "infractions": {k: len(inf.get(k, [])) for k in _SCORED if inf.get(k)},
            "min_speed_infractions": len(inf.get("min_speed_infractions", [])),
            "duration_game_s": rec.get("meta", {}).get("duration_game"),
        })
    return rows


def _means(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    n = len(rows)
    scored_is = [r["IS"] for r in rows if r["IS"] is not None]
    return {
        "n": n,
        "DS": round(sum(r["DS"] for r in rows) / n, 2),
        "RC": round(sum(r["RC"] for r in rows) / n, 2),
        "IS": round(sum(scored_is) / len(scored_is), 3) if scored_is else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rows = [row for s in args.sessions if s.is_dir() for row in summarize(s)]
    if not rows:
        print("no session directories found", file=sys.stderr)
        return 1
    groups = sorted({(r["benchmark"], r["access"]) for r in rows})
    means = {f"{b}/{a}": _means([r for r in rows if (r["benchmark"], r["access"]) == (b, a)])
             for b, a in groups}
    if args.json:
        print(json.dumps({"routes": rows, "mean_by_benchmark_access": means}, indent=2))
        return 0

    print("| session | benchmark | access | route | town | status | DS | RC | IS | infractions | cmds | wall s |")
    print("|---|---|---|---|---|---|---:|---:|---:|---|---:|---:|")
    for r in rows:
        inf = ", ".join(f"{k}={v}" for k, v in r["infractions"].items()) or "-"
        is_str = "-" if r["IS"] is None else f"{r['IS']:.3f}"
        cmds = "-" if r["commands"] is None else r["commands"]
        wall = "-" if r["wall_s"] is None else r["wall_s"]
        print(f"| {r['session']} | {r['benchmark']} | {r['access']} | {r['route_id'] or '-'} | {r['town'] or '-'} | "
              f"{r['status']} | {r['DS']:.2f} | {r['RC']:.2f} | {is_str} | {inf} | {cmds} | {wall} |")
    print()
    for key, m in means.items():
        print(f"**{key}: mean over {m['n']} route(s):** DS {m['DS']} · RC {m['RC']} · IS {m['IS']}")
    if any(b == "fail2drive" for b, _ in groups):
        print("\nFail2Drive is scored in Base/Generalization pairs: "
              "`python tools/f2d_score.py <sessions>` for the drop.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
