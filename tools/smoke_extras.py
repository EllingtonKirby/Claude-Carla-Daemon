#!/usr/bin/env python3
"""Scripted live check of the privileged extras (no model): drive a route along its nav points and
print what the extras report -- actor behaviours, emergency vehicles, rules, scenarios, predict and
plan_check -- and how the logged predictions compare with where the car actually went.

    ACCESS=privileged PRIV_EXTRAS=1 PID_MAX_THROTTLE=1.0 bash scripts/run_session.sh 25378 SESSION &
    python tools/smoke_extras.py SESSION --addr 127.0.0.1:8799
"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_probe.client import ProbeClient  # noqa: E402

WHY = "scripted smoke test of the privileged extras; not a driving decision"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("session", type=Path)
    ap.add_argument("--addr", default="127.0.0.1:8790")
    ap.add_argument("--max-plans", type=int, default=40)
    a = ap.parse_args()
    c = ProbeClient(a.addr, wait_s=1200)
    seen_types, seen_beh, scen_seen = set(), set(), set()
    for k in range(a.max_plans):
        obs = c.call("percept")["observe"]
        for x in obs["actors_nearest"]:
            seen_types.add((x.get("type"), x.get("role"), x.get("emergency")))
            seen_beh.add((x.get("scenario"), x.get("behaviour"), x.get("reacts_to_you")))
        for sc in obs.get("scenarios", []):
            scen_seen.add((sc.get("type"), sc.get("started"), json.dumps(sc.get("parameters"))[:120]))
        if k in (0, 10, 20):
            print("rules:", json.dumps(obs["rules"])[:700])
            print("scenarios:", json.dumps(obs["scenarios"])[:600])
        nav = (obs.get("nav") or {}).get("route_points_ego")
        if not nav:
            break
        chk = c.call("plan_check", waypoints=nav, target_speed=6, ticks=30)
        if k % 10 == 0:
            p = chk["plan_check"]
            print(f"plan {k}: end speed {p['end_speed']} distance {p['distance_m']} band {p['max_band_m']} worst {p.get('worst_case_m')}"
                  f" top {p['clearances'][:2]}")
        r = c.call("apply_plan", waypoints=nav, target_speed=6, ticks=30, why=WHY)
        if (r.get("result") or {}).get("reason") in ("route_end", "shutdown") or not r.get("ok", False):
            break
    print("\nactor types seen:")
    for t in sorted(seen_types, key=str):
        print("  ", t)
    print("behaviours seen:")
    for b in sorted(seen_beh, key=str):
        print("  ", b)
    print("scenarios seen:", sorted(scen_seen, key=str))
    try:
        c.call("shutdown", why="smoke test finished")
    except Exception:
        pass


if __name__ == "__main__":
    main()
