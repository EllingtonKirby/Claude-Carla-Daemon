#!/usr/bin/env python3
"""Drivers with no language model in the loop: the reference points an LLM driver is measured against.

    python -m carla_probe.baseline_driver <ADDR> --policy vla|route [--ticks 40] [--speed 5.0]

    vla    the learned policy alone: `drive mode=plain` (no text) until the route ends. What
           SimLingo scores on this route with nothing said to it -- the number a language-supervised
           run must beat to have shown the language helped.
    route  the null driver: the sparse nav points handed straight to `apply_plan` at a constant
           speed, blind to traffic, lights and the scenario. Any driver scoring below this on a
           route did worse than not thinking at all.

Both send every driving command with a `why`, so decisions.jsonl records what they are. Launched
by scripts/run_eval.sh with DRIVER=baseline_vla | baseline_route, or by hand against a session
started with scripts/run_session.sh.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_probe.client import ProbeClient  # noqa: E402

WHY = {
    "vla": "baseline: the learned policy drives alone, told nothing",
    "route": "baseline: follow the sparse nav points at a constant speed, blind to the scene",
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("addr")
    ap.add_argument("--policy", choices=sorted(WHY), required=True)
    ap.add_argument("--ticks", type=int, default=40, help="ticks per command (default %(default)s)")
    ap.add_argument("--speed", type=float, default=5.0, help="route policy: target speed, m/s")
    ap.add_argument("--wait", type=float, default=900.0)
    args = ap.parse_args()

    probe = ProbeClient(args.addr, wait_s=args.wait)
    status = probe.call("status")
    assert status.get("ok"), status
    n = 0
    while True:
        if args.policy == "vla":
            reply = probe.call("drive", mode="plain", ticks=args.ticks, why=WHY["vla"])
        else:
            nav = (probe.call("percept")["observe"].get("nav") or {}).get("route_points_ego")
            if not nav or len(nav) < 2:
                print("no route points left")
                break
            reply = probe.call("apply_plan", waypoints=nav, ticks=args.ticks, target_speed=args.speed, why=WHY["route"])
        n += 1
        if not reply.get("ok"):
            print(f"command refused: {reply.get('error')}")
            return 1
        res = reply.get("result") or {}
        st = reply.get("status") or {}
        if n % 10 == 0:
            print(f"cmd {n:>4}  step {st.get('step'):>5}  v={st.get('speed_mps')}  "
                  f"progress={(st.get('route') or {}).get('progress_pct')}%  {res.get('vla') or ''}", flush=True)
        if res.get("reason") in ("route_end", "shutdown"):
            print(f"done: {res.get('reason')} after {n} commands")
            break
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
