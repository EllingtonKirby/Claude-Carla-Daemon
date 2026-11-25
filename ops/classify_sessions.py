#!/usr/bin/env python3
"""Classify every ladder / privileged session: valid, or an infrastructure failure to requeue.

    python3 classify_sessions.py [--json]

  valid        the driver ran, was never refused by a rate limit, and the route has a result record
  cutoff       the driver was refused by a usage / rate limit (rate_limit_event status=rejected, or an
               api_error ending) -- infra, the score is not the driver's
  pre_driver   no driver ever ran: aborted at load or at the isolation check, or hung before driving
  incomplete   a driver ran but there is no result record (crash after start) -- inspect by hand
  running      a live run_eval.sh is on it -- leave it alone
Real driving failures (collisions, timeouts, a driver that gave up) are `valid`.
"""
import json
import sys
from pathlib import Path

ROOTS = [Path.home() / "workspace/ccd_base/sessions/priv_opus", Path.home() / "workspace/ccd_base/sessions/priv_fable",
         Path.home() / "workspace/ccd_exp/sessions/ladder", Path.home() / "workspace/ccd_priv2/sessions/priv_v2",
         Path.home() / "workspace/ccd_priv2/sessions/ladder_plus",
         Path.home() / "workspace/ccd_priv2/sessions/a0plus"]


def _live_sessions():
    """Session dirs a running run_eval.sh names: these are in progress, never to be quarantined."""
    import subprocess
    out = subprocess.run(["pgrep", "-af", "scripts/run_eval.sh"], capture_output=True, text=True).stdout
    return [line.split()[-1] for line in out.splitlines() if line.strip()]


LIVE = _live_sessions()


def classify(d: Path):
    if any(str(d).endswith(x.rstrip("/").split("sessions/", 1)[-1]) for x in LIVE):
        return "running", "a run_eval.sh is driving it now"
    dj = d / "driver.json"
    if not dj.exists():
        return "pre_driver", "no driver.json"
    drv = json.loads(dj.read_text())
    tr = d / "driver_transcript.jsonl"
    if "driver_exit_code" not in drv and not tr.exists():
        return "pre_driver", "driver never started"
    limited = False
    if tr.exists():
        for line in tr.open():
            if '"rate_limit_event"' in line and '"status":"rejected"' in line:
                limited = True
            if '"terminal_reason":"api_error"' in line:
                limited = True
    if limited:
        return "cutoff", "rate limit refused the driver"
    try:
        recs = json.loads((d / "result.json").read_text())["_checkpoint"]["records"]
    except Exception:
        recs = []
    if not recs:
        return "incomplete", "driver ran, no result record"
    r = recs[0]
    return "valid", f"{r['status']} DS {r['scores']['score_composed']:.1f}"


def main():
    rows = []
    for root in ROOTS:
        for d in sorted(list(root.glob("*/a*/route_*")) + list(root.glob("a*/route_*"))):
            if not d.is_dir() or d.name.endswith(".claim"):
                continue
            rel = d.relative_to(root.parent)
            kind, why = classify(d)
            rows.append({"session": str(rel), "path": str(d), "kind": kind, "why": why})
    if "--json" in sys.argv:
        print(json.dumps(rows, indent=1))
        return
    from collections import Counter
    for r in rows:
        print(f"{r['kind']:<11} {r['session']:<60} {r['why']}")
    print(Counter(r["kind"] for r in rows))


if __name__ == "__main__":
    main()
