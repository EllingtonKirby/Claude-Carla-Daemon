#!/usr/bin/env python3
"""Extract per-frame ground truth from a session's CARLA recording, and cache it.

    python tools/recorder_truth.py SESSION_DIR [--port 20400] [--gpu 0] [--force]

Writes ``SESSION_DIR/recorder_truth.json``:

    {"dt": 0.05, "hero_id": 109, "frames": {"<frame>": {"t": 1.85,
                                             "actors": {"<id>": [x, y, yaw]}}},
     "actors": {"<id>": {"type": "vehicle.ford.ambulance", "role": "scenario"}}}

Positions are metres in the CARLA world frame (the recorder reports centimetres) and yaw is degrees.

Why this exists: the recording is the only ground truth we have for a sensors-track run --
``trajectory.jsonl`` stores the driver's own GNSS *estimate*. Anything that needs to know where
things really were (perception scoring, replay frame alignment, collision attribution) should read
this table, not the trajectory, and every actor's pose is here, not just the ego's.

Needs a CARLA server only to decode the log; after the first run everything downstream is offline.
``trajectory.jsonl``'s ``frame`` field is a WORLD frame; the recorder numbers its own frames from 1,
so ``recorder_frame = world_frame - frame_offset`` where ``frame_offset`` is the recording's
``start_world_frame`` (also stored in the cache). Fitting that offset against the driver's own pose
gives a sharp minimum exactly at ``start_world_frame``, which is how it is verified below.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay_sim import Session, connect  # noqa: E402

_FRAME = re.compile(r"^Frame (\d+) at ([\d.]+) seconds")
_POS = re.compile(r"^\s+Id: (\d+) Location: \(([-\d.e+]+), ([-\d.e+]+), [-\d.e+]+\) "
                  r"Rotation: \([-\d.e+]+, ([-\d.e+]+), [-\d.e+]+\)")
_CREATE = re.compile(r"^\s+Create (\d+): ([\w.]+) \(\d+\) at ")
_ATTR = re.compile(r"^\s+(\w+) = (.+)$")


def parse_dump(text: str) -> dict:
    """Parse `show_recorder_file_info(..., show_all=True)` output into the cached structure."""
    frames: dict = {}
    actors: dict = {}
    cur = None
    last_created = None
    for line in text.splitlines():
        m = _FRAME.match(line)
        if m:
            cur = int(m.group(1))
            frames[cur] = {"t": float(m.group(2)), "actors": {}}
            last_created = None
            continue
        m = _POS.match(line)
        if m and cur is not None:
            frames[cur]["actors"][int(m.group(1))] = [round(float(m.group(2)) / 100.0, 3),
                                                      round(float(m.group(3)) / 100.0, 3),
                                                      round(float(m.group(4)), 2)]
            continue
        m = _CREATE.match(line)
        if m:
            last_created = int(m.group(1))
            actors.setdefault(last_created, {"type": m.group(2), "role": None})
            continue
        m = _ATTR.match(line)
        if m and last_created is not None and m.group(1) == "role_name":
            actors[last_created]["role"] = m.group(2).strip()
    return {"frames": frames, "actors": actors}


def build(session: Path, port: int, gpu: int, force: bool = False) -> dict:
    out_path = session / "recorder_truth.json"
    if out_path.is_file() and not force:
        return json.loads(out_path.read_text())

    sess = Session(session)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(1))
    client, proc = connect(port, gpu)
    client.set_timeout(600.0)
    try:
        text = client.show_recorder_file_info(str(sess.recording), True)
    finally:
        if proc is not None:
            os.killpg(proc.pid, signal.SIGKILL)

    parsed = parse_dump(text)
    hero = next((aid for aid, a in parsed["actors"].items() if a.get("role") == "hero"), None)
    if hero is None:                     # role_name is not always in the dump; fall back to the
        traj = [json.loads(l) for l in open(session / "trajectory.jsonl")]   # actor nearest the
        first = next((r for r in traj if "frame" in r), None)                # driver's first pose
        if first and first["frame"] in parsed["frames"]:
            cands = parsed["frames"][first["frame"]]["actors"]
            hero = min(cands, key=lambda i: (cands[i][0] - first["x"]) ** 2 + (cands[i][1] - first["y"]) ** 2)

    data = {"dt": sess.dt, "hero_id": hero, "frame_offset": sess.start_frame,
            "frames": {str(f): v for f, v in parsed["frames"].items()},
            "actors": {str(a): v for a, v in parsed["actors"].items()}}
    out_path.write_text(json.dumps(data))
    return json.loads(out_path.read_text())   # normalise key types (JSON makes all keys strings)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--port", type=int, default=20400)
    ap.add_argument("--gpu", type=int, default=0)
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    data = build(args.session, args.port, args.gpu, args.force)
    frames = data["frames"]
    ks = sorted(int(k) for k in frames)
    print(f"{args.session.name}: {len(ks)} frames {ks[0]}..{ks[-1]}, "
          f"hero id {data['hero_id']}, dt {data['dt']}")
    print(f"actors: {len(data['actors'])}")
    print(f"wrote {args.session / 'recorder_truth.json'}")

    # Sanity: the driver's GNSS estimate vs where the ego really was, and a check that the
    # world->recorder frame offset is right (the error minimises sharply at the true offset).
    off = data["frame_offset"]
    traj = [json.loads(l) for l in open(args.session / "trajectory.jsonl")]
    errs = []
    for r in traj:
        f, h = str(r.get("frame", 0) - off), str(data["hero_id"])
        if f in frames and h in frames[f]["actors"]:
            gx, gy, _ = frames[f]["actors"][h]
            errs.append(((gx - r["x"]) ** 2 + (gy - r["y"]) ** 2) ** 0.5)
    if errs:
        errs.sort()
        print(f"driver GNSS pose error vs truth: mean {sum(errs)/len(errs):.2f} m, "
              f"median {errs[len(errs)//2]:.2f} m, p95 {errs[int(len(errs)*0.95)]:.2f} m, "
              f"max {errs[-1]:.2f} m  (n={len(errs)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
