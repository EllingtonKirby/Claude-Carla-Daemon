#!/usr/bin/env python3
"""Measure the step<->frame alignment error of a session replay.

    python tools/diagnose_alignment.py SESSION_DIR [--from-step 0] [--to-step 400]
                                         [--port 20400] [--gpu 0]

``replay_sim.py`` labels each rendered frame with the agent step it believes that frame belongs to,
and overlays the decision in force at that step. This script checks that belief: it replays the
recording exactly the way ``replay_sim.py`` does, reads the replayed hero's TRUE speed on every
tick, and cross-correlates that series against ``trajectory.jsonl``'s speed for the step the
renderer would have labelled. The lag that maximises correlation is the renderer's alignment error,
in ticks (1 tick = fixed_delta_seconds, normally 0.05 s).

Speed is the right probe: it comes from the SPEED pseudosensor (true ego speed), not from the noisy
GNSS pose, and it varies enough along a route to localise a lag.

Reports the best lag, the correlation at zero lag, and how much the overlay text would be out by.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
from pathlib import Path

import carla
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from replay_sim import Session, connect  # noqa: E402


def replayed_speeds(client: carla.Client, sess: Session, first: int, last: int) -> dict:
    """{rec_frame_label: true hero (x, y)} using replay_sim.py's own frame bookkeeping."""
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = sess.dt
    world.apply_settings(settings)

    warmup = 2
    start_frame = max(sess.start_frame + 1, first - warmup)
    client.replay_file(str(sess.recording), sess.time_of_frame(start_frame), 0.0, 0, False)
    world.apply_settings(settings)   # replay_file() disables synchronous mode
    weather, _ = sess.weather()
    if weather is not None:
        world.tick()          # the replayer (re)loads the map on the first tick
        start_frame += 1
        world.set_weather(weather)

    out, hero, rec_frame = {}, None, start_frame
    try:
        while rec_frame <= last:
            world.tick()
            rec_frame += 1   # the tick just advanced the world to this frame
            if hero is None:
                found = [a for a in world.get_actors().filter("vehicle.*")
                         if a.attributes.get("role_name") == "hero"]
                hero = found[0] if found else None
            elif rec_frame >= first:
                # The replayer sets transforms directly and never populates velocity, so
                # get_velocity() is identically zero here: derive speed from displacement.
                loc = hero.get_transform().location
                out[rec_frame] = (float(loc.x), float(loc.y))
    finally:
        client.stop_replayer(False)
        settings.synchronous_mode = False
        world.apply_settings(settings)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--from-step", type=int, default=0)
    ap.add_argument("--to-step", type=int, default=400)
    ap.add_argument("--max-lag", type=int, default=40, help="ticks to search either way")
    ap.add_argument("--port", type=int, default=20400)
    ap.add_argument("--gpu", type=int, default=0)
    ap.add_argument("--dump", type=Path, default=None,
                    help="write step,replay_x,replay_y,traj_x,traj_y CSV for offline analysis")
    args = ap.parse_args()

    sess = Session(args.session)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(1))
    client, proc = connect(args.port, args.gpu)
    client.set_timeout(300.0)
    try:
        first = sess.step_to_frame[args.from_step]
        last = sess.step_to_frame[min(args.to_step, max(sess.step_to_frame))]
        replay = replayed_speeds(client, sess, first, last)
    finally:
        if proc is not None:
            import os
            os.killpg(proc.pid, signal.SIGKILL)

    # What the renderer believes: frame F belongs to step frame_to_step[F].
    # What the agent recorded:     step S was at trajectory position traj_xy[S].
    # The replayer sets true transforms, so comparing positions localises a lag far more sharply
    # than speed does (metres travelled per tick >> the ~0.5 m GNSS noise in trajectory.jsonl).
    # Reference is the recording itself (recorder_truth.json), not trajectory.jsonl: the latter is
    # the driver's GNSS estimate. recorder_frame = world_frame - frame_offset.
    gt = json.loads((sess.dir / "recorder_truth.json").read_text())
    off, hero_id = gt["frame_offset"], str(gt["hero_id"])
    traj_xy = {}
    for step, wframe in sess.step_to_frame.items():
        cell = gt["frames"].get(str(wframe - off))
        if cell and hero_id in cell["actors"]:
            x, y, _ = cell["actors"][hero_id]
            traj_xy[int(step)] = (x, y)

    labelled = {}
    for f, xy in replay.items():
        s_ = sess.frame_to_step.get(f)
        if s_ is not None:
            labelled[s_] = xy

    if args.dump:
        with open(args.dump, "w") as fh:
            fh.write("step,replay_x,replay_y,traj_x,traj_y\n")
            for s_ in sorted(set(labelled) & set(traj_xy)):
                fh.write(f"{s_},{labelled[s_][0]:.3f},{labelled[s_][1]:.3f},"
                         f"{traj_xy[s_][0]:.3f},{traj_xy[s_][1]:.3f}\n")
        print(f"wrote {args.dump}")

    steps = sorted(set(labelled) & set(traj_xy))
    if len(steps) < 50:
        sys.exit(f"only {len(steps)} overlapping steps; widen the range")
    lo, hi = steps[0], steps[-1]

    def mean_err(lag: int) -> float:
        """Mean distance between the replayed pose labelled step S and the agent's pose at S+lag."""
        errs = [float(np.hypot(labelled[s][0] - traj_xy[s + lag][0],
                               labelled[s][1] - traj_xy[s + lag][1]))
                for s in steps if lo <= s + lag <= hi]
        return float(np.mean(errs)) if len(errs) >= 50 else float("nan")

    scores = {lag: mean_err(lag) for lag in range(-args.max_lag, args.max_lag + 1)}
    scores = {k: v for k, v in scores.items() if not np.isnan(v)}
    best = min(scores, key=scores.get)

    print(f"session: {sess.dir.name}")
    print(f"steps compared: {len(steps)} ({lo}..{hi}), dt={sess.dt}s")
    print(f"mean pose error at lag 0 : {scores.get(0, float('nan')):.2f} m")
    print(f"best lag                 : {best:+d} ticks ({best * sess.dt:+.2f} s), "
          f"mean error {scores[best]:.2f} m")
    print()
    for lag in sorted(scores):
        if abs(lag) <= 6 or lag == best:
            mark = "  <-- best" if lag == best else ""
            print(f"  lag {lag:+3d} ({lag * sess.dt:+5.2f}s): {scores[lag]:7.2f} m{mark}")
    print()
    if best == 0:
        print("ALIGNED: the frame labelled step S really is the world at step S.")
    else:
        print(f"MISALIGNED by {best:+d} ticks: the frame labelled step S shows the world at "
              f"step {best:+d} relative to it.")
        print("The overlay's decision/why text is therefore drawn on the wrong frames by that much.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
