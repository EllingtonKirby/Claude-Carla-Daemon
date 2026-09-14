---
name: drive-carla
description: Drive a CARLA Bench2Drive or Fail2Drive route through the probe daemon, on the sensors-only (SENSORS-track-eligible) or privileged track, either as a benchmark run or for a manual session. Use when asked to drive a route, run the benchmark, or take control of a probe session.
---

# Drive a CARLA route

You are the driver. There is no autopilot: the ego brakes (`hold`) until you command it.

## 1. Launch

1. Pick the benchmark and the track.
   - `BENCHMARK=bench2drive` (default) or `BENCHMARK=fail2drive`. Fail2Drive is a *generalization*
     benchmark scored in `Base_*` / `Generalization_*` route pairs; it needs its own CARLA build
     and its routes are named, not numbered. Ask if the request does not say which.
   - `sensors` (cameras + ego sensors + route plan) unless the user asks for `privileged`
     (ground-truth scene). If the request is ambiguous, ask.
2. `source scripts/env.sh` (fails with a clear message if setup is missing; see
   `docs/HANDOFF.md` §2). Each Bash call is a fresh shell: re-source it, or call
   `.venv/bin/python -m carla_probe.client` directly.
3. Start the session in the background (it blocks until the route ends):
   `BENCHMARK=<benchmark> ACCESS=<track> bash scripts/run_session.sh <ROUTE>`. On Bench2Drive the
   standard benchmark is route 2091 and the smoke route is 25378; on Fail2Drive routes are named
   (`Base_Animals_0075`) or given by id (`75`), and `benchmark/routes/f2d_*.txt` lists the sets.
   Note the session dir it prints.
4. `python -m carla_probe.client --wait 600 status` returns once the route has loaded. Check that
   `status.access` is the track you intended. If launch fails, read `SESSION_DIR/evaluator.log`
   and `carla.log` (allowed before the first driving command).
   **Every Fail2Drive route is Town13, a large map: the load takes several minutes.** Use
   `--wait 900` and do not treat a quiet evaluator as a hang.

## 2. Drive loop

Read `docs/OPERATOR_GUIDE.md` §3–8 once before the first command (§7 is the sensors-track
technique). Then repeat:

1. **Look.**
   - *Sensors:* `observe` (speed, route points, next turn) plus `get_camera` (read the JPGs:
     `CAM_FRONT` always, and side/back cameras at junctions, merges and lane changes).
   - *Privileged:* `observe` (lane, route, lights, signs, nearest actors), plus `render_bev` when the
     geometry is unclear.
2. **Decide the next 1–2 s.** Build `apply_plan` waypoints (ego frame: `fwd` ahead, `left` to the
   LEFT) from the route points and the lane. Set `target_speed` to a safe speed, and 0 to stop.
3. **Advance.** Use `apply_plan ... ticks=<5-40> why="..."`: short near other road users, longer
   on empty road. Privileged only: add `stop_on_collision=true`. Every driving command needs `why`:
   what you observed and why this command follows from it (it is logged for review).
4. **Check the reply** (`result.reason`, status; on privileged also `new_infractions` and
   `status.score`) and adapt.

Stop when an advance returns `route_end` or the connection closes.

## 3. Rules for benchmark runs

Follow `benchmark/PROTOCOL.md`:
- probe protocol only, with no reading of route XMLs or scenario code;
- no config or code edits;
- one attempt;
- `why="..."` on every driving command, stating the reason you actually acted on;
- no `shutdown` unless the session is unrecoverable.

**On the sensors track, open only files whose paths the daemon returns.** Never read
`evaluator.log`, `carla.log`, `result.json` or other session files while driving.

Thinking time is free; ticks are not. On Bench2Drive the budget is 4000 ticks and 60 s standing
still outside a scenario fails the route; on Fail2Drive there is no tick cap, the route ends on its
own timeout, and the standing-still limit is 180 s. Do not rely on either: drive as if the tighter
one applied.

Fail2Drive's routes are short (median ~210 m) and its sensors-track route plan is coarse -- often
little more than a start and an end point. That is correct, not a fault: lean on the cameras for
the road shape, and on the route points only for where you are going.

## 4. Finish

`python tools/summarize_results.py <SESSION_DIR>`. Report the benchmark, the track, DS / RC / IS,
status, infractions, commands and wall time, plus a short account of what went well and badly. On
Fail2Drive, one route is never a result: the headline number is the Base -> Generalization drop
over many pairs (`python tools/f2d_score.py <SESSION_DIRS>`), so say what the single route showed
and do not present it as a Fail2Drive score. If a CARLA
process is left behind, run `bash scripts/clean_carla.sh <CARLA_PORT>`.
