---
name: drive-carla
description: Drive a CARLA Bench2Drive route through the probe daemon, on the sensors-only (Bench2Drive-eligible) or privileged track, either as a benchmark run or for a manual session. Use when asked to drive a route, run the benchmark, or take control of a probe session.
---

# Drive a CARLA route

You are the driver. There is no autopilot: the ego brakes (`hold`) until you command it.

## 1. Launch

1. Pick the track. Use `sensors` (cameras + ego sensors + route plan, the Bench2Drive-eligible
   inputs) unless the user asks for `privileged` (ground-truth scene). If the request is ambiguous,
   ask.
2. `source scripts/env.sh` (fails with a clear message if setup is missing; see
   `docs/HANDOFF.md` §2). Each Bash call is a fresh shell: re-source it, or call
   `.venv/bin/python -m carla_probe.client` directly.
3. Start the session in the background (it blocks until the route ends):
   `ACCESS=<track> bash scripts/run_session.sh <ROUTE_ID>`. The standard benchmark is route 2091
   and the smoke route is 25378. Note the session dir it prints.
4. `python -m carla_probe.client --wait 600 status` returns once the route has loaded. Check that
   `status.access` is the track you intended. If launch fails, read `SESSION_DIR/evaluator.log`
   and `carla.log` (allowed before the first driving command).

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

Thinking time is free; ticks are not (4000-tick budget, and 60 s standing still outside a
scenario fails the route).

## 4. Finish

`python scripts/summarize_results.py <SESSION_DIR>`. Report the track, DS / RC / IS, status,
infractions, commands and wall time, plus a short account of what went well and badly. If a CARLA
process is left behind, run `bash scripts/clean_carla.sh <CARLA_PORT>`.
