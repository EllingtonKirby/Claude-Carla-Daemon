---
name: drive-carla
description: Drive a CARLA Bench2Drive or Fail2Drive route through the probe daemon, on the sensors-only (SENSORS-track-eligible) or privileged track, either as a benchmark run or for a manual session. Use when asked to drive a route, run the benchmark, or take control of a probe session.
---

# Drive a CARLA route

You are the driver. There is no autopilot: the ego brakes (`hold`) until you command it.

## 1. Launch

1. Pick the benchmark and the track.
   - `BENCHMARK=bench2drive` (default) or `BENCHMARK=fail2drive`. Ask if the request does not say
     which. Fail2Drive needs its own CARLA build and is run and reported differently:
     `benchmark/FAIL2DRIVE.md` has the details, and is operator-only -- do not open it while
     driving a Fail2Drive route.
   - `sensors` (cameras + ego sensors + route plan) unless the user asks for `privileged`
     (ground-truth scene). If the request is ambiguous, ask.
2. `source scripts/env.sh` (fails with a clear message if setup is missing; see
   `docs/HANDOFF.md` §2). Each Bash call is a fresh shell: re-source it, or call
   `.venv/bin/python -m carla_probe.client` directly.
3. Start the session in the background (it blocks until the route ends):
   `BENCHMARK=<benchmark> ACCESS=<track> bash scripts/run_session.sh <ROUTE>`. On Bench2Drive the
   standard benchmark is route 2091 and the smoke route is 25378; Fail2Drive routes are named, and
   `benchmark/FAIL2DRIVE.md` lists the sets. Note the session dir it prints.
4. `python -m carla_probe.client --wait 600 status` returns once the route has loaded. Check that
   `status.access` is the track you intended. If launch fails, read `SESSION_DIR/evaluator.log`
   and `carla.log` (allowed before the first driving command).
   **A large map takes several minutes to load.** Use `--wait 900` when the route is on one, and
   do not treat a quiet evaluator as a hang.

## 2. Drive loop

Read `docs/OPERATOR_GUIDE.md` §3–10 once before the first command (§6 is this loop, §7 the plan
geometry, §8 the sensors-track technique). Then repeat it, **every decision, for the whole route** —
a route is tens of these cycles, not one:

1. **Look.**
   - *Sensors:* `percept` — speed, route points, next turn, and a JPG path from every camera. Open
     `CAM_FRONT` always, and the side/back cameras at junctions, merges, lane changes and whenever
     something may be coming up behind you.
   - *Privileged:* `percept` (lane, route, lights, signs, nearest actors, plus the camera JPGs),
     and `render_bev` when the geometry is unclear.
2. **Decide the next 1–2 s.** Build `apply_plan` waypoints (ego frame: `fwd` ahead, `left` to the
   LEFT) from the route points and the lane — spaced 2–3 m apart, reaching about 10 m past where
   the burst will end, because the controller steers by a point only a few metres ahead and ignores
   everything past it (§7). Set `target_speed` to a safe speed, and 0 to stop.
3. **Advance and look in one command.** `apply_plan ... ticks=<5-40> percept=true why="..."` returns
   the next look inside the reply that reports the outcome, so a decision costs one round trip and
   you are never planning from a stale picture. Short bursts near other road users, longer on empty
   road. Privileged only: add `stop_on_collision=true`. Every driving command needs `why`: what you
   observed and why this command follows from it (it is logged for review).
4. **Check the reply** (`result.reason`, `status.plan_done`; on privileged also `new_infractions`
   and `status.score`), open the images it names, and go back to step 2.

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
scenario fails the route). `status.ticks_left` counts the budget down; where it is `null` the
benchmark has no tick cap and the route ends on its own timeout. Do not lean on that: drive as if
the 4000-tick budget applied either way.

On the sensors track the route plan can be coarse -- sometimes little more than a start and an end
point. That is the benchmark, not a fault: lean on the cameras for the road shape, and on the route
points only for where you are going.

## 4. Finish

`python tools/summarize_results.py <SESSION_DIR>`. Report the benchmark, the track, DS / RC / IS,
status, infractions, commands and wall time, plus a short account of what went well and badly. On
Fail2Drive a single route is never a result: say what the route showed, and see
`benchmark/FAIL2DRIVE.md` for how that benchmark is actually reported. If a CARLA
process is left behind, run `bash scripts/clean_carla.sh <CARLA_PORT>`.
