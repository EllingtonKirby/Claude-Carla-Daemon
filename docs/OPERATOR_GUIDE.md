# Operator guide: driving a probe session

How to launch a session, read the scene, and drive a route well. It is scenario-agnostic on
purpose: discover each scenario from live state.

## 1. The model of the world

- One session = one route. The leaderboard runs CARLA synchronously at **20 Hz**
  (1 tick = 0.05 s).
- The sim is **frozen whenever no time-advancing command is pending**, so deliberation is free.
- A route has a hard budget of **4000 ticks (200 s driven)**; `status.ticks_left` counts down.
  The leaderboard also fails the route if the ego is **blocked for 60 s**, **deviates from the
  route** by more than 30 m, or trips a scenario timeout.
- You are the only driver. At launch the ego is in `hold` mode (braking) and stays there until you
  command something.

## 2. Launch and connect

```bash
source scripts/env.sh
ACCESS=sensors bash scripts/run_session.sh 2091 &     # or ACCESS=privileged
python -m carla_probe.client --wait 600 status        # returns once CARLA + the route have loaded (1-3 min)
```

The client defaults to `127.0.0.1:8790` (`$CARLA_PROBE_ADDR` or a leading `HOST:PORT` argument
overrides it). Syntax: `python -m carla_probe.client CMD key=value ...`, with values parsed as
JSON. `repl` opens an interactive prompt. `help` lists every command for the session's access level.

If an advance outlasts your shell timeout, **the daemon finishes it anyway**. Reconnect and run
`status`; do not re-issue the advance.

## 3. Access levels

`status.access` tells you which one you are in. The daemon enforces it: a refused request returns
`ok: false` and changes nothing.

| | **sensors** (Bench2Drive-eligible) | **privileged** |
|---|---|---|
| scene | cameras only | actors, parked cars, lanes, traffic lights + stop lines, stop signs |
| ego | speed, IMU, GNSS, pose estimated from GNSS + compass (~0.5 m noise) | ground-truth pose, speed limit |
| route | leaderboard's sparse GPS plan: route points ahead, next turn command, progress | dense route, same fields |
| feedback | none: no score, collisions or infractions | live score, `new_infractions`, collision contacts |
| `render_bev` | navigation map: route, route points, your plan | full scene |
| refused | `get_score`, `run_until_event collision/infraction`, `stop_on_collision` | nothing |

## 4. Commands

Every reply carries `status`. An advancing command replies once, when the sim pauses again, with
`result.reason` and `ticks_advanced` (privileged sessions add `collisions_during` and
`new_infractions`). Items marked \* are privileged-only.

**Every driving command** (`apply_plan`, `set_control`, `hold`, `tick`, `run_until_frame`,
`run_until_event`, `shutdown`) **must include `why="..."`**: what you observed and why this command
follows from it, in a sentence or two. A command without one is refused (`ok: false`) and changes
nothing. Each accepted command is logged with its justification and outcome, so decisions can be
reviewed against the scene after the run. Write the reason you actually acted on.

| command | args | effect |
|---|---|---|
| `status` | | step, `ticks_left`, access, mode, speed, last control, route progress (+ speed limit, collision contacts and live score when privileged) |
| `get_state` | | everything the access level allows |
| `get_score`\* | | live DS / RC / IS estimate plus every infraction event |
| `percept` | `cams?` `k?` | **the per-decision view, and the only one.** Privileged: lane, route, relevant lights / stop signs, `k` nearest actors and parked cars. Sensors: ego status and route. Both: `frames_dir` plus a `files` name from **every** camera for this tick — open `<frames_dir>/<name>`. `percept=true` on a driving command returns the next one in the reply that reports the outcome, so a decision costs one round trip |
| `render_bev` | | PNG path (contents per §3) |
| `dump_frame` | | state.json + cameras + bev.png into `frames/frame_<step>/` |
| `apply_plan` | `waypoints` `ticks` `target_speed?` `stop_on_collision?`\* | **primary driving primitive**: PID-track ego-frame waypoints for `ticks` |
| `set_control` | `steer` `throttle` `brake` `ticks` `stop_on_collision?`\* | raw control (steer + = right) |
| `hold` | `ticks?` | brake; with `ticks`, also advance |
| `tick` | `n` `stop_on_collision?`\* | advance in the current mode |
| `run_until_frame` | `frame` | advance until `step >= frame` |
| `run_until_event` | `event` `max_ticks?` | `stopped` \| `plan_done` \| `collision`\* \| `infraction`\* |
| `record` | `on` `every?` | log camera frames to `camera_log/` (then `tools/make_video.py`) |
| `shutdown` | | abort the route (scored as an agent failure) |

## 5. Frames: read this first

- **Ego frame: `fwd` = metres ahead, `left` = metres to the LEFT.** `percept`, the `ego` blocks,
  `nav.route_points_ego`, `render_bev` (forward is up) and `apply_plan` waypoints all use it. A
  positive `left` is on your left **regardless of world heading**. Reason in this frame, never in
  world compass directions.
- Privileged actor fields: `v_fwd < 0` means the actor is coming toward you; `v_left > 0` means it is
  moving toward your left. `yaw_rel_deg` is its heading relative to yours (+ = turned left, ±180 =
  oncoming).
- The raw `loc` / `rot` / `velocity` / `pose_estimate` fields are **CARLA world**, which is
  **left-handed** (+y is to the right of +x, and yaw increases clockwise). If you ever convert by
  hand: `fwd = dx·cos(yaw) + dy·sin(yaw)`, `left = dx·sin(yaw) − dy·cos(yaw)`. Prefer the ego-frame
  fields.

## 6. Driving with `apply_plan`

- `waypoints = [[fwd, left], ...]`, at least 2 points, in the CURRENT ego frame. The plan is pinned
  to the world when issued, so later ticks track the same path as the ego moves.
- **Speed.** With `target_speed` (m/s) the controller holds that speed along the path. Without it,
  waypoints are `plan_interval_s` (0.5 s) apart, so spacing encodes speed (3 m apart ≈ 6 m/s).
  `target_speed=0` brakes to a stop.
- **Geometry.** Use 5–8 points reaching 15–40 m ahead; the steering aims about 12 m out
  (`pid.aim_dist`). Hand-drawn curves drift through sharp turns, so anchor waypoints to something
  real: the lane (privileged `lane.*`, or the lane markings you see on camera) and
  `nav.route_points_ego`.
- **When the plan is consumed** (its endpoint is no longer ahead), `plan_done` becomes true and
  the ego brakes. Issue the next plan before that point to keep moving.
- **Burst length.** Use 20–40 ticks (1–2 s) on open road and 5–15 near interactions. Re-observe
  between bursts.

## 7. Driving on the sensors track

- **The scene exists only in the cameras.** Take a `percept` every decision: open `CAM_FRONT` always,
  plus the side cameras at junctions and merges, and `CAM_BACK` before lane changes or reversing
  your speed plan.
- **Distances from images.** The JPGs may be written at a reduced resolution, so work from the image
  you actually opened rather than from a fixed pixel count. `CAM_FRONT` has a 100° horizontal FOV,
  which makes its focal length ≈ 0.42 × the image width W in pixels. An object of real height H
  metres spanning h pixels is then ≈ 0.42·W·H/h metres away. A car is about 1.5 m tall and 1.8 m
  wide, and a lane about 3.5 m wide.
- **Closing speed** comes from the change in distance between two frames a known number of ticks
  apart. Compare frames before committing to a gap.
- **Route points are approximate.** The plan is sparse, so the polyline between route waypoints
  cuts corners, and the pose estimate carries ~0.5 m GNSS noise. Use `nav.route_points_ego` and
  `next_command` for *where* to go, and the visible lane for *exactly where* to drive.
- **No speed limit and no feedback.** Read speed signs; otherwise drive with surrounding traffic.
  Nothing tells you about a collision or infraction. A sharp negative `accel_fwd_mps2` spike that
  you did not command is a strong hint that you hit something.
- **Traffic lights and stop signs**: find them on camera, and stop at the stop line on your side of
  the junction.

## 8. Hard-won lessons (both tracks)

- **Short bursts near other road users.** Long advances on a stale read are how cars get
  rear-ended. Nosing into a stopped vehicle can wedge the ego in an **unrecoverable traffic-manager
  gridlock** (pinned front and back; nothing moves even at full throttle).
- **Gap acceptance** (unprotected turns, merges): time the gap to each conflicting vehicle, commit
  only when the whole manoeuvre fits, and do not stop halfway across a lane.
- **Stop signs**: come to a full stop (speed ≈ 0) at the line before proceeding.
- **Red lights**: stop before the stop line.
- **Emergency vehicles**: a vehicle with flashing lights behind you has priority, and yielding is
  not finished until it is actually **past you**. It wants the lane you are in, so **get out of that
  lane**: change lane if there is one, or pull clear onto the shoulder. Slow enough to make the move
  safely and make it promptly -- decisiveness is the point. **Do not stop in its lane.** A following
  emergency vehicle stops behind you, the gap freezes, and it can never get by; standing still in
  front of one is the one action that guarantees it never passes, however far right you sit. It
  approaches from behind, so it is only ever visible in `CAM_BACK`; nothing in the route, the front
  cameras or the ego sensors will warn you.
- **Privileged `collision_contacts` counts contacts, not scored collisions.** The leaderboard counts
  sustained contact as one event; scored infractions are in `status.score`.
- **Blocked timer**: standing still for 60 s outside a scenario fails the route. Waiting at a light
  is fine if you move off when it turns green.
- **Keep up with the flow of traffic.** Driving far below the speed of the traffic around you is an
  infraction, not a safe default.

## 9. Operating efficiently

- One `percept` per decision, plus `render_bev` when the geometry is unclear (privileged).
  `percept=true` on the driving command folds the next look into the reply you were already waiting
  for, so a decision costs one round trip rather than two.
- A percept writes a JPG from every camera but you choose which to open. Opening one costs tokens
  and it stays in your context; the daemon writing it does not. Look wide when the scene is
  ambiguous, and take the front camera alone when it is not.
- **`CAM_BACK` is in every percept.** Whatever is behind you -- a vehicle closing, an emergency
  vehicle, traffic you would cut in front of -- is only ever visible there. Front-only looking is a
  blind spot you chose.
- Deliberation is free; the tick budget is not. Don't creep for no reason.
- On route end the advance returns reason `route_end`; `result.json` holds the official scores.

## 10. Artifacts

`SESSION_DIR/`: `result.json`, `session.json` (access, seed, commit), `commands.jsonl` (every command
with wall time), `decisions.jsonl` / `decisions.txt` (your driving commands, justifications and
outcomes), `trajectory.jsonl` (per-tick pose and control; the GNSS estimate on the sensors track),
a CARLA recording of the whole simulation, `frames/`, `camera_log/`, `evaluator.log`, `carla.log`,
`probe_done.json`.
`python tools/make_video.py SESSION_DIR` turns `camera_log/` into an mp4. On the sensors track,
see `benchmark/PROTOCOL.md` rule 6 before opening any of these mid-run.
