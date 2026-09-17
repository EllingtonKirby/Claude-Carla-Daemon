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

**Every driving command** (`apply_plan`, `set_control`, `drive`, `hold`, `tick`, `run_until_frame`,
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
| `drive` | `text` `ticks` `mode?` `stop_on_collision?`\* | **when the session has the learned policy on (§7a):** tell the policy what you see and what to do; it drives for `ticks`. Refused otherwise |
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
  you actually opened rather than from a fixed pixel count. `CAM_FRONT` has a 110° horizontal FOV,
  which makes its focal length ≈ 0.35 × the image width W in pixels. An object of real height H
  metres spanning h pixels is then ≈ 0.35·W·H/h metres away. A car is about 1.5 m tall and 1.8 m
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

## 7a. Driving with `drive`: the learned policy

Some sessions run with a learned driving policy (SimLingo, a vision-language-action model). Its
`status` shows `vla` while it is in charge, and `help` lists `drive`; when the policy is off,
`drive` is refused and nothing else changes. With it on, **`percept` returns `CAM_FRONT` only**:
the policy sees exactly that camera, and you must reason from the same image it does.

The policy gets one front image, your speed, two points of the route, and **your sentence**. It
then steers and controls speed at every tick for `ticks`, on its own perception, until you speak
again. So you are the supervisor: you look, you decide what the situation is and what to do, you
say it, and the policy executes between your decisions, reacting to what happens in between.

- `mode=commentary` (default): **a description of the situation and the action**, in the
  policy's own phrasebook below. This is what it was trained to say to itself before acting.
- `mode=instruction`: **an order** it was trained to obey even against the route: a lane change,
  a speed, a stop, pulling into a parking lane. `mode=safety` is the same order, but the policy may
  refuse one it considers unsafe (its reply then says so in `language`).
- `mode=plain`: no text; the policy alone. Use it when you have nothing to add.

**Speak the phrasebook.** The policy was trained on templated sentences; free prose is outside
its experience. A commentary is two sentences, *manoeuvre* then *speed*:

| manoeuvre | speed |
|---|---|
| `Follow the route.` | `Accelerate to drive with the target speed.` |
| `Turn left.` / `Turn right.` | `Maintain your current speed.` / `Maintain the reduced speed.` |
| `Prepare to do a lane change.` | `Decelerate to stay behind the car in 12 meters.` |
| `Stay on your current lane to go around the construction site.` | `Accelerate to follow the car in 20 meters.` |
| `Stay on your current lane to give way to the emergency vehicle.` | `Remain stopped to stay behind the car.` / `Stop now to stay behind the car.` |
| `Stay on your current lane to overtake the bikes on your lane.` | `Decelerate because the traffic light is red.` / `Accelerate because the traffic light is green.` |
| `Steer clear of the parked vehicle.` / `Avoid the accident on your lane.` | `Decelerate to drive through the junction.` / `Accelerate to drive through the junction.` |
| `Return to your original route after avoiding the obstacle.` | `Wait for a gap in the traffic before changing lanes.` |
| `Avoid the vehicle that is opening its door.` | `Decelerate due to the pedestrian in 8 meters.` |

`<OBJECT>` is a plain noun (`car`, `truck`, `bike`, `pedestrian`, `traffic light`, `stop sign`),
`<DISTANCE>` is `N meters`. Instructions are shorter: `Change one lane to the left.`, `Slow down.`,
`Accelerate.`, `Stop.`, `Drive at 5 m/s.`, `Change to the parking lane on the right.`,
`Turn left at the next intersection.`

**What it will and will not do for you.** It holds the lane and follows the route by itself, and
it brakes for what it sees in front. It does *not* see behind or beside you, it does not know the
scenario, and a sentence telling it to accelerate overrides its own caution: told "Accelerate to
drive with the target speed" at a red light, it accelerates. Say what is true. Use `ticks` of
10–40 and look again; the reply's `result.vla` reports what it said back (`language`), the speed
it chose (`desired_speed_mps`) and how many times it was queried.

## 8. Hard-won lessons (both tracks)

- **Short bursts near other road users.** Long advances on a stale read are how cars get
  rear-ended. Nosing into a stopped vehicle can wedge the ego in an **unrecoverable traffic-manager
  gridlock** (pinned front and back; nothing moves even at full throttle).
- **Gap acceptance** (unprotected turns, merges): time the gap to each conflicting vehicle, commit
  only when the whole manoeuvre fits, and do not stop halfway across a lane.
- **Stop signs**: come to a full stop (speed ≈ 0) at the line before proceeding.
- **Red lights**: stop as close to the stop line as you can **with the signal still in view**.
  There is a band, and both edges cost you. Too far back and it is unreadable: a signal is a handful
  of pixels at 30 m and only a few more at 15 m, so pulling up ten metres short of the line is not
  the cautious choice -- it parks you outside the range at which you can read the one thing you are
  waiting on, and no amount of re-reading fixes an image that cannot answer the question. Too close
  and it leaves the frame entirely: `CAM_FRONT` sees about 62 degrees vertically from a 1.7 m mount,
  so a signal on a mast arm 5 m up drops off the top of the image inside roughly 5 m, and one 6 m up
  inside about 7 m. If the signal has vanished from the top of the frame you are too close to read
  it, not looking at a dark lamp -- check a signal head further away, or the cross traffic, rather
  than trusting your last reading.
  Between those limits, keep re-reading it: signals change, so a colour you read a while ago is
  stale, and a lamp you cannot resolve is unknown rather than still-what-it-last-showed. Cross
  traffic and the cars around you are corroborating evidence -- if the queue beside you moves off
  and nothing is crossing, your own reading is probably out of date.
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
- **Use all the cameras.** A percept writes a JPG from every one, and the side and rear views show
  things the front simply cannot. Opening an image costs tokens and it stays in your context, but
  driving on a partial view costs more than the tokens save.
- **Look at least once a second.** You are blind between decisions: nothing is rendered to you while
  a driving command runs, so a 60-tick advance is three seconds in which the world changes and you
  do not see it. Keep `ticks` at or under 20 (one second) whenever anything is happening -- traffic
  near you, a junction, a signal, a manoeuvre in progress -- and spend longer advances only on empty
  road you have just looked at. A stale look is the most expensive thing you can act on.
- **Check `CAM_BACK` at least once a second**, which means on essentially every decision. A vehicle
  closing on you, an emergency vehicle, traffic you are about to pull in front of: none of it
  appears in `CAM_FRONT`, in the route, or in the ego sensors. Something following you gives you no
  cue at all to look, so the rear check cannot be something you do when the forward view looks
  ambiguous -- by the time the forward view tells you to look behind, whatever was there has
  already decided the outcome.
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
