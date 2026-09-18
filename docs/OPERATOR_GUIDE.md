# Operator guide: driving a probe session

How to launch a session, read the scene, and drive a route well. It is scenario-agnostic on
purpose: discover each scenario from live state.

## 1. The model of the world

- One session = one route. The leaderboard runs CARLA synchronously at **20 Hz**
  (1 tick = 0.05 s).
- The sim is **frozen whenever no time-advancing command is pending**, so deliberation is free.
- A route has a hard budget of **4000 ticks (200 s driven)**; `status.ticks_left` counts down.
  (Where `ticks_left` comes back `null` the benchmark has no tick cap and the route ends on its own
  timeout; drive as if the budget applied anyway.)
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
| `clearance` | `points` `waypoints?` | how close your **body** comes to points you believe are at ego-frame `[[fwd,left],...]`: now, and over a path (the one given, else the plan in flight). Also `points=` on `percept`, in the same round trip as looking |
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

### 5.1 You are not a point

Every ego-frame distance in a reply — `route_points_ego`, an actor's `fwd`/`left`, a plan waypoint
— is measured from the ego **origin**. Your bodywork is not there. On the Bench2Drive hero it
reaches **2.44 m ahead of the origin, 2.45 m behind, and 0.92 m to each side** (`get_state` →
`ego_body`, from `config.yaml`). The front camera is mounted a further 0.8 m forward still, so a
car you judge "3 m away" in `CAM_FRONT` is about **1.4 m from your bumper**.

This matters more than it sounds. Two cars touch when their centres reach the sum of their
half-extents: roughly **4.8 m nose to nose and 2.0 m flank to flank**. A "3 m gap" ahead is not a
gap. Measured over a whole corpus, the single largest cause of lost score was committing to a
manoeuvre with a stated margin of half a metre to two metres — margins that were already contact.

So: **a distance is not a clearance until you subtract your own extent on that side.** Rather than
do that arithmetic in your head, hand the daemon the points you believe are there:

```
clearance points=[[3.0, -1.2], [8.0, 0.4]]
clearance points=[[3.0, -1.2]] waypoints=[[4,0],[8,4]]     # ...and over a candidate path
percept points=[[3.0, -1.2]]                               # ...in the same round trip as looking
```

It answers from your own estimates and your own dimensions — it cannot see anything you cannot —
and returns `now_m` (clearance at your current pose) and `along_plan.min_m` (the closest your body
comes while following the path). **Negative means you drive through it.** The sweep rotates the
whole rectangle along the path, so it catches the case that pure forward reasoning misses: a rear
corner swinging into something your nose has already cleared. `swept.corridor_width_m` says how
much road the manoeuvre actually needs, which is more than the width of the car whenever it bends.

Two caveats. The path checked is the one you asked for, not the one the controller will track —
it aims at a single lookahead point and lags by roughly 0.2 m — so keep margin beyond zero. And
the points are *your* estimates: the answer is only as good as they are.

## 6. The drive loop

Look, plan, advance, **repeat**. All three, every decision. The loop is the technique; everything
in §7 only decides how well one turn of it goes.

```
percept                                                            # the first look
apply_plan waypoints=[...] target_speed=6 ticks=20 percept=true why="..."
apply_plan waypoints=[...] target_speed=6 ticks=20 percept=true why="..."
...
```

- **Look on every decision, not once per route.** `percept=true` on the driving command returns the
  next look inside the same reply that reports the advance, so looking costs no extra round trip and
  no extra command. Nothing watches the road between your commands: there is no autopilot, no
  collision warning, and on the sensors track no feedback of any kind. A plan issued from a picture
  taken thirty seconds ago is a guess about a scene that has moved.
- **Open the images the percept names.** The reply carries *paths*, not pixels: an image you did
  not open is an image you did not see. `CAM_FRONT` every decision; the side cameras at junctions,
  merges and anything crossing; `CAM_BACK` about once a second.
- **The route is not the scene.** `nav.route_points_ego` tells you where the road goes, and it says
  exactly the same thing whether the lane ahead is empty or has a stopped truck in it. Driving on
  route points alone tracks the route accurately into whatever is standing on it.
- **Read the reply before the next plan.** `result.reason` says why the advance ended (`ticks` for
  a completed `apply_plan` burst, or `stopped` / `plan_done` / `max_ticks` for a `run_until_event`,
  `route_end` at the finish). `status` carries speed, route progress and `ticks_left`, and
  **`status.plan_done: true` means the plan ran out before the burst did** — the ego spent the rest
  of it braking, so the plan was too short (§7).

## 7. Driving with `apply_plan`

`waypoints = [[fwd, left], ...]`, at least 2 points, in the CURRENT ego frame. The plan is pinned to
the world when issued, so later ticks track the same path as the ego moves along it.

**What the controller actually reads.** Each tick it starts from the waypoint nearest the ego, walks
forward along the plan, and steers at the single point it finds **L metres along**, interpolated
between the two waypoints that bracket it. L grows with speed: ≈2.9 m at 1 m/s, 4.8 m at 3 m/s,
7.8 m at 6 m/s, and it caps at 10.5 m above ~8.8 m/s. **Nothing beyond L steers the car.** A plan
reaching 40 m is not read 40 m ahead; the far end only decides when the plan runs out.

So a plan has exactly two jobs: be *accurate* where the steering reads, and be *long enough* to
outlive the burst.

- **Space the points 2–3 m apart over at least the first 10 m.** A segment is the resolution of your
  steering: with points 5 m apart the controller aims at a straight line drawn across the curve you
  meant. And because the walk starts at the *nearest* waypoint, which is behind you about half the
  time, up to half your spacing is spent walking backwards before any of L counts forward — coarse
  spacing quietly shortens the lookahead as well as blunting it.
- **Reach ≈ burst distance + 10 m.** The burst covers `target_speed × ticks × 0.05` metres; add the
  lookahead so the steering still has path in front of it on the last tick. At 6 m/s for 20 ticks
  that is 6 + 10 ≈ 16 m, which at 2.5 m spacing is 6–7 points. Reaching much farther buys nothing:
  you will replace the plan within a second anyway, and spreading the same points over more distance
  is exactly how the near geometry gets blunted.
- **When the plan runs out** — its last point is behind you, or within 1.5 m — `plan_done` fires and
  the ego **brakes for the whole remainder of the burst**, however many ticks are left. Size the
  reach so this never happens mid-burst; `status.plan_done` in the reply tells you when it did.
  Points beyond ~12 m may be spaced as widely as you like: nothing steers by them.
- **Always pass `target_speed`** (m/s). Without it the controller reads the speed off your spacing
  (mean spacing ÷ `plan_interval_s`, 0.5 s), so the close spacing above would command a crawl, and
  the implied speed would drift as the plan is consumed. `target_speed=0` brakes to a stop.
- **Straight ahead is not "stay in the lane".** A plan of points with `left = 0` freezes the lateral
  offset you have *right now*: half a metre off centre, and it holds you half a metre off centre for
  the whole burst. Put the points where the car should be, not where it already is.
- **Anchor the geometry to something real.** Hand-drawn curves drift through sharp turns, so build
  the near points from the lane (privileged `lane.*`, or the lane markings you can see on camera)
  and the far ones from `nav.route_points_ego`. A route point 40 m out tells you where you are
  going; it tells you nothing about where the road is in the next 10 m.
- **Burst length:** 20–40 ticks (1–2 s) on open road, 5–15 near other road users.
- **Don't crawl.** Driving far below the traffic around you is a scored infraction in its own
  right, and it also shrinks L: at 1 m/s the controller steers at a point under 3 m away and
  follows every wobble in your first two waypoints. Pick a speed you can justify and hold it.

## 8. Driving on the sensors track

- **The scene exists only in the cameras** — which is why §6's look-every-decision rule is not
  advice here but the whole of your perception.
- **Distances from images.** The JPGs may be written at a reduced resolution, so work from the image
  you actually opened rather than from a fixed pixel count. `CAM_FRONT` has a 100° horizontal FOV,
  which makes its focal length ≈ 0.42 × the image width W in pixels. An object of real height H
  metres spanning h pixels is then ≈ 0.42·W·H/h metres away. A car is about 1.5 m tall and 1.8 m
  wide, and a lane about 3.5 m wide.
- **Closing speed** comes from the change in distance between two frames a known number of ticks
  apart. Compare frames before committing to a gap.
- **Route points are approximate.** The plan is sparse, so the polyline between route waypoints
  cuts corners, and the pose estimate carries ~0.5 m GNSS noise. Use `nav.route_points_ego` and
  `next_command` for *where* to go, and the visible lane for *exactly where* to drive — which is
  also what §7's near waypoints need.
- **No speed limit and no feedback.** Read speed signs; otherwise drive with surrounding traffic.
  Nothing tells you about a collision or infraction. A sharp negative `accel_fwd_mps2` spike that
  you did not command is a strong hint that you hit something.
- **Traffic lights and stop signs**: find them on camera, and stop at the stop line on your side of
  the junction.

## 9. Hard-won lessons (both tracks)

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

## 10. Operating efficiently

- **Use all the cameras.** A percept writes a JPG from every one, and the side and rear views show
  things the front simply cannot. Opening an image costs tokens and it stays in your context, but
  driving on a partial view costs far more than the tokens save. In particular, a vehicle closing on
  you from behind, an emergency vehicle, or traffic you are about to pull in front of appears in
  none of `CAM_FRONT`, the route or the ego sensors: nothing gives you a cue to look, so the rear
  check has to be unconditional rather than something you do when the forward view looks ambiguous.
- `render_bev` is worth a call when the geometry is unclear (privileged).
- Deliberation is free; the tick budget is not. Don't creep for no reason. On an empty straight
  road take *longer bursts* rather than fewer looks — with `percept=true` the look is already in
  the reply, so the thing to economise is ticks per decision, never decisions per route.
- On route end the advance returns reason `route_end`; `result.json` holds the official scores.

## 11. Artifacts

`SESSION_DIR/`: `result.json`, `session.json` (access, seed, commit), `commands.jsonl` (every command
with wall time), `decisions.jsonl` / `decisions.txt` (your driving commands, justifications and
outcomes), `trajectory.jsonl` (per-tick pose and control; the GNSS estimate on the sensors track),
a CARLA recording of the whole simulation, `frames/`, `camera_log/`, `evaluator.log`, `carla.log`,
`probe_done.json`.
`python tools/make_video.py SESSION_DIR` turns `camera_log/` into an mp4. On the sensors track,
see `benchmark/PROTOCOL.md` rule 6 before opening any of these mid-run.
