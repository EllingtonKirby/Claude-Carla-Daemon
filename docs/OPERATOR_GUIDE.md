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

A third level, **dense**, is the sensors column exactly, plus ground-truth depth and segmentation
arrays for every camera in each `percept` (§8d). It refuses what sensors refuses.

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
| `clearance` | `points` `waypoints?` | how close your **body** comes to points you believe are at ego-frame `[[fwd,left],...]`: now, and over a path (the one given, else the plan in flight). Also `points=` on `percept`, in the same round trip as looking |
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

### 5.1 You are not a point

Every ego-frame distance in a reply — `route_points_ego`, an actor's `fwd`/`left`, a plan waypoint
— is measured from the ego **origin**. Your bodywork is not there. On the Bench2Drive hero it
reaches **2.44 m ahead of the origin, 2.45 m behind, and 0.92 m to each side** (`get_state` →
`ego_body`, from `config.yaml`). `CAM_FRONT` is **not** at the bumper: it is mounted at x = **-1.5 m**
(1.5 m *behind* the origin, about 3.9 m behind your front bumper) and 2.0 m up (`get_state` ->
`ego_body.cameras` has every mount). A distance you measure from a `CAM_FRONT` image is from the
camera: subtract ~3.9 m to get the gap to your front bumper. Anything about 4 m from the camera is
already touching you.

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

## 6a. The five questions: how to choose the speed

Your `target_speed` is not a judgment call; it is the answer to five questions, asked in this
order on every decision, from the percept you just opened. **Write the answers at the start of
`why=`** as `Q1:n Q2:y Q3:n Q4:n Q5:n`, then your reasoning. The answers are how your decisions
are reviewed afterwards, so answer what you actually see.

- **Q1 - Slow zone.** Am I on an urban street and in, or within ~30 m of, an intersection?
  Yes: cap `target_speed` at **7 m/s**.
- **Q2 - Follow.** Is there a vehicle or a cyclist *ahead in my lane, going my way*, close enough
  (< 4 m + 0.25 x my speed, plus your estimation margin) or slow enough that I should ease off to
  hold the gap? Parked, broken-down or blocking vehicles are not followed; they are Q5.
  Yes: `target_speed` = its speed, or lower until the gap is safe.
- **Q3 - Stop line.** Does a red or amber light for *my* lane (§9), or a stop sign I have not yet
  stopped at, require me to stop at a line ahead? Yes: plan to the line with `target_speed=0`.
  A stop sign is done only after >= 1.5 s below 0.1 m/s within 3 m of the line; you keep that
  memory across looks, nothing does it for you.
- **Q4 - Path conflict.** If I keep driving my route at the intended speed for the next 2 s, will
  I hit or come close to *anything*: a crossing, merging, oncoming or pulling-out car, an opening
  door, a pedestrian or a crossing cyclist (including one moving toward my path further ahead), a
  cone or debris? A car in the lane I am merging into counts. Yes: **stop now** (`target_speed=0`,
  or `hold`). `clearance` on your plan (§5) turns the distances you estimate into body clearances.
- **Q5 - Overtake.** Is my lane blocked by a stationary obstacle, and is the oncoming lane clear
  for ~64 m? Yes: commit -- `target_speed` up to **11 m/s**, Q1-Q3 no longer apply, **Q4 still
  applies on every decision** until you are back in your lane. Blocked but not clear: wait behind it.

**Combine:** Q5 -> overtake unless Q4 · else Q4 or Q3 -> stop · else Q2 -> follow · else Q1 ->
<= 7 m/s · else the speed limit (a posted sign; otherwise 10 m/s).

**Then decide how long before you look again.** Any yes to Q2-Q4 means `ticks <= 20`; only an
all-no answer licenses a long burst.

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
  you actually opened rather than from a fixed pixel count. `CAM_FRONT` has a 110° horizontal FOV,
  which makes its focal length ≈ 0.35 × the image width W in pixels. An object of real height H
  metres spanning h pixels is then ≈ 0.35·W·H/h metres away. A car is about 1.5 m tall and 1.8 m
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

## 8a. Driving with `drive`: the learned policy

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

## 8b. The code tool

Some sessions give you a code tool (the prompt says so). You write Python files into your code
dir with Write / Edit, and run one per Bash call with `python -m carla_probe.code_runner FILE.py
[args]`. A script sees `numpy` and `cv2`, may read the frames a `percept` returns and
`<frames_dir>/../cameras.json`, and may write only inside your code dir. Everything else is refused
with `code tool: denied`. It cannot send probe commands: driving stays a decision you make and
justify. Output is truncated, so print numbers, not arrays.

**What it is for: measuring instead of eyeballing.** Most collisions on this benchmark are not
missed objects but misjudged margins (§5.1): the driver sees the car and gets the gap wrong by
about its own half-length. A script can do the geometry exactly.

- **`cameras.json`** gives, per camera, the mount (CARLA vehicle frame: x forward, y RIGHT, z up,
  yaw + = right) and the pinhole intrinsics of every file a percept writes, keyed by kind
  (`jpg`, `full_jpg`, `depth_npy`, `inst_png`). Use the intrinsics of the file you load.
- With the code tool on, each percept also writes **`<cam>_full.jpg`** at native resolution.
- **Ground plane.** A pixel on the road at row v below the horizon is at forward distance
  `z ≈ fy · h_cam / (v − cy)` in the camera frame (camera height `h_cam` = mount z, pitch 0), and
  lateral offset `x = (u − cx) · z / fx` (right positive). Rotate by the mount yaw and add the mount
  position to get the vehicle frame, then flip y to get the probe's ego `left`. The contact point
  of a car with the road is the bottom of its body in the image, which is what this projects.
- **Your own car is in the frame**: the bottom rows of `CAM_FRONT` and `CAM_BACK` show the ego's
  bonnet and boot. Anything a script finds there is you.
- **Measure to bodies.** Ego-frame distances are to your origin; your bumper is `ego_body.front_m`
  ahead of it (§5.1), and `clearance points=...` does that subtraction for you.
- Build up a small library across the drive (`lib.py` imported by short scripts) rather than
  rewriting the same projection each time: a script that worked at step 200 is free at step 900.
- Pausing is free; script run time is not counted against the route. Keep each script under the
  tool's timeout (120 s).

## 8c. The perception service

When the prompt names a perception service, your scripts may call it -- and no other network
address. It runs learned models on a camera image, so its answers are estimates, as good as the
models (Bench2Drive-eligible: they see only your cameras). From a script:

```python
from carla_probe.perception_client import PerceptionClient
pc = PerceptionClient()                               # address from the environment
depth = pc.depth(path)                                # HxW float32, metres along the optical axis
labels, segments = pc.segment(path, prompts=["car", "pedestrian", "bicycle", "truck"])
# labels: HxW int32 segment id (0 = none); segments: [{"id", "label", "score", "area", "bbox": [x0,y0,x1,y1]}]
```

`path` is a frame file a percept returned; outputs are at that image's resolution, so use the
matching intrinsics. `pc.info()` says which models are loaded and whether `segment` honours free
`prompts` (an open-vocabulary model) or returns a fixed label set. Monocular metric depth is
typically good to ~5-10% at 10-30 m and worse far away or on glass; prefer the median over an
object's mask to any single pixel.

## 8d. Ground-truth arrays (dense track)

On `access: dense` each percept also writes, per camera:

- `<cam>_depth.npy`: float16 HxW, metres along the optical axis (exact).
- `<cam>_inst.png`: 3-channel uint8. Read with `cv2.imread(p)[..., ::-1]`: channel 0 is the
  semantic tag (names in `cameras.json` `semantic_tags`: 14 car, 15 truck, 16 bus, 12 pedestrian,
  18 motorcycle, 19 bicycle, 1 road, 24 road_line, ...), channels 1·256 + 2 an instance id, the
  same for every pixel of one object.

These are exact, but they are pixels, not a scene list: finding the nearest car in your path is
still your script's job. They start from the second tick of the route.

**Your own car is in the pictures.** The ego's bonnet fills the bottom of `CAM_FRONT` and its boot
the bottom of `CAM_BACK`, segmented as a car (one instance id, the same in every camera, at a
depth of 1-2 m). Exclude that instance before looking for the nearest vehicle; the same applies to
a perception model's masks (§8c), which will report it as a car a few metres away.

## 8e. Privileged extras: who everyone is, the rules, and where your car will go

Some privileged sessions add the following (the prompt says so; `predict` answers only in them).

**Every actor says who it is and how it treats you.** `actors_nearest` entries carry `type`
(e.g. `vehicle.ford.ambulance`), `role` (`scenario` or `background`), `emergency`, the `scenario`
it belongs to, its `behaviour` and `reacts_to_you`, read from the running scenario's code. Take
`reacts_to_you` literally: a flow vehicle "at constant speed" will NOT brake for you, whatever the
gap looks like; background traffic brakes only for a vehicle ahead of it in its own lane.

**`rules`** gives the scorer's rules with the live state it judges you on:
- `stop_sign.scorer_state`: `engaged` (you are in a stop's zone) and `satisfied` (the scorer has
  counted your stop). A stop counts only below 0.1 m/s with your origin within 4 m of the trigger
  centre: `next_on_route.origin_to_trigger_m` and `inside_counting_radius` tell you whether a stop
  here would count. Stopping short and creeping through does not.
- `emergency_vehicle.nearby`: emergency vehicles near you and whether they are in your lane.
  Yielding means leaving their lane, not slowing in it.

**`scenarios`** lists the scenarios on the route within 150 m: `type`, `task` (what you have to
do), `trigger_ahead_m`, whether it has `started`, and its `parameters` (flow speeds, spacing, ...).

**Your own car's motion.** The plan controller is measured (tools/controller_model.json, row
"plus"). It launches at full throttle until it is within 1 m/s of your target: from standstill it
reaches ~3.8 m/s in 1 s and 90% of 9 m/s in ~1.7 s, overshoots by up to ~10%, then holds the target
to about +-0.15 m/s. One exception: the car's automatic gearbox hunts when cruising between 5.0 and
6.7 m/s (+-0.5 m/s swings, and the benchmark forbids holding a gear), so a target_speed in that band
is held at 4.9 or 6.8 m/s, whichever is nearer. `predict` already accounts for this; ask for 4.9 or
6.8 directly if you care which. Within 25% over
the target it coasts; beyond that it brakes gently; asked to stop it brakes fully, from 12 m/s in
~1.2 s and from 5 m/s in ~0.7 s. The first ~0.5 s of a route the car does not move.
It follows your path closely but not exactly: the 95th-percentile sideways error is ~0.05-0.15 m
on gentle paths, ~0.2-0.3 m on a 3.5 m lane change over 15 m at 6-9 m/s or a 20-35 m radius curve,
and up to ~0.4 m on a 12 m radius curve at 6 m/s. Plan your margins on top of that, not inside it.
- `predict waypoints=... [target_speed] [ticks]` returns where the ego will be at each tick of
  that plan -- speed, pose in today's ego frame, and `band_m`, the measured sideways error there.
  Nothing moves.
- `plan_check` (same arguments) adds every nearby actor and parked car: the closest body-to-body
  clearance over the burst with others extrapolated at their current velocity, and `worst_case_m`
  with your band subtracted. Below 0 means your own tracking error alone could make contact.
  Others are extrapolated straight: a vehicle that reacts to you will not keep its velocity, one
  that does not react will.

Use `plan_check` before any plan near another road user, and treat `worst_case_m` as the margin
you actually have.

## 9. Hard-won lessons (both tracks)

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
