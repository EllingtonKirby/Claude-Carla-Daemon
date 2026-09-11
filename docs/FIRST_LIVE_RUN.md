# First live run: validation checklist

**Status: this repo has never been run against a live CARLA simulator.** Everything was developed on
a machine with no GPU or CARLA. The offline tests pass (`pytest`: unit tests plus fake-simulator
end-to-end sessions for both access levels), and `scripts/setup_bench2drive.sh` has been run for
real. Every call into the real CARLA / leaderboard API is unverified.

This document is the handoff for whoever runs it first on a GPU machine. Work through the steps
in order; each one gates the next. When a step fails, fix the code, add a regression test where
possible (`tests/test_agent_fakesim.py` can mimic most CARLA behaviour), and re-run from that step.
Record what you find in §6.

Background reading: `README.md`, `docs/DESIGN.md`, `docs/OPERATOR_GUIDE.md` §3–6.

## 0. Machine requirements

- Linux, NVIDIA GPU with ≈ 8 GB+ VRAM free, ≈ 30 GB disk for CARLA.
- `git`, `patch`, `wget`, and `uv` (or `python3.10`). Internet access for setup.
- Free ports: 20000 (CARLA RPC), 40000 (traffic manager), 8790 (probe socket). Override with
  `CARLA_PORT`, `TM_PORT`, `PROBE_PORT` if taken.

## 1. Setup

```bash
CARLA_ROOT=$HOME/carla bash scripts/setup_carla.sh   # skip if CARLA 0.9.15 with Town11-15 exists; export CARLA_ROOT to it
bash scripts/setup_bench2drive.sh
source scripts/env.sh
pytest
```

**Pass:**
- `env.sh` ends with `env ready`, and its import check lists `ok: carla_probe.agent`;
- `pytest` reports all tests passing;
- `routes/bench2drive220/route_25378.xml` exists.

**Likely trouble:**
- **Pinned dependencies fail to install.** `env.sh` pins `numpy==1.26.4`,
  `opencv-python-headless==4.5.4.58`, `setuptools<81` and `carla==0.9.15` (a cp310 wheel). Relax a
  pin only if it doesn't install, and note the change in §6.
- **Town12 is missing.** `ls $CARLA_ROOT/CarlaUE4/Content/Carla/Maps/Town12` must show files.
  Route 25378 is Town03 and doesn't need it; route 2091 does.

## 2. Running a session as an agent

`run_session.sh` blocks until the route ends. Start it in the background, then poll:

```bash
ACCESS=privileged bash scripts/run_session.sh 25378 > /tmp/session.out 2>&1 &   # or a background shell task
python -m carla_probe.client --wait 600 status
```

- The session dir is printed on the launcher's third line (`/tmp/session.out`). Logs are in
  `SESSION_DIR/evaluator.log` and `carla.log`.
- CARLA plus the route take 1–3 minutes to load before `status` answers.
- While paused, the evaluator looking "hung" is correct behaviour. An advance that outlives your
  shell timeout still completes: reconnect and `status`, and never re-issue the advance.
- Between sessions, if anything is left behind: `bash scripts/clean_carla.sh 20000`.

## 3. Privileged smoke run (route 25378, Town03)

| # | command | pass | if it fails, look at |
|---|---|---|---|
| 1 | `status` | `ok: true`, `step: 0`, `mode: hold`, `access: privileged`, `speed_limit_mps` a number, `score` **not null** | `score: null` means the patched evaluator did not call `set_scenario`; check the patch applied (`grep set_scenario third_party/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py`). A crash before this step points at sensor validation (camera mounts must be ≤ 3 m from origin, ≤ 8 cameras) or the collision-sensor spawn in `_ensure_collision_sensor` |
| 2 | `observe` | `nav.route_points_ego[0]` ≈ `[5, ~0]` (5 m ahead, small `left`); `lane` not null with `lane_width_m` ≈ 3–4; nearby vehicles listed with plausible `fwd`/`left` | route points far away or behind: route projection in `set_global_plan` / `_update_route_progress` |
| 3 | `render_bev`, then open the PNG | grey lanes under the blue route, ego (red) at the bottom centre facing up, vehicles on lanes, traffic lights as coloured dots | lanes offset or rotated: `_local_lanes` / `_yaw_rel_ccw`; no lanes: `CarlaDataProvider.get_map().get_waypoint`; exception mentioning `CityObjectLabel` or `get_level_bbs`: `_parked_vehicles`. Also note how long the command took |
| 4 | `get_camera` | 4 JPGs that look like the road ahead / left / right / behind | camera ids in `config.yaml` |
| 5 | `apply_plan 'waypoints=[[4,0],[8,0],[12,0],[16,0],[20,0]]' target_speed=5 ticks=40` | `result.reason: ticks`, `speed_mps` ≈ 3–5, `mode: plan`, the car moved forward, not sideways | steering sign or aim: `pid.py` (steer + = right); plan anchoring: `geometry.ego_to_world` |
| 6 | `get_state` | if a traffic light is within 80 m: `traffic_lights[].state` is Red / Green / Yellow; if one is on the route: `stop_line_on_route_ahead_m` is a number | `_traffic_lights` (`get_stop_waypoints`), `_stop_signs` (`Transform.transform`) |
| 7 | `set_control steer=0 throttle=0 brake=1 ticks=40`, then `run_until_event event=stopped max_ticks=200` | reason `stopped`, `speed_mps` < 0.1 | |
| 8 | Drive to the end: repeat `observe` + `apply_plan` along `nav.route_points_ego` (`target_speed` 6–8, `ticks` 20–40), or use `drive-carla` | final advance returns `reason: route_end`; `result.json` has a record; the launcher **exits by itself** and prints the summary table | launcher never exits: the teardown watcher in `run_session.sh` (waits for `probe_done.json`, then `global_record` or 120 s). No `probe_done.json`: `ProbeAgent.destroy` |

Also check that `SESSION_DIR/commands.jsonl` and `trajectory.jsonl` are populated.

## 4. Sensors smoke run (route 25378)

New session: `ACCESS=sensors bash scripts/run_session.sh 25378`.

| # | command | pass | if it fails, look at |
|---|---|---|---|
| 1 | `status` | `access: sensors`; **no** `score`, `speed_limit_mps` or `collision_contacts` keys | |
| 2 | `get_state` | `ego_status.gnss` lat/lon plausible; `pose_estimate.x/y` within metres of the privileged run's start (compare `trajectory.jsonl` step 0 of both sessions) and **not ~1e5**; `pose_estimate.yaw_deg` matches the privileged run's start yaw within a few degrees | x/y wrong: `geometry.solve_geo_reference` / `gps_to_world` or the `GPS` input layout (`[lat, lon, alt]`). Yaw off by 90° or 180°, or mirrored: `geometry.compass_to_yaw_deg` or the `IMU` layout (compass is the last element) |
| 3 | `observe` | `nav.route_points_ego[0]` ≈ `[5, ~0]` (loose: the route plan is sparse and GNSS noisy, so within ~1–2 m) | route points behind or to the side: yaw conversion (step 2) |
| 4 | `get_score` | refused: `ok: false`, error mentions `access=sensors` | `_denied` |
| 5 | `apply_plan 'waypoints=[[4,0],[8,0],[12,0],[16,0],[20,0]]' target_speed=5 ticks=40` | moves straight ahead, no oscillation | weaving: GNSS noise reaching the PID; try `gnss_position_gain` 0.1 in `config.yaml` |
| 6 | `render_bev` | route + route points + ego only (no lanes / actors) | |
| 7 | Drive a stretch using cameras + route points, then let it end or `shutdown` | it stays in its lane using route points + camera; `lateral_offset_m` stays within ~1–2 m on straight road | |

## 5. First benchmark attempts (optional, after §3–4 pass)

- `ACCESS=sensors bash scripts/run_session.sh 2091` (needs Town12), driven with the `drive-carla`
  skill under `benchmark/PROTOCOL.md`. Then `python scripts/summarize_results.py sessions/*`.
- Things worth measuring for the docs:
  - `render_bev` time on Town12 (the lane-graph expansion);
  - wall-clock per `apply_plan` tick batch;
  - whether the 4 default cameras slow the sim noticeably.

## 6. Results log

Append one entry per validation run (date, machine/GPU, CARLA version, what passed, what was fixed
and in which commit). Once §3 and §4 pass:
- remove the "never been run" status at the top of this file;
- drop "not yet run against a live CARLA" from `docs/DESIGN.md` and `README.md` if present;
- update `docs/DESIGN.md` "Known limitations" with anything learned.

_No entries yet._
