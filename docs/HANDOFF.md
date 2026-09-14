# Handoff: set up, validate, run and record an eval

This is the one document to read before operating this repo. It covers what the repo is for (§1),
setup (§2), validating an install (§3), running an eval (§4), what a run records and how to
analyze it (§5), and troubleshooting (§6). Deeper references: `docs/OPERATOR_GUIDE.md` (the
driver's manual), `docs/DESIGN.md` (internals) and `benchmark/PROTOCOL.md` (benchmark rules).

**Two roles.** The *operator* (you, reading this) sets up machines, launches runs and collects
results. The *driver* is the LLM agent being evaluated. A driver under evaluation gets only the
driver prompt, `docs/OPERATOR_GUIDE.md`, the `drive-carla` skill and `benchmark/PROTOCOL.md`, plus
whatever the daemon returns. Never pass it anything else about a route: not its scenario, its
contents or past results on it. Don't record route-specific findings in the repo either, since
drivers can read it. Keep run results in `sessions/` (gitignored) or outside the repo.

## 1. What this repo is for

It measures how well an LLM drives a car in closed loop, scored by an unmodified driving benchmark.

- **Simulator and scoring.** [CARLA](https://carla.org) 0.9.15 runs a
  [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) route: a fixed path through a town
  with traffic and one scripted scenario. The Bench2Drive leaderboard scores the drive: **DS**
  (driving score) = **RC** (route completion, %) × **IS** (infraction penalty, a product of
  per-infraction multipliers).
- **The daemon.** `carla_probe/agent.py` is the leaderboard's agent. The leaderboard steps CARLA
  synchronously at 20 Hz and calls the agent every tick, and the agent blocks on a TCP socket until
  it gets a command. So the world is **frozen while the driver thinks**, and time advances only
  through the driver's commands (mainly `apply_plan`: ego-frame waypoints tracked by a PID
  controller for N ticks). A route has a 4000-tick (200 s) budget.
- **The driver** talks to the daemon with `python -m carla_probe.client CMD key=value ...`
  (one JSON reply per command). No learned policy is involved: every control decision is the
  driver's.
- **Two tracks**, enforced by the daemon and never averaged together:
  - `sensors`: only inputs a Bench2Drive SENSORS-track agent gets (cameras, speedometer, IMU,
    GNSS, the sparse route plan). The main benchmark.
  - `privileged`: the ground-truth scene (actors, lanes, lights, signs, live score). Useful for
    development and as an upper bound; comparable only to privileged/expert agents.
- **Route sets** (`benchmark/routes/`): `smoke.txt` (install check, not a benchmark),
  `standard.txt` (the pinned single-route benchmark) and `dev10.txt` (a broader 10-route set).
  Route ids map to `routes/bench2drive220/route_<id>.xml`, which drivers must not read.

## 2. Setup

**Machine:** Linux, an NVIDIA GPU with ≥ 8 GB free VRAM per concurrent session, ≈ 30 GB disk for
CARLA, internet for setup; `git`, `patch`, `wget`, and `uv` (or `python3.10`); `ffmpeg` with libx264 for videos. The system `python3`
may be old; the scripts don't rely on it beyond stdlib basics.

```bash
# 1. CARLA 0.9.15 + the Town11-15 maps. If a 0.9.15 install already exists, point CARLA_ROOT at it
#    (the script then only checks it). Town12/13 are needed by most benchmark routes.
#    LOOK FOR AN EXISTING INSTALL FIRST (e.g. /home/carla, $HOME/carla): as of 2026-09 the upstream
#    download in setup_carla.sh FAILS -- tiny.carla.org redirects to a BunnyCDN host that answers
#    "Domain suspended or not configured" (403), and the carla-releases S3 bucket 403s too. Check
#    for CarlaUE4.sh and CarlaUE4/Content/Carla/Maps/Town12/ before assuming you must download.
export CARLA_ROOT=$HOME/carla          # env.sh defaults to $HOME/carla; export it in every shell otherwise
bash scripts/setup_carla.sh

# 2. Fetch Bench2Drive 0.0.4 into third_party/, apply patches/bench2drive-0.0.4-probe.patch,
#    split its 220 routes into routes/bench2drive220/.
bash scripts/setup_bench2drive.sh

# 3. Python 3.10 venv (.venv) with pinned deps, and PYTHONPATH for the leaderboard. Works from bash
#    or zsh. The first run installs; later runs are a fast no-op.
source scripts/env.sh

# 4. Offline tests (unit + fake-simulator sessions for both tracks; no GPU needed).
pytest
```

**Pass:** `env.sh` prints `env ready` (the first run also lists `ok: carla_probe.agent`), `pytest`
passes, `routes/bench2drive220/route_25378.xml` exists, and
`ls $CARLA_ROOT/CarlaUE4/Content/Carla/Maps/Town12` shows files.

**A `.venv` whose interpreter has vanished** looks like a missing venv but is not: `uv` creates it
with a *managed* CPython under `~/.local/share/uv/python/`, and `.venv/bin/python` is a symlink into
that. If the managed runtime is removed (or the home directory moves between machines), the symlink
dangles while `.venv/lib/.../site-packages` stays fully populated, and `env.sh` tries to rebuild from
scratch. Don't: reinstall `uv` and run `uv python install 3.10`, which restores the exact path the
symlink expects, and the existing venv works again with no package downloads.

**Agent shells:** in Claude Code each Bash call is a fresh shell, so `source scripts/env.sh`
doesn't persist. Either prefix each command with it, or call `.venv/bin/python` directly; the
client is stdlib-only.

**Ports** per session: CARLA RPC `CARLA_PORT` (20000; CARLA also uses the next two ports),
traffic manager `TM_PORT` (`CARLA_PORT`+20000) and the probe socket `PROBE_PORT` (8790).

## 3. Validate an install

Do this on every new machine, and after changing `agent.py`, `config.yaml` or the patch. It
uses the smoke route 25378 (Town03). Here you drive by hand as the operator; this is not an eval.
When a step fails, fix the code, add a regression test where possible
(`tests/test_agent_fakesim.py` mimics most CARLA behaviour), and re-run from that step.

Launch (it blocks until the route ends, so run it in the background) and wait for the load
(≈ 1 min on Town03, ≈ 2 min on Town12):

```bash
source scripts/env.sh
ACCESS=privileged bash scripts/run_session.sh 25378 > /tmp/session.out 2>&1 &
python -m carla_probe.client --wait 600 status
```

Every driving command (`apply_plan`, `set_control`, `hold`, `tick`, `run_until_*`, `shutdown`) needs a
justification, even here: append `why="checklist step N"` to the commands below, or they are refused.
The session dir is printed on the launcher's `session:` line. While paused, the evaluator looking
"hung" is correct. An advance that outlives your shell timeout still completes: reconnect with
`status`, and never re-issue it.

### 3.1 Privileged smoke run

| # | command | pass | if it fails, look at |
|---|---|---|---|
| 1 | `status` | `ok: true`, `step: 0`, `mode: hold`, `access: privileged`, `speed_limit_mps` a number, `score` **not null** | `score: null`: the patched evaluator did not call `set_scenario` (`grep set_scenario third_party/Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py`). Crash before this: `SESSION_DIR/evaluator.log`; sensor validation (camera mounts ≤ 3 m from origin, ≤ 8 cameras), `setup` / `set_global_plan` |
| 2 | `percept` | `nav.route_points_ego[0]` ≈ `[5, ~0]`; `lane` not null with `lane_width_m` ≈ 3–4; nearby vehicles with plausible `fwd`/`left` | route points far away or behind: `set_global_plan` / `_update_route_progress` |
| 3 | `render_bev`, open the PNG | grey lanes under the blue route, ego (red) bottom-centre facing up, vehicles on lanes | lanes offset or rotated: `_local_lanes` / `_yaw_rel_ccw`; no lanes: `CarlaDataProvider.get_map().get_waypoint`; errors about `get_level_bbs`: `_parked_vehicles` |
| 4 | open the 4 JPGs `percept` returned | the road ahead / left / right / behind | camera ids in `config.yaml` |
| 5 | `apply_plan 'waypoints=[[4,0],[8,0],[12,0],[16,0],[20,0]]' target_speed=5 ticks=40` | `result.reason: ticks`, `speed_mps` ≈ 3–5, `mode: plan`, the car moved forward, not sideways | steering sign: `pid.py` (steer + = right); plan anchoring: `geometry.ego_to_world` |
| 6 | `get_state` | traffic lights within 80 m have a state Red / Green / Yellow; stop signs list `trigger_ego` | `_traffic_lights`, `_stop_signs` |
| 7 | `set_control steer=0 throttle=0 brake=1 ticks=40`, then `run_until_event event=stopped max_ticks=200` | reason `stopped`, `speed_mps` < 0.1 | |
| 8 | drive to the end (`percept` + `apply_plan` along `nav.route_points_ego`, 20–40 ticks at a time) | final advance returns `route_end`; `result.json` has a record; the launcher **exits by itself with status 0** and prints a summary table | launcher never exits: the teardown watcher in `run_session.sh`; no `probe_done.json`: `ProbeAgent.destroy` |
| 9 | after the launcher exits: `python tools/replay_sim.py SESSION_DIR frames --from-step 100 --to-step 100 --view front` | the image matches the driver's view at step 100 (compare with `percept` output or `camera_log/`); `score_events.jsonl` rows match `result.json` infractions; `decisions.txt` lists every driving command with its `why` and outcome | no `*_rep0.log`: `SIM_RECORD` / the evaluator's `--record`; no `*_rep0.json`: the patch (re-run `setup_bench2drive.sh` after deleting `third_party/Bench2Drive`); misaligned replay: `replay_sim.Session.time_of_frame` |

Also check that `commands.jsonl` and `trajectory.jsonl` in the session dir are populated.

### 3.2 Sensors smoke run

New session: `ACCESS=sensors bash scripts/run_session.sh 25378 > /tmp/session.out 2>&1 &`.

| # | command | pass | if it fails, look at |
|---|---|---|---|
| 1 | `status` | `access: sensors`; **no** `score`, `speed_limit_mps` or `collision_contacts` keys | |
| 2 | `get_state` | `pose_estimate.x/y` within ~1 m of the privileged run's start (`trajectory.jsonl` step 0 of both sessions), **not ~1e5**; `yaw_deg` within a few degrees of it | x/y: `geometry.solve_geo_reference` / `gps_to_world`, GPS layout `[lat, lon, alt]`. Yaw off by 90°/180° or mirrored: `geometry.compass_to_yaw_deg`, IMU layout (compass last) |
| 3 | `percept` | `nav.route_points_ego[0]` within ~1–2 m of `[5, 0]` | yaw conversion (step 2) |
| 4 | `get_score` | refused: `ok: false`, mentions `access=sensors` | `_denied` |
| 5 | `apply_plan 'waypoints=[[4,0],[8,0],[12,0],[16,0],[20,0]]' target_speed=5 ticks=40` | straight ahead, no weaving (`steer` in `trajectory.jsonl` stays small) | GNSS noise reaching the PID: `gnss_position_gain` |
| 6 | `render_bev` | route + route points + ego only (no lanes / actors) | |
| 7 | drive to the end with route points + cameras | `lateral_offset_m` stays within ~1 m on straight road; `route_end`; launcher exits 0 | |

Between sessions, if a CARLA or evaluator process is left behind: `bash scripts/clean_carla.sh 20000`.

## 4. Run an eval

### 4.1 Rules (summary of `benchmark/PROTOCOL.md`; that file is authoritative)

- One fresh session per route, one attempt; crashed or aborted runs count as failures (DS 0).
- Every driving command carries `why="..."`, which the daemon enforces and logs. Justifications are
  reviewed as part of the result (§5.3).
- Fixed settings: `TM_SEED=0`, default `carla_probe/config.yaml` except `access`, `AGENT_MAX_TICKS`
  unset. Report any deviation.
- The driver harness is part of the definition. `DRIVER=drive_loop` (default) is the bespoke loop;
  `DRIVER=claude_p` is the original `claude -p` driver. Their numbers are separate baselines, never
  averaged. For `drive_loop`, report `PROMPT_VARIANT` too (`baseline` is the headline; `guided` is
  an intervention under test).
- The driver uses only the probe protocol: no route XMLs, scenario or simulator code, logs, config
  edits or human hints. On the sensors track it may open only files whose paths the daemon returns.
- Traffic is seeded but not bit-exact, and it reacts to the ego: run several attempts per route
  when comparing models.

### 4.2 Procedure: `scripts/run_eval.sh`

```bash
source scripts/env.sh
bash scripts/run_eval.sh <ROUTE_ID> <sensors|privileged> <MODEL> > /tmp/eval_<ROUTE_ID>.out 2>&1 &
# e.g.  bash scripts/run_eval.sh 2091 sensors claude-opus-5 > /tmp/eval_2091.out 2>&1 &
```

It blocks until the route is scored, so run it in the background and read its output file for
progress. `drive_loop` needs Anthropic API credentials in the environment (`ANTHROPIC_API_KEY`, or
an `ant auth login` profile); `run_eval.sh` checks the SDK is installed before launching CARLA, but
a missing credential only surfaces on the driver's first request. A Claude Code background task is killed with its session, taking CARLA and the driver
down mid-route with no score. For runs that may outlive your session, detach it:
`setsid nohup bash scripts/run_eval.sh ... > /tmp/eval.out 2>&1 < /dev/null &`. After a kill, run
`bash scripts/clean_carla.sh <CARLA_PORT>`, and report the run as aborted (`benchmark/PROTOCOL.md`
rule 2). Env overrides: `GPU_RANK`, `CARLA_PORT`,
`PROBE_PORT` (for parallel runs), `SIM_RECORD`, `RECORD_CAMERA_EVERY` (§5.1), `LOAD_TIMEOUT_S`,
`DRIVER`, `PROMPT_VARIANT`, `EFFORT`, `KEEP_IMAGES`, `ISOLATION_CHECK=0`, `CHECK_MODEL`. What it
does:

1. Launches `scripts/run_session.sh` into a fresh `sessions/<stamp>_route_<id>_<track>/` and waits
   until the route is loaded and paused, checking the reported `access`.
2. Runs the driver, recording what it is into `driver.json`, and streams its output into the
   session directory (`driver_loop.jsonl`, or `driver_transcript.jsonl` for `claude_p`).
3. If the driver exits while the route is still running, aborts the session with `shutdown` (an
   agent failure) and records `driver_stopped_before_route_end` in `driver.json`. Then it waits
   for scoring and prints the summary.

**The two drivers.** `DRIVER=drive_loop` (default) runs `carla_probe/drive_loop.py`: a purpose-built
loop on the Messages API whose entire action space is the probe tools. It cannot open a file, so
PROTOCOL rule 7 holds by construction, and it needs no sandbox. Its system prompt and tool schemas
are the benchmark definition — snapshotted to the session as `prompt.txt` / `tools.json` and hashed
into `driver.json`.

`DRIVER=claude_p` runs the original `claude -p` driver, kept so the runs made with it stay
reproducible. It is a general-purpose agent with Bash and Read, so `run_eval.sh` additionally builds
it a sandbox (fresh empty directory outside the repo, Claude Code sandbox off, no user/project
settings, tools limited to Bash and Read, `dontAsk` permissions allowing only the probe client,
reads under the session's `frames/` and the three driver documents; session files, routes and
`third_party/` denied) and runs an **isolation check**: a cheap model tries to `cat` / `ls` / `Read`
the session directory, and any success aborts the run. That check matters — in testing, a default
`claude -p` started in the repo could `grep` `result.json` and read `evaluator.log` even with
`--allowedTools` and `--permission-mode dontAsk`, because read-only shell commands and reads inside
the workspace were auto-approved, and a host command sandbox can auto-approve all of Bash. Keep it
on for `claude_p`; it catches Claude Code upgrades that change permission behaviour.

Either way the driver's prompt is part of the benchmark definition: changing it makes results
non-comparable.

Notes:
- **Monitoring** without disturbing the driver: `tail -f $SESSION_DIR/commands.jsonl` (one line per
  command). Don't connect a second client mid-run: the daemon serves one client at a time.
- **Parallel runs:** one GPU per session (`GPU_RANK`), with distinct `CARLA_PORT` (stride ≥ 10),
  `PROBE_PORT` and session dirs. Throughput is dominated by camera rendering, so a second session
  on the same GPU slows both.
- **Interactive (non-benchmark) use:** open Claude Code in the repo and ask it to drive a route. The
  `drive-carla` skill launches the session itself. That driver is *not* isolated, so don't count
  such runs as benchmark results.

## 5. Recording and analysis

### 5.1 What a run records

Everything lands in `SESSION_DIR` (gitignored under `sessions/`). On the sensors track the driver
may open none of it except the images the daemon returns (the eval sandbox enforces this).

| file | written by | contents |
|---|---|---|
| `session.json`, `config.yaml` | launcher | route file, access, TM seed, ports, tick budget, recording settings, repo commit; the exact daemon config |
| `result.json` | leaderboard | **official scores**: DS / RC / IS, status, every scored infraction with its message |
| `RouteScenario_<id>_rep0.log` | CARLA recorder (`SIM_RECORD=1`, default) | **replayable simulation log**: every actor's state on every tick, from before the first agent step to the end. Replay with `tools/replay_sim.py` (§5.2) |
| `RouteScenario_<id>_rep0.json` | patched evaluator | world frame at recorder start and tick length, which map agent steps to replay time |
| `score_events.jsonl` | daemon, **both tracks** | **infraction timing**: one row per scoring event as it appears: `step`, sim time `t`, `frame_seen`, and the event's `type`, own CARLA `frame` and `message`. Criteria that update an event in place (e.g. `outside_route_lanes` percentages) log each revision; `result.json` keeps only the last. Write-only: never shown to the driver |
| `decisions.jsonl`, `decisions.txt` | daemon | **driving decision justifications**: every accepted driving command with `step`, `t`, `frame`, arguments and the driver's `why`, then its outcome (reason, ticks, step, speed). `.txt` is the same, human-readable |
| `commands.jsonl` | daemon | every command (including observations and refused ones) with wall time and step, and `advance_done` rows |
| `trajectory.jsonl` | daemon | per tick: step, sim time, CARLA `frame`, x / y / yaw / speed, mode, steer / throttle / brake. **Sensors track: the GNSS pose estimate**; ground truth is in the recording |
| `frames/frame_<step>/` | daemon | every image the driver requested (`percept`, `render_bev`, `dump_frame`) |
| `camera_log/step_<step>.jpg` | daemon | periodic frames from `record.camera_id`, when `RECORD_CAMERA_EVERY=N` is set (or the driver used `record`) |
| `evaluator.log`, `carla.log`, `launcher.log`, `probe_done.json` | leaderboard, CARLA, launcher, daemon | per-criterion table at route end, crashes, launch progress, final status |
| `driver_loop.jsonl` | `carla_probe/drive_loop.py` | one row per tool call from the bespoke driver loop: tool, `why`, outcome, step, route progress, and the turn's token usage split into cache reads / writes. Only written by that loop, not by `run_eval.sh` |
| `driver.json`, `prompt.txt`, `settings.json` | `run_eval.sh` | model, Claude Code version, prompt hash, start/end, driver exit code, whether it stopped before the route ended; the exact prompt and sandbox |
| `driver_transcript.jsonl`, `driver_stderr.log` | `run_eval.sh` | the driver's full stream: reasoning text, every tool call and result (images inline as base64), token usage |

Recording knobs (both allowed in benchmark runs, `benchmark/PROTOCOL.md` rule 3):
`SIM_RECORD=0` disables the CARLA recording (on by default; a few hundred KB per simulated minute on
a small town, more with dense traffic); `RECORD_CAMERA_EVERY=N` logs a camera frame every N ticks.
Pass them to `run_eval.sh` or `run_session.sh` as environment variables.

**Joins.** `step` is the common key: `decisions.jsonl` / `score_events.jsonl` / `commands.jsonl` /
`trajectory.jsonl` / `frames/frame_<step>/` / `camera_log/step_<step>.jpg`. `trajectory.jsonl`
maps steps to CARLA frames, and `replay_sim.py` maps frames to recording time. Transcript tool calls
are in command order.

### 5.2 Analysis tools

- `python tools/summarize_results.py SESSION_DIR...`: markdown table (or `--json`) of DS / RC / IS,
  status, scored infractions, command count and wall time, with means per track.
- `python tools/replay_sim.py SESSION_DIR info | collisions | frames ...` replays the recording
  on a CARLA server (it starts one on `--port`/`--gpu` if none answers; use a port no live session
  uses, default 20200).
  - `info`: map, duration, and the step ↔ frame ↔ replay-time range.
  - `collisions`: the recorder's collision list (every actor pair, not only scored ones), with
    approximate steps; the recorder prints whole seconds, so use `score_events.jsonl` for exact steps.
  - `frames --from-step A --to-step B [--view chase|front|top] [--every N] [--video]`: renders the
    drive from any viewpoint, with the route's weather, into `SESSION_DIR/replay/`. Each image is
    overlaid with the step, the decision in force (command + `why`) and any scoring event at that
    tick. The `front` view reproduces the driver's `CAM_FRONT`.
- `python tools/recorder_truth.py SESSION_DIR`: decodes the recording into
  `SESSION_DIR/recorder_truth.json` -- every actor's true pose (metres, world frame) on every frame,
  plus each actor's type and role. Needs a CARLA server once; everything downstream is then offline.
  This is the only ground truth for a sensors-track run: `trajectory.jsonl` holds the driver's own
  GNSS *estimate*. Map with `recorder_frame = world_frame - frame_offset` (`frame_offset` is in the
  file, and equals the recording's `start_world_frame`). The tool reports the driver's pose error
  against truth as a sanity check; a large value means the offset is wrong, not that GNSS is bad.
- `python tools/diagnose_alignment.py SESSION_DIR --from-step A --to-step B`: verifies that the
  frame `replay_sim.py` labels step S really is the world at step S, by replaying and scoring the
  ego's position against `recorder_truth.json` over a range of lags. Prints the best lag and the
  mean error; aligned means best lag 0 at ~0.00 m. Run it after touching the replay path.
- `python tools/make_video.py SESSION_DIR`: mp4 from `camera_log/`.
- Videos (`--video`, `make_video.py`) are H.264 (`yuv420p`) encoded by ffmpeg, so they play in browsers and
  common players. Replays run at real time (10 fps with `--every 2`).

### 5.3 Reviewing decisions

To judge whether a driver's decisions fit the scene:

1. Read `decisions.txt` alongside `score_events.jsonl`. Every infraction has a step, so the decision
   in force when it happened is the last `[i] step ...` entry at or before that step.
2. For each decision worth checking, render its window, e.g.
   `replay_sim.py SESSION_DIR frames --from-step <decision step> --to-step <result step> --view chase --every 5`.
   Use `--view front` for what the driver's main camera saw, and `chase` or `top` for context it
   couldn't see.
3. Compare the `why` with the scene. Did the driver see what it claims (the car ahead, the light,
   the gap)? Does the command follow from it (speed, path, duration)? Did the outcome match what the
   justification expected?
4. On the sensors track, also check what the driver actually looked at: the images under `frames/`
   and the transcript's Read calls at that step.

## 6. Troubleshooting

- **Replays drift out of sync with their overlay.** `client.replay_file()` silently turns
  synchronous mode OFF (settings come back `synchronous_mode=False, fixed_delta_seconds=None`), so
  the world advances at wall-clock rate (~0.023 s/tick observed) instead of `fixed_delta_seconds`.
  Anything counting ticks as recorded frames then drifts without bound -- tens of metres of ego
  travel by mid-route. `replay_sim.py` re-applies the settings straight after `replay_file`; keep
  that, and re-check with `diagnose_alignment.py` after any change to the replay loop.
- **Parallel replays or sessions kill each other at startup.** A CARLA server binds THREE
  consecutive ports (rpc, rpc+1, rpc+2). Ports spaced less than 4 apart mean one lane's rpc port is
  its neighbour's secondary port, and adjacent servers die -- every other lane fails, and the
  surviving set flips between attempts. Space `--port` / `CARLA_PORT` by at least 4 (10 is
  comfortable). A collision can also destabilise a server that is already mid-render, not just
  block a new one.
- **`pgrep -f <pattern>` matches the shell running it** when the pattern appears in its own command
  line, so a cleanup or wait loop can kill itself or never exit. Assemble such patterns at runtime.
- **`status` never answers.** Look at `SESSION_DIR/carla.log` (CARLA died, e.g. no GPU or Vulkan)
  and `evaluator.log` (agent setup errors, missing map). Town12/13 load for ~2 min.
- **`Address already in use` / CARLA won't bind.** Another session holds the ports:
  `bash scripts/clean_carla.sh <CARLA_PORT>`. The traffic manager port (`CARLA_PORT`+20000) must be
  free too.
- **`env.sh` says patched Bench2Drive not found** although setup ran: make sure it is *sourced*
  from the repo's `scripts/` (it resolves the repo from its own path).
- **Pinned dependencies fail to install** (`numpy==1.26.4`, `opencv-python-headless==4.5.4.58`,
  `setuptools<81`, `carla==0.9.15` cp310 wheel): relax a pin only if it doesn't install, and
  record the change.
- **Launcher never exits after `route_end`.** The evaluator can hang in teardown after writing
  `result.json`. The launcher kills it once `probe_done.json` and the global record exist, or
  after 120 s; `scripts/clean_carla.sh` is the manual fallback.
- **`pkill -f` / `clean_carla.sh` killed your own shell.** `-f` matches full command lines, so
  never run it from a shell command that itself contains the pattern (e.g. inside `bash -c "..."`).
- **Route is skipped instantly.** The session dir already had a `result.json`; always use a fresh
  one.
