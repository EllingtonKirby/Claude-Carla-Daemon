# Benchmark protocol

What a result from this repo means, and the rules that keep results comparable.

## Task

An LLM agent drives one route, end to end, through the probe daemon. No learned driving policy is
involved: every control decision comes from the agent's commands. The leaderboard scores the route
exactly as it scores any other agent.

## Benchmarks

Two, selected with `BENCHMARK=` and recorded in every `session.json` and `driver.json`:
`bench2drive` (the default) and `fail2drive`. They are **different benchmarks** -- different
routes, simulator build, route budget and success rule -- and their scores must never be pooled,
averaged or directly compared. Everything specific to the second one, including how it is run and
reported, is in `benchmark/FAIL2DRIVE.md`.

That file sits deliberately outside what a driver may read, because Fail2Drive is a **held-out test
set**: its routes, scenario definitions and assets are vendored in this repo so the benchmark is
reproducible, not so that anything may be read off them (rule 6). The rules below are written to
hold on either benchmark without naming what is in either.

## Tracks

The daemon enforces what the driver can see (`access` in the session's `config.yaml`, set with
`ACCESS=` on `scripts/run_session.sh`). The two tracks are different benchmarks: never average
across them.

| track | launch | the driver sees | comparable to |
|---|---|---|---|
| **sensors** | `ACCESS=sensors bash scripts/run_session.sh <route>` | cameras, speedometer, IMU, GNSS, and the leaderboard's route plan (sparse GPS waypoints + turn commands) | SENSORS-track agents on the same benchmark |
| **privileged** | `ACCESS=privileged bash scripts/run_session.sh <route>` | ground truth: actors, lanes, traffic lights, stop signs, speed limit, live score and infractions | privileged / expert agents only |
| **dense** | `ACCESS=dense bash scripts/run_session.sh <route>` | the sensors view, plus ground-truth depth and instance segmentation per camera | nothing published: an upper bound for pixel-level perception |

Both benchmarks offer both tracks, so a result is a (benchmark, track) pair: four buckets, none of
which may be averaged with another. The sensors track uses only inputs a SENSORS-track agent
receives. The daemon refuses
everything else: scene queries return only ego sensor data, and `get_score`, collision/infraction
events and `stop_on_collision` are rejected. One caveat: the driver still chooses *when* to look
(it may request camera frames at any paused tick). That is more than a fixed-rate policy has, but
it adds no information beyond what the sensors show.

## Route sets

**Bench2Drive.** Route ids map to `routes/bench2drive220/route_<id>.xml`.

| set | file | purpose |
|---|---|---|
| smoke | `benchmark/routes/smoke.txt` | install check (Town03, no large map) |
| standard | `benchmark/routes/standard.txt` | the pinned single-route benchmark (route 2091) |
| dev10 | `benchmark/routes/dev10.txt` | broader 10-route set |

**Fail2Drive.** Named routes, grouped into three sets; see `benchmark/FAIL2DRIVE.md`.

## Rules

1. **Fresh session per route, confined driver**: launch benchmark runs with
   `scripts/run_eval.sh <id> <track> <model>` (`docs/HANDOFF.md` §4), which uses a fresh session
   directory and a driver whose reach is limited to the probe protocol. Never reuse a session dir
   (the leaderboard would skip the route). Runs driven interactively are not benchmark results.
2. **The driver harness is part of the benchmark definition.** The driver is `claude -p` with the
   canonical prompt in `run_eval.sh`, which is hashed into `driver.json` as `prompt_sha256`:
   changing the prompt, the documents it points at, or the daemon's command surface makes results
   non-comparable, so report the repo commit alongside every score. It runs on whatever the
   `claude` CLI is logged in to — normally a Claude subscription, in which case the reported
   `total_cost_usd` is an API-equivalent figure rather than a bill. Being a general-purpose agent
   with Bash and Read, it needs the driver sandbox and the pre-flight isolation check, both of
   which `run_eval.sh` applies.
3. **One attempt per route.** Report crashed or aborted sessions as failures (DS 0); do not rerun
   until you get a good one.
4. **Fixed settings:** `TM_SEED=0`, the default `carla_probe/config.yaml` apart from `access`,
   and `AGENT_MAX_TICKS`, `AGENT_BLOCKED_MIN_SPEED`, `AGENT_BLOCKED_MAX_TIME` and
   `AGENT_SCENARIO_TIMEOUT` all unset -- which leaves whichever benchmark is running on its own
   stock budget (`status.ticks_left` counts it down, and is `null` where there is no tick cap).
   Report any deviation next to the result. Recording knobs that don't change what the driver sees are allowed: `SIM_RECORD` (on by
   default) and `RECORD_CAMERA_EVERY` (periodic camera frames to `camera_log/`).
5. **Justified decisions:** every driving command carries `why="..."` (enforced by the default
   `require_justification: true`). Justifications are part of the result and are reviewed against
   the replayed scene.
6. **Probe protocol only.** No reading route XMLs, scenario definitions or simulator code -- which
   includes the vendored benchmark trees under `third_party/` -- no editing the repo's code or
   config during a run, and no human hints. With the code tool (`CODE_TOOL=on`, OPERATOR_GUIDE
   §8b) the driver may write and run its own scripts in its code dir; that is a harness setting,
   recorded in `driver.json`, and runs with and without it are different measurements. A perception
   service (`PERCEPTION_ADDR`) sees only the cameras, so it keeps a sensors run eligible.
7. **Sensors track: no side channels.** The session directory holds privileged data
   (`evaluator.log`, `carla.log`, `result.json`). While driving, open only the files whose paths the
   daemon returns (`percept`, `render_bev`, `dump_frame`). Reading logs is allowed only to
   diagnose a launch failure before the first driving command, or after the route has ended.
8. **No `shutdown`** unless the session is unrecoverable; it scores as an agent failure.
9. Pausing is free: thinking time between commands does not consume route time.

## Reporting

```bash
python tools/summarize_results.py sessions/<run_1> sessions/<run_2> ...     # both benchmarks
```

The summary groups means by benchmark *and* track. Report for each route: DS (driving score), RC (route
completion), IS (infraction penalty), status, scored infractions, plus `cmds` (commands sent) and
`wall s` (session wall-clock time). Report the mean DS / RC / IS per track. Include the track, the
model name, the repo commit (in each `session.json`) and the CARLA version, plus the driver
`prompt_sha256` from `driver.json` -- two runs of the same model under different harnesses are not
the same measurement. Report the benchmark too; it is in `session.json` and `driver.json`.

Fail2Drive is reported differently; see `benchmark/FAIL2DRIVE.md`.

**Caveat on reproducibility:** the traffic manager is seeded, but CARLA traffic is not bit-exact
across runs, and traffic *reacts to the ego*, so different driving produces different traffic.
Expect run-to-run variance; for model comparisons, run several attempts per route and report the
spread.
