# Benchmark protocol

What a result from this repo means, and the rules that keep results comparable.

## Task

An LLM agent drives one route, end to end, through the probe daemon. No learned driving policy is
involved: every control decision comes from the agent's commands. The leaderboard scores the route
exactly as it scores any other agent.

## Benchmarks

Two, selected with `BENCHMARK=` and recorded in every `session.json` and `driver.json`. They are
**different benchmarks** and their scores must never be pooled, averaged or directly compared.

| | `bench2drive` (default) | `fail2drive` |
|---|---|---|
| what it measures | closed-loop driving across 220 routes and 44 scenarios | closed-loop **generalization**: 100 route pairs, each `Generalization_*` route a shifted version of an in-distribution `Base_*` reference |
| headline number | mean DS | the **Base → Generalization drop** in the harmonic mean of DS and success rate |
| tree | fetched + patched, `third_party/Bench2Drive` | vendored, `third_party/fail2drive` |
| simulator | stock CARLA 0.9.15 | **its own CARLA build**, with 30 novel assets baked in |
| towns | 7 towns | Town13 only (a large map) |
| route budget | hard cap of 4000 ticks (200 s at 20 Hz) | no tick cap; the route ends on its own `RouteTimeoutBehavior` |
| route succeeds if | every infraction list empty except `min_speed_infractions` | every infraction list empty except `min_speed_infractions` **and** `outside_route_lanes` |
| sensors-track route plan | a point at most every 50 m | a point at most every 200 m (see below) |
| report with | `tools/summarize_results.py` | `tools/f2d_score.py` |

The last two rows are the ones that catch people out. The same drive scores differently under the
two success rules, and on the sensors track a Fail2Drive driver is given a **much coarser** route
plan than a Bench2Drive driver: that is stock Fail2Drive and what every model on its public
leaderboard receives, and it matters because F2D routes are short (median 211 m, 92 of 200 under
200 m), so a typical route's plan is little more than its start and end.

Running F2D routes against a stock CARLA 0.9.15 does **not** error: CARLA substitutes default
blueprints, so a scenario that should spawn a cow spawns a car and the run measures nothing it
claims to. `scripts/setup_fail2drive.sh` and `scripts/env.sh` check the novel-asset packs for
files; never bypass that check to make a run start.

**Fail2Drive is a held-out test set.** Its routes, scenario definitions and novel assets must never
be used to train or tune a driver. They are vendored in this repo so the benchmark is reproducible;
the driver harnesses cannot reach them (`drive_loop` has no filesystem access at all, and
`claude_p`'s sandbox denies `third_party/fail2drive/**`).

## Tracks

The daemon enforces what the driver can see (`access` in the session's `config.yaml`, set with
`ACCESS=` on `scripts/run_session.sh`). The two tracks are different benchmarks: never average
across them.

| track | launch | the driver sees | comparable to |
|---|---|---|---|
| **sensors** | `ACCESS=sensors bash scripts/run_session.sh <route>` | cameras, speedometer, IMU, GNSS, and the leaderboard's route plan (sparse GPS waypoints + turn commands) | SENSORS-track agents on the same benchmark |
| **privileged** | `ACCESS=privileged bash scripts/run_session.sh <route>` | ground truth: actors, lanes, traffic lights, stop signs, speed limit, live score and infractions | privileged / expert agents only |

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

**Fail2Drive.** Routes are named `<Split>_<Scenario>_<id>` in `third_party/fail2drive/fail2drive_split/`;
`run_session.sh` and `run_eval.sh` accept either the name or the bare id. Every set is whole pairs,
because a route without its twin cannot produce a drop.

| set | file | routes | purpose |
|---|---|---:|---|
| f2d_pair | `benchmark/routes/f2d_pair.txt` | 2 | plumbing check: the install, the novel-asset CARLA build and the pair scoring. Not a result. |
| f2d_smoke | `benchmark/routes/f2d_smoke.txt` | 8 | one pair per scenario class; exercises every class table. Still not a result. |
| f2d_full | `benchmark/routes/f2d_full.txt` | 200 | the benchmark |

A Fail2Drive number quoted from anything less than a substantial, balanced share of the 200 routes
is not a Fail2Drive score. `tools/f2d_score.py` always prints its coverage
(`n/100 Base + n/100 Generalization, n/100 complete pairs`); quote that line with the number.

## Rules

1. **Fresh session per route, confined driver**: launch benchmark runs with
   `scripts/run_eval.sh <id> <track> <model>` (`docs/HANDOFF.md` §4), which uses a fresh session
   directory and a driver whose reach is limited to the probe protocol. Never reuse a session dir
   (the leaderboard would skip the route). Runs driven interactively are not benchmark results.
2. **The driver harness is part of the benchmark definition.** `run_eval.sh` offers two, selected
   with `DRIVER=`. They are different configurations: report which one, and never average or
   directly compare their scores.
   - `drive_loop` (default): `carla_probe/drive_loop.py`, a loop on the Messages API whose entire
     action space is the probe tools. It has no filesystem access, so rule 7 holds by construction
     rather than by enforcement. Its system prompt and tool schemas ARE the definition: both are
     snapshotted into the session (`prompt.txt`, `tools.json`) and hashed into `driver.json` as
     `definition_sha256`. Report `PROMPT_VARIANT` -- `baseline` is the headline configuration;
     `guided` adds driving guidance and is an intervention under test, not the reference.
   - `claude_p`: the original `claude -p` driver, kept so runs made with it stay reproducible.
     Being a general-purpose agent with Bash and Read, it needs the driver sandbox and the
     pre-flight isolation check, both of which `run_eval.sh` applies.
3. **One attempt per route.** Report crashed or aborted sessions as failures (DS 0); do not rerun
   until you get a good one.
4. **Fixed settings:** `TM_SEED=0`, the default `carla_probe/config.yaml` apart from `access`,
   and `AGENT_MAX_TICKS`, `AGENT_BLOCKED_MIN_SPEED`, `AGENT_BLOCKED_MAX_TIME` and
   `AGENT_SCENARIO_TIMEOUT` all unset -- which is the stock budget of whichever benchmark is
   running (Bench2Drive: a 4000-tick, 200 s cap; Fail2Drive: no tick cap at all). Report any
   deviation next to the result. Recording knobs that don't change what the driver sees are allowed: `SIM_RECORD` (on by
   default) and `RECORD_CAMERA_EVERY` (periodic camera frames to `camera_log/`).
5. **Justified decisions:** every driving command carries `why="..."` (enforced by the default
   `require_justification: true`). Justifications are part of the result and are reviewed against
   the replayed scene.
6. **Probe protocol only.** No reading route XMLs, scenario definitions or simulator code, no editing
   code or config during a run, and no human hints. On Fail2Drive this covers `third_party/fail2drive/`,
   whose vendored routes and scenarios are the held-out test set itself.
7. **Sensors track: no side channels.** The session directory holds privileged data
   (`evaluator.log`, `carla.log`, `result.json`). While driving, open only the files whose paths the
   daemon returns (`get_camera`, `render_bev`, `dump_frame`). Reading logs is allowed only to
   diagnose a launch failure before the first driving command, or after the route has ended.
8. **No `shutdown`** unless the session is unrecoverable; it scores as an agent failure.
9. Pausing is free: thinking time between commands does not consume route time.

## Reporting

```bash
python tools/summarize_results.py sessions/<run_1> sessions/<run_2> ...     # both benchmarks
python tools/f2d_score.py sessions/f2d_*                                   # Fail2Drive only
```

The summary groups means by benchmark *and* track. Report for each route: DS (driving score), RC (route
completion), IS (infraction penalty), status, scored infractions, plus `cmds` (commands sent) and
`wall s` (session wall-clock time). Report the mean DS / RC / IS per track. Include the track, the
model name, the repo commit (in each `session.json`) and the CARLA version, plus the driver
harness and, for `drive_loop`, the prompt variant and `definition_sha256` (all in `driver.json`) --
two runs of the same model under different harnesses are not the same measurement. Report the
benchmark too; it is in `session.json` and `driver.json`.

**Reporting Fail2Drive.** The per-route table is not the result. Report, from
`tools/f2d_score.py`: the coverage line, Base and Generalization DS / SR / HM, and the **drop** in
each. A drop needs both sides, so a partial run has no headline number -- the tool prints `n/a`
rather than a drop, and that `n/a` is the honest answer, not a missing value to fill in with the
per-split numbers. Per-class and per-scenario tables say *where* the model breaks; quote them with
their `n`, which is often small enough that one route moves the row.

**Caveat on reproducibility:** the traffic manager is seeded, but CARLA traffic is not bit-exact
across runs, and traffic *reacts to the ego*, so different driving produces different traffic.
Expect run-to-run variance; for model comparisons, run several attempts per route and report the
spread.
