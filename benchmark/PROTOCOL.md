# Benchmark protocol

What a result from this repo means, and the rules that keep results comparable.

## Task

An LLM agent drives one Bench2Drive route, end to end, through the probe daemon. No learned
driving policy is involved: every control decision comes from the agent's commands. The
leaderboard scores the route exactly as it scores any other agent.

## Tracks

The daemon enforces what the driver can see (`access` in the session's `config.yaml`, set with
`ACCESS=` on `scripts/run_session.sh`). The two tracks are different benchmarks: never average
across them.

| track | launch | the driver sees | comparable to |
|---|---|---|---|
| **sensors** | `ACCESS=sensors bash scripts/run_session.sh <id>` | cameras, speedometer, IMU, GNSS, and the leaderboard's route plan (sparse GPS waypoints + turn commands) | Bench2Drive SENSORS-track agents |
| **privileged** | `ACCESS=privileged bash scripts/run_session.sh <id>` | ground truth: actors, lanes, traffic lights, stop signs, speed limit, live score and infractions | privileged / expert agents only |

The sensors track uses only inputs a Bench2Drive-eligible agent receives. The daemon refuses
everything else: scene queries return only ego sensor data, and `get_score`, collision/infraction
events and `stop_on_collision` are rejected. One caveat: the driver still chooses *when* to look
(it may request camera frames at any paused tick). That is more than a fixed-rate policy has, but
it adds no information beyond what the sensors show.

## Route sets

| set | file | purpose |
|---|---|---|
| smoke | `benchmark/routes/smoke.txt` | install check (Town03, no large map) |
| standard | `benchmark/routes/standard.txt` | the pinned single-route benchmark (route 2091) |
| dev10 | `benchmark/routes/dev10.txt` | broader 10-route set |

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
   `AGENT_MAX_TICKS` unset (the stock 4000-tick, 200 s budget). Report any deviation next to the
   result. Recording knobs that don't change what the driver sees are allowed: `SIM_RECORD` (on by
   default) and `RECORD_CAMERA_EVERY` (periodic camera frames to `camera_log/`).
5. **Justified decisions:** every driving command carries `why="..."` (enforced by the default
   `require_justification: true`). Justifications are part of the result and are reviewed against
   the replayed scene.
6. **Probe protocol only.** No reading route XMLs, scenario definitions or simulator code, no editing
   code or config during a run, and no human hints.
7. **Sensors track: no side channels.** The session directory holds privileged data
   (`evaluator.log`, `carla.log`, `result.json`). While driving, open only the files whose paths the
   daemon returns (`percept`, `render_bev`, `dump_frame`). Reading logs is allowed only to
   diagnose a launch failure before the first driving command, or after the route has ended.
8. **No `shutdown`** unless the session is unrecoverable; it scores as an agent failure.
9. Pausing is free: thinking time between commands does not consume route time.

## Reporting

```bash
python tools/summarize_results.py sessions/<run_1> sessions/<run_2> ...
```

The summary groups means by track. Report for each route: DS (driving score), RC (route
completion), IS (infraction penalty), status, scored infractions, plus `cmds` (commands sent) and
`wall s` (session wall-clock time). Report the mean DS / RC / IS per track. Include the track, the
model name, the repo commit (in each `session.json`) and the CARLA version, plus the driver
harness and, for `drive_loop`, the prompt variant and `definition_sha256` (all in `driver.json`) --
two runs of the same model under different harnesses are not the same measurement.

**Caveat on reproducibility:** the traffic manager is seeded, but CARLA traffic is not bit-exact
across runs, and traffic *reacts to the ego*, so different driving produces different traffic.
Expect run-to-run variance; for model comparisons, run several attempts per route and report the
spread.
