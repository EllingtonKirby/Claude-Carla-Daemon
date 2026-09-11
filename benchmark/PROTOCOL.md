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

1. **Fresh session per route**: `scripts/run_session.sh <id>` with the default session directory.
   Never reuse a session dir (the leaderboard would skip the route).
2. **One attempt per route.** Report crashed or aborted sessions as failures (DS 0); do not rerun
   until you get a good one.
3. **Fixed settings:** `TM_SEED=0`, the default `carla_probe/config.yaml` apart from `access`,
   `AGENT_MAX_TICKS` unset (the stock 4000-tick, 200 s budget). Report any deviation next to the
   result.
4. **Probe protocol only.** No reading route XMLs, scenario definitions or simulator code, no editing
   code or config during a run, and no human hints.
5. **Sensors track: no side channels.** The session directory holds privileged data
   (`evaluator.log`, `carla.log`, `result.json`). While driving, open only the files whose paths the
   daemon returns (`get_camera`, `render_bev`, `dump_frame`). Reading logs is allowed only to
   diagnose a launch failure before the first driving command, or after the route has ended.
6. **No `shutdown`** unless the session is unrecoverable; it scores as an agent failure.
7. Pausing is free: thinking time between commands does not consume route time.

## Reporting

```bash
python scripts/summarize_results.py sessions/<run_1> sessions/<run_2> ...
```

The summary groups means by track. Report for each route: DS (driving score), RC (route
completion), IS (infraction penalty), status, scored infractions, plus `cmds` (commands sent) and
`wall s` (session wall-clock time). Report the mean DS / RC / IS per track. Include the track, the
model name, the repo commit (in each `session.json`) and the CARLA version.

**Caveat on reproducibility:** the traffic manager is seeded, but CARLA traffic is not bit-exact
across runs, and traffic *reacts to the ego*, so different driving produces different traffic.
Expect run-to-run variance; for model comparisons, run several attempts per route and report the
spread.
