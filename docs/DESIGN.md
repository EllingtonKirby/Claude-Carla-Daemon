# Design notes

## The agent is the hook

`leaderboard_evaluator.py` runs CARLA in **synchronous mode** and calls the agent's `run_step()`
once per tick; the world advances only after it returns. So an agent that blocks on a socket
**pauses the whole simulation**. There is no separate service racing the tick loop, and no
leaderboard logic had to change to make pausing work. The daemon lives inside the agent process:

```
CarlaUE4 (GPU, --carla-rpc-port)
   ▲ rpc
leaderboard_evaluator.py  (sync mode, owns the tick loop, scores the route)
   │ run_step() every tick
ProbeAgent  carla_probe/agent.py
   ├─ CommandServer: JSON lines over TCP, blocks run_step while paused
   └─ control: hold | plan (PID tracks client waypoints) | raw
   ▲ TCP
carla_probe.client (stdlib CLI / REPL / ProbeClient class): Claude or a human
```

One evaluator process drives one route. The route and scenario are bound at startup.

## Pause semantics

- The sim pauses whenever no time-advancing command is pending, including on tick 0.
- Advancing commands (`tick`, `run_until_*`, `apply_plan`, `set_control`, `hold ticks=`) register
  an advance, and `run_step` returns control until the advance's stop condition holds. The single
  reply is then sent from the tick where it stops.
- If the route ends mid-advance, `destroy()` sends the reply with reason `route_end`.
- A dropped client just returns the server to `accept`. Nothing is lost.
- Watchdogs: the agent, simulation and client-timeout watchdogs all ride the leaderboard's
  `--timeout`, which `run_session.sh` sets to one week. The cost is that a genuine CARLA hang is
  yours to notice.

## Access levels

`access` in the config selects what the driver may see. It is enforced in the agent, not by
convention. `_denied()` rejects forbidden requests *before* a command touches any state, and every
view has a separate sensors-only builder instead of a filtered privileged one, so a new privileged
field cannot leak by default.

**privileged.** The ego pose is the hero actor's CARLA transform, and the route is the
leaderboard's dense world-coordinate plan. Scene queries read actors, level bounding boxes
(parked cars), the lane graph, traffic lights and stop signs straight from the simulator. The live
score mirrors `StatisticsManager.compute_route_statistics` over the running scenario's criteria
(the patched evaluator hands the scenario over through `set_scenario`); `result.json` remains
authoritative.

**sensors.** Only Bench2Drive SENSORS-track inputs:
- RGB cameras, speedometer, IMU and GNSS. The leaderboard adds its standard noise to IMU and GNSS.
- The route plan the base agent class keeps: the downsampled GPS plan with its `RoadOption`
  commands.

The ego pose is estimated:
- **Geo reference.** The town's reference is solved from the plan's first point, which the
  leaderboard provides in both GPS and world form (`geometry.solve_geo_reference`).
- **Position.** GNSS fixes are mapped to world coordinates with the inverse of CARLA's GNSS
  projection, then smoothed by a dead-reckoning complementary filter (`gnss_position_gain`).
- **Heading.** Comes from the IMU compass.
- **Route.** Projected through the same conversion, so route and ego share one frame.

The hero actor is never queried, the collision sensor is never attached, and the live score is
never computed for a reply. `render_bev` draws only route, route points, plan and a nominal ego box.

**Write-only analysis logs (both tracks).** `score_events.jsonl` reads the scenario's criteria every
tick in sensors sessions too (`_scenario_events` → `_record_score_events`). Its output goes only to
that file and never into a reply; `test_sensors_session` checks this. `decisions.jsonl` / `.txt`
log the driver's own commands and justifications (`require_justification`).

The original agent recovered the geo reference with `fsolve` on equations that carried a stray
factor of `lat_ref`. The rewrite bisects the latitude equation (it involves `lat_ref` alone) and
solves longitude in closed form. It is round-trip-tested against the forward projection.

What enforcement cannot cover: the session directory still holds privileged files
(`evaluator.log`, `result.json`). On the sensors track, reading them mid-run is forbidden by the
benchmark protocol (rule 6), not blocked technically. In sensors sessions, `trajectory.jsonl` logs
the pose estimate, not ground truth.

## Frames

Every ego-frame quantity is one rigid transform away from CARLA world (`carla_probe/geometry.py`,
unit-tested). Each access level has exactly one world frame: ground truth for privileged, the
GNSS projection for sensors. In the original model-driven daemon, mixing a GPS-derived frame with
CARLA world coordinates (they differ by a large translation) caused real bugs.

## The Bench2Drive patch (`patches/bench2drive-0.0.4-probe.patch`)

Applied to Bench2Drive 0.0.4 by `scripts/setup_bench2drive.sh`. It touches only the MIT-licensed
`leaderboard/` and `scenario_runner/` trees.

| file | change | why |
|---|---|---|
| `leaderboard_evaluator.py` | does not spawn its own CARLA server; no free-port hunting; fail fast on connection errors | the launcher owns the server and ports |
| | `weather.xml` resolved via `LEADERBOARD_ROOT` | stock path is CWD-relative |
| | `agent.destroy(results)`; `agent.set_scenario(route_scenario)` | final record + live score |
| | large-map load (Town12/13) done in async mode, sync restored afterwards | libcarla crash on some hosts when loading large maps in sync mode |
| | world settings are not reset on exit | keeps a reused server in sync mode |
| | with `--record`, writes `<route>_rep<i>.json` (world frame at recorder start, tick length) next to the recording | maps agent steps to replay time (`tools/replay_sim.py`) |
| `scenario_manager.py` | `AGENT_MAX_TICKS` env (default 4000) | analysis runs only; the benchmark uses the default |
| `route_scenario.py` | `AGENT_BLOCKED_MIN_SPEED` / `AGENT_BLOCKED_MAX_TIME` env (defaults 0.1 m/s, 60 s) | analysis runs only |
| `atomic_behaviors.py` | `AGENT_SCENARIO_TIMEOUT` env cap (unset = stock) | analysis runs only |
| `route_parser.py`, `scenario_parser.py` | `getchildren()` → `list()` | removed in Python 3.9 |

With those variables unset, scoring semantics are stock Bench2Drive.

## The vendored Fail2Drive tree (`third_party/fail2drive/`)

Fail2Drive is MIT, so unlike Bench2Drive it is **vendored rather than fetched and patched**: the
probe's changes are an ordinary diff in git history (`git log -p third_party/fail2drive/`) instead of a
patch file. `third_party/fail2drive/VENDORED.md` pins the upstream commit and records what was left out
and why. `scripts/setup_fail2drive.sh` installs the two pieces git cannot carry: the F2D CARLA
build (12.8 GB download, ~30 GB installed) and `scenario_runner/speed_limits/*.npy` (75 MB).

F2D's `scenario_runner` has diverged from Bench2Drive's -- new scenario classes, scenarios embedded
in the route XMLs, a different success rule -- so it is a sibling tree, never a merge. Both forks
ship packages literally named `leaderboard` and `srunner`, and importing the wrong one does **not**
raise; it silently scores the route under the other benchmark's rules. `scripts/env.sh` therefore
*removes* the other fork from `PYTHONPATH` rather than putting this one first.

Changes to the vendored tree, each marked `# probe:` in place:

| file | change | why |
|---|---|---|
| `leaderboard_evaluator.py` | `torch` becomes an optional import | stock F2D imports it at module scope only to seed it; the probe driver is not a torch model and `env.sh` does not install torch |
| | `agent.destroy(results)`; `_register_statistics` returns the record | final record reaches the agent |
| | `agent.set_scenario(route_scenario)` | privileged live score |
| | with `--record`, writes `<route>_rep<i>.json` next to the recording | maps agent steps to replay time (`tools/replay_sim.py`) |
| | the world load drops to async mode, restoring sync right after | libcarla crashes loading a Large Map in sync mode on some hosts, and **all 200 F2D routes are Town13** |
| | world settings are not reset on exit | keeps a reused server in sync mode |
| | accepts `--gpu-rank`, ignored | B2D's evaluator spawns its own CARLA and needs it; F2D's does not. Keeps `run_session.sh` one code path |
| `autonomous_agent.py` | back-ports `get_hero()` from B2D's fork | `agent.py` resolves the ego through it on the privileged track |
| `route_scenario.py` | `AGENT_BLOCKED_MIN_SPEED` / `AGENT_BLOCKED_MAX_TIME` env (defaults 0.1 m/s, 180 s = F2D's own values) | analysis runs only |
| `scenario_manager.py` | `AGENT_MAX_TICKS` env (unset = no cap) | analysis runs only. Stock F2D has **no** tick cap; B2D's fork caps hard at 4000 |
| `atomic_behaviors.py` | `AGENT_SCENARIO_TIMEOUT` env cap (unset = stock) | analysis runs only |

With those variables unset, scoring semantics are stock Fail2Drive.

**Deliberately not back-ported: the 50 m global-plan downsample.** B2D's fork calls
`downsample_route(plan, 50)`, F2D's calls it with `200` -- a maximum distance between plan points,
so the sensors-track driver sees a much coarser route on F2D. Matching B2D would make the two
benchmarks' sensors numbers comparable to each other, at the cost of making F2D numbers
incomparable with its public leaderboard, which is the comparison that matters for a held-out
benchmark. The single-line change is in `autonomous_agent.set_global_plan` if that trade is ever
worth revisiting; it would be a deviation to record next to any result.

`get_metric_info()` is the other B2D addition, and is **not** back-ported: nothing here calls it.

## Known limitations

- **No seek or rewind.** CARLA has no full-state snapshot. Traffic is seeded (`TM_SEED`) but not
  bit-exact, and it reacts to the ego.
- **Teardown hang.** After a route ends, the evaluator sometimes hangs *after* writing
  `result.json`, and it ignores SIGINT. `run_session.sh` kills it once the agent has been
  destroyed and the global record is written, or after 120 s. `scripts/clean_carla.sh` is the
  manual fallback.
- **Contact counter.** The daemon's collision sensor counts contacts; the leaderboard debounces
  sustained contact into one event.
- **Maps.** 7 of the dev10 routes need the Town11–15 additional maps (`setup_carla.sh` installs
  them). Some installs lack Town14.
- **No reverse.** Controls are steer / throttle / brake only, so an ego stopped nose-first against a
  wall, fence or vehicle cannot back out. Fail2Drive's `Wall` and `FullyBlocked` scenarios are
  built around obstacles a driver may need to reverse away from, so this limitation bites hardest
  there.
- **Fail2Drive's sensors-track route plan is coarse.** 200 m between plan points against routes
  whose median length is 211 m, so a typical route's plan is little more than its start and end,
  plus whatever the downsampler adds at junctions. This is stock F2D (see above), but it means the
  sensors track leans much harder on the cameras there than it does on Bench2Drive.
- **The privileged scene has no static layout.** Fences, poles, walls and curbs are not in
  `percept`, `get_state` or the BEV, and junction polygons only approximate the paved area. At
  junction corners, check the cameras.
- **BEV lane coverage.** Lanes are expanded through Driving lanes only, so a carriageway beyond a
  median is not drawn (its vehicles still are).
- **Throughput.** Every camera renders every tick, so the camera set dominates sim speed: with the
  4 default cameras a session runs well below real time. Fewer or smaller cameras run faster, but
  change the benchmark config.
- **Replay weather.** CARLA's recorder does not store weather; `replay_sim.py` applies the route's
  start weather, so routes whose weather changes along the way replay with the start weather.
- **No authentication** on the socket. Bind to localhost unless the network is trusted.

## Provenance

Ported from an internal model-debugging daemon. Everything tied to a learned driving model was
removed, including model autopilot, proposal inspection and selection, and score reweighting.
Driving is now fully in the client's hands, and scene state comes directly from the simulator
instead of the model's GPS-based frames.
