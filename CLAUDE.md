# claude_carla_daemon

A CARLA / Bench2Drive probe daemon that lets an LLM drive a route through a JSON-lines socket and
be scored by the stock leaderboard.

**Start with `docs/HANDOFF.md`**: purpose, setup, install validation (§3; re-run it on a new machine
or after changing `agent.py`, `config.yaml` or the patch), evals, and recording. Internals are in
`docs/DESIGN.md`.

- Benchmark runs: `scripts/run_eval.sh` (isolated headless driver). To drive a route interactively,
  use the `drive-carla` skill; such runs are not benchmark results.
- Never commit run results, session data or route-specific findings (scenario contents, tactics
  that work on a given route, scorer internals). Drivers under evaluation can read this repo; keep
  results in `sessions/` (gitignored) or outside the repo.
- `carla_probe/agent.py` runs inside the leaderboard process and needs carla/leaderboard/srunner.
  `geometry.py`, `pid.py`, `bev.py`, `server.py` and `client.py` do not, and are unit-tested in
  `tests/` (`pytest`, no simulator).
- Keep `client.py` pure stdlib.
- Every config knob lives in `carla_probe/config.yaml` and is read with `cfg["key"]`: no code-side
  defaults.
- Access levels (`access: privileged | sensors`) are enforced in `agent.py`. Any new command or
  field must be decided for BOTH levels. Sensors-only views are built separately (`_sensor_*`) from
  Bench2Drive-eligible inputs only, and must never touch `hero_actor`, the world, the map or the
  scenario. The one exception is write-only analysis logs that never reach a reply
  (`score_events.jsonl`). `tests/test_agent_fakesim.py::test_sensors_session` checks for leaks: extend it.
- Every driving command needs `why=` (`require_justification`); scripts that send one (e.g.
  `run_eval.sh`'s `shutdown`) must pass it too.
- Frames: CARLA world is left-handed (y right, yaw clockwise). The client-facing ego frame is
  x forward, y LEFT. All conversions go through `geometry.py`.
- Bench2Drive is fetched and patched by `scripts/setup_bench2drive.sh` into `third_party/`, which is
  not committed. To change leaderboard behaviour, edit `patches/bench2drive-0.0.4-probe.patch`
  and document the change in `docs/DESIGN.md`. With its env knobs unset the patch must keep stock
  scoring semantics.
