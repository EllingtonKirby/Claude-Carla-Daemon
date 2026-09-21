# claude_carla_daemon

A CARLA / Bench2Drive probe daemon that lets an LLM drive a route through a JSON-lines socket and
be scored by the stock leaderboard.

**Start with `docs/HANDOFF.md`**: purpose, setup, install validation (§3; re-run it on a new machine
or after changing `agent.py`, `config.yaml` or the patch), evals, and recording. Internals are in
`docs/DESIGN.md`.

- Benchmark runs: `scripts/run_eval.sh`. To drive a route interactively, use the `drive-carla`
  skill; such runs are not benchmark results.
- Never commit run results, session data or route-specific findings (scenario contents, tactics
  that work on a given route, scorer internals). Drivers under evaluation can read this repo; keep
  results in `sessions/` (gitignored) or outside the repo.
- Three top-level homes, and code goes in exactly one:
  `carla_probe/` the daemon and the driver — anything a benchmark run executes;
  `scripts/` shell only, what sets up and launches a run; no Python lives here;
  `tools/` offline analysis, never on the path of a run.
- `carla_probe/agent.py` runs inside the leaderboard process and needs carla/leaderboard/srunner.
  `geometry.py`, `pid.py`, `bev.py`, `server.py` and `client.py` do not, and are unit-tested in
  `tests/` (`pytest`, no simulator).
- Keep `client.py` pure stdlib.
- `carla_probe/vla_server.py` (the learned policy behind `drive`) runs in ITS OWN interpreter,
  `third_party/vla_venv` (Python 3.8 + torch; `scripts/setup_vla.sh`), never in the daemon's. The
  daemon talks to it over HTTP with stdlib only. Its outputs are Bench2Drive-eligible on both
  tracks; `test_drive_learned_policy` runs it against a fake service at both levels.
- The driver is `claude -p`, launched by `run_eval.sh`, so runs are billed to whatever the `claude`
  CLI is logged in to — normally a Claude subscription. A bespoke Messages-API driver existed
  briefly (`carla_probe/drive_loop.py`, removed 2026-09-14) and is recoverable from git history:
  don't rebuild one without an API key and a decision to fund runs that way. A subscription-backed
  run's `total_cost_usd` is an API-equivalent figure, not a bill.
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
