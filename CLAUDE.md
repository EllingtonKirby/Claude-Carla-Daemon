# claude_carla_daemon

A CARLA / Bench2Drive probe daemon that lets an LLM drive a route through a JSON-lines socket and
be scored by the stock leaderboard. Read `README.md` first, then `docs/DESIGN.md`.

**Status: never run against a live CARLA.** On a GPU machine, work through `docs/FIRST_LIVE_RUN.md`
before anything else, and log results in its §6.

- To drive a route or run the benchmark, use the `drive-carla` skill.
- `carla_probe/agent.py` runs inside the leaderboard process and needs carla/leaderboard/srunner.
  `geometry.py`, `pid.py`, `bev.py`, `server.py` and `client.py` do not, and are unit-tested in
  `tests/` (`pytest`, no simulator).
- Keep `client.py` pure stdlib.
- Every config knob lives in `carla_probe/config.yaml` and is read with `cfg["key"]`: no code-side
  defaults.
- Access levels (`access: privileged | sensors`) are enforced in `agent.py`. Any new command or
  field must be decided for BOTH levels. Sensors-only views are built separately (`_sensor_*`) from
  Bench2Drive-eligible inputs only, and must never touch `hero_actor`, the world, the map or the
  scenario. `tests/test_agent_fakesim.py::test_sensors_session` checks for leaks: extend it.
- Frames: CARLA world is left-handed (y right, yaw clockwise). The client-facing ego frame is
  x forward, y LEFT. All conversions go through `geometry.py`.
- Bench2Drive is fetched and patched by `scripts/setup_bench2drive.sh` into `third_party/`, which is
  not committed. To change leaderboard behaviour, edit `patches/bench2drive-0.0.4-probe.patch`
  and document the change in `docs/DESIGN.md`. With its env knobs unset the patch must keep stock
  scoring semantics.
