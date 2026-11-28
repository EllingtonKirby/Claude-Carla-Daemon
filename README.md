# CARLA coding-agent benchmark

Measure how well coding agents drive CARLA routes in closed loop. Start with
[AGENTS.md](AGENTS.md) for Codex or Claude Code, or go straight to [setup](docs/HANDOFF.md#2-setup).

The **probe daemon** is a CARLA leaderboard agent that pauses the simulation every tick and waits
for JSON commands on a socket. The agent reads the scene, decides, and advances time with
plan-level commands (`apply_plan`: ego-frame waypoints tracked by a PID). Pausing is free, so the
agent can think as long as it needs. Routes are scored by unmodified leaderboard metrics: driving
score, route completion and infraction penalty.

Two benchmarks, selected with `BENCHMARK=` and never pooled:
- **`bench2drive`** (default): [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) 0.0.4,
  220 routes over 7 towns. Headline number: mean driving score.
- **`fail2drive`**: [Fail2Drive](https://github.com/autonomousvision/fail2drive) (IROS 2026), 100
  route *pairs* in Town13 with 17 unseen scenarios and 30 novel assets. Each `Generalization_*`
  route is a shifted version of an in-distribution `Base_*` reference, and the headline number is
  the **drop** between them, not either absolute score.

Three tracks are enforced by the daemon and reported separately.
- **`sensors`**: only SENSORS-track-eligible inputs. Cameras, speedometer, IMU, GNSS and the
  leaderboard's route plan; the ego pose is estimated from GNSS + compass.
- **`privileged`**: the ground-truth scene. Actors, lanes, traffic lights, stop signs, speed limit
  and a live score.
- **`dense`**: the sensors view plus ground-truth depth and instance segmentation.

```
Driver ──(carla_probe.client, JSON lines)──▶ ProbeAgent ◀─run_step─ leaderboard (sync) ◀─rpc─▶ CARLA
```

## Requirements

- Linux with an NVIDIA GPU that can run CARLA 0.9.15 (≈8 GB VRAM or more), about 30 GB of disk
  (plus ~30 GB more for Fail2Drive, which needs its own CARLA build)
- `git`, `patch`, `wget`, and [uv](https://docs.astral.sh/uv/) (or `python3.10`); `ffmpeg` (with libx264) to build videos
- The client alone needs only Python ≥ 3.8 and no packages

## Setup

```bash
CARLA_ROOT=$HOME/carla bash scripts/setup_carla.sh   # CARLA 0.9.15 + Town11-15 maps
bash scripts/setup_bench2drive.sh                    # fetch Bench2Drive 0.0.4 + apply patch + split routes
source scripts/env.sh                                # venv + pinned deps + PYTHONPATH
pytest                                               # offline unit tests (no simulator needed)
```

For Fail2Drive, additionally (the leaderboard and routes are already vendored at
`third_party/fail2drive/`; this installs F2D's own CARLA build and the runtime speed-limit maps):

```bash
F2D_CARLA_ROOT=$HOME/f2d_carla bash scripts/setup_fail2drive.sh
BENCHMARK=fail2drive source scripts/env.sh
```

## Run a session

```bash
ACCESS=sensors bash scripts/run_session.sh 25378 &   # smoke route; 2091 is the standard benchmark
python -m carla_probe.client --wait 600 status       # waits for CARLA + route load
python -m carla_probe.client observe
python -m carla_probe.client percept cams='["CAM_FRONT"]'      # state + a JPG path
python -m carla_probe.client apply_plan 'waypoints=[[4,0],[8,0],[12,0],[16,0],[20,0]]' target_speed=6 ticks=40 \
    why="lane ahead is clear; follow it at 6 m/s"      # every driving command carries a justification
```

When the route ends, the session directory (`sessions/<stamp>_route_<id>/`) holds `result.json`,
a replayable CARLA recording, timed scoring events, the decision log with every justification, and the
full command log (`docs/HANDOFF.md` §5):

```bash
python tools/summarize_results.py sessions/*      # per route, grouped by benchmark and track
python tools/f2d_score.py sessions/f2d_*          # Fail2Drive: Base vs Generalization, and the drop
```

## Run an evaluation

The scored launcher supports headless Claude Code and Codex. Authenticate the CLI you want
to run and select a model available to that account. The route-following baseline uses the same
scoring pipeline and needs no model account.

```bash
DRIVER=baseline_route bash scripts/run_eval.sh 2091 sensors route-follower
bash scripts/run_eval.sh 2091 sensors '<claude-model-id>'
DRIVER=codex bash scripts/run_eval.sh 2091 sensors gpt-6.2-sol
```

Follow [the experiment sequence](docs/HANDOFF.md#43-first-experiments) for smoke tests, repeated
Bench2Drive runs, and the separate Fail2Drive check.

For practice and debugging, use [the interactive workflow](docs/HANDOFF.md#44-interactive-driving-with-codex-or-claude-code)
from either Codex or Claude Code. For scored Codex runs, see
[the adapter and its isolation preflight](docs/HANDOFF.md#45-scored-codex-runs).

## Docs

- [`docs/HANDOFF.md`](docs/HANDOFF.md): **start here.** Purpose, setup, install validation,
  running an isolated eval (`scripts/run_eval.sh`), what a run records, troubleshooting.
- [`docs/OPERATOR_GUIDE.md`](docs/OPERATOR_GUIDE.md): commands, frames, driving technique, lessons
- [`docs/DESIGN.md`](docs/DESIGN.md): how the pause works, the Bench2Drive patch and the vendored
  Fail2Drive tree, limitations
- [`benchmark/PROTOCOL.md`](benchmark/PROTOCOL.md): route sets, rules, reporting
- [`benchmark/FAIL2DRIVE.md`](benchmark/FAIL2DRIVE.md): the Fail2Drive benchmark -- what it
  measures, its route sets, how to report it. Operator-only: the driver sandbox denies it.

## Layout

```
carla_probe/        the daemon: agent.py (leaderboard agent) server.py client.py geometry.py
                    pid.py bev.py config.yaml
scripts/            shell only, everything needed to set up and launch a run: setup_carla.sh
                    setup_bench2drive.sh setup_fail2drive.sh env.sh run_session.sh run_eval.sh
                    clean_carla.sh
tools/              offline analysis, none of it needed to run an eval: summarize_results.py
                    f2d_score.py replay_sim.py recorder_truth.py diagnose_alignment.py make_video.py
                    split_routes.py
patches/            the Bench2Drive leaderboard patch
third_party/        one subdirectory per benchmark tree: Bench2Drive/ (fetched and patched at setup,
                    not committed) and fail2drive/ (vendored: leaderboard, scenario_runner, 200
                    routes; MIT)
benchmark/          route sets + protocol
tests/              offline unit tests
```

## License

This repository is MIT-licensed (see `LICENSE`). Bench2Drive is **not** included: it is fetched
at setup, and its top-level license is CC BY-NC-ND 4.0 (non-commercial), while its `leaderboard/`
and `scenario_runner/` trees are MIT (CARLA). Fail2Drive **is** included, under `third_party/fail2drive/`:
it is MIT throughout, so a modified copy may be redistributed; see `third_party/fail2drive/VENDORED.md`
for the pinned upstream commit and what was left out. Fail2Drive is a **held-out** benchmark — its
routes, scenarios and assets must never be used for training or tuning. CARLA is MIT, and its
assets are CC-BY.
