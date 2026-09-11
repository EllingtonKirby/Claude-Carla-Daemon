# claude_carla_daemon

Benchmark how well an LLM agent (Claude) drives a car in closed loop.

The **probe daemon** is a [Bench2Drive](https://github.com/Thinklab-SJTU/Bench2Drive) leaderboard
agent that pauses the CARLA simulation every tick and waits for JSON commands on a socket. The
agent reads the scene, decides, and advances time with plan-level commands (`apply_plan`:
ego-frame waypoints tracked by a PID). Pausing is free, so the agent can think as long as it needs.
Routes are scored by the unmodified Bench2Drive metrics: driving score, route completion and
infraction penalty.

Two tracks, enforced by the daemon:
- **`sensors`**: only Bench2Drive-eligible inputs. Cameras, speedometer, IMU, GNSS and the
  leaderboard's route plan; the ego pose is estimated from GNSS + compass.
- **`privileged`**: the ground-truth scene. Actors, lanes, traffic lights, stop signs, speed limit
  and a live score.

```
Claude ──(carla_probe.client, JSON lines)──▶ ProbeAgent ◀─run_step─ leaderboard (sync) ◀─rpc─▶ CARLA
```

## Requirements

- Linux with an NVIDIA GPU that can run CARLA 0.9.15 (≈8 GB VRAM or more), about 30 GB of disk
- `git`, `patch`, `wget`, and [uv](https://docs.astral.sh/uv/) (or `python3.10`)
- The client alone needs only Python ≥ 3.8 and no packages

## Setup

```bash
CARLA_ROOT=$HOME/carla bash scripts/setup_carla.sh   # CARLA 0.9.15 + Town11-15 maps
bash scripts/setup_bench2drive.sh                    # fetch Bench2Drive 0.0.4 + apply patch + split routes
source scripts/env.sh                                # venv + pinned deps + PYTHONPATH
pytest                                               # offline unit tests (no simulator needed)
```

## Run a session

```bash
ACCESS=sensors bash scripts/run_session.sh 25378 &   # smoke route; 2091 is the standard benchmark
python -m carla_probe.client --wait 600 status       # waits for CARLA + route load
python -m carla_probe.client observe
python -m carla_probe.client get_camera cams='["CAM_FRONT"]'   # prints a JPG path
python -m carla_probe.client apply_plan 'waypoints=[[4,0],[8,0],[12,0],[16,0],[20,0]]' target_speed=6 ticks=40
```

When the route ends, the session directory (`sessions/<stamp>_route_<id>/`) holds `result.json`
and the full command log:

```bash
python scripts/summarize_results.py sessions/*
```

## Letting Claude drive

Open Claude Code in this repo and ask it to drive a route. For example, "Run the standard
benchmark route and drive it." The `drive-carla` skill (`.claude/skills/drive-carla/SKILL.md`)
covers the launch, the observe → plan → advance loop, and the rules in
[`benchmark/PROTOCOL.md`](benchmark/PROTOCOL.md).

## Docs

- [`docs/FIRST_LIVE_RUN.md`](docs/FIRST_LIVE_RUN.md): **start here on a new GPU machine**. The code
  has not yet been validated against a live CARLA; this is the step-by-step checklist with pass
  criteria.
- [`docs/OPERATOR_GUIDE.md`](docs/OPERATOR_GUIDE.md): commands, frames, driving technique, lessons
- [`docs/DESIGN.md`](docs/DESIGN.md): how the pause works, the Bench2Drive patch, limitations
- [`benchmark/PROTOCOL.md`](benchmark/PROTOCOL.md): route sets, rules, reporting

## Layout

```
carla_probe/        agent.py (leaderboard agent) server.py client.py geometry.py pid.py bev.py config.yaml
scripts/            setup_carla.sh setup_bench2drive.sh env.sh run_session.sh clean_carla.sh
                    summarize_results.py make_video.py split_routes.py
patches/            the Bench2Drive leaderboard patch
benchmark/          route sets + protocol
tests/              offline unit tests
```

## License

This repository is MIT-licensed (see `LICENSE`). Bench2Drive is **not** included: it is fetched
at setup, and its top-level license is CC BY-NC-ND 4.0 (non-commercial), while its `leaderboard/`
and `scenario_runner/` trees are MIT (CARLA). CARLA is MIT, and its assets are CC-BY.
