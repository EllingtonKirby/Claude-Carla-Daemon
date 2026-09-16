# Vendored Fail2Drive

Upstream: <https://github.com/autonomousvision/fail2drive> (MIT), pinned at

```
bceb18afb7d62e06e990739e013e750d92ad7a88   "Update citation for IROS 2026 acceptance"
```

Every file in this directory **except this one** is byte-identical to that commit. Nothing here
has been adapted to this repo yet; the probe's changes land in a separate commit on top, so
`git log -p third_party/fail2drive/` reads as "upstream, then exactly what we changed".

Fail2Drive is a held-out CARLA v2 benchmark: 100 route *pairs*, each `Generalization_*` route a
shifted version of an in-distribution `Base_*` reference. Its routes, scenario definitions and
novel assets must never be used to train or tune a driver.

## What is vendored, and what is not

Redistributing a modified copy is allowed here (MIT) in a way it is not for Bench2Drive
(CC BY-NC-ND 4.0, hence `scripts/setup_bench2drive.sh` + `patches/`).

| Vendored | Why |
|---|---|
| `leaderboard/` (minus `docs/`) | the evaluator the probe agent runs under |
| `scenario_runner/` (minus `Docs/`, `speed_limits/*.npy`) | F2D's `srunner` fork: new scenario classes, a different success rule |
| `fail2drive_split/` | the 200 route XMLs |
| `tools/f2d_result_parser.py` | upstream's scorer; the authoritative reference for the scoring rules |
| `LICENSE`, `README.md` | the licence these files are redistributed under, and upstream's own docs |

| Not vendored | Why |
|---|---|
| `scenario_runner/speed_limits/*.npy` (75 MB) | **required at runtime** (`srunner/.../timer.py`, `atomic_behaviors.py`, `background_activity.py` read `$SCENARIO_RUNNER_ROOT/speed_limits/<Town>_speed_limits.npy`). Too big for git; `scripts/setup_fail2drive.sh` fetches them into this tree, and `.gitignore` keeps them out. `compute_speed_limits.py`, which regenerates them, **is** vendored. |
| `team_code/` (216 MB) | F2D's own TransFuser++ / PDM-Lite baselines. The driver here is `carla_probe/agent.py`. |
| `toolbox/` (279 MB) | route and obstacle authoring. F2D is held out; this repo never regenerates routes. |
| `leaderboard/docs/`, `scenario_runner/Docs/` (20 MB) | Jekyll doc sites. |
| `slurm_evaluate.py`, `eval_num_jobs.txt` | F2D's SLURM driver. Replaced by `scripts/run_session.sh` / `run_eval.sh`. |
| `tools/` (rest), `assets/`, `.github/` | Docker images, renderers, route bridges, logos, issue templates — none on the path of a run. |
| `environment.yml`, `env_vars.sh` | conda env and paths. This repo uses `scripts/env.sh`. |
| `README.zh-CN.md`, `README.ja.md` | translations of the vendored `README.md`. |

## Re-verifying

```bash
git clone https://github.com/autonomousvision/fail2drive.git /tmp/f2d
git -C /tmp/f2d checkout bceb18afb7d62e06e990739e013e750d92ad7a88
diff -rq --exclude=.git --exclude=.github --exclude=docs --exclude=Docs \
         --exclude='*.npy' --exclude=.gitignore /tmp/f2d third_party/fail2drive | grep -v '^Only in /tmp/f2d'
# prints nothing at the vendoring commit
```

Manifest of this tree as vendored (before any probe change):

```
find third_party/fail2drive -type f ! -name VENDORED.md | LC_ALL=C sort | xargs sha256sum | sha256sum
-> 20465082696bde8edd40a19cd60c47f8f69936165f9e586e71b316914d3e1de5
```

## Citation

```bibtex
@inproceedings{Gerstenecker2026Fail2Drive,
  author    = {Gerstenecker, Simon and Geiger, Andreas and Renz, Katrin},
  title     = {Fail2Drive: Benchmarking Closed-Loop Driving Generalization},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2026},
}
```
