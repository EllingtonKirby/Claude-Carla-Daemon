# Operator scripts (hornet container)

Snapshot of the scripts that ran the privileged-ladder experiments from `~/workspace` on the hornet
container (2026-11-18 to 2026-11-21). They are written for that machine and reference absolute paths
there. Not used by the daemon or the drivers.

- `lanes3.sh` runs a job list in parallel lanes, gated by `quota_gate.sh`: the 19:00-09:00 Paris run
  window, five-hour and seven-day usage limits, and an optional cap file (`~/workspace/quota_cap`).
  `lanes.sh` and `lanes2.sh` are earlier versions.
- `carla_watchdog.sh` ends and requeues a session whose CARLA server died.
- `classify_sessions.py` sorts session folders into valid, cutoff, pre_driver, incomplete and running.
- `dashboard_data.py`, `build_takeaways.py` and `DASHBOARD_REFRESH.md` rebuild the results dashboard's CSVs.
- `render_*.sh` and `capture_all.sh` render commentary videos. `teach_launch.sh` and `b4_watch.sh` ran the teacher/student step.
- `ladder_*_after*.sh` chain one batch after another. `apply_guide_camera_fix.py` applied commit 0bfa4af.
- `hornet_envrc` is the environment file (CARLA path, SDL_VIDEODRIVER=dummy, venv).
- `HANDOFF.md` is the session-to-session handoff for these experiments: run state, user rules, results, next steps.
