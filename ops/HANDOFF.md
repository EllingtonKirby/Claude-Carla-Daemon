# Handoff: CARLA privileged-ladder experiments (updated 2026-11-24)

Session-to-session handoff for the operator (not for drivers; it holds no route contents). Read it
first when resuming. It used to live at `~/workspace/HANDOFF.md`; it now lives here, in the
`privileged-v2` worktree (`~/workspace/ccd_priv2/ops/HANDOFF.md`). `docs/HANDOFF.md` is a different
document: the repo's general operator manual.

Claude memory (`~/.claude/projects/-home-ekirby-mack-ekirby-runs-carla-hornet*-work/memory/`) has
the background; this file has the current state and the open tasks.

## PRs (opened 2026-11-24)

- GitHub access on a fresh container: `bash ~/mack/ekirby/code/cluster-relay/claude-skill/github-setup.sh`
  (SSH push + `gh`; put `~/.local/bin` on PATH).
- Fork `EllingtonKirby/Claude-Carla-Daemon` (remote `origin`); upstream `valeoai/CarlaClaudeDaemon`
  (remote `valeoai`). Do not open PRs upstream unless the user asks; it would notify that org.
- `privileged-v2` → base `sensors-control-knobs` (a fork copy of upstream `valeoai/sensors-control-knobs`).
  Don't use the fork's `main` as base: it is far behind (82 commits, vendored fail2drive).
  `privileged-ladder` is an ancestor of `privileged-v2` and needs no separate PR.
- `oracle-frames-deny-fix` → base `sensors-control-knobs`: one Sep 21 commit (6ef0435, BLUE oracle
  permissions) that sat on local `main` and was never pushed. Its parent is upstream main 3eb6811,
  which the base contains, so the PR shows only that commit. Upstream main is where it belongs.

## Run state: everything is stopped

- No lanes are running. Check with `ps -eo pid,args | grep -E "[l]anes3|[r]un_eval"`.
- The last session, B2i+ 28198, finished: Completed, DS 100. `classify_sessions.py`: 96 valid, 4 cutoff.
- `~/workspace/quota_cap` = `0.0` until 2026-11-26 08:00 UTC: every lane blocks. Raise it or delete it
  only when the user says to run.
- Not run (budget cap reached): B0i+ 25378, B0i+ 3514, B2i+ 3514. Their `.claim` folders were set by
  hand, and each is logged in `sessions/ladder_plus/RELAUNCHES.md`.

## User rules (these override defaults)

- Claude-driven evals only 19:00–09:00 Paris (enforced by hours_gate in `quota_gate.sh`, POSIX TZ).
- The user sets a budget cap per batch. Enforce it with `quota_cap`. Count your own work too: a long
  coding session costs several % of the weekly budget. The weekly window resets ~Friday midday.
- Keep check-ins few and lean (one or two a night). Don't poll hourly. No manual Claude calls
  (e.g. resumed debriefs cost ~1.3x a drive).
- Hold off on repeat attempts unless asked.
- Keep work moving without asking for routine launches once a direction is agreed. Tell the user at
  once if something blocks.
- Never edit the shared checkout `~/workspace/claude_carla_daemon` (it has someone's uncommitted
  JOURNAL.md). Work in the worktrees.
- Don't commit session results where drivers can read them.

## Where things are

- Worktrees:
  - `~/workspace/ccd_base`: A0 baseline, detached at bd0bb10.
  - `~/workspace/ccd_exp`: branch `privileged-ladder`.
  - `~/workspace/ccd_priv2`: branch `privileged-v2`, the current one.
- Env: `source ~/workspace/hornet_envrc`. CARLA 0.9.15 needs `SDL_VIDEODRIVER=dummy`. Tests:
  `cd ~/workspace/ccd_priv2 && .venv/bin/python -m pytest -q tests` (269 pass).
- Operator scripts live in `~/workspace/` and are mirrored in `ops/` (README there).
  - `lanes3.sh <repo> <jobs> <n> <first_slot>`: slots 6–8. Slot s = CARLA 20000+10s, probe 8790+s;
    slot 5 is skipped. Never share a slot.
  - `classify_sessions.py`: valid / cutoff / pre_driver / incomplete / running. Quarantine to `_infra/`
    and requeue infra failures; never rerun real driving failures.
  - Dashboard: https://claude.ai/artifact/K9qzj5LpoCTtoAQESyh37B. Refresh procedure:
    `ops/DASHBOARD_REFRESH.md`; blob ids in `dashboard_blobs.json`; `build_takeaways.py`.
  - Commentary videos: `~/workspace/dash_videos/` (rendered by `render_*.sh`).
- Sessions (gitignored):
  - `sessions/priv_v2/` (A1);
  - `sessions/ladder_plus/{B0_sensors,B2_dense_code,B0_sensors_guidefix,B0_sensors_info,B2_dense_code_info}/a1/`;
  - older runs in `ccd_base/sessions/priv_opus` (A0) and `ccd_exp/sessions/ladder` (B0–B4, old controller).
- Each session has `debrief.md`: the closing account under fixed headings (run_eval `DEBRIEF=prompt`).

## Results so far (Bench2Drive dev10, Opus, attempt 1 unless noted)

| Arm | Mean DS |
|---|---|
| Sep 11 sensors reference | 61.5 |
| B0+ (cameras, plus controller) | 68.4 |
| B0g+ (camera mount fixed in the guide) | 69.0 |
| B0i+ (B0g+ + info extras) | 60.9 on 8 routes (B0g+ 65.0 on the same 8) |
| B2+ (GT depth/masks + code) | 81.6 |
| B2i+ (B2+ + info extras) | 84.8 on 9 routes (B2+ 79.5 on the same 9) |
| A0 (privileged) | 88.3 |
| A1 Privileged+ (10 routes x 3 attempts) | 98.8, 29/30 at 100 |

- One attempt per route has a median spread of ~27 DS, so camera-arm differences under ~10 are noise.
- Info extras = rules text, scenarios, predict, and plan_check on driver-estimated obstacles.
- Takeaway: the camera arms lose on perception (wrong positions, lanes, headings), not on missing rules
  or planning tools. Driver debriefs' causal claims are leads, not proof (the camera-mount fix did not
  lift B0).

## Candidate next steps (none agreed yet; ask the user)

1. Suggest the §5.1 camera-mount fix (0bfa4af) and the oracle fix (6ef0435) to whoever owns the shared
   checkout and upstream.
2. Improve camera-arm perception (lane assignment, oncoming vs. leading, unclassified pedestrians):
   helpers in the perception library, then remeasure.
3. Separate information from controller on A1: A0 on the plus controller without extras.
4. A wider route set than dev10 (privileged is saturated at 98.8).
5. Repeat attempts on the camera arms (on hold per the user).
6. Run the three skipped info-extras jobs if wanted: remove their `.claim` folders and set a cap.
