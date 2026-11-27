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
- `strong-perception` → base `privileged-v2` (PR #3, stacked): the strong perception backend.
- `oracle-frames-deny-fix` → base `sensors-control-knobs`: one Sep 21 commit (6ef0435, BLUE oracle
  permissions) that sat on local `main` and was never pushed. Its parent is upstream main 3eb6811,
  which the base contains, so the PR shows only that commit. Upstream main is where it belongs.

## Run state (2026-11-25 05:30 UTC): everything is stopped

- Nothing is running. `~/workspace/quota_cap` = `0.0` until 2026-11-26 08:00 UTC: Wednesday night
  off (the user's call). seven_day was 0.65 at the last gate (00:04 UTC on 11-25), ~0.67 after the
  last three drives. The weekly window resets ~Friday midday.
- Done overnight 11-24/25, all 20 sessions valid (`classify_sessions.py` now includes `a0plus`):
  - A0+ (`sessions/a0plus/a1`): privileged, plus controller, no extras. Mean 96.4 over 10 routes.
  - B3s+ (`sessions/ladder_plus/B3s_strong_perc/a1`): cameras + code tool + strong perception. Mean
    81.9 over 10 routes. Every driver called the service: 241 of 462 script runs used it (5-44 per drive).
- Nothing is queued. Next runs need the user's go and a new cap.

## Strong perception (branch `strong-perception`, PR #3)

- Offline check, no Claude usage: `tools/eval_dense_estimators.py` on the 223 dense route scenes in
  `ccd_exp/sessions/scenes` (results in `sessions/perception_eval/{gt,legacy,strong}.json`).
  Depth |rel| median at 0-10 / 10-20 / 20-40 m: legacy 1.31 / 0.93 / 0.45, strong 0.07 / 0.09 / 0.18.
  Actor recall (median position error) at the same bands: legacy 0.69 (2.5 m) / 0.51 (3.9 m) /
  0.28 (3.5 m); strong 0.94 (1.5 m) / 0.89 (1.7 m) / 0.59 (1.9 m); the same pipeline on ground truth
  0.97 (1.4 m) / 0.96 (1.7 m) / 0.68 (1.8 m). Strong is close to the dense ground truth, so B3s+ is
  expected near B2+ (81.6) if drivers use it. Only 12 pedestrians in the scenes: no pedestrian verdict.
- Env: `~/workspace/venv_perception2` (Python 3.12, `scripts/setup_perception2.sh`). The HF token
  (SAM 3 access) is in `~/.cache/huggingface/token` and `~/mack/ekirby/.hf_cluster/token`; never commit
  it. `uv python install 3.11 3.12` on a fresh container: both perception venvs link to uv's interpreters.
- ~0.67 s per image for depth + six SAM 3 prompts on the L40S; 9 GB VRAM.
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
  - `~/workspace/ccd_priv2`: branch `strong-perception` (stacked on `privileged-v2`), the current one.
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
| B3s+ (cameras + code + Depth Pro 2 / SAM 3 service) | 81.9 |
| A0 (privileged) | 88.3 |
| A0+ (A0 on the plus controller, no extras) | 96.4 |
| A1 Privileged+ (10 routes x 3 attempts) | 98.8, 29/30 at 100 |

- One attempt per route has a median spread of ~27 DS, so camera-arm differences under ~10 are noise.
- Info extras = rules text, scenarios, predict, and plan_check on driver-estimated obstacles.
- Takeaway: the camera arms lose on perception (wrong positions, lanes, headings), not on missing rules
  or planning tools. Driver debriefs' causal claims are leads, not proof (the camera-mount fix did not
  lift B0).

## Driver perception library (branch `driver-lib`, PR #4) -- built 11-25, not yet driven

- Agreed with the user 11-25: present TOOLS, not solutions. No harness path smoothing (rejected as
  too much of a hint). Built instead: `carla_probe/driver_lib/` (camera geometry, object footprints
  from the perception service, lane markings + LaneTracker, zoom), installed per session with
  `CODE_LIB=carla_probe/driver_lib`; and an OPERATOR_GUIDE §8 rule on not chasing lane-estimate noise
  (applies to every camera run from now on).
- Why: camera arms flip steering ~50 times per 10 s of straight cruising (privileged 1-5); each new
  camera plan moves the path sideways by a median 14-24 cm (privileged 1-4 cm). The library also
  tests token savings: drivers rewrote ~300-390 lines of perception code per drive.
- Offline (tools/eval_driver_lib.py, sessions/perception_eval/driver_lib.json): footprints 77%
  recall, 0.97 m nearest-point error, 0.47 m sideways (Depth Pro median point: 1.85 m); lanes found
  in ~60% of frames, centre within 0.15 m when found; tracker halves jitter, poor when coasting.
- The user (11-25): no strict A/B needed. Run only B3s+L (new guide + library), 10 routes, on
  Thursday night 11-26, and compare tokens with the earlier B3s+/B2+ runs. Scheduled:
  `~/workspace/b3sL_thursday.sh` (running, sleeps until 17:55 UTC 11-26; cancel with
  `pkill -f b3sL_thursday.sh`) sets cap 0.95 (raised from 0.80 by the user on 11-26) until 11-27 12:00 UTC, starts the strong service, runs
  `sessions/ladder_plus/jobs_b3sL.txt`, stops the service and sets the cap back to 0.0. Log:
  `sessions/ladder_plus/lanes_b3sL.log`. `jobs_lib.txt` also holds a no-library control, not scheduled.

## TransFuser v6 (LEAD) -- reference numbers and planned perception tool (11-26)

- The user (11-26): the study is how an agentic MLLM drives and what tools it needs, not SOTA.
  TransFuser v6 is to be used ONLY as a perception tool (boxes, BEV map with lanes and light states),
  never its plan. Its standalone score is a reference for us, not an arm.
- Reference run, no Claude usage: `~/workspace/tfv6_dev10.sh visiononly_resnet34` (env
  `~/workspace/venv_lead`, Python 3.10, run with LD_LIBRARY_PATH unset; checkpoint downloaded to
  `~/workspace/lead/outputs/checkpoints/visiononly_resnet34`, MIT licence). Results in
  `~/workspace/lead_runs/visiononly_resnet34/<route>/checkpoint_endpoint.json`. First pass: 17569,
  2091, 25381, 3255 at 100; 25378 at 70 (did not yield to the ambulance). The other five crashed at
  CARLA start-up (port reuse) and were rerun with a free-port wait.
- Reference result (11-26, complete): TransFuser v6 vision-only alone, 97.0 mean on dev10 (100 on nine,
  70 on 25378: did not yield to the ambulance). A reference, not an arm.
- B5 (built 11-26, branch `b5-scene`, PR #5, worktree `~/workspace/ccd_b5`; not yet driven): cameras +
  code + strong perception + driver library + TransFuser perception. `run_eval.sh TFV6=1`.
  - The leaderboard allows at most 8 RGB cameras, so with TFV6 the agent keeps CAM_FRONT and CAM_BACK
    and replaces CAM_FRONT_LEFT/RIGHT with TransFuser's 6-camera ring (the user chose this, 11-26);
    percept defaults to CAM_FRONT, TFV6_1, TFV6_3, CAM_BACK.
  - `carla_probe/tfv6_server.py` (venv_lead, port 8785): boxes + bird's-eye map only, never the plan.
    The perception service forwards POST /scene to it (`--tfv6-addr`); `lib/scene.py` reads it.
  - Offline (`tools/eval_driver_lib.py --scene` on `ccd_b5/sessions/scenes_tfv6`, 30 scenes, 2
    routes): objects 89% recall, 0.29 m nearest-point error, 0.15 m sideways; lanes found in 7/7
    frames with a clean ground-truth lane, centre within 0.31 m.
  - Launch: `~/workspace/b5_launch.sh [START_UTC] [CAP]` with ccd_priv2 checked out on b5-scene
    (do that only after the Thursday B3s+L run is done); job list `sessions/ladder_plus/jobs_b5.txt`.
- Teacher/student: rerun from driver_lib as the seed (agreed 11-26), 3 rounds per arm, offline first;
  practice-route experiment (same-scenario Bench2Drive route before each test route) proposed.
  Budget for both to be set after the Thursday-night B3s+L results.

## Repo layout from 11-26 evening

- PRs #1, #3, #4, #5 are merged; `main` on the fork has everything (PR #2, the BLUE oracle fix, is
  still open). Work from `main`: `~/workspace/claude_carla_daemon_codex` is on the `main` branch;
  `~/workspace/ccd_priv2` (where the lanes run) sits on the same commit, detached. New work goes on a
  branch and a PR; the user merges, then both worktrees are brought back to `origin/main`.
- `ccd_b5`, `ccd_exp`, `ccd_base` are old worktrees, fully merged (`ccd_b5/sessions/scenes_tfv6` holds
  the TransFuser validation captures).

## Thursday 11-26 night

- The quota gate read `allowed_warning` (weekly usage above ~80%) as blocked, so B3s+L sat idle from
  18:01 to 21:33 UTC. Fixed in `quota_gate.sh` (allowed_warning counts as allowed; the limits and the
  cap still apply) and resumed with `b3sL_resume.sh`.
- B3s+L uses a library frozen at the PR #4 merge (`~/workspace/frozen_libs/driver_lib_b3sL`, no
  `scene` module), so it is the condition that was agreed even though `main` now has scene.py.

## B3s+L results (Thursday 11-26 night, run 22:33 to 02:08 Paris; all 10 valid)

- Mean DS 70.6 (B3s+ 81.9, B2+ 81.6). Per route: 17569 60, 2091 11.1 (ran out of time, RC 47),
  25378 70, 25381 100, 25424 68.9, 26405 30.5, 27494 70, 28198 95.2, 3255 100, 3514 100.
  Vehicle collisions 5 in 10 drives (B3s+ 2): 2091 x2, 26405 x2, 17569 x1; red light 27494; ambulance 25378.
- Tokens did NOT go down: API-equivalent $16.7 per drive (B3s+ $13.2, B2+ $12.8), thinking 77k (71k),
  wall 34 min (30). Drivers wrote much less code (105 lines per drive against 280) but ran more scripts
  (54 against 46) and opened more images (56 against 44).
- Library use: every driver read lib/README.md and used objects, lanes and zoom; only one (26405) used
  LaneTracker.
- Wobble unchanged: steering reversals 56 per 10 s of straight cruising (B3s+ 47, A1 2.6); plan-to-plan
  path shift at 8 m median 20 cm (24), p90 74 cm (85).
- One attempt per route; a ~11-point gap is at the edge of the noise.
- Transcript dig (11-27): the cost rise is two long, failed drives, not the library. Median cost per
  drive $12.4 (B3s+ $13.2); cost per decision flat (~$0.40-0.47). 2091 ran all 3,999 steps (61
  decisions, $24.7: waited 500 ticks for a gap, then a full-lock manual turn into traffic) and 26405
  took 56 decisions ($35.2: went the wrong side of a parked police car on the GPS offset). Cost is
  re-reading the context every turn (17-23M cached input tokens per drive against ~75k generated), so
  writing less code (a few thousand tokens) cannot move it; fewer turns per decision and smaller
  contexts would. Drivers built one per-decision look.py (lanes + tracker + objects, ~25 runs per
  drive) and zoomed more (11 zoom reads against 7 crops).
- Weekly usage 0.88 at the last gate (00:24 UTC 11-27); the cap is 0.0 until the weekly reset.

## Order of runs (proposed 11-26)

1. Thu night: B3s+L (perception library + guide rule), 10 routes. Done: see above.
2. Fri 11-27 night: B5 (TransFuser scene tool on top of B3s+L), 10 routes, SCHEDULED:
   `~/workspace/b5_launch.sh "2026-11-27 17:55" 0.20` (running, waiting). Library frozen at the
   current main in `~/workspace/frozen_libs/driver_lib_b5`. End to end checked without Claude usage
   (TransFuser behind the perception service's /scene, called from lib inside code_runner).
2b. Then B5c: B5 with AUTOCOMPACT=100k (the user's order, 11-27), after the B5 morning check:
   `~/workspace/b5c_launch.sh "<start UTC>" <cap>`, job list `sessions/ladder_plus/jobs_b5c.txt`.
   Why: no drive ever compacted (context grows to 300-430k under `auto`); replaying the drives with
   compaction at 100k to a 30k summary roughly halves input tokens.
3. Next: training routes with a working memory (not built yet): 10 training routes driven in
   sequence with a shared memory, then the 10 test routes with it frozen.
4. Parked: teacher/student.

## Candidate next steps (none agreed yet; ask the user)

1. Suggest the §5.1 camera-mount fix (0bfa4af) and the oracle fix (6ef0435) to whoever owns the shared
   checkout and upstream.
2. Improve camera-arm perception (lane assignment, oncoming vs. leading, unclassified pedestrians):
   helpers in the perception library, then remeasure.
3. Separate information from controller on A1: A0 on the plus controller without extras.
4. A wider route set than dev10 (privileged is saturated at 98.8).
5. Repeat attempts on the camera arms (on hold per the user).
6. Run the three skipped info-extras jobs if wanted: remove their `.claim` folders and set a cap.
