# Experiment spec: train on training routes, then evaluate (B3s+T)

Status: agreed with the user on 2026-11-27, **not implemented**. Written so that another agent
(Claude or Codex) can build and run it. Read `ops/HANDOFF.md` first for the user's standing rules
(run window, budget caps, no edits to the shared checkout, etc.).

## Question

Every drive so far starts cold: a fresh session that forgets everything at the end. Drivers keep
asking for tools (a cross-frame tracker with closing speeds, forward prediction against moving
traffic) and rebuild perception code on every drive, but never build anything that only pays off
later. When we built a library for them (B3s+L) we decided what the tools were.

**If the agent gets a training phase on separate routes, with a toolbox that persists between drives
and full ground-truth feedback after each one, does it build the tools it needs itself, and do they
help on the evaluation routes?**

The agent is told that the toolbox persists and that later drives, including the evaluation, will
use it as it is left. It is **not** told what to build.

## Arms compared

| arm | what it is | status |
|---|---|---|
| B3s+ | cameras + code tool + perception service (Depth Pro 2 + SAM 3), cold start | done, 81.9 |
| B3s+E | B3s+ with `EGO_STATE=1` (ego.json next to every frame), cold start | job list ready (`sessions/ladder_plus/jobs_b3se.txt`), needs PR #10 |
| B3s+L | B3s+ with our perception library and the lane-noise guide rule | done, 70.6 |
| **B3s+T** | **B3s+E settings, plus the toolbox the agent built in training (frozen)** | this spec |

All on Bench2Drive dev10, one attempt per route, Opus, `PID_PRESET=plus`, the current driver guide.

## Training routes (10, one per evaluation scenario type)

Chosen deterministically: for each dev10 route, the lowest-numbered Bench2Drive route outside dev10
with the same scenario type. None of them is an evaluation route.

| scenario type | eval route (town) | training route (town) | other same-type routes |
|---|---|---|---|
| ParkingExit | 3514 (Town13) | **1956** (Town12) | 3520, 26393, 26435 |
| ParkingCrossingPedestrian | 3255 (Town13) | **3248** (Town13) | 18252, 24206, 24294 |
| StaticCutIn | 26405 (Town15) | **2709** (Town12) | 2715, 25358, 26396 |
| HazardAtSideLane | 25381 (Town05) | **1790** (Town12) | 1792, 25300, 25439 |
| YieldToEmergencyVehicle | 25378 (Town03) | **3364** (Town13) | 3373, 3378, 3380 |
| ConstructionObstacleTwoWays | 25424 (Town11) | **1825** (Town12) | 1833, 2606, 20920 |
| NonSignalizedJunctionLeftTurn | 2091 (Town12) | **2084** (Town12) | 2086, 2881, 26990 |
| BlockedIntersection | 27494 (Town04) | **2204** (Town12) | 2215, 2989, 27532 |
| SequentialLaneChange | 17569 (Town12) | **17563** (Town12) | 17598, 17635, 17655 |
| SignalizedJunctionLeftTurnEnterFlow | 28198 (Town15) | **28099** (Town04) | 28241, 28243, 28330 |

Training order: the table order above, fixed. Training must run **one drive at a time** (each drive
reads the toolbox the previous review left), so one lane, not three.

Scenario types overlap between training and evaluation, and so do some towns (Town12, Town13); the
routes themselves do not. This is the same kind of split as Bench2Drive's own training data. Nothing
derived from an evaluation route may enter the toolbox.

## Phases

### Training drive k (k = 1..10)

- Settings: exactly B3s+E (`CODE_TOOL=on PERCEPTION_ADDR=... EGO_STATE=1 PID_PRESET=plus`), plus the
  toolbox (below). Normal debrief.
- Before the drive: copy `TOOLBOX_DIR` (the current toolbox) into the driver's code dir as
  `toolbox/` (writable).
- After the drive: copy the driver's `toolbox/` back into `TOOLBOX_DIR`, and archive it as
  `TOOLBOX_DIR/../toolbox_after_drive_k/`.
- **Ground truth is recorded during the drive but hidden from the driver.** For every percept, the
  agent writes, outside `frames/` (which the driver can read), e.g. `SESSION_DIR/truth/frame_<step>/`:
  every actor within 50 m (type, ego-frame position, velocity, heading, size: the privileged
  `actors_nearest` / `parked_nearest` fields), the ego's true pose and speed, the dense route and lane
  geometry ahead (privileged `lane` / `nav`), traffic-light states and stop lines, and the
  ground-truth depth and instance arrays for each camera (`gt_dense.py`, already implemented for the
  dense track). The driver's permissions (`run_eval.sh` writes an allowlist; `--permission-mode
  dontAsk` refuses everything else) must not allow `truth/`. Verify that with the isolation check.

### Review k (training only, right after drive k)

A separate `claude -p` session (same model), not a resumed drive: resuming costs about 1.3x a drive
because the resumed context is not served from cache. It sees:

- the drive's frames (`frames/`), `ego.json` files, its commands and decisions (`commands.jsonl`,
  `decisions.txt`), the debrief, and the final score with every infraction (`result.json`);
- the ground truth from `truth/` for every frame;
- the toolbox (writable), and the code tool with read access to `frames/` and `truth/`, so it can run
  its own perception code against the truth and measure its errors.

Its task, in full (do not add hints about what to build):

> You just drove a training route. This folder holds everything about that drive, including the
> ground truth for every frame you saw, which you did not have while driving. Your toolbox (toolbox/)
> is what you will have on future drives, including the final evaluation on routes you have not seen;
> nothing else carries over. Use the ground truth to find where your perception and decisions were
> wrong, and update the toolbox (code and notes) so future drives go better. Keep toolbox/README.md
> current: it is the first thing you will read on the next drive.

Budget the review (for example a turn or wall-clock cap) and log its cost separately.

### Evaluation (after drive and review 10)

- Freeze the toolbox (`toolbox_final/`, hashed into `driver.json` like `CODE_LIB`).
- Run the 10 dev10 routes with B3s+E settings, the frozen toolbox copied in as `toolbox/`, no review,
  normal debrief. Parallel lanes are fine.
- Prompt for training and evaluation drives (added by `run_eval.sh` when a toolbox is given):

> Your toolbox from earlier drives is at <code dir>/toolbox: read toolbox/README.md before driving.
> (Training only:) Anything you leave in toolbox/ will be there on later drives, including the
> final evaluation.

## Measurements

- Driving: DS per route and mean, infractions, against B3s+, B3s+E, B3s+L.
- Cost: API-equivalent $ per drive (mean and median), $ per decision, turns, max context, thinking
  tokens (`ops/tool_use_summary.py`); training cost (drives + reviews) reported separately.
- What it built: toolbox size and contents after each review (diff the archives); which tools the
  evaluation drives actually called (transcripts); whether a cross-frame tracker appears, when, and
  how accurate it is against ground truth (score it like `tools/eval_driver_lib.py`).
- Steering wobble (`ops/steering_wobble.py`, `ops/steering_wobble_plans.py`).

## To implement

1. `EGO_STATE=1` (PR #10) merged.
2. `run_eval.sh`: `TOOLBOX_DIR=<dir>` and `TOOLBOX_MODE=train|eval` (copy in; copy back and archive in
   train mode; prompt lines above; hash in `driver.json`).
3. Agent: hidden ground-truth logging per percept into `SESSION_DIR/truth/` (reuse the privileged
   observe builders and `gt_dense.py`); confirm the driver cannot read it.
4. `scripts/run_review.sh <session_dir> <toolbox_dir>`: the review session (prompt above, permissions:
   read session frames/truth/logs, write toolbox, code tool with read access to truth).
5. An operator script that runs drive 1, review 1, drive 2, ... in order, then the evaluation lanes,
   under `quota_gate` and the run window.

## Budget and rules

- About 30 Claude sessions: 10 training drives, 10 reviews, 10 evaluation drives. With drives at
  roughly 1% of the weekly quota each and reviews probably less, expect about 25% of a week. The
  user sets the cap (`~/workspace/quota_cap`).
- Claude-driven runs only 19:00-09:00 Paris (`quota_gate.sh`). Training is sequential, so it may need
  two nights; evaluation can run in parallel afterwards.
- Keep results out of anything drivers can read (sessions/ is gitignored; `ops/` is not on the
  driver's allowlist).
