# Fail2Drive

Everything specific to the `fail2drive` benchmark: what it measures, how its routes are named and
grouped, and how a result is reported. `benchmark/PROTOCOL.md` stays authoritative for the rules
that apply to both benchmarks.

**This file is operator-facing and the driver must not read it.** Fail2Drive is a held-out test
set, and the paragraphs below name its scenario classes, its novel assets and its success rule --
the sort of thing that primes a driver for what is coming. `drive_loop` has no filesystem access at
all; `claude_p`'s sandbox allows `PROTOCOL.md` and `OPERATOR_GUIDE.md` but denies this file, and
`third_party/fail2drive/` with it. Keep it that way: if something here turns out to be needed while
driving, phrase it in `PROTOCOL.md` without the Fail2Drive specifics, the way the tick-cap and
route-plan notes there already are.

## What it is

[Fail2Drive](https://github.com/autonomousvision/fail2drive) (IROS 2026), vendored at
`third_party/fail2drive/` (MIT; see `third_party/fail2drive/VENDORED.md`).

| | `bench2drive` (default) | `fail2drive` |
|---|---|---|
| what it measures | closed-loop driving across 220 routes and 44 scenarios | closed-loop **generalization**: 100 route pairs, each `Generalization_*` route a shifted version of an in-distribution `Base_*` reference |
| headline number | mean DS | the **Base → Generalization drop** in the harmonic mean of DS and success rate |
| tree | fetched + patched, `third_party/Bench2Drive` | vendored, `third_party/fail2drive` |
| simulator | stock CARLA 0.9.15 | **its own CARLA build**, with 30 novel assets baked in |
| towns | 7 towns | Town13 only (a large map) |
| route budget | hard cap of 4000 ticks (200 s at 20 Hz) | no tick cap; the route ends on its own `RouteTimeoutBehavior` |
| route succeeds if | every infraction list empty except `min_speed_infractions` | every infraction list empty except `min_speed_infractions` **and** `outside_route_lanes` |
| sensors-track route plan | a point at most every 50 m | a point at most every 200 m (see below) |
| report with | `tools/summarize_results.py` | `tools/f2d_score.py` |

The last two rows are the ones that catch people out. The same drive scores differently under the
two success rules, and on the sensors track a Fail2Drive driver is given a **much coarser** route
plan than a Bench2Drive driver: that is stock Fail2Drive and what every model on its public
leaderboard receives, and it matters because F2D routes are short (median 211 m, 92 of 200 under
200 m), so a typical route's plan is little more than its start and end.

Running F2D routes against a stock CARLA 0.9.15 does **not** error: CARLA substitutes default
blueprints, so a scenario that should spawn a cow spawns a car and the run measures nothing it
claims to. `scripts/setup_fail2drive.sh` and `scripts/env.sh` check the novel-asset packs for
files; never bypass that check to make a run start.

**Fail2Drive is a held-out test set.** Its routes, scenario definitions and novel assets must never
be used to train or tune a driver. They are vendored in this repo so the benchmark is reproducible,
not so that anything may be read off them.

## Route sets

Routes are named `<Split>_<Scenario>_<id>` in `third_party/fail2drive/fail2drive_split/`;
`run_session.sh` and `run_eval.sh` accept either the name or the bare 4-digit id, and a
`Generalization_*` id is its `Base_*` id plus 1000. Every set below is whole pairs, because a route
without its twin cannot produce a drop.

| set | file | routes | purpose |
|---|---|---:|---|
| f2d_pair | `benchmark/routes/f2d_pair.txt` | 2 | plumbing check: the install, the novel-asset CARLA build and the pair scoring. Not a result. |
| f2d_smoke | `benchmark/routes/f2d_smoke.txt` | 8 | one pair per scenario class; exercises every class table. Still not a result. |
| f2d_full | `benchmark/routes/f2d_full.txt` | 200 | the benchmark |

A Fail2Drive number quoted from anything less than a substantial, balanced share of the 200 routes
is not a Fail2Drive score. `tools/f2d_score.py` always prints its coverage
(`n/100 Base + n/100 Generalization, n/100 complete pairs`); quote that line with the number.

## Running one

Setup is `docs/HANDOFF.md` §2b (F2D needs its own CARLA build); the smoke run is §3.3.

```bash
BENCHMARK=fail2drive source scripts/env.sh
BENCHMARK=fail2drive ACCESS=privileged bash scripts/run_session.sh Base_BadParking_0000
BENCHMARK=fail2drive DRIVER=claude_p bash scripts/run_eval.sh Base_BadParking_0000 sensors claude-opus-5
```

Every route loads Town13 and takes several minutes; use `--wait 900` on the first `status` and do
not read a quiet evaluator as a hang.

## Reporting

The per-route table is not the result. Report, from `tools/f2d_score.py`: the coverage line, Base
and Generalization DS / SR / HM, and the **drop** in each. A drop needs both sides, so a partial
run has no headline number -- the tool prints `n/a` rather than a drop, and that `n/a` is the
honest answer, not a missing value to fill in with the per-split numbers. Per-class and
per-scenario tables say *where* the model breaks; quote them with their `n`, which is often small
enough that one route moves the row.

Everything `PROTOCOL.md` asks for alongside a result applies here too: the track, the model, the
repo commit, the CARLA version, the driver harness, and the benchmark itself (`session.json` and
`driver.json` both record it).
