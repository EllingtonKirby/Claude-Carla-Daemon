# Journal: what we have measured, and what it taught us

A running log of work on the benchmark itself — experiments run, what they showed, and the
measurement lessons that came out of them. `docs/HANDOFF.md` says how to operate the repo; this
file says what we have learned by operating it, including the things that turned out to be wrong.

**Operator-only, and it must stay that way.** Drivers under evaluation can read this repo, so this
file carries no route ids, no per-route scores, no scenario content and no tactics — only
aggregates and method. When an entry needs route-level detail, the detail lives in the session dir
or in a gitignored note and the entry says where.

**§1 is the durable part**: lessons that should change how the next experiment is designed. §2 is
the dated log. Add to both — an entry with no lesson is fine, a lesson with no entry behind it is
not.

## 1. Standing lessons

### 1.1 A single attempt does not measure a route

Measured on the 10-route Bench2Drive sensors dev set, three attempts per route, one commit, one
`TM_SEED`: the **median per-route DS spread is 27 points**, and the worst route spans 65. Two of
the ten routes returned an identical score on all three attempts; the rest did not come close.

The spread is not machine noise. Traffic is seeded but reacts to the ego, so a few ticks'
difference early compounds into a different interaction later, and most routes have one interaction
that decides the score. `benchmark/PROTOCOL.md` already says to expect run-to-run variance; the
number says how much.

What follows:

- **A per-route DS delta from one attempt each way is not evidence.** It was not worth reporting,
  and we reported some anyway before measuring this.
- **Classify routes before comparing them.** A route whose repeated attempts agree can carry a
  claim on its own. A route that swings 40 points cannot, no matter how large the delta looks.
- **Three attempts per arm is the floor** for a mean over ~10 routes, which puts the standard error
  of the mean near 5 DS. Treat a mean shift under ~10 points as unresolved, which is what the
  protocol's rule of thumb said all along.
- Report the spread, not just the mean. A table of means hides exactly the thing that decides
  whether the means mean anything.

### 1.2 Prefer per-decision metrics to per-route scores

A 10-route corpus yields 10 DS numbers and several hundred plans. The plans are the tighter
instrument by two orders of magnitude, and they measure the behaviour a guide change is trying to
change, rather than its noisy downstream consequence.

In the one guide A/B run so far, the per-plan geometry moved decisively and legibly while the DS
means stayed inside their own noise. Design experiments so the primary endpoint is something
counted per decision — plan geometry, look cadence, what the driver opened — and treat DS as the
slow confirmatory endpoint it is. `tools/plan_lint.py` exists for this.

### 1.3 Count what the driver did, not what you assume it did

A finding that "the driver looks once per route" was wrong, and wrong in a way worth remembering:
it counted standalone `percept` commands and missed `percept=true` ridden on driving commands,
which is how the driver actually looks. The error survived because the cheap query (`cmd ==
"percept"` in `commands.jsonl`) looked like the right one.

Cross-check any behavioural count against a second, independent source before building on it. For
looking, that is the driver's own tool calls in `driver_transcript.jsonl`: if the transcript shows
90 images opened on a route your counter scored as one look, the counter is wrong. More generally,
a metric that is easy to compute is rarely the metric you meant.

### 1.4 Only make a documentation change you can measure

Guide edits are cheap to write and expensive to validate, so they accumulate unmeasured. In the
first measured A/B, the section that gave a concrete, checkable geometric rule changed driver
behaviour exactly as specified; the section that restated a loop the driver was already following
changed nothing measurable. Both felt equally worth writing.

Before editing `docs/OPERATOR_GUIDE.md` or the `drive-carla` skill, name the counter that would
move if the edit works. If there isn't one, the edit is a readability change — fine, but do not
expect it in the scores, and do not credit it afterwards.

### 1.5 Pin the commit, and pool only within it

`benchmark/PROTOCOL.md` rule 2 makes the harness — prompt, the documents it points at, the daemon's
command surface — part of the benchmark definition. So attempts pool only if they ran at the same
repo commit, and every reported score carries that commit.

The practical consequence on a shared checkout is that two commits cannot run at once: comparisons
run in phases, one commit at a time, rather than interleaved. Budget for that; it doubles the
wall-clock of an A/B.

### 1.6 Operational hazards of running corpora in parallel

- **Never let a batch script delete a session dir it might not own.** Make it require an explicit
  corpus prefix and refuse to overwrite an existing dir unless forced. A re-run that silently
  overwrote its own baseline would be invisible afterwards.
- **Stagger launches that share a filesystem.** Parallel slots starting in the same second raced to
  create the shared venv and two of them died. Seconds of stagger, or a pre-warmed venv.
- Launch slots in parallel but do not let one slow `ssh` abort the loop: no `set -e` around the
  launcher, and a timeout that accounts for a cold connection.
- More on the port spacing, `pgrep -f` and script-editing hazards in `docs/HANDOFF.md` §6.

## 2. Log

Newest first. One entry per experiment: what ran, at which commit, what it showed, what it changed.

### 2026-09-17 — guide rewrite A/B, and a noise floor

*Bench2Drive, sensors track, 10-route dev set, `claude-opus-5`, stock config, `TM_SEED=0`.*

Three strands, run on 7 GPUs across containers on one host:

1. **Baseline corpus** at the pre-rewrite commit, one attempt per route, analysed with
   `tools/plan_lint.py`. It overturned two of the six guide rules drafted before measuring: plans
   were not too short (1 in 313 was), and `target_speed` was already passed on every plan. What
   survived was that plans over-reached by about 6×, putting the controller's aim point inside a
   single coarse segment on two thirds of plans.
2. **Guide rewrite** (`docs/OPERATOR_GUIDE.md` §6–§11) built on that, then re-run over the same
   routes at the new commit. Per-plan geometry moved as specified and the over-reach problem
   largely disappeared; tracking error did not improve, and one lint flag got worse.
3. **Repeatability**, three attempts per route at the baseline commit — §1.1.

Also corrected: the "driver looks once per route" miscount (§1.3), and a mid-run claim that an
infraction class had disappeared, which was an artefact of the summary table omitting it. Read the
raw `result.json` before claiming an infraction class changed.

Route-level numbers and the full write-up stay out of the repo: `tmp/guide-corpus-findings.md` and
`tmp/guide-rewrite-ab.md` on the run host, alongside the session dirs.

New tooling from this round: `tools/plan_lint.py` (offline plan geometry linter) and
`tools/percept_video.py` (one video panel per decision, from the images the driver opened).
