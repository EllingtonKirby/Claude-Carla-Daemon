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
- **Stability has to hold in both arms, and it often does not.** A baseline that repeats bounds the
  *baseline's* noise; it says nothing about the arm you are comparing it to. Both routes that
  repeated exactly in the baseline above came back bimodal under the guide change, swinging 40 and
  50 points. Once the second arm was measured, **no route on the dev set was stable in both arms**,
  so three attempts supports no per-route claim at all — only the pooled one. `tools/pool_report.py`
  enforces this; the earlier version classified on the baseline alone and would have reported both
  of those routes as clean wins.

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

### 1.7 A defect you eliminate is not a defect that mattered

The first guide A/B removed its target completely: the geometric fault the rewrite was written to
fix fell from 71% of plans to 0.4%, over ~900 plans per arm, with every supporting measure moving
the same way. DS did not follow. Pooled over the dev set the mean rose 9 points with a 95%
interval of −4 to +23, and no route was stable enough in both arms to carry a delta of its own.

The lesson is not that per-decision metrics are unreliable — §1.2 stands, and the geometry result
is by far the most certain thing this corpus has produced. It is that **a per-decision metric is a
proxy, and eliminating it proves the edit worked, not that the edit mattered.** The two questions
are separate and need separating in advance:

- *Did the change land?* Per-decision counters answer this, tightly, with a few hundred samples.
- *Did it help?* Only the outcome answers this, and it costs two orders of magnitude more samples.

So: when a guide change fully lands and the score does not move, do not quietly re-report the
proxy as the result, and do not keep spending attempts hoping the outcome catches up.

**Amended 2026-09-18, and the amendment is the more useful half.** The original wording went on to
say that a flat score means "the behaviour was not what was costing points". That was wrong here.
A near-miss counter built later on the same 60 sessions found the rewrite cut the rate of plans
leaving under half a metre by 5.4 points, t = −4.49, eight of nine routes improving — the change
did improve the driving, and DS was simply too blunt to show it (§1.11). The rule survives; the
inference drawn from it does not. A flat outcome licenses "this experiment did not resolve
whether it helped", and nothing stronger. Before concluding that a behaviour does not matter, look
for a denser measure of the same thing — the third option, between the proxy and the outcome, is
usually there and is usually already recorded.

### 1.8 Decompose the noise before buying more of the experiment

When a pooled delta lands inside its interval, the reflex is to run more attempts. Check first
which axis the uncertainty is actually on, because two components need different purchases:

- **attempt noise** — each arm's per-route mean is itself sampled, and *more attempts* shrink it;
- **route-to-route variation in the true effect** — the change may help some routes and hurt
  others, and only *more routes* shrink that. More attempts never will.

On the first A/B the split was roughly 80/20, so attempts were the efficient axis — but with a
floor: even infinitely many attempts on ten routes left the standard error at about half its
observed value. `tools/pool_report.py` prints the split and projects the standard error at other
experiment sizes; read it before ordering a re-run. It is also worth asking whether resolving the
delta would change any decision, which is a cheaper question and was answered "no" the first time.

### 1.9 Rank causes by what they cost, not by how often they fire

The scorer logs every infraction as an event, and the natural first move is to count the events.
On the first corpus that ranking is not merely imprecise, it is inverted: the two most frequent
classes fired 1088 and 837 times and cost about 3 DS a session between them, while the class that
fired 37 times cost 20 — more than every other cause combined.

The reason is structural and will not go away: leaderboard 2.0 has two kinds of penalty. Some
classes multiply the score once per event; others scale it once by the fraction of the route
affected, however many events they log. A frequency table mixes the two and reads as though the
percentage-based classes dominate.

So before choosing what to study, convert to points: for each class, how much DS would come back
if that class alone were removed. `tools/collisions.py` prints that ledger. The entries overlap —
penalties multiply, so they are a ranking rather than a partition — but a ranking is what the
decision needs, and it took one offline pass over sessions already on disk.

### 1.10 The driver writes down its reasoning; read it against the ground truth

Every command carries a `why`, and at the moment something goes wrong that sentence says what the
driver believed. Put it beside the recorder's account of where things actually were and the error
stops being "the driver crashed" and becomes a specific false belief with a magnitude.

That comparison is what separated the two explanations we could not choose between from the score
alone. "The driver does not look" died on it: on 45 of 49 collisions it had opened a camera within
a second of impact, and on 43 of 46 it had opened the camera covering the bearing the hit came
from. What survived was narrower and quantified — the driver names the object it is about to hit,
states a margin, and the margin is short by roughly its own half-extent.

Two working rules. Read the `why` at the event, not the session summary: the belief that matters is
the one held at the moment of commitment, and it is usually contradicted by the previous decision's
`why` as well. And check the driver's claims about the daemon rather than repeating them — two
runs independently reported that the controller will not reverse, which was worth confirming in the
code (it is true) before it went into a finding.

### 1.11 When the outcome is too rare to measure, measure what produces it

A 60-session corpus yields about 50 collisions, which is not enough to tell whether a change
helped: the first guide A/B moved DS by 9 points with an interval four times that wide, and
resolving it would have cost ~160 more sessions. The same corpus contains about 1,800 plans, and
for each one the recording says how close the car came to anything. A near miss is the same
behaviour as a collision with a better draw, so the dense quantity and the sparse one measure the
same thing — and the dense one is already on disk.

The pattern generalises past collisions. Before buying more samples of a rare outcome, ask what
continuous quantity it is the tail of, and whether the recording already contains it. The rare
event is what you care about; its precursor is what you can afford to measure.

Two things this only works with:

- **Bodies, not centres.** The whole reason the metric is interesting is the distinction between
  a distance to an object and the room left beside it, which means every actor's extent has to be
  known. `tools/actor_extents.json` measures each blueprint once against a live server.
- **A normaliser.** A per-decision rate falls whenever decisions get shorter, whether or not
  anything improved. `clearance_lint` reports distance driven per plan next to the rate for
  exactly this reason, and the first result had to survive that check before it could be believed.

And one discipline: pick the threshold before looking, then show the answer across a range of
them. A near-miss rate at one cut-off is a choice; the same sign and size from 0.15 m to 1.0 m is
a finding.

## 2. Log

Newest first. One entry per experiment: what ran, at which commit, what it showed, what it changed.

### 2026-09-18 — a near-miss counter, and the guide rewrite turns out to have worked

*The same 60 sessions again; no new runs. Ground truth decoded for all 60 this time, not just the
39 that logged a collision — restricting to those would have biased the clearance distribution
toward the dangerous ones.*

Built to give the footprint change (`4867728`) something to be measured by, since DS cannot resolve
anything at this corpus size (§1.11). `tools/clearance_lint.py` reports, once per `apply_plan`, the
smallest gap between the ego's body and any other road user's body over the burst, from the
recording. **1,812 plans against 49 collisions**, and the pass takes 14 seconds.

**Validated against the scorer before being used.** Per session, 52 of 60 agree on whether anything
was hit. The one false negative is a contact with map geometry, which the counter excludes by
design. The false positives are a ~9% plan-level over-count concentrated on bicycles and
pedestrians, whose blueprint boxes are generous next to the physics — so the overlap count is an
upper bound and the near-miss rate is the measurement.

**The result: the guide rewrite worked after all.** Paired over the nine steering routes, the rate
of plans leaving under half a metre fell by 5.4 points, sd 3.6, SE 1.2, t = −4.49 on 8 df, with
eight of nine routes improving. On the same 60 sessions DS moved +9.3 with t = 1.64 and an
interval from −4 to +23. The entry below concluded "the behaviour was not what was costing the
points"; the honest correction is narrower — **the change did improve the driving, and DS was too
blunt to show it.** §1.7 still stands on its own terms (eliminating a proxy does not prove the
proxy mattered), but its worked example was wrong about this corpus.

Two checks before believing the size of it. Plan length did not change (4.3 → 4.4 m driven per
plan), so the rate is not falling because plans got shorter; normalised, near misses per 100 m
driven fall 4.79 → 3.78, a 21% reduction. And the effect holds at every threshold from 0.15 m to
1.0 m and disappears at 1.5 m, which is the right shape: what went away were the tight passes, not
ordinary driving.

It is post-hoc — the counter was built after the collision analysis, on the same corpus — so this
wants confirming on fresh data before it is leaned on. Its intended use is as the pre-change
baseline for the footprint work, with the prediction written down in advance: the near-miss rate
should fall below the current arm's 16.7%, paired per route, detectable at t ≈ 3 on 30 sessions
rather than the ~160 DS would need.

Route-level rates stay out of the repo: `tmp/near-miss-baseline.md` in the gitignored scratch dir
of the working copy and beside the sessions on the run host.

New tooling: `tools/clearance_lint.py` and `tools/actor_extents.py` (blueprint extents, measured
once against a live server and cached, because the recording stores where every actor was but not
how big it is). One bug worth naming, caught by reading the output rather than by a test: the
arms printed in alphabetical order, so `BEFORE → AFTER` rendered as `AFTER → BEFORE` and the delta
read with its sign reversed. Direction is the one thing a reader takes straight off a report.

### 2026-09-18 — where the points actually go: a post-mortem of every collision

*Same 60 sessions as the A/B below; no new runs. CARLA recorder ground truth decoded for the 39
sessions that logged a collision, then 49 events reconstructed offline.*

The A/B left the question "if plan geometry was not costing the score, what is?" open. Answering it
needed no compute, only the sessions already on disk.

**The ledger.** Route completion is nearly free — 55 of 60 sessions finish, mean route score 94.9 —
so the entire loss is penalty. Vehicle contact accounts for 57% of all DS lost, more than every
other cause combined; the two classes that fire most often account for 8% (§1.9).

**What the driver was doing.** 49 collisions. 36 of them with no brake applied, 33 of those on the
throttle; only 2 under sustained braking. Median speed at impact 2.0 m/s, and below 1 m/s on a
third of them. These are not failures to stop in time — they are low-speed drives into things.

**It is not attention.** The driver had opened a camera within a second of impact on 45 of 49
events, and the camera covering the bearing the hit came from on 43 of 46. Exactly one event is
"a camera covered it and the driver never opened it". Two more fell in a blind sector of the rig
itself, between where the front-quarter cameras stop and where the rear camera starts — a rig
property with a measurable fix. The withdrawn "driver never looks" finding (§1.3) is now not just
unsupported but contradicted.

**It is metric estimation.** Contact registers when the two centres reach the sum of their
half-extents: a median 4.7 m nose to nose, 2.0 m flank to flank. Those are exactly the separations
the driver quotes in `why` as the clearance it is working with. It measures to the object and
spends the whole margin, reserving nothing for its own body — its bumper is about 2.4 m ahead of
the point it reasons from. On a single-pass reading of all 49, 22 are that error and 6 more are the
same error in time rather than space (time-to-arrival of oncoming traffic at a turn). Together 57%.

**Headroom is real.** Five of 49 are events no change to the driver would prevent — struck from
behind, or stationary and struck. The benchmark is not scoring a lottery.

Route-level detail, the per-event dump and the classification stay out of the repo:
`tmp/collision-postmortem.md` in the gitignored scratch dir of the working copy and beside the
sessions on the run host.

New tooling: `tools/collisions.py`, with tests. Three traps it now pins, all of which produce a
confident wrong answer rather than an error: the location in a collision message is the **ego's**
own position, not the struck actor's (median 0.21 m from the driver's pose over 49 events), so
using it as the hazard's position makes every impact look head-on; the recorder numbers its frames
from 1, so a missing offset returns poses from a different moment that still look like driving; and
a CARLA camera yaw is clockwise, so a sign error swaps which side the driver was looking at. The
first measure of "did the driver react" was also wrong and had to be replaced — any brake tick in
the five seconds before impact is true of almost every collision, because the controller brakes
whenever a plan runs out. Only the unbroken brake run that reaches the impact separates trying to
stop from happening to have braked.

Not done, and not authorised: mounting a range sensor, reporting the ego footprint in replies,
letting the controller reverse, or closing the blind sector. The post-mortem argues for them in
that order.

### 2026-09-18 — the guide rewrite A/B, completed at three attempts per arm

*Bench2Drive, sensors track, 10-route dev set, `claude-opus-5`, stock config, `TM_SEED=0`.
Three attempts per route in each arm, 60 sessions, each arm at a single pinned commit
(§1.5, verified from `session.json:repo_commit`).*

Completes the comparison that the 2026-09-17 entry below left at one attempt in the rewrite arm.

**Geometry: the rewrite landed in full.** The aim-point fault it targeted went from 71% of plans
to 0.4%; median aim bracket more than halved, waypoints per plan rose, reach fell toward the
distance a burst actually covers. ~900 plans per arm, so none of this is noise.

**Score: no measurable change.** Pooled over the nine steering routes the mean rose 9.3 DS, sd of
the per-route deltas 17.1, SE 5.7, t = 1.64 on 8 df, 95% CI −3.8 to +22.5; six of nine routes
improved (sign test p ≈ 0.5). Not one route was stable in both arms. Both of the routes that had
looked like clean wins on a single attempt were deterministic in the baseline and bimodal under
the rewrite — see §1.1.

Decision: **no further attempts.** The rewrite is kept on the geometry, which is unambiguous and
is what the edit was for; the DS question is left open rather than bought down, because ~160 more
sessions would move a p-value without changing what we would do (§1.7, §1.8). The open problem
moves upstream: the driver already opens more than two camera images per decision and still
collides, so what it fails to extract from them — not how it draws the path afterwards — is where
the score is going.

Two regressions logged and not explained: the straight-plan flag rose from 9% to 15% despite the
rewrite adding a warning about it (the benign reading is mechanical — shorter reach means fewer
plans extend into a bend — and is unverified), and worst-case tracking error grew while the median
held.

Tooling: `tools/pool_report.py` and `tools/looks.py` promoted out of the gitignored scratch dir
with tests. Four bugs were found in the pooler while promoting it, every one of which rendered as
a plausible result rather than an error — flags read as dict keys so every violation rate printed
0.0%, stability judged on the baseline arm alone, the route id scanned out of the path so a parent
directory could become the route, and the report-only route hardcoded. An instrument a conclusion
rests on belongs in `tools/` with tests, not in `tmp/`.

Route-level numbers stay out of the repo: `tmp/guide-ab-results.md`, in the gitignored scratch dir
of the working copy and beside the session dirs on the run host.

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
`tmp/guide-rewrite-ab.md` in the gitignored scratch dir of the working copy.

New tooling from this round: `tools/plan_lint.py` (offline plan geometry linter) and
`tools/percept_video.py` (one video panel per decision, from the images the driver opened).
