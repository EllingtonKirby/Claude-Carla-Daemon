#!/usr/bin/env python3
"""Index every collision in a corpus and reconstruct the decision that produced it.

    python tools/collisions.py BEFORE=dev10,base2,base3 AFTER=guide2,new2,new3
    python tools/collisions.py RUN=dev10 --jsonl tmp/collisions.jsonl --dump

Offline once ``recorder_truth.json`` is cached; needs no simulator itself.

Why this exists: a score says the drive was bad and ``tools/plan_lint.py`` says whether the plans
were drawable, but neither says what the driver was doing when it hit something. On the first
measured corpus, vehicle collisions were **57% of all lost DS** -- more than every other infraction
class combined, and an order of magnitude more than the classes that fire most often. There are
far fewer collisions than plans, but far more of them than there are routes, so they are the
tightest instrument available for the outcome itself rather than a proxy for it
(docs/JOURNAL.md §1.2, §1.7).

Each event is reduced to the things that separate the ways a collision happens:

* **geometry** -- where the other actor was relative to the ego when they touched, and whether
  either was moving. An impact behind the rear axle of a stationary ego is not a driving error.
* **reaction** -- whether the brake went on at all in the seconds before, and how long before.
* **attention** -- when the driver last opened a camera, which cameras, and whether any of them
  covered the bearing the hit came from.
* **intent** -- the plan in flight and the ``why`` the driver wrote when it sent it.

The classification into "never looked" / "looked and did not see" / "saw it and drove in anyway"
is a reading of those fields, not something this tool decides. It prints the measurements and the
driver's own words; ``--dump`` lays out one event per screen for that reading.

Three traps, each of which silently yields a confident wrong answer:

1. **The location in the collision message is the EGO's, not the other actor's.** It reads
   ``Agent collided against object with type=... and id=... at (x=..)``, which parses naturally as
   the thing that was hit. It is not: measured against the driver's own pose over 49 events the
   median separation is 0.21 m and the maximum 0.60 m. Every bearing here comes from
   ``recorder_truth.json`` instead, which has the other actor's pose as well.
2. **``trajectory.jsonl`` holds the driver's GNSS estimate, not the truth.** It is good enough for
   speed, throttle and brake, which is all it is used for below. Poses come from the recording.
3. **The recorder numbers its own frames from 1**, so a frame key is ``world_frame -
   frame_offset``. Getting that wrong yields poses from a different moment that still look
   plausible. :func:`alignment_error` checks the mapping against the driver's own track and the
   report refuses truth-derived fields if it is off.

Camera coverage is read from the session's own ``config.yaml`` snapshot, so a run that changed the
rig is scored against the rig it had. A CARLA camera yaw is clockwise; ego-frame bearing is the
negation (carla_probe/geometry.py).
"""

from __future__ import annotations

import argparse
import collections
import glob
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "tools"))

from carla_probe import geometry  # noqa: E402
from pool_report import route_matcher, route_of, repo_commit  # noqa: E402

DEFAULT_GLOB = "sessions/{prefix}_r*_sensors"

COLLISION_TYPES = ("collisions_vehicle", "collisions_pedestrian", "collisions_layout")

WINDOW_S = 5.0
"""How far back from the impact to look for a reaction. Five seconds is about three decisions at
the observed cadence, so it spans the decision that hit and the one that set it up."""

BRAKE_ON = 0.5
"""Brake command above this counts as a reaction rather than a trim."""

STOPPED_MPS = 0.5
"""At or below this an actor is treated as stationary. Above idle creep, below a walking pace."""

REAR_DEG = 110.0
"""Impact bearing beyond this is behind the ego's shoulder: it was hit, it did not hit."""

LATE_BRAKE_S = 1.0
"""Braking later than this before impact is a reaction to the collision, not an avoidance of it."""

ALIGN_TOLERANCE_M = 2.0
"""Median hero-vs-GNSS separation above this means the recorder frame mapping is wrong; the
sensors-track estimate itself tracks truth far better than this (trap 1 above)."""

PENALTY_COEF = {
    "collisions_pedestrian": 0.50,
    "collisions_vehicle": 0.60,
    "collisions_layout": 0.65,
    "red_light": 0.70,
    "stop_infraction": 0.80,
    "scenario_timeouts": 0.70,
    "yield_emergency_vehicle_infractions": 0.70,
}
"""Leaderboard 2.0's multiplicative penalties, one factor per logged event (Bench2Drive 0.0.4,
``leaderboard/utils/statistics_manager.py``). ``outside_route_lanes`` and ``min_speed_infractions``
are percentage-based: they scale the penalty once, by how much of the route was affected, rather
than once per event -- which is why a class that fires a thousand times can cost almost nothing
and a class that fires thirty times can cost more than all the others together. Never rank
infraction classes by how often they fire. :func:`loss_ledger` reconstructs the recorded
``score_penalty`` from this table and reports the mismatch, so a table gone stale is visible."""

_MSG = re.compile(r"type=(\S+) and id=(\d+) at \(x=(-?[\d.]+), y=(-?[\d.]+), z=(-?[\d.]+)\)")
_FRAME_PATH = re.compile(r"/frames/frame_0*(\d+)/([A-Za-z_]+)\.(?:jpg|jpeg|png)")
_BURIED_Z = -100.0
"""The recorder parks an actor that is not yet in play hundreds of metres below the map with a
believable x/y (tools/recorder_truth.py). Below this, the pose is not a pose."""


# --------------------------------------------------------------------------- reading a session

def _jsonl(path: str) -> List[dict]:
    if not os.path.exists(path):
        return []
    out = []
    with open(path) as fh:
        for line in fh:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def parse_event(row: dict) -> dict:
    """A ``score_events.jsonl`` row with the collision message decomposed.

    ``ego_xy_reported`` is named for what it is: the leaderboard writes the EGO's location into
    the message, not the struck actor's (trap 1 in the module docstring). Nothing downstream may
    use it as a position for ``other_id``.
    """
    ev = dict(row)
    m = _MSG.search(row.get("message", ""))
    if m:
        ev["other_type"], ev["other_id"] = m.group(1), int(m.group(2))
        ev["ego_xy_reported"] = (float(m.group(3)), float(m.group(4)))
    return ev


def collision_events(session_dir: str) -> List[dict]:
    """Collision rows from ``score_events.jsonl``, with the message parsed."""
    return [parse_event(e) for e in _jsonl(os.path.join(session_dir, "score_events.jsonl"))
            if e.get("type") in COLLISION_TYPES]


def trajectory(session_dir: str) -> Dict[int, dict]:
    return {r["step"]: r for r in _jsonl(os.path.join(session_dir, "trajectory.jsonl")) if "step" in r}


def issued(session_dir: str) -> List[dict]:
    """Commands the driver sent, in order, from ``decisions.jsonl`` (the rows carrying ``cmd``)."""
    return [d for d in _jsonl(os.path.join(session_dir, "decisions.jsonl")) if d.get("cmd")]


def looks_by_step(session_dir: str) -> Dict[int, set]:
    """Camera frames the driver actually OPENED, as ``{frame step: {camera, ...}}``.

    From its own tool calls, not from the reply it was handed: a look that returns four paths and
    is followed by no reads is not a look (docs/JOURNAL.md §1.3, tools/looks.py).
    """
    out: Dict[int, set] = collections.defaultdict(set)
    path = os.path.join(session_dir, "driver_transcript.jsonl")
    if not os.path.exists(path):
        return {}
    with open(path) as fh:
        for line in fh:
            for step, cam in _FRAME_PATH.findall(line):
                out[int(step)].add(cam)
    return dict(out)


def cameras(session_dir: str) -> List[dict]:
    """The rig this run used, from its own config snapshot. Empty if the snapshot is missing."""
    path = os.path.join(session_dir, "config.yaml")
    if not os.path.exists(path):
        return []
    cams, inside = [], False
    for raw in open(path):
        line = raw.rstrip("\n")
        if re.match(r"^cameras:\s*$", line):
            inside = True
            continue
        if inside:
            if not line.startswith((" ", "\t", "-")) and line.strip():
                break
            m = re.search(r"\{(.*)\}", line)
            if not m:
                continue
            d = {}
            for field in m.group(1).split(","):
                if ":" not in field:
                    continue
                k, v = field.split(":", 1)
                d[k.strip()] = v.strip()
            try:
                cams.append({"id": d["id"], "yaw": float(d["yaw"]), "fov": float(d["fov"])})
            except (KeyError, ValueError):
                continue
    return cams


def covering_cameras(cams: Sequence[dict], bearing_deg: float) -> List[str]:
    """Which cameras of the rig see the given ego-frame bearing (0 ahead, + left).

    A CARLA camera yaw is clockwise about a left-handed frame, so it points at ego bearing -yaw.
    """
    out = []
    for c in cams:
        if abs(geometry.wrap_deg(bearing_deg - (-c["yaw"]))) <= c["fov"] / 2.0:
            out.append(c["id"])
    return out


# --------------------------------------------------------------------------- ground truth

def truth(session_dir: str) -> Optional[dict]:
    path = os.path.join(session_dir, "recorder_truth.json")
    if not os.path.exists(path):
        return None
    try:
        return json.loads(open(path).read())
    except (json.JSONDecodeError, OSError):
        return None


def pose(tr: dict, world_frame: int, actor_id) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) of an actor at a world frame, or None if absent or not yet in play."""
    rec = tr["frames"].get(str(int(world_frame) - int(tr["frame_offset"])))
    if not rec:
        return None
    p = rec["actors"].get(str(actor_id))
    if not p or (len(p) > 3 and p[3] < _BURIED_Z):
        return None
    return float(p[0]), float(p[1]), float(p[2])


def alignment_error(tr: dict, traj: Dict[int, dict]) -> Optional[float]:
    """Median metres between the hero's recorded pose and the driver's own estimate.

    This is the check that the frame mapping is right. It cannot be separated from GNSS error, so
    it is a ceiling on the mapping error rather than a measurement of it -- which is all that is
    needed, because a mapping that is off by even a few frames puts the hero metres away.
    """
    errs = []
    for row in traj.values():
        if "frame" not in row:
            continue
        p = pose(tr, row["frame"], tr["hero_id"])
        if p:
            errs.append(math.hypot(p[0] - row["x"], p[1] - row["y"]))
    if not errs:
        return None
    errs.sort()
    return errs[len(errs) // 2]


def _speed(tr: dict, world_frame: int, actor_id, dt: float, back: int = 5) -> Optional[float]:
    a = pose(tr, world_frame - back, actor_id)
    b = pose(tr, world_frame, actor_id)
    if not a or not b:
        return None
    return math.hypot(b[0] - a[0], b[1] - a[1]) / (back * dt)


def truth_geometry(tr: dict, ev: dict, dt: float) -> dict:
    """Where the other actor was, and how each of them was moving, at the moment of contact."""
    out: dict = {"truth": False}
    frame = ev.get("frame")
    if frame is None or ev.get("other_id") is None:
        return out
    hero = pose(tr, frame, tr["hero_id"])
    other = pose(tr, frame, ev["other_id"])
    if not hero or not other:
        return out
    fwd, left = geometry.world_to_ego(hero[:2], hero[2], [other[0], other[1]])[0]
    ego_v = _speed(tr, frame, tr["hero_id"], dt)
    other_v = _speed(tr, frame, ev["other_id"], dt)
    out.update({
        "truth": True,
        "bearing_deg": round(math.degrees(math.atan2(left, fwd)), 1),
        "range_m": round(math.hypot(fwd, left), 2),
        "ego_speed_truth": None if ego_v is None else round(ego_v, 2),
        "other_speed_truth": None if other_v is None else round(other_v, 2),
        "other_heading_rel_deg": round(geometry.wrap_deg(other[2] - hero[2]), 1),
    })
    return out


# --------------------------------------------------------------------------- reconstruction

def _window(traj: Dict[int, dict], step: int, dt: float, window_s: float) -> List[dict]:
    first = step - int(round(window_s / dt))
    return [traj[s] for s in sorted(traj) if first <= s <= step]


def reaction(rows: Sequence[dict], dt: float) -> dict:
    """What the vehicle was doing when it hit, and for how long it had been doing it.

    ``brake_lead_s`` -- the first brake tick anywhere in the window -- is recorded but is close to
    useless on its own, and knowing why matters: the controller brakes for the remainder of a
    burst as soon as the plan runs out (``plan_done`` in carla_probe/pid.py), so on this corpus it
    reads a median of 4.25 s, meaning little more than "a plan ended recently". ``brake_run_s``,
    the unbroken run of braking that reaches the impact, is the one that separates a driver that
    was trying to stop from one that happened to brake earlier for an unrelated reason.
    """
    if not rows:
        return {"brake_lead_s": None, "brake_ticks": 0, "brake_run_s": None,
                "v_max": None, "v_impact": None}
    impact = rows[-1]
    braking = [r for r in rows if (r.get("brake") or 0.0) > BRAKE_ON]
    run = 0
    for r in reversed(rows):
        if (r.get("brake") or 0.0) > BRAKE_ON:
            run += 1
        else:
            break
    speeds = [r.get("speed", 0.0) for r in rows]
    return {
        "brake_lead_s": round((impact["step"] - braking[0]["step"]) * dt, 2) if braking else None,
        "brake_ticks": len(braking),
        "brake_run_s": round(run * dt, 2),
        "brake_at_impact": impact.get("brake"),
        "v_max": round(max(speeds), 2),
        "v_impact": round(impact.get("speed", 0.0), 2),
        "throttle_at_impact": impact.get("throttle"),
        "steer_at_impact": impact.get("steer"),
    }


def decision_in_flight(cmds: Sequence[dict], step: int, dt: float) -> dict:
    """The command that was running when the impact happened, and the one before it."""
    prior = [c for c in cmds if c.get("step", -1) <= step]
    if not prior:
        return {}
    cur = prior[-1]
    args = cur.get("args") or {}
    wps = args.get("waypoints") or []
    out = {
        "decision_i": cur.get("i"),
        "decision_cmd": cur.get("cmd"),
        "decision_step": cur.get("step"),
        "decision_age_s": round((step - cur.get("step", step)) * dt, 2),
        "target_speed": args.get("target_speed"),
        "n_wps": len(wps),
        "ticks": args.get("ticks"),
        "percept": args.get("percept"),
        "why": cur.get("why"),
    }
    if len(prior) > 1:
        out["prev_why"] = prior[-2].get("why")
    return out


def attention(looks: Dict[int, set], step: int, dt: float, window_s: float) -> dict:
    """When the driver last opened a camera before the impact, and which ones."""
    before = [s for s in looks if s <= step]
    if not before:
        return {"last_look_step": None, "last_look_age_s": None, "cams_last_look": [],
                "cams_in_window": [], "looks_in_window": 0}
    last = max(before)
    first = step - int(round(window_s / dt))
    in_window = [s for s in before if s >= first]
    cams_w: set = set()
    for s in in_window:
        cams_w |= looks[s]
    return {
        "last_look_step": last,
        "last_look_age_s": round((step - last) * dt, 2),
        "cams_last_look": sorted(looks[last]),
        "cams_in_window": sorted(cams_w),
        "looks_in_window": len(in_window),
    }


def flags(rec: dict) -> List[str]:
    """Mechanical observations, not a verdict. The verdict needs the ``why`` text read."""
    out = []
    v_max = rec.get("v_max")
    if v_max is not None and v_max <= STOPPED_MPS:
        out.append("ego_stopped")
    if rec.get("truth"):
        b = abs(rec["bearing_deg"])
        out.append("rear_impact" if b > REAR_DEG else "side_impact" if b > 50 else "front_impact")
        ov = rec.get("other_speed_truth")
        if ov is not None and ov <= STOPPED_MPS:
            out.append("other_stationary")
        covering = rec.get("cams_covering") or []
        if not covering:
            out.append("blind_sector")          # no camera saw that bearing: not an attention fault
        elif not set(covering) & set(rec.get("cams_in_window") or []):
            out.append("never_looked_that_way")
    if (rec.get("v_impact") or 0.0) < -STOPPED_MPS:
        out.append("reversing")
    run = rec.get("brake_run_s")
    if run is None or run <= 0.0:
        out.append("not_braking_at_impact")
        if (rec.get("throttle_at_impact") or 0.0) > 0.1:
            out.append("on_the_throttle")
    elif run < LATE_BRAKE_S:
        out.append("braking_started_late")
    else:
        out.append("braking_throughout")
    return out


def reconstruct(session_dir: str, route: Optional[str], arm: str,
                window_s: float = WINDOW_S) -> List[dict]:
    traj = trajectory(session_dir)
    events = collision_events(session_dir)
    if not events or not traj:
        return []
    steps = sorted(traj)
    dt = 0.05
    if len(steps) > 1:
        spans = [(traj[b]["t"] - traj[a]["t"]) / (b - a) for a, b in zip(steps, steps[1:])
                 if b > a and "t" in traj[a] and "t" in traj[b]]
        if spans:
            dt = round(sorted(spans)[len(spans) // 2], 4) or 0.05
    cmds, looks, cams = issued(session_dir), looks_by_step(session_dir), cameras(session_dir)
    tr = truth(session_dir)
    align = alignment_error(tr, traj) if tr else None
    use_truth = tr is not None and align is not None and align <= ALIGN_TOLERANCE_M

    out = []
    for n, ev in enumerate(events, 1):
        step = ev.get("step")
        rows = _window(traj, step, dt, window_s)
        rec = {
            "arm": arm, "route": route, "session": os.path.basename(session_dir),
            "commit": repo_commit(session_dir), "n": n, "type": ev["type"],
            "step": step, "t": ev.get("t"), "frame": ev.get("frame"),
            "other_type": ev.get("other_type"), "other_id": ev.get("other_id"),
            "align_err_m": None if align is None else round(align, 2),
        }
        rec.update(reaction(rows, dt))
        rec.update(decision_in_flight(cmds, step, dt))
        rec.update(attention(looks, step, dt, window_s))
        if use_truth:
            rec.update(truth_geometry(tr, ev, dt))
            if rec.get("truth"):
                rec["cams_covering"] = covering_cameras(cams, rec["bearing_deg"])
        else:
            rec["truth"] = False
        rec["flags"] = flags(rec)
        out.append(rec)
    return out


# --------------------------------------------------------------------------- where DS goes

def _record(session_dir: str) -> Optional[dict]:
    """The scored record, or None if the route started but never finished.

    ``result.json`` exists from route start with an empty ``records`` list, so its presence counts
    starts (docs/JOURNAL.md, tools/pool_report.py).
    """
    try:
        recs = json.load(open(os.path.join(session_dir, "result.json")))["_checkpoint"]["records"]
    except (OSError, ValueError, KeyError):
        return None
    return recs[0] if recs else None


def loss_ledger(session_dirs: Sequence[str]) -> dict:
    """How many DS points each infraction class costs, per session.

    A class's entry is *the points that removing that class alone would return*, holding the rest
    of the session fixed. The entries therefore do not partition the loss exactly -- penalties
    multiply, so removing two classes returns more than the sum of removing each -- but they rank
    the classes by what they are worth, which is the question worth asking before choosing what to
    study. ``route not completed`` is the DS lost to distance never driven rather than to any
    infraction.
    """
    loss: collections.Counter = collections.Counter()
    ds, rc, pen, residuals = [], [], [], []
    n = 0
    for d in session_dirs:
        rec = _record(d)
        if not rec:
            continue
        n += 1
        s_ds = rec["scores"]["score_composed"]
        s_rc = rec["scores"]["score_route"]
        s_pen = rec["scores"]["score_penalty"]
        ds.append(s_ds); rc.append(s_rc); pen.append(s_pen)
        inf = rec.get("infractions", {})
        discrete = 1.0
        for k, coef in PENALTY_COEF.items():
            discrete *= coef ** len(inf.get(k, []))
        for k, coef in PENALTY_COEF.items():
            hits = len(inf.get(k, []))
            if hits:
                loss[k] += s_rc * (s_pen / coef ** hits) - s_ds
        residual = s_pen / discrete if discrete else float("nan")
        residuals.append(residual)
        if residual < 0.999:
            loss["speed and lane (percentage-based)"] += s_rc * (s_pen / residual) - s_ds
        loss["route not completed"] += (100.0 - s_rc) * s_pen
    if not n:
        return {}
    residuals.sort()
    return {
        "sessions": n,
        "mean_ds": sum(ds) / n, "mean_rc": sum(rc) / n, "mean_penalty": sum(pen) / n,
        "completed": sum(1 for x in rc if x > 99.5),
        "loss": {k: v / n for k, v in loss.items()},
        "residual_median": residuals[len(residuals) // 2],
        "max_residual": residuals[-1],
    }


def print_ledger(led: dict) -> None:
    if not led:
        print("no scored sessions")
        return
    print("%d scored sessions: mean DS %.1f = route %.1f x penalty %.3f; %d finished the route"
          % (led["sessions"], led["mean_ds"], led["mean_rc"], led["mean_penalty"], led["completed"]))
    if led["max_residual"] > 1.001:
        print("  WARNING: reconstructed penalty exceeds the recorded one (max residual %.3f):"
              " PENALTY_COEF no longer matches the leaderboard" % led["max_residual"])
    total = sum(led["loss"].values()) or 1.0
    print("\nDS per session returned by removing each cause alone (they overlap; a ranking, not a split)")
    for k, v in sorted(led["loss"].items(), key=lambda kv: -kv[1]):
        print("  %-38s %5.1f   %5.1f%%" % (k, v, 100.0 * v / total))


# --------------------------------------------------------------------------- reporting

def _parse_arm(spec: str) -> Tuple[str, List[str]]:
    name, _, prefixes = spec.partition("=")
    return name, [p for p in prefixes.split(",") if p]


def sessions(specs: Sequence[str], pattern: str = DEFAULT_GLOB) -> List[Tuple[str, Optional[str], str]]:
    """(arm, route, dir) for every session the arm specs name, in order."""
    out = []
    for spec in specs:
        name, prefixes = _parse_arm(spec)
        for prefix in prefixes:
            matcher = route_matcher(pattern, prefix)
            for d in sorted(glob.glob(pattern.format(prefix=prefix))):
                out.append((name, route_of(matcher, d), d))
    return out


def gather(specs: Sequence[str], pattern: str = DEFAULT_GLOB,
           window_s: float = WINDOW_S) -> List[dict]:
    out = []
    for name, route, d in sessions(specs, pattern):
        out.extend(reconstruct(d, route, name, window_s))
    return out


def _tally(recs: Sequence[dict], key) -> "collections.Counter":
    c: collections.Counter = collections.Counter()
    for r in recs:
        v = key(r)
        for item in (v if isinstance(v, list) else [v]):
            c[item] += 1
    return c


def report(recs: Sequence[dict]) -> None:
    if not recs:
        print("no collisions found")
        return
    arms = sorted({r["arm"] for r in recs})
    n_truth = sum(1 for r in recs if r.get("truth"))
    print("%d collision events, %d with recorder ground truth" % (len(recs), n_truth))
    missing = sorted({r["session"] for r in recs if not r.get("truth")})
    if missing:
        print("  no usable truth in %d session(s): %s" % (len(missing), " ".join(missing[:6])))

    def table(title, counter, total):
        print("\n%s" % title)
        for k, v in counter.most_common():
            print("  %-28s %4d  %5.1f%%" % (k, v, 100.0 * v / total if total else 0.0))

    table("by type", _tally(recs, lambda r: r["type"]), len(recs))
    for a in arms:
        sub = [r for r in recs if r["arm"] == a]
        print("\narm %s: %d events over %d sessions"
              % (a, len(sub), len({r["session"] for r in sub})))
    table("observations (an event can carry several)", _tally(recs, lambda r: r["flags"]), len(recs))

    tru = [r for r in recs if r.get("truth")]
    if tru:
        print("\nimpact bearing (0 = dead ahead, + = ego's left)")
        buckets = [("ahead    |b|<=30", lambda b: abs(b) <= 30),
                   ("fwd quarter 30-70", lambda b: 30 < abs(b) <= 70),
                   ("abeam     70-110", lambda b: 70 < abs(b) <= 110),
                   ("behind    >110", lambda b: abs(b) > 110)]
        for label, test in buckets:
            hits = [r for r in tru if test(r["bearing_deg"])]
            left = sum(1 for r in hits if r["bearing_deg"] > 0)
            print("  %-20s %4d   (left %d / right %d)" % (label, len(hits), left, len(hits) - left))
        moving = [r for r in tru if (r.get("other_speed_truth") or 0) > STOPPED_MPS]
        print("\n  other actor moving: %d / %d" % (len(moving), len(tru)))
        stopped = [r for r in tru if (r.get("v_max") or 0) <= STOPPED_MPS]
        print("  ego stationary throughout the window: %d / %d" % (len(stopped), len(tru)))

    print("\nwhat the vehicle was doing at the moment of contact")
    runs = [r.get("brake_run_s") or 0.0 for r in recs]
    print("  not braking                       %4d   (of which on the throttle: %d)"
          % (sum(1 for x in runs if x <= 0.0),
             sum(1 for r in recs if "on_the_throttle" in r["flags"])))
    print("  braking for <%.1fs                 %4d" % (LATE_BRAKE_S,
                                                        sum(1 for x in runs if 0 < x < LATE_BRAKE_S)))
    print("  braking for >=%.1fs and still hit  %4d" % (LATE_BRAKE_S,
                                                        sum(1 for x in runs if x >= LATE_BRAKE_S)))
    print("  reversing into it                 %4d" % sum(1 for r in recs if "reversing" in r["flags"]))
    speeds = sorted(r["v_impact"] for r in recs if r.get("v_impact") is not None)
    if speeds:
        print("  speed at impact: median %.1f m/s, max %.1f, below 1 m/s on %d of %d"
              % (speeds[len(speeds) // 2], speeds[-1],
                 sum(1 for x in speeds if abs(x) < 1.0), len(speeds)))

    print("\nattention")
    ages = sorted(r["last_look_age_s"] for r in recs if r.get("last_look_age_s") is not None)
    if ages:
        print("  time since the driver last opened a camera: median %.1fs, max %.1fs"
              % (ages[len(ages) // 2], ages[-1]))
    blind = [r for r in recs if "blind_sector" in r["flags"]]
    unseen = [r for r in recs if "never_looked_that_way" in r["flags"]]
    print("  impact bearing outside every camera's field of view   %4d" % len(blind))
    print("  covered by a camera the driver did not open in window  %4d" % len(unseen))


def dump(recs: Sequence[dict]) -> None:
    for r in recs:
        print("=" * 100)
        print("%s  route %s  %s #%d  step %s  t=%.1fs  [%s]"
              % (r["session"], r["route"], r["type"], r["n"], r["step"], r.get("t") or 0.0,
                 " ".join(r["flags"])))
        if r.get("truth"):
            print("  hit %s at bearing %+.0f deg, %.1f m; other %s (%.1f m/s), ego %.1f m/s"
                  % (r.get("other_type"), r["bearing_deg"], r["range_m"],
                     "moving" if (r.get("other_speed_truth") or 0) > STOPPED_MPS else "stationary",
                     r.get("other_speed_truth") or 0.0, r.get("ego_speed_truth") or 0.0))
            print("  cameras covering that bearing: %s" % (", ".join(r.get("cams_covering") or []) or "NONE"))
        print("  ego: v=%.1f m/s (max %.1f in window), throttle %s brake %s, braking for %ss"
              % (r.get("v_impact") or 0.0, r.get("v_max") or 0.0, r.get("throttle_at_impact"),
                 r.get("brake_at_impact"), r.get("brake_run_s")))
        print("  last look %s steps ago (%s s), opened %s; in window %s"
              % (None if r.get("last_look_step") is None else r["step"] - r["last_look_step"],
                 r.get("last_look_age_s"), ", ".join(r.get("cams_last_look") or []) or "nothing",
                 ", ".join(r.get("cams_in_window") or []) or "nothing"))
        print("  plan #%s issued %ss earlier, %s wps, target_speed %s"
              % (r.get("decision_i"), r.get("decision_age_s"), r.get("n_wps"), r.get("target_speed")))
        if r.get("prev_why"):
            print("  before: %s" % r["prev_why"])
        print("  WHY:    %s" % r.get("why"))


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arms", nargs="+", metavar="NAME=prefix[,prefix...]",
                    help="named arm and the session-dir prefixes that make it up")
    ap.add_argument("--glob", default=DEFAULT_GLOB,
                    help="session-dir template; '{prefix}' is substituted (default: %(default)s)")
    ap.add_argument("--window", type=float, default=WINDOW_S, metavar="S",
                    help="seconds before impact to search for a reaction (default: %(default)s)")
    ap.add_argument("--jsonl", metavar="PATH", help="write one JSON record per event")
    ap.add_argument("--dump", action="store_true", help="print each event in full, for reading")
    ap.add_argument("--no-ledger", action="store_true",
                    help="skip the DS-loss ledger that says whether collisions are worth studying")
    args = ap.parse_args(argv)

    dirs = sessions(args.arms, args.glob)
    if not args.no_ledger:
        print_ledger(loss_ledger([d for _, _, d in dirs]))
        print()
    recs = gather(args.arms, args.glob, args.window)
    if args.jsonl:
        with open(args.jsonl, "w") as fh:
            for r in recs:
                fh.write(json.dumps(r) + "\n")
        print("wrote %d records to %s\n" % (len(recs), args.jsonl))
    report(recs)
    if args.dump:
        print()
        dump(recs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
