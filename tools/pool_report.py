#!/usr/bin/env python3
"""Compare two arms of a benchmark A/B over repeated attempts, and say what the result supports.

    python tools/pool_report.py BEFORE=dev10,base2,base3 AFTER=guide2,new2,new3

An *arm* is one or more session-dir prefixes. Each prefix is one attempt at the whole route set,
so three prefixes is three attempts per route. Both arms must have been run at a single commit
each: the driver harness is part of the benchmark definition (benchmark/PROTOCOL.md rule 2), so
attempts only pool within one commit. This tool reads ``session.json:repo_commit`` and refuses to
pool an arm that mixes commits.

Why this exists rather than a mean of each arm: on this benchmark a route's own attempt-to-attempt
spread is larger than any guidance change has ever moved it. Measured on the 10-route sensors dev
set, the median per-route spread over three attempts at one commit was 27 DS and the worst was 65.
A single-attempt delta is therefore not evidence, and neither is a difference of arm means quoted
without its noise. What this prints instead:

* every attempt's DS, per route, for both arms, with each arm's spread;
* which routes are *stable in both arms* -- the only routes where a per-route delta can be read.
  A route that repeats exactly in the baseline bounds the BASELINE's noise, not the other arm's,
  and the two come apart: the routes that made the strongest case for one guidance change were
  deterministic in the baseline and bimodal under the change;
* the paired test over routes, which is the headline number and usually smaller than it looks;
* a decomposition of that uncertainty into attempt noise (buy it down with more attempts) and
  genuine between-route variation in the effect (buy it down with more routes, not more
  attempts), with the projected standard error at other attempt counts. This is the number to
  look at before spending compute on a rerun.

Geometry and look-counts come from tools/plan_lint.py and tools/looks.py over every plan in the
arm: thousands of samples rather than tens, and in practice the part of the comparison that
carries a signal at all.

Nothing here is route-specific and nothing is written: per-route findings must stay out of the
repo (CLAUDE.md), so redirect the output to ``sessions/`` or outside the tree.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
import looks as looks_mod  # noqa: E402
from plan_lint import analyse_session  # noqa: E402

REPO = Path(__file__).resolve().parent.parent

DEFAULT_GLOB = "sessions/{prefix}_r*_sensors"
"""Where an attempt's sessions live.

This is a convention of the wrappers around ``scripts/run_eval.sh`` that drive a whole route set,
NOT something the repo enforces: ``run_eval.sh`` is told its session dir. Override with --glob if
your runs are laid out differently. The template needs a ``{prefix}`` and the path must carry the
route as a ``_r<route>_`` segment, which is what the route is keyed on.
"""

def route_matcher(pattern: str, prefix: str) -> "re.Pattern":
    """Turn the session-dir template into a regex that captures the route.

    Deriving the route from the template rather than scanning the path for ``_r<something>_`` is
    not pedantry: the scan matches the FIRST such segment anywhere in the path, so a parent
    directory whose name happens to contain one silently becomes the route id and every attempt
    collapses into one bogus route.
    """
    out = []
    for part in re.split(r"([*?])", pattern.format(prefix=prefix)):
        if part == "*":
            out.append("([^/]+)")
        elif part == "?":
            out.append("[^/]")
        else:
            out.append(re.escape(part))
    return re.compile("".join(out) + "/?$")


def route_of(matcher: "re.Pattern", path: str) -> Optional[str]:
    m = matcher.search(path)
    return m.group(1) if m and m.groups() else None

AIM_GAP_M = 4.0
"""Aim-point bracket wider than this counts as ``aim_gap``; matches tools/plan_lint.py's default."""

STABLE_SPREAD = 10.0
"""DS points. At or below this, a route's own attempts agree closely enough that a delta larger
than the spread cannot be explained by that route's noise. Above it, the outcome hinges on one
traffic interaction and the delta is not evidence either way."""

MIN_ATTEMPTS_FOR_STABLE = 2
"""A single attempt has a spread of zero by construction, which is not stability. Guard it."""

GEOM_FLAGS = ("aim_gap", "straight_ray", "reach_short", "stalled", "no_target_speed", "two_points")


# --------------------------------------------------------------------------- reading sessions

def read_report_only(path: Optional[Path] = None) -> set:
    """Routes that are reported but never used to steer a decision.

    The pinned standard benchmark route is simultaneously a member of the dev set. Letting its
    number steer guidance would tune on the headline route, so it is printed and then excluded
    from every pooled statistic. Read from benchmark/routes/standard.txt rather than hardcoded,
    so re-pinning the standard route cannot silently leave this stale.
    """
    path = path or REPO / "benchmark" / "routes" / "standard.txt"
    if not path.exists():
        return set()
    out = set()
    for line in path.read_text().splitlines():
        entry = line.split("#", 1)[0].strip()
        if entry:
            out.add(entry)
    return out


def score(session_dir: str) -> Optional[Dict]:
    """DS/RC/status for a FINISHED route, else None.

    ``result.json`` is created when the route *starts*, with an empty ``_checkpoint.records``, so
    its existence counts starts rather than finishes. Anything that waits on or counts completed
    routes must test that ``records`` is non-empty -- a barrier that only stat()ed this file
    reported a 20-route sweep as three-quarters done three minutes after launch.
    """
    path = os.path.join(session_dir, "result.json")
    if not os.path.exists(path):
        return None
    try:
        with open(path) as fh:
            records = json.load(fh)["_checkpoint"]["records"]
    except (json.JSONDecodeError, KeyError, OSError):
        return None
    if not records:
        return None
    rec = records[0]
    return {"ds": rec["scores"]["score_composed"],
            "rc": rec["scores"]["score_route"],
            "status": rec["status"],
            "infractions": {k: len(v) for k, v in rec["infractions"].items() if v}}


def repo_commit(session_dir: str) -> Optional[str]:
    path = os.path.join(session_dir, "session.json")
    try:
        with open(path) as fh:
            return json.load(fh).get("repo_commit")
    except (json.JSONDecodeError, OSError):
        return None


def arm(prefixes: Sequence[str], pattern: str = DEFAULT_GLOB,
        warn=None) -> Tuple[Dict[str, List[float]], Dict[str, List[str]], set]:
    """-> ({route: [ds per attempt]}, {route: [session dir]}, {repo_commit seen})."""
    warn = warn if warn is not None else (lambda m: print(m, file=sys.stderr))
    ds: Dict[str, List[float]] = {}
    dirs: Dict[str, List[str]] = {}
    commits = set()
    for prefix in prefixes:
        matcher = route_matcher(pattern, prefix)
        for d in sorted(glob.glob(pattern.format(prefix=prefix))):
            route = route_of(matcher, d)
            if route is None:
                warn("  skip %s: the glob has no * to read the route from" % d)
                continue
            s = score(d)
            if s is None:
                warn("  skip %s: unscored (no finished record)" % d)
                continue
            ds.setdefault(route, []).append(s["ds"])
            dirs.setdefault(route, []).append(d)
            commits.add(repo_commit(d) or "unknown")
    return ds, dirs, commits


# --------------------------------------------------------------------------- statistics

def spread(values: Sequence[float]) -> float:
    return max(values) - min(values) if len(values) > 1 else 0.0


def is_stable(values: Sequence[float], limit: float = STABLE_SPREAD) -> bool:
    """A single attempt is never stable, however tight its (zero) spread looks."""
    return len(values) >= MIN_ATTEMPTS_FOR_STABLE and spread(values) <= limit


def paired_delta(before: Dict[str, List[float]], after: Dict[str, List[float]],
                 exclude: set = frozenset()) -> Optional[Dict]:
    """Paired comparison over the routes both arms ran, plus a variance decomposition.

    The per-route delta varies for two different reasons, and they have different remedies:

    * each arm mean is itself noisy (``sigma_w^2/k`` per arm), which more ATTEMPTS shrinks;
    * the change may genuinely help some routes and hurt others (``tau^2``), which more attempts
      do NOT shrink -- only more routes do.

    ``tau2`` is the residual after subtracting the sampling part, so it is an estimate and can come
    out negative when attempt noise already explains everything; it is clamped at zero and should
    be read as "no detectable heterogeneity", not "none".
    """
    routes = sorted(r for r in set(before) & set(after) if r not in exclude)
    if len(routes) < 2:
        return None
    deltas = [statistics.mean(after[r]) - statistics.mean(before[r]) for r in routes]
    n = len(deltas)
    mean = statistics.mean(deltas)
    sd = statistics.stdev(deltas)
    se = sd / math.sqrt(n)

    # Pooled within-route, within-arm variance across both arms.
    ss, df = 0.0, 0
    for src in (before, after):
        for r in routes:
            v = src[r]
            if len(v) > 1:
                ss += statistics.variance(v) * (len(v) - 1)
                df += len(v) - 1
    var_w = ss / df if df else 0.0

    # Sampling part of each delta's variance, averaged over routes (handles unbalanced arms).
    sampling = statistics.mean(var_w / len(before[r]) + var_w / len(after[r]) for r in routes)
    tau2 = max(statistics.variance(deltas) - sampling, 0.0)

    return {"routes": routes, "deltas": deltas, "n": n, "mean": mean, "sd": sd, "se": se,
            "t": mean / se if se else float("inf"),
            "improved": sum(1 for d in deltas if d > 0),
            "var_within": var_w, "df_within": df, "sampling_var": sampling, "tau2": tau2,
            "attempts": min(min(len(before[r]), len(after[r])) for r in routes)}


def projected_se(stats: Dict, attempts: int, routes: Optional[int] = None) -> float:
    """SE of the mean delta if the same experiment were run with `attempts` per route per arm."""
    routes = routes or stats["n"]
    per_route = 2.0 * stats["var_within"] / attempts + stats["tau2"]
    return math.sqrt(per_route / routes)


def t_crit_95(df: int) -> float:
    """Two-sided 95% t critical value. Small table: this tool only ever has a handful of routes."""
    table = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306,
             9: 2.262, 10: 2.228, 12: 2.179, 15: 2.131, 20: 2.086, 30: 2.042, 60: 2.000}
    for k in sorted(table):
        if df <= k:
            return table[k]
    return 1.960


# --------------------------------------------------------------------------- per-plan geometry

def geometry(dirs: Dict[str, List[str]], aim_gap_m: float = AIM_GAP_M, warn=None) -> Dict:
    """Pooled per-plan geometry over every session in an arm.

    plan_lint reports a plan's violations in a ``flags`` LIST; reading them as boolean keys
    (``p.get("aim_gap")``) silently yields 0% for every flag, which looks exactly like "the
    guidance change did nothing" rather than like a bug. The list membership below is the fix, and
    tests/test_pool_report.py pins it.
    """
    warn = warn if warn is not None else (lambda m: print(m, file=sys.stderr))
    counts = {f: 0 for f in GEOM_FLAGS}
    plans = 0
    wps, reach, bracket, speed, rms = [], [], [], [], []
    for session_dirs in dirs.values():
        for d in session_dirs:
            try:
                res = analyse_session(Path(d), aim_gap_m)
            except Exception as exc:  # noqa: BLE001 - one unreadable session must not kill the run
                warn("  geometry skip %s: %s" % (d, exc))
                continue
            for p in res["plans"]:
                plans += 1
                flags = p.get("flags", [])
                for f in GEOM_FLAGS:
                    if f in flags:
                        counts[f] += 1
                wps.append(p["n_wps"])
                reach.append(p["reach_m"])
                bracket.append(p["aim_gap_m"])
                speed.append(p["speed"])
                if p.get("track_rms_m") is not None:
                    rms.append(p["track_rms_m"])
    n = max(plans, 1)
    med = lambda v: statistics.median(v) if v else float("nan")  # noqa: E731
    out = {"plans": float(plans)}
    for f in GEOM_FLAGS:
        out[f + "%"] = 100.0 * counts[f] / n
    out.update({"wps": med(wps), "reach_m": med(reach), "bracket_m": med(bracket),
                "speed_ms": med(speed), "track_rms_m": med(rms),
                "track_max_m": max(rms) if rms else float("nan")})
    return out


def look_rates(dirs: Dict[str, List[str]]) -> Dict:
    """Looks and camera frames per plan, pooled over an arm (see tools/looks.py)."""
    flat = [d for ds in dirs.values() for d in ds]
    tot = looks_mod.totals(flat)
    plans = max(tot["plans"], 1)
    return {"plans": float(tot["plans"]), "looks": float(tot["looks"]),
            "imgs": float(tot["imgs"]),
            "looks/plan": tot["looks"] / plans, "imgs/plan": tot["imgs"] / plans}


# --------------------------------------------------------------------------- reporting

def _fmt_attempts(values: Sequence[float], width: int) -> str:
    return "".join("%7.1f" % v for v in values) + "       " * (width - len(values))


def _parse_arm(spec: str) -> Tuple[str, List[str]]:
    name, _, prefixes = spec.partition("=")
    return name, [p for p in prefixes.split(",") if p]


def report(a_spec: str, b_spec: str, *, pattern: str = DEFAULT_GLOB,
           stable_spread: float = STABLE_SPREAD, aim_gap_m: float = AIM_GAP_M,
           report_only: Optional[set] = None, skip_geometry: bool = False,
           out=sys.stdout) -> int:
    say = lambda *a: print(*a, file=out)  # noqa: E731
    an, ap = _parse_arm(a_spec)
    bn, bp = _parse_arm(b_spec)
    report_only = read_report_only() if report_only is None else report_only

    ads, adirs, acommits = arm(ap, pattern)
    bds, bdirs, bcommits = arm(bp, pattern)
    if not ads or not bds:
        say("no scored sessions for %s" % (an if not ads else bn))
        return 2

    say("%s: %d routes, commit %s" % (an, len(ads), ", ".join(sorted(acommits))))
    say("%s: %d routes, commit %s" % (bn, len(bds), ", ".join(sorted(bcommits))))
    mixed = [n for n, c in ((an, acommits), (bn, bcommits)) if len(c) > 1]
    if mixed:
        say("")
        say("REFUSING TO POOL: arm(s) %s span more than one commit. The driver harness is part of"
            % ", ".join(mixed))
        say("the benchmark definition (benchmark/PROTOCOL.md rule 2), so attempts at different")
        say("commits are attempts at different benchmarks and do not pool.")
        return 1
    say("")

    na = max((len(v) for v in ads.values()), default=0)
    nb = max((len(v) for v in bds.values()), default=0)
    order = sorted(set(ads) | set(bds), key=lambda r: -statistics.mean(ads.get(r, [0.0])))

    head = ("route  " + "%-*s" % (7 * na, an) + " mean  sprd | "
            + "%-*s" % (7 * nb, bn) + " mean  sprd |  delta")
    say(head)
    say("-" * len(head))
    stable, unstable = [], []
    for r in order:
        a, b = ads.get(r, []), bds.get(r, [])
        if not a or not b:
            continue
        ma, mb, sa, sb = statistics.mean(a), statistics.mean(b), spread(a), spread(b)
        tag = "  (report-only)" if r in report_only else ""
        say("%-6s %s %5.1f %5.1f | %s %5.1f %5.1f | %+6.1f%s"
            % (r, _fmt_attempts(a, na), ma, sa, _fmt_attempts(b, nb), mb, sb, mb - ma, tag))
        if r in report_only:
            continue
        row = (r, ma, mb, sa, sb, len(a), len(b))
        (stable if is_stable(a, stable_spread) and is_stable(b, stable_spread)
         else unstable).append(row)

    say("")
    stats = paired_delta(ads, bds, exclude=report_only)
    if stats:
        tc = t_crit_95(stats["n"] - 1)
        lo, hi = stats["mean"] - tc * stats["se"], stats["mean"] + tc * stats["se"]
        gm_a = statistics.mean(statistics.mean(ads[r]) for r in stats["routes"])
        gm_b = statistics.mean(statistics.mean(bds[r]) for r in stats["routes"])
        say("Paired over %d routes (%d excluded as report-only):  %s %.1f -> %s %.1f"
            % (stats["n"], len(report_only & (set(ads) & set(bds))), an, gm_a, bn, gm_b))
        say("  mean delta %+.1f   sd %.1f   SE %.1f   t %.2f (df %d)   95%% CI %+.1f to %+.1f"
            % (stats["mean"], stats["sd"], stats["se"], stats["t"], stats["n"] - 1, lo, hi))
        say("  %d/%d routes improved" % (stats["improved"], stats["n"]))
        if lo <= 0.0 <= hi:
            say("  -> the interval spans zero: this experiment does not establish a DS change.")
        say("")
        say("Where that uncertainty comes from (per-route attempt sd %.1f DS, df %d):"
            % (math.sqrt(stats["var_within"]), stats["df_within"]))
        obs = statistics.variance(stats["deltas"])
        say("  attempt noise      %5.1f of %5.1f variance (%2.0f%%)  -- more ATTEMPTS shrink this"
            % (stats["sampling_var"], obs, 100.0 * stats["sampling_var"] / obs if obs else 0.0))
        say("  route-to-route     %5.1f of %5.1f variance (%2.0f%%)  -- only more ROUTES shrink this"
            % (stats["tau2"], obs, 100.0 * stats["tau2"] / obs if obs else 0.0))
        say("")
        say("  projected SE (and t for the observed %+.1f) at other experiment sizes:"
            % stats["mean"])
        for k in (stats["attempts"], stats["attempts"] * 2, stats["attempts"] * 4):
            se = projected_se(stats, k)
            say("    %2d attempts x %2d routes  (%3d sessions)  SE %.1f  t %.2f"
                % (k, stats["n"], 2 * k * stats["n"], se, stats["mean"] / se if se else 0.0))
        for rr in (stats["n"] * 2, stats["n"] * 4):
            se = projected_se(stats, stats["attempts"], rr)
            say("    %2d attempts x %2d routes  (%3d sessions)  SE %.1f  t %.2f"
                % (stats["attempts"], rr, 2 * stats["attempts"] * rr, se,
                   stats["mean"] / se if se else 0.0))
        floor = math.sqrt(stats["tau2"] / stats["n"]) if stats["tau2"] else 0.0
        say("    floor at %d routes, infinite attempts: SE %.1f" % (stats["n"], floor))
    say("")

    say("STABLE routes (spread <= %.0f in BOTH arms) -- a larger delta here is real:" % stable_spread)
    for r, ma, mb, sa, sb, _ka, _kb in stable:
        say("  %-6s %5.1f -> %5.1f  (%+.1f)   spread %.1f / %.1f" % (r, ma, mb, mb - ma, sa, sb))
    if not stable:
        say("  none -- no route repeats itself closely enough in both arms to carry a delta")
    say("")
    say("UNSTABLE routes -- at least one arm's own attempts disagree; not evidence either way:")
    for r, ma, mb, sa, sb, ka, kb in unstable:
        if ka < MIN_ATTEMPTS_FOR_STABLE or kb < MIN_ATTEMPTS_FOR_STABLE:
            why = "too few attempts (%d/%d)" % (ka, kb)
        elif sa > stable_spread and sb > stable_spread:
            why = "both"
        else:
            why = an if sa > stable_spread else bn
        say("  %-6s %5.1f -> %5.1f  (%+.1f)   spread %.1f / %.1f   unstable: %s"
            % (r, ma, mb, mb - ma, sa, sb, why))
    if not unstable:
        say("  none")

    if skip_geometry:
        return 0
    say("")
    ga, gb = geometry(adirs, aim_gap_m), geometry(bdirs, aim_gap_m)
    la, lb = look_rates(adirs), look_rates(bdirs)
    say("Per-plan geometry and look rate, pooled over every plan in each arm")
    say("  %-16s %10s %10s" % ("", an, bn))
    keys = ["plans"] + [f + "%" for f in GEOM_FLAGS] + \
           ["wps", "reach_m", "bracket_m", "speed_ms", "track_rms_m", "track_max_m"]
    for k in keys:
        say("  %-16s %10.1f %10.1f" % (k, ga[k], gb[k]))
    for k in ("looks/plan", "imgs/plan"):
        say("  %-16s %10.1f %10.1f" % (k, la[k], lb[k]))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("arms", nargs=2, metavar="NAME=prefix[,prefix...]",
                    help="the two arms; each prefix is one attempt at the whole route set")
    ap.add_argument("--glob", default=DEFAULT_GLOB,
                    help="session-dir template, needs {prefix} and a _r<route>_ segment "
                         "(default: %(default)s)")
    ap.add_argument("--stable-spread", type=float, default=STABLE_SPREAD, metavar="DS",
                    help="per-arm spread at or below which a route counts as stable "
                         "(default: %(default)s)")
    ap.add_argument("--aim-gap", type=float, default=AIM_GAP_M, metavar="M",
                    help="aim-bracket width that counts as aim_gap (default: %(default)s)")
    ap.add_argument("--no-report-only", action="store_true",
                    help="include the pinned standard route in the pooled statistics")
    ap.add_argument("--no-geometry", action="store_true",
                    help="scores only; skips the per-plan pass, which is the slow part")
    args = ap.parse_args(argv)
    return report(args.arms[0], args.arms[1], pattern=args.glob,
                  stable_spread=args.stable_spread, aim_gap_m=args.aim_gap,
                  report_only=set() if args.no_report_only else None,
                  skip_geometry=args.no_geometry)


if __name__ == "__main__":
    sys.exit(main())
