#!/usr/bin/env python3
"""Score probe sessions as a Fail2Drive result: Base vs Generalization, and the drop.

    python tools/f2d_score.py sessions/f2d_*              # markdown tables
    python tools/f2d_score.py sessions/f2d_* --json       # machine-readable

Fail2Drive is a *generalization* benchmark. Its 100 route pairs are built so that each
`Generalization_*` route is a shifted version of an in-distribution `Base_*` reference, and the
headline number is not either absolute score -- it is the **drop from Base to Generalization**.
tools/summarize_results.py reports one route at a time and cannot say that; this tool can.

Scoring follows upstream `third_party/fail2drive/tools/f2d_result_parser.py`, which stays the
authoritative reference (this is a stdlib reimplementation, not a copy: upstream needs pandas and
reads a single merged results tree, while a probe run is one session directory per route):

  DS       score_composed
  RC       score_route
  Success  EVERY infraction list empty, except min_speed_infractions and outside_route_lanes.
           Note Bench2Drive ignores only the first, so the same drive scores differently under the
           two benchmarks -- never pool them.
  HM       harmonic mean of DS and the success rate (in %), the figure the paper ranks on
  Drop %   100 * gen / base - 100, per split, per class and per scenario

Per-class figures are route-weighted across every route in the class, as upstream does, not an
unweighted mean of per-scenario means.

Two deliberate divergences from upstream, both reporting-only:

  * A partial run reports `n/a`, not a drop. Upstream's pct_change(0.0, 99.9) yields a
    credible-looking -100.0% when the truth is that the Generalization side has not run yet.
  * A session with NO result.json counts as DS 0 / not-successful rather than being dropped,
    because benchmark/PROTOCOL.md rule 3 reports a crashed or aborted session as a failure. A
    session WITH a record that is unscorable (`Failed - Agent couldn't be set up`, or a missing or
    non-numeric score) is excluded from every denominator and listed separately, as upstream does:
    CARLA pre-populates every infraction list as empty for a route the agent never ran, so counting
    such a record would score it as a *perfect* drive.

Stdlib only.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

SPLIT_DIR = Path(__file__).resolve().parents[1] / "third_party/fail2drive" / "fail2drive_split"
ROUTE_FILE_RE = re.compile(r"^(Base|Generalization)_(.+)_(\d{4})\.xml$")
ROUTE_ID_RE = re.compile(r"RouteScenario_(\d+)")

# A Generalization route's id is its Base reference's id + 1000, with the same scenario family --
# verified over all 100 pairs of the vendored split. That is what makes pair coverage reportable.
PAIR_OFFSET = 1000

IGNORED_INFRACTIONS_FOR_SUCCESS = {"min_speed_infractions", "outside_route_lanes"}
UNSCORABLE_STATUSES = {"Failed - Agent couldn't be set up"}

# Upstream's taxonomy and paper ordering, copied from third_party/fail2drive/tools/f2d_result_parser.py.
SCENARIO_CLASSES = {
    "Robustness": ["RightConstruction", "OppositeConstruction", "ImageOnObject",
                   "PassableObstacles", "PedestrianCrowd"],
    "Visual-lat": ["BadParking", "ConstructionPermutations", "CustomObstacles"],
    "Visual-lon": ["ObscuredStop", "HardBrake", "RightOfWay", "Animals", "PedestrianOtherBlocker"],
    "Behavior": ["ConstructionPedestrian", "Wall", "FullyBlocked", "PedestriansOnRoad"],
}
SCENARIO_TO_CLASS = {s: c for c, ss in SCENARIO_CLASSES.items() for s in ss}
SCENARIO_PAPER_ORDER = [
    "Wall", "FullyBlocked", "PedestriansOnRoad", "ConstructionPedestrian",
    "BadParking", "ConstructionPermutations", "CustomObstacles",
    "ObscuredStop", "HardBrake", "RightOfWay", "Animals", "PedestrianOtherBlocker",
    "RightConstruction", "OppositeConstruction", "ImageOnObject", "PassableObstacles",
    "PedestrianCrowd",
]
CLASS_PAPER_ORDER = ["Visual-lon", "Visual-lat", "Behavior", "Robustness"]
SPLITS = ("Base", "Generalization")


def route_map(split_dir: Path = SPLIT_DIR) -> Dict[int, Tuple[str, str]]:
    """route id -> (split, scenario), read off the vendored route filenames."""
    mapping: Dict[int, Tuple[str, str]] = {}
    for xml in sorted(split_dir.glob("*.xml")):
        m = ROUTE_FILE_RE.match(xml.name)
        if m:
            split, scenario, route_id = m.groups()
            mapping[int(route_id)] = (split, scenario.replace("TwoWays", ""))
    if not mapping:
        raise SystemExit(f"no Fail2Drive route XMLs in {split_dir} (is third_party/fail2drive/ present?)")
    return mapping


def is_success(record: Dict[str, Any]) -> bool:
    for name, entries in (record.get("infractions") or {}).items():
        if name not in IGNORED_INFRACTIONS_FOR_SUCCESS and entries:
            return False
    return True


def _route_id(session: Path, record: Optional[Dict[str, Any]]) -> Optional[int]:
    """The route id: from the record's route_id, else from what the launcher recorded."""
    if record:
        m = ROUTE_ID_RE.search(str(record.get("route_id") or ""))
        if m:
            return int(m.group(1))
    meta = session / "session.json"
    if meta.is_file():
        try:
            stem = str(json.loads(meta.read_text()).get("route") or "")
        except json.JSONDecodeError:
            return None
        m = ROUTE_FILE_RE.match(stem + ".xml")
        if m:
            return int(m.group(3))
    return None


def read_session(session: Path, routes: Dict[int, Tuple[str, str]]) -> Dict[str, Any]:
    row: Dict[str, Any] = {"session": session.name, "benchmark": None, "access": None,
                           "route_id": None, "split": None, "scenario": None,
                           "DS": None, "RC": None, "success": None, "status": None,
                           "usable": False, "note": None}
    meta = session / "session.json"
    if meta.is_file():
        try:
            m = json.loads(meta.read_text())
            row["benchmark"], row["access"] = m.get("benchmark"), m.get("access")
        except json.JSONDecodeError:
            pass

    result = session / "result.json"
    records = []
    if result.is_file():
        try:
            records = json.loads(result.read_text()).get("_checkpoint", {}).get("records") or []
        except json.JSONDecodeError:
            records = []
    record = records[0] if records else None
    row["route_id"] = _route_id(session, record)
    if row["route_id"] is None or row["route_id"] not in routes:
        row["note"] = "route not in the Fail2Drive split"
        return row
    row["split"], row["scenario"] = routes[row["route_id"]]

    if record is None:
        # PROTOCOL rule 3: a crashed or aborted session is a failure, not a missing measurement.
        row.update(status="no result", DS=0.0, RC=0.0, success=False, usable=True,
                   note="no result.json: scored as a failure (PROTOCOL rule 3)")
        return row

    row["status"] = record.get("status")
    scores = record.get("scores") or {}
    ds, rc = scores.get("score_composed"), scores.get("score_route")
    if row["status"] in UNSCORABLE_STATUSES or not isinstance(ds, (int, float)) \
            or not isinstance(rc, (int, float)):
        # CARLA leaves every infraction list empty for a route the agent never ran, so an
        # ungated record here would score as a perfect drive. Drop it from every denominator.
        row["note"] = f"unscorable record ({row['status']}): excluded"
        return row
    row.update(DS=float(ds), RC=float(rc), success=is_success(record), usable=True)
    return row


def harmonic_mean(a: float, b: float) -> float:
    if math.isnan(a) or math.isnan(b) or a < 0 or b < 0:
        return float("nan")
    return 0.0 if a == 0 or b == 0 else 2.0 / (1.0 / a + 1.0 / b)


def pct_change(new: float, old: float) -> float:
    if old == 0 or math.isnan(new) or math.isnan(old):
        return float("nan")
    return 100.0 * new / old - 100.0


def aggregate(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Route-weighted DS / SR / HM per split, over whatever rows are given."""
    if not rows:
        return {"n": 0, "DS": float("nan"), "SR": float("nan"), "HM": float("nan")}
    ds = sum(r["DS"] for r in rows) / len(rows)
    sr = 100.0 * sum(1 for r in rows if r["success"]) / len(rows)
    return {"n": len(rows), "DS": ds, "SR": sr, "HM": harmonic_mean(ds, sr)}


def _by_split(rows: List[Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
    return {s: aggregate([r for r in rows if r["split"] == s]) for s in SPLITS}


def score(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    usable = [r for r in rows if r["usable"]]
    overall = _by_split(usable)
    out: Dict[str, Any] = {
        "overall": overall,
        "drop": {k: pct_change(overall["Generalization"][k], overall["Base"][k])
                 for k in ("DS", "SR", "HM")},
        "partial": overall["Base"]["n"] == 0 or overall["Generalization"]["n"] == 0,
        "by_class": {}, "by_scenario": {},
    }
    for cls in CLASS_PAPER_ORDER:
        members = set(SCENARIO_CLASSES[cls])
        per = _by_split([r for r in usable if r["scenario"] in members])
        out["by_class"][cls] = {**per, "drop_HM": pct_change(per["Generalization"]["HM"],
                                                             per["Base"]["HM"])}
    for sc in SCENARIO_PAPER_ORDER:
        per = _by_split([r for r in usable if r["scenario"] == sc])
        if per["Base"]["n"] == 0 and per["Generalization"]["n"] == 0:
            continue
        out["by_scenario"][sc] = {**per, "class": SCENARIO_TO_CLASS[sc],
                                  "drop_HM": pct_change(per["Generalization"]["HM"],
                                                        per["Base"]["HM"])}

    ids = {r["route_id"] for r in usable}
    base_ids = {i for i in ids if i < PAIR_OFFSET}
    out["coverage"] = {
        "Base": len(base_ids), "Generalization": len(ids) - len(base_ids), "total_routes": 200,
        "complete_pairs": sum(1 for i in base_ids if i + PAIR_OFFSET in ids),
        "total_pairs": 100,
        "excluded": [{"session": r["session"], "note": r["note"]} for r in rows if not r["usable"]],
    }
    return out


def _fmt(x: float, suffix: str = "") -> str:
    return "n/a" if isinstance(x, float) and math.isnan(x) else f"{x:.1f}{suffix}"


def _drop(x: float, partial: bool) -> str:
    return "n/a" if partial else _fmt(x, "%")


def report(s: Dict[str, Any]) -> None:
    base, gen = s["overall"]["Base"], s["overall"]["Generalization"]
    cov = s["coverage"]
    print(f"# Fail2Drive: {cov['Base']}/100 Base + {cov['Generalization']}/100 Generalization routes, "
          f"{cov['complete_pairs']}/100 complete pairs\n")
    print("| split | n | DS | SR % | HM |")
    print("|---|---:|---:|---:|---:|")
    for name, agg in (("Base", base), ("Generalization", gen)):
        print(f"| {name} | {agg['n']} | {_fmt(agg['DS'])} | {_fmt(agg['SR'])} | {_fmt(agg['HM'])} |")
    d = s["drop"]
    print(f"\n**Drop (Base -> Generalization):** DS {_drop(d['DS'], s['partial'])} · "
          f"SR {_drop(d['SR'], s['partial'])} · HM {_drop(d['HM'], s['partial'])}")
    if s["partial"]:
        missing = "Generalization" if gen["n"] == 0 else "Base"
        print(f"\n> drop n/a: the {missing} side has no scored routes. This is a partial run, not a "
              f"score of zero -- Fail2Drive's headline number needs both sides.")

    for title, table, key in (("Per class", s["by_class"], "class"),
                              ("Per scenario", s["by_scenario"], "scenario")):
        if not table:
            continue
        print(f"\n## {title}\n")
        extra = "| class " if key == "scenario" else ""
        print(f"| {key} {extra}| Base n | Base HM | Gen n | Gen HM | drop % |")
        print(f"|---|{'---|' if key == 'scenario' else ''}---:|---:|---:|---:|---:|")
        for name, agg in table.items():
            b, g = agg["Base"], agg["Generalization"]
            cls = f"| {agg['class']} " if key == "scenario" else ""
            partial = b["n"] == 0 or g["n"] == 0
            print(f"| {name} {cls}| {b['n']} | {_fmt(b['HM'])} | {g['n']} | {_fmt(g['HM'])} | "
                  f"{_drop(agg['drop_HM'], partial)} |")

    if cov["excluded"]:
        print(f"\n## Excluded ({len(cov['excluded'])})\n")
        for e in cov["excluded"]:
            print(f"- `{e['session']}`: {e['note']}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--split-dir", type=Path, default=SPLIT_DIR,
                    help="Fail2Drive route XMLs (default %(default)s)")
    args = ap.parse_args()

    routes = route_map(args.split_dir)
    rows = [read_session(s, routes) for s in args.sessions if s.is_dir()]
    if not rows:
        print("no session directories found", file=sys.stderr)
        return 1
    wrong = [r["session"] for r in rows if r["benchmark"] not in (None, "fail2drive")]
    if wrong:
        print(f"ERROR: not Fail2Drive sessions: {', '.join(wrong)}", file=sys.stderr)
        print("       Bench2Drive and Fail2Drive have different routes, route budgets and success "
              "rules; their scores must never be pooled.", file=sys.stderr)
        return 1

    s = score(rows)
    if args.json:
        print(json.dumps({"routes": rows, **s}, indent=2))
        return 0
    report(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
