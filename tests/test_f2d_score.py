"""Offline tests for the Fail2Drive scorer. No simulator, no pandas, no network."""

from __future__ import annotations

import ast
import json
import math
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import f2d_score as f2d  # noqa: E402

UPSTREAM_PARSER = Path(__file__).resolve().parents[1] / "third_party/fail2drive" / "tools" / "f2d_result_parser.py"


def _upstream_constants():
    """Read upstream's constants without importing it (it needs pandas)."""
    tree = ast.parse(UPSTREAM_PARSER.read_text())
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    return out


# --------------------------------------------------------------------------- the scoring rules
def test_taxonomy_matches_upstream():
    """The classes, the paper orders and the ignored infractions are upstream's, not ours."""
    up = _upstream_constants()
    assert f2d.SCENARIO_CLASSES == up["SCENARIO_CLASSES"]
    assert f2d.SCENARIO_PAPER_ORDER == up["SCENARIO_PAPER_ORDER"]
    assert f2d.CLASS_PAPER_ORDER == up["CLASS_PAPER_ORDER"]
    assert f2d.IGNORED_INFRACTIONS_FOR_SUCCESS == up["IGNORED_INFRACTIONS_FOR_SUCCESS"]
    assert set(f2d.SCENARIO_PAPER_ORDER) == set(f2d.SCENARIO_TO_CLASS)


def test_route_file_regex_behaves_like_upstream():
    # Upstream compiles its regex, so compare behaviour rather than source text.
    assert f2d.ROUTE_FILE_RE.match("Base_Animals_0075.xml").groups() == ("Base", "Animals", "0075")
    assert f2d.ROUTE_FILE_RE.match("Generalization_PedestriansOnRoad_1085.xml").groups() \
        == ("Generalization", "PedestriansOnRoad", "1085")
    assert f2d.ROUTE_FILE_RE.match("route_2091.xml") is None


def test_success_ignores_exactly_two_infractions():
    assert f2d.is_success({"infractions": {}})
    assert f2d.is_success({"infractions": {"min_speed_infractions": ["x"], "outside_route_lanes": ["y"],
                                           "collisions_vehicle": []}})
    # Bench2Drive ignores only min_speed_infractions; Fail2Drive also ignores outside_route_lanes.
    assert not f2d.is_success({"infractions": {"collisions_vehicle": ["hit"]}})
    assert not f2d.is_success({"infractions": {"red_light": ["ran"]}})


@pytest.mark.parametrize("a,b,expected", [
    (100.0, 100.0, 100.0), (50.0, 50.0, 50.0), (0.0, 99.9, 0.0), (99.9, 0.0, 0.0),
])
def test_harmonic_mean(a, b, expected):
    assert f2d.harmonic_mean(a, b) == pytest.approx(expected)


def test_harmonic_mean_nan_on_negative():
    assert math.isnan(f2d.harmonic_mean(-1.0, 50.0))
    assert math.isnan(f2d.harmonic_mean(float("nan"), 50.0))


def test_pct_change():
    assert f2d.pct_change(50.0, 100.0) == pytest.approx(-50.0)
    assert f2d.pct_change(120.0, 100.0) == pytest.approx(20.0)
    assert math.isnan(f2d.pct_change(50.0, 0.0))


# --------------------------------------------------------------------------- the vendored split
def test_route_map_is_the_full_benchmark():
    rm = f2d.route_map()
    assert len(rm) == 200
    assert sum(1 for s, _ in rm.values() if s == "Base") == 100
    assert sum(1 for s, _ in rm.values() if s == "Generalization") == 100
    assert set(sc for _, sc in rm.values()) == set(f2d.SCENARIO_PAPER_ORDER)


def test_every_base_route_has_its_generalization_twin():
    """The pairing the coverage line reports: gen id == base id + 1000, same scenario."""
    rm = f2d.route_map()
    for rid, (split, scenario) in rm.items():
        if split == "Base":
            twin = rm.get(rid + f2d.PAIR_OFFSET)
            assert twin == ("Generalization", scenario), rid


# --------------------------------------------------------------------------- reading sessions
def _session(tmp_path, name, *, route, record=None, benchmark="fail2drive", write_result=True):
    d = tmp_path / name
    d.mkdir()
    (d / "session.json").write_text(json.dumps({"benchmark": benchmark, "access": "sensors",
                                                "route": route}))
    if write_result:
        (d / "result.json").write_text(json.dumps({"_checkpoint": {"records": [record]}}))
    return d


def _record(route_id, ds, rc=100.0, infractions=None, status="Completed"):
    return {"route_id": f"RouteScenario_{route_id}_rep0", "status": status,
            "scores": {"score_composed": ds, "score_route": rc, "score_penalty": 1.0},
            "infractions": infractions or {}}


def test_reads_a_scored_session(tmp_path):
    rm = f2d.route_map()
    row = f2d.read_session(_session(tmp_path, "s", route="Base_Animals_0075",
                                    record=_record(75, 88.0)), rm)
    assert (row["usable"], row["split"], row["scenario"], row["DS"], row["success"]) \
        == (True, "Base", "Animals", 88.0, True)


def test_route_id_falls_back_to_session_json(tmp_path):
    """A record with no parseable route_id still resolves through the launcher's metadata."""
    rec = _record(75, 88.0)
    rec["route_id"] = None
    row = f2d.read_session(_session(tmp_path, "s", route="Base_Animals_0075", record=rec), f2d.route_map())
    assert row["route_id"] == 75 and row["usable"]


def test_missing_result_is_a_failure_not_a_gap(tmp_path):
    """PROTOCOL rule 3: a crashed session is reported as DS 0, and counts."""
    row = f2d.read_session(_session(tmp_path, "s", route="Base_Animals_0075", write_result=False),
                           f2d.route_map())
    assert row["usable"] and row["DS"] == 0.0 and row["success"] is False


def test_unscorable_record_is_excluded(tmp_path):
    """CARLA leaves the infraction lists empty for a route the agent never ran."""
    rm = f2d.route_map()
    bad = _record(75, 0.0, status="Failed - Agent couldn't be set up")
    assert f2d.read_session(_session(tmp_path, "a", route="Base_Animals_0075", record=bad), rm)["usable"] is False
    nan_scores = _record(76, None)
    assert f2d.read_session(_session(tmp_path, "b", route="Base_Animals_0076", record=nan_scores), rm)["usable"] is False


# --------------------------------------------------------------------------- aggregation
def _rows(*specs):
    """(route_id, split, scenario, DS, success) -> scorer rows."""
    return [{"session": f"s{i}", "benchmark": "fail2drive", "access": "sensors", "route_id": rid,
             "split": sp, "scenario": sc, "DS": ds, "RC": ds, "success": ok, "status": "Completed",
             "usable": True, "note": None}
            for i, (rid, sp, sc, ds, ok) in enumerate(specs)]


def test_drop_is_reported_for_a_complete_pair():
    s = f2d.score(_rows((75, "Base", "Animals", 100.0, True),
                        (1075, "Generalization", "Animals", 50.0, False)))
    assert s["partial"] is False
    assert s["overall"]["Base"]["HM"] == pytest.approx(100.0)
    assert s["overall"]["Generalization"]["HM"] == pytest.approx(0.0)   # SR 0 -> HM 0
    assert s["drop"]["DS"] == pytest.approx(-50.0)
    assert s["coverage"]["complete_pairs"] == 1


def test_partial_run_reports_na_not_minus_100():
    """Upstream's pct_change(0, 99.9) would print a credible-looking -100.0%."""
    s = f2d.score(_rows((75, "Base", "Animals", 99.9, True)))
    assert s["partial"] is True
    assert s["coverage"]["complete_pairs"] == 0
    assert f2d._drop(s["drop"]["DS"], s["partial"]) == "n/a"
    # ... and adding the missing side restores a real number.
    s2 = f2d.score(_rows((75, "Base", "Animals", 99.9, True),
                         (1075, "Generalization", "Animals", 49.95, True)))
    assert s2["partial"] is False
    assert s2["drop"]["DS"] == pytest.approx(-50.0)


def test_class_aggregation_is_route_weighted():
    """Visual-lon holds Animals and HardBrake; 2 Animals routes must outweigh 1 HardBrake route."""
    s = f2d.score(_rows((75, "Base", "Animals", 0.0, False),
                        (76, "Base", "Animals", 0.0, False),
                        (35, "Base", "HardBrake", 90.0, True)))
    # route-weighted DS = (0 + 0 + 90) / 3 = 30; an unweighted mean of scenario means would be 45.
    assert s["by_class"]["Visual-lon"]["Base"]["DS"] == pytest.approx(30.0)
    assert s["by_class"]["Visual-lon"]["Base"]["n"] == 3


def test_unrelated_scenarios_do_not_appear():
    s = f2d.score(_rows((75, "Base", "Animals", 50.0, True)))
    assert set(s["by_scenario"]) == {"Animals"}
    assert s["by_class"]["Behavior"]["Base"]["n"] == 0


def test_unscored_record_is_a_failure_not_a_perfect_run(tmp_path):
    """The evaluator starts a record at status 'Started' with zeroed scores and every infraction
    list EMPTY. Read at face value that is a successful DS 0 route, which would inflate the success
    rate; compute_route_statistics is what replaces 'Started' with a real status."""
    started = _record(75, 0.0, rc=0.0, status="Started")
    row = f2d.read_session(_session(tmp_path, "s", route="Base_Animals_0075", record=started),
                           f2d.route_map())
    assert row["usable"] is True          # PROTOCOL rule 3: it counts, as a failure
    assert row["DS"] == 0.0 and row["success"] is False
    assert "never scored" in row["note"]


def test_empty_records_list_is_the_pre_run_stub(tmp_path):
    """result.json is written before the route runs, with records: []."""
    d = tmp_path / "s"
    d.mkdir()
    (d / "session.json").write_text(json.dumps({"benchmark": "fail2drive", "access": "sensors",
                                                "route": "Base_Animals_0075"}))
    (d / "result.json").write_text(json.dumps({"_checkpoint": {"records": []},
                                               "entry_status": "Started"}))
    row = f2d.read_session(d, f2d.route_map())
    assert row["usable"] is True and row["DS"] == 0.0 and row["success"] is False


@pytest.mark.parametrize("status,usable", [
    ("Perfect", True), ("Completed", True), ("Failed - Agent crashed", True),
    ("Failed - Simulation crashed", True),
    ("Started", True),                                    # counted, as a failure
    ("Failed - Agent couldn't be set up", False),         # excluded, as upstream does
])
def test_status_decides_scorable_vs_excluded(tmp_path, status, usable):
    row = f2d.read_session(
        _session(tmp_path, status.replace(" ", "_").replace("'", ""), route="Base_Animals_0075",
                 record=_record(75, 70.0, status=status)), f2d.route_map())
    assert row["usable"] is usable
    if status == "Started":
        assert row["DS"] == 0.0 and row["success"] is False   # never the record's own 70.0
