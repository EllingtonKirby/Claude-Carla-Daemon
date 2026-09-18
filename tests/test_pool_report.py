"""Offline tests for the A/B pooler. No simulator.

Most of these pin mistakes that were actually made and that a reader cannot see in the output,
because each one degrades silently into a plausible-looking number rather than an error:

* counting ``result.json`` files counts route STARTS, not finishes;
* reading plan_lint's ``flags`` list as boolean keys reports 0% for every violation, which reads
  as "the change did nothing";
* calling a route stable on one arm's spread, or on a single attempt's spread of zero.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import pool_report as pr  # noqa: E402


# --------------------------------------------------------------------------- session reading

def _session(tmp_path: Path, name: str, *, ds=None, records_empty=False, no_result=False,
             commit="abc1234") -> Path:
    d = tmp_path / "sessions" / name
    d.mkdir(parents=True)
    (d / "session.json").write_text(json.dumps({"repo_commit": commit}))
    if no_result:
        return d
    records = [] if records_empty else [{
        "scores": {"score_composed": ds, "score_route": 100.0},
        "status": "Completed",
        "infractions": {"min_speed_infractions": ["x"], "collisions_vehicle": []},
    }]
    (d / "result.json").write_text(json.dumps({"_checkpoint": {"records": records}}))
    return d


def test_a_started_route_is_not_a_finished_one(tmp_path):
    """result.json exists from route start with an empty records list. Counting the file counts
    starts; a barrier built that way called a sweep three-quarters done three minutes in."""
    started = _session(tmp_path, "arm_r1234_sensors", records_empty=True)
    assert (started / "result.json").exists()
    assert pr.score(str(started)) is None


def test_a_finished_route_yields_its_scores(tmp_path):
    d = _session(tmp_path, "arm_r1234_sensors", ds=61.5)
    s = pr.score(str(d))
    assert s["ds"] == 61.5 and s["rc"] == 100.0 and s["status"] == "Completed"
    assert s["infractions"] == {"min_speed_infractions": 1}  # empty lists dropped


def test_a_missing_or_corrupt_result_is_skipped_not_fatal(tmp_path):
    assert pr.score(str(_session(tmp_path, "arm_r1_sensors", no_result=True))) is None
    d = _session(tmp_path, "arm_r2_sensors", ds=50.0)
    (d / "result.json").write_text("{not json")
    assert pr.score(str(d)) is None


def test_the_route_is_read_from_the_glob_not_scanned_out_of_the_path():
    """Scanning the path for `_r<something>_` matches the FIRST such segment anywhere in it, so a
    parent directory containing one becomes the route id and every attempt collapses into one
    bogus route. Deriving it from the template is what makes that impossible."""
    m = pr.route_matcher("/runs/test_route_and_things/sessions/{prefix}_r*_sensors", "one")
    assert pr.route_of(m, "/runs/test_route_and_things/sessions/one_r1234_sensors") == "1234"
    # A prefix that itself contains `_r` must not shift the capture either.
    m2 = pr.route_matcher("sessions/{prefix}_r*_sensors", "base_run")
    assert pr.route_of(m2, "sessions/base_run_r1234_sensors") == "1234"
    # Non-numeric route names (Fail2Drive) work the same way.
    m3 = pr.route_matcher("sessions/{prefix}_r*_sensors", "a1")
    assert pr.route_of(m3, "sessions/a1_rMergeIntoTraffic_3_sensors") == "MergeIntoTraffic_3"


def test_arm_keys_by_route_and_collects_commits(tmp_path):
    _session(tmp_path, "one_r1234_sensors", ds=50.0, commit="aaa")
    _session(tmp_path, "two_r1234_sensors", ds=70.0, commit="aaa")
    _session(tmp_path, "one_r5678_sensors", ds=10.0, commit="aaa")
    _session(tmp_path, "two_r5678_sensors", records_empty=True, commit="aaa")
    ds, dirs, commits = pr.arm(["one", "two"], str(tmp_path / pr.DEFAULT_GLOB), warn=lambda m: None)
    assert ds == {"1234": [50.0, 70.0], "5678": [10.0]}
    assert len(dirs["1234"]) == 2
    assert commits == {"aaa"}


# --------------------------------------------------------------------------- stability

def test_spread_and_stability():
    assert pr.spread([50.0, 50.0, 50.0]) == 0.0
    assert pr.spread([50.0, 100.0]) == 50.0
    assert pr.is_stable([50.0, 52.0, 55.0])
    assert not pr.is_stable([50.0, 100.0])


def test_one_attempt_is_never_stable():
    """A single attempt has a spread of zero by construction. That is not stability, and treating
    it as such is how a one-attempt delta gets reported as a real effect."""
    assert pr.spread([50.0]) == 0.0
    assert not pr.is_stable([50.0])


def test_stability_needs_both_arms(tmp_path, capsys):
    """A baseline that repeats bounds the BASELINE's noise, not the other arm's. The route that
    made the strongest case for one guidance change repeated exactly three times in the baseline
    and then came back bimodal under the change. Route ids here are synthetic: per-route results
    stay out of the repo (CLAUDE.md)."""
    for name, ds in [("b1_r1111_sensors", 50.0), ("b2_r1111_sensors", 50.0),
                     ("b3_r1111_sensors", 50.0),
                     ("a1_r1111_sensors", 100.0), ("a2_r1111_sensors", 50.0),
                     ("a3_r1111_sensors", 100.0),
                     ("b1_r2222_sensors", 60.0), ("b2_r2222_sensors", 61.0), ("b3_r2222_sensors", 62.0),
                     ("a1_r2222_sensors", 70.0), ("a2_r2222_sensors", 71.0), ("a3_r2222_sensors", 72.0)]:
        _session(tmp_path, name, ds=ds)
    rc = pr.report("BEFORE=b1,b2,b3", "AFTER=a1,a2,a3",
                   pattern=str(tmp_path / pr.DEFAULT_GLOB), report_only=set(),
                   skip_geometry=True, out=sys.stdout)
    out = capsys.readouterr().out
    assert rc == 0
    stable = out.split("STABLE routes")[1].split("UNSTABLE")[0]
    assert "2222" in stable and "1111" not in stable
    assert "1111" in out.split("UNSTABLE")[1]
    assert "unstable: AFTER" in out


# --------------------------------------------------------------------------- report-only route

def test_report_only_comes_from_the_pinned_route_file(tmp_path):
    f = tmp_path / "standard.txt"
    f.write_text("# The pinned single-route benchmark.\n2091  # Town12\n")
    assert pr.read_report_only(f) == {"2091"}
    assert pr.read_report_only(tmp_path / "absent.txt") == set()


def test_the_repos_pinned_route_is_what_gets_excluded():
    """Hardcoding the route number here would go stale the moment the standard route is re-pinned,
    and the failure mode is tuning on the headline route without noticing."""
    pinned = pr.read_report_only()
    assert pinned and all(r.isdigit() for r in pinned)


def test_report_only_route_is_printed_but_excluded_from_the_pooled_stats(tmp_path, capsys):
    pinned = sorted(pr.read_report_only())[0]
    for arm_prefix, ds in [("b1", 10.0), ("b2", 12.0), ("a1", 90.0), ("a2", 92.0)]:
        _session(tmp_path, "%s_r%s_sensors" % (arm_prefix, pinned), ds=ds)
    for arm_prefix, ds in [("b1", 50.0), ("b2", 52.0), ("a1", 55.0), ("a2", 57.0)]:
        _session(tmp_path, "%s_r700_sensors" % arm_prefix, ds=ds)
        _session(tmp_path, "%s_r701_sensors" % arm_prefix, ds=ds)
    pr.report("BEFORE=b1,b2", "AFTER=a1,a2", pattern=str(tmp_path / pr.DEFAULT_GLOB),
              skip_geometry=True, out=sys.stdout)
    out = capsys.readouterr().out
    assert "(report-only)" in out
    # The pinned route's +80 would dominate; the pooled delta must be the other routes' +5.
    assert "mean delta +5.0" in out


# --------------------------------------------------------------------------- commit provenance

def test_an_arm_spanning_two_commits_is_refused(tmp_path, capsys):
    """PROTOCOL rule 2: the driver harness is part of the benchmark, so attempts at different
    commits are attempts at different benchmarks."""
    _session(tmp_path, "b1_r1_sensors", ds=50.0, commit="aaa")
    _session(tmp_path, "b2_r1_sensors", ds=50.0, commit="bbb")
    _session(tmp_path, "a1_r1_sensors", ds=60.0, commit="ccc")
    rc = pr.report("BEFORE=b1,b2", "AFTER=a1", pattern=str(tmp_path / pr.DEFAULT_GLOB),
                   report_only=set(), skip_geometry=True, out=sys.stdout)
    assert rc == 1
    assert "REFUSING TO POOL" in capsys.readouterr().out


# --------------------------------------------------------------------------- paired statistics

def test_paired_delta_arithmetic():
    before = {"a": [50.0, 50.0], "b": [60.0, 60.0], "c": [10.0, 10.0]}
    after = {"a": [60.0, 60.0], "b": [50.0, 50.0], "c": [40.0, 40.0]}
    st = pr.paired_delta(before, after)
    assert st["deltas"] == [10.0, -10.0, 30.0]
    assert st["mean"] == pytest.approx(10.0)
    assert st["improved"] == 2
    # Both arms repeat exactly, so no attempt noise: all the variance is between routes.
    assert st["var_within"] == pytest.approx(0.0)
    assert st["tau2"] == pytest.approx(400.0)


def test_paired_delta_excludes_report_only_routes():
    before = {"a": [50.0, 50.0], "b": [50.0, 50.0], "z": [0.0, 0.0]}
    after = {"a": [60.0, 60.0], "b": [60.0, 60.0], "z": [99.0, 99.0]}
    assert pr.paired_delta(before, after, exclude={"z"})["mean"] == pytest.approx(10.0)
    assert pr.paired_delta(before, after)["mean"] == pytest.approx(39.666, abs=1e-2)


def test_variance_splits_into_attempt_noise_and_route_heterogeneity():
    """The split is the whole point: attempt noise is bought down with more attempts, route
    heterogeneity only with more routes. Here every route has the same true effect, so the
    observed spread of deltas is attempt noise and tau^2 must come out at zero."""
    before = {"a": [40.0, 60.0], "b": [30.0, 70.0], "c": [45.0, 55.0], "d": [20.0, 80.0]}
    after = {r: [v + 10.0 for v in vs] for r, vs in before.items()}
    st = pr.paired_delta(before, after)
    assert st["mean"] == pytest.approx(10.0)
    assert st["tau2"] == pytest.approx(0.0)   # clamped: deltas are identical, nothing left over


def test_projected_se_falls_with_attempts_and_floors_at_route_heterogeneity():
    before = {"a": [40.0, 60.0], "b": [30.0, 70.0], "c": [45.0, 55.0]}
    after = {"a": [55.0, 65.0], "b": [50.0, 60.0], "c": [50.0, 70.0]}
    st = pr.paired_delta(before, after)
    se2, se4, se8 = (pr.projected_se(st, k) for k in (2, 4, 8))
    assert se2 > se4 > se8
    import math
    assert se8 > math.sqrt(st["tau2"] / st["n"]) or st["tau2"] == 0.0
    # More routes at the same attempt count also shrinks it.
    assert pr.projected_se(st, 2, routes=st["n"] * 4) < se2


def test_paired_delta_needs_at_least_two_shared_routes():
    assert pr.paired_delta({"a": [1.0]}, {"a": [2.0]}) is None
    assert pr.paired_delta({"a": [1.0]}, {"b": [2.0]}) is None


# --------------------------------------------------------------------------- geometry

class _FakeLint:
    """Stands in for plan_lint so the flag-reading contract can be tested on its own."""

    def __init__(self, plans):
        self.plans = plans

    def __call__(self, path, aim_gap_m):
        return {"plans": self.plans}


def _plan(**kw):
    base = {"n_wps": 5, "reach_m": 25.0, "aim_gap_m": 5.0, "speed": 3.5,
            "track_rms_m": 0.2, "flags": []}
    base.update(kw)
    return base


def test_violations_are_read_from_the_flags_list_not_as_keys(monkeypatch):
    """plan_lint reports violations in a `flags` LIST. Reading them as boolean keys yields 0% for
    every flag -- which looks exactly like "the guidance change eliminated the defect" rather than
    like a bug, and did, until the run crashed for an unrelated reason."""
    plans = [_plan(flags=["aim_gap"]), _plan(flags=["aim_gap", "straight_ray"]),
             _plan(flags=[]), _plan(flags=[])]
    monkeypatch.setattr(pr, "analyse_session", _FakeLint(plans))
    g = pr.geometry({"r": ["ignored"]})
    assert g["plans"] == 4
    assert g["aim_gap%"] == pytest.approx(50.0)
    assert g["straight_ray%"] == pytest.approx(25.0)
    assert g["reach_short%"] == pytest.approx(0.0)


def test_geometry_medians(monkeypatch):
    plans = [_plan(n_wps=4, reach_m=10.0, aim_gap_m=1.0, track_rms_m=0.1),
             _plan(n_wps=8, reach_m=20.0, aim_gap_m=3.0, track_rms_m=0.9),
             _plan(n_wps=6, reach_m=30.0, aim_gap_m=2.0, track_rms_m=0.3)]
    monkeypatch.setattr(pr, "analyse_session", _FakeLint(plans))
    g = pr.geometry({"r": ["ignored"]})
    assert g["wps"] == 6 and g["reach_m"] == 20.0 and g["bracket_m"] == 2.0
    assert g["track_rms_m"] == pytest.approx(0.3)
    assert g["track_max_m"] == pytest.approx(0.9)


def test_one_unreadable_session_does_not_kill_the_arm(monkeypatch):
    def boom(path, aim_gap_m):
        raise ValueError("no commands.jsonl")
    monkeypatch.setattr(pr, "analyse_session", boom)
    assert pr.geometry({"r": ["a", "b"]}, warn=lambda m: None)["plans"] == 0


def test_the_plan_keys_this_tool_reads_are_the_ones_plan_lint_emits(tmp_path):
    """A contract test against the real linter: the first version of this tool read `n_wp`,
    `reach` and `aim_bracket`, none of which exist, and only surfaced as a crash after the score
    table had already printed."""
    pytest.importorskip("numpy")
    pytest.importorskip("yaml")
    import plan_lint
    import yaml

    d = tmp_path / "session"
    d.mkdir()
    cfg = yaml.safe_load((ROOT / "carla_probe" / "config.yaml").read_text())
    (d / "config.yaml").write_text(yaml.safe_dump(cfg))
    speed, tick_s = 6.0, 0.05
    with open(d / "trajectory.jsonl", "w") as f:
        for step in range(41):
            f.write(json.dumps({"step": step, "t": round(step * tick_s, 2), "frame": step,
                                "x": round(speed * tick_s * step, 3), "y": 0.0, "yaw": 0.0,
                                "speed": speed, "mode": "plan", "pose": "ground_truth",
                                "steer": 0.0, "throttle": 0.5, "brake": 0.0}) + "\n")
    with open(d / "commands.jsonl", "w") as f:
        f.write(json.dumps({"t_wall": 0.0, "step": 0, "cmd": "apply_plan",
                            "waypoints": [[2, 0.1], [4, 0.2], [6, 0.3], [9, 0.5], [13, 0.6],
                                          [18, 0.6], [24, 0.6]],
                            "ticks": 40, "target_speed": speed, "why": "test",
                            "advancing": True, "ok": None}) + "\n")

    plan = plan_lint.analyse_session(d, pr.AIM_GAP_M)["plans"][0]
    for key in ("n_wps", "reach_m", "aim_gap_m", "speed", "track_rms_m", "flags"):
        assert key in plan, key
    assert isinstance(plan["flags"], list)
    g = pr.geometry({"r": [str(d)]})
    assert g["plans"] == 1
