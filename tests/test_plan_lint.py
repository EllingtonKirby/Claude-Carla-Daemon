"""Offline tests for the plan linter. No simulator: a synthetic session is enough, because every
check is arithmetic on what the driver sent and where the ego then went."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import plan_lint  # noqa: E402

TICK_S, SPEED = 0.05, 6.0          # 0.3 m per tick
GOOD = [[2, 0.1], [4, 0.2], [6, 0.3], [9, 0.5], [13, 0.6], [18, 0.6], [24, 0.6]]


def _session(tmp_path: Path, plans, *, cfg_overrides=None, straight=True) -> Path:
    """A session whose ego drives straight along +x at SPEED, with `plans` as its apply_plan log.

    Each plan is ``(step, waypoints, ticks, target_speed)``. The real config.yaml is used, so the
    linter is tested against the tuning that actually ships rather than a stand-in.
    """
    d = tmp_path / "session"
    d.mkdir(exist_ok=True)
    cfg = yaml.safe_load((ROOT / "carla_probe" / "config.yaml").read_text())
    cfg["pid"].update((cfg_overrides or {}).pop("pid", {}))
    cfg.update(cfg_overrides or {})
    (d / "config.yaml").write_text(yaml.safe_dump(cfg))

    last_step = max(s + t for s, _w, t, _ts in plans) + 1
    with open(d / "trajectory.jsonl", "w") as f:
        for step in range(last_step):
            f.write(json.dumps({"step": step, "t": round(step * TICK_S, 2), "frame": step,
                                "x": round(SPEED * TICK_S * step, 3), "y": 0.0 if straight else 0.4,
                                "yaw": 0.0, "speed": SPEED, "mode": "plan", "pose": "ground_truth",
                                "steer": 0.0, "throttle": 0.5, "brake": 0.0}) + "\n")
    with open(d / "commands.jsonl", "w") as f:
        for step, wps, ticks, ts in plans:
            row = {"t_wall": 0.0, "step": step, "cmd": "apply_plan", "waypoints": wps,
                   "ticks": ticks, "why": "test", "advancing": True, "ok": None}
            if ts is not None:
                row["target_speed"] = ts
            f.write(json.dumps(row) + "\n")
    return d


def _one(tmp_path, wps, ticks=40, target_speed=SPEED, **kw):
    res = plan_lint.analyse_session(_session(tmp_path, [(0, wps, ticks, target_speed)], **kw), 4.0)
    assert res["n_plans"] == 1
    return res["plans"][0]


def test_a_well_formed_plan_raises_nothing(tmp_path):
    assert _one(tmp_path, GOOD)["flags"] == []


def test_reach_is_measured_against_the_burst_not_a_fixed_distance(tmp_path):
    # 40 ticks at 6 m/s covers 12 m, so the plan must reach past 12 + 1.5.
    assert _one(tmp_path, GOOD, ticks=40)["margin_m"] == pytest.approx(24.0 - 13.5, abs=0.05)
    assert "reach_short" not in _one(tmp_path, GOOD, ticks=40)["flags"]
    # The same plan over a longer burst outruns itself.
    assert "reach_short" in _one(tmp_path, GOOD, ticks=90)["flags"]


def test_a_short_plan_is_flagged_and_the_stall_confirms_it(tmp_path):
    p = _one(tmp_path, [[2, 0.0], [5, 0.1], [8, 0.2]], ticks=40)
    assert "reach_short" in p["flags"] and "stalled" in p["flags"]
    # plan_done trips when the last waypoint is within 1.5 m: 8 - 1.5 = 6.5 m, at 0.3 m/tick.
    assert p["done_at_tick"] == pytest.approx(6.5 / (SPEED * TICK_S), abs=1)
    assert p["braking_ticks"] == 40 - p["done_at_tick"]


def test_missing_target_speed_reports_what_the_spacing_commands(tmp_path):
    p = _one(tmp_path, [[10, 0.0], [20, 0.0]], target_speed=None)
    assert "no_target_speed" in p["flags"]
    assert p["implied_speed"] == pytest.approx(20.0)   # 10 m of spacing over plan_interval_s


def test_the_aim_gap_is_the_segment_bracketing_the_lookahead(tmp_path):
    # At 6 m/s the aim sits ~7.8 m out. Two points at 10 and 20 m leave it inside the 10 m gap
    # between the EGO and the first waypoint, so the path near the car is a chord.
    p = _one(tmp_path, [[10, 0.0], [20, 0.0]])
    assert p["aim_m"] == pytest.approx(7.77, abs=0.05)
    assert p["aim_gap_m"] == pytest.approx(10.0) and "aim_gap" in p["flags"]
    assert "aim_gap" not in _one(tmp_path, GOOD)["flags"]


def test_the_aim_gap_is_skipped_when_the_aim_snaps_to_a_waypoint(tmp_path):
    p = _one(tmp_path, [[10, 0.0], [20, 0.0]], cfg_overrides={"pid": {"steer_mode": "fixed"}})
    assert p["aim_m"] is None and "aim_gap" not in p["flags"]


def test_a_straight_ray_and_a_two_point_plan_are_both_called_out(tmp_path):
    assert set(_one(tmp_path, [[10, 0.0], [20, 0.0]])["flags"]) >= {"straight_ray", "two_points"}
    assert "straight_ray" not in _one(tmp_path, GOOD)["flags"]


def test_track_rms_measures_the_path_the_ego_was_actually_given(tmp_path):
    # The ego drives straight down y=0; a plan that asks for 0.6 m of left is not followed.
    assert _one(tmp_path, GOOD)["track_rms_m"] > 0.2
    # A straight-ahead plan from the same pose is followed exactly.
    assert _one(tmp_path, [[6, 0.0], [12, 0.0], [24, 0.0]])["track_rms_m"] == pytest.approx(0.0, abs=1e-6)


def test_a_torn_log_and_a_burst_past_the_end_do_not_crash(tmp_path):
    d = _session(tmp_path, [(0, GOOD, 40, SPEED)])
    with open(d / "commands.jsonl", "a") as f:
        f.write('{"step": 99, "cmd": "apply_pl')          # session killed mid-write
    with open(d / "trajectory.jsonl", "a") as f:
        f.write(json.dumps({"step": 999, "cmd": "x"}) + "\n")
    res = plan_lint.analyse_session(d, 4.0)
    assert res["n_plans"] == 1
    assert plan_lint.report(res)[0].startswith(str(d))
