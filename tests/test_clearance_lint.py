"""Offline tests for the near-miss counter. No simulator.

The counter exists because collisions are too rare to measure a change with: 49 in a 60-session
corpus against several hundred plans. Its job is to say how much room the driver actually left,
per plan, which means the arithmetic has to be body-to-body -- the failure being measured is
exactly the habit of treating a centre-to-centre distance as clearance.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import actor_extents as ax  # noqa: E402
import clearance_lint as cl  # noqa: E402

CAR = {"front_m": 2.44, "rear_m": 2.45, "half_width_m": 0.92}
TABLE = {"vehicle.hero": CAR, "vehicle.other": CAR,
         "walker.pedestrian.0001": {"front_m": 0.2, "rear_m": 0.2, "half_width_m": 0.25}}


# --------------------------------------------------------------------------- polygon geometry

def test_two_cars_side_by_side_are_measured_flank_to_flank():
    a = cl.corners(0.0, 0.0, 0.0, CAR)
    b = cl.corners(0.0, 3.0, 0.0, CAR)
    assert cl.polygon_distance(a, b) == pytest.approx(3.0 - 2 * 0.92)


def test_two_cars_nose_to_nose_are_measured_bumper_to_bumper():
    a = cl.corners(0.0, 0.0, 0.0, CAR)
    b = cl.corners(6.0, 0.0, 180.0, CAR)
    assert cl.polygon_distance(a, b) == pytest.approx(6.0 - 2.44 - 2.44, abs=1e-6)


def test_overlapping_bodies_are_zero_not_negative():
    a = cl.corners(0.0, 0.0, 0.0, CAR)
    assert cl.polygon_distance(a, cl.corners(1.0, 0.0, 0.0, CAR)) == 0.0
    assert cl.polygon_distance(a, cl.corners(0.0, 0.0, 90.0, CAR)) == 0.0


def test_a_diagonal_gap_is_the_hypotenuse_not_either_axis():
    # b sits ahead and to the left, its REAR bumper facing a's nose: 3 m of x gap, 4 m of y gap.
    a = cl.corners(0.0, 0.0, 0.0, CAR)
    b = cl.corners(CAR["front_m"] + 3.0 + CAR["rear_m"], 2 * CAR["half_width_m"] + 4.0, 0.0, CAR)
    assert cl.polygon_distance(a, b) == pytest.approx(5.0)


def test_the_body_is_built_on_the_carla_basis_so_a_left_handed_yaw_survives():
    """The world frame is left-handed: at yaw 90 the nose points to +y and the body's RIGHT side
    is toward -x. Assuming a right-handed rotation here mirrors every body in the corpus."""
    nose_north = cl.corners(0.0, 0.0, 90.0, CAR)
    assert max(y for _x, y in nose_north) == pytest.approx(2.44)
    assert min(x for x, _y in nose_north) == pytest.approx(-0.92)


# --------------------------------------------------------------------------- what counts

def test_only_road_users_and_scenario_props_count():
    assert cl._is_obstacle("vehicle.audi.tt")
    assert cl._is_obstacle("walker.pedestrian.0048")
    assert cl._is_obstacle("static.prop.constructioncone")
    # Map furniture: thousands per route, and the recorder gives no extent for any of them.
    assert not cl._is_obstacle("static.prop.mesh")
    # Logical trigger volumes, not bodies.
    assert not cl._is_obstacle("traffic.traffic_light")
    assert not cl._is_obstacle("traffic.stop")
    assert not cl._is_obstacle("sensor.camera.rgb")
    assert not cl._is_obstacle("spectator")


def test_an_unmeasured_type_falls_back_loudly():
    seen = set()
    got = ax.extent_of(TABLE, "vehicle.unheard.of", seen)
    assert got == ax.FALLBACK and seen == {"vehicle.unheard.of"}
    assert ax.extent_of(TABLE, "vehicle.other", seen) == CAR and seen == {"vehicle.unheard.of"}


# --------------------------------------------------------------------------- burst boundaries

def test_a_plan_cut_short_is_only_charged_the_stretch_it_drove():
    """The driver can supersede a plan before its ticks run out. Charging it the requested ticks
    would blame it for a stretch a later plan was driving."""
    cmds = [{"i": 1, "step": 0, "cmd": "apply_plan", "args": {"ticks": 40, "waypoints": [[5, 0]]}},
            {"i": 2, "step": 10, "cmd": "apply_plan", "args": {"ticks": 40, "waypoints": [[5, 0]]}}]
    got = cl.bursts(cmds, last_step=200)
    assert [(b["step0"], b["step1"]) for b in got] == [(0, 10), (10, 50)]


def test_a_burst_never_runs_past_the_end_of_the_recording():
    cmds = [{"i": 1, "step": 0, "cmd": "apply_plan", "args": {"ticks": 400, "waypoints": [[5, 0]]}}]
    assert cl.bursts(cmds, last_step=37)[0]["step1"] == 37


def test_non_plan_commands_are_not_plans_but_do_end_one():
    cmds = [{"i": 1, "step": 0, "cmd": "apply_plan", "args": {"ticks": 40, "waypoints": [[5, 0]]}},
            {"i": 2, "step": 5, "cmd": "hold", "args": {}}]
    got = cl.bursts(cmds, last_step=200)
    assert len(got) == 1 and got[0]["step1"] == 5


# --------------------------------------------------------------------------- end to end

def _session(tmp_path: Path, name="arm_r1111_sensors", *, other_xy=(20.0, 3.0), n=120,
             offset=27, hero_id=10, other_id=11, other_type="vehicle.other", plan_ticks=60,
             misalign=False):
    """An ego driving down +x past one parked car, with a matching recording."""
    d = tmp_path / "sessions" / name
    d.mkdir(parents=True)
    (d / "session.json").write_text(json.dumps({"repo_commit": "abc1234"}))
    rows, frames = [], {}
    for s in range(n):
        x = 0.4 * s
        rows.append({"step": s, "t": round(s * 0.05, 3), "frame": 38 + s,
                     "x": x, "y": 0.0, "yaw": 0.0, "speed": 8.0})
        frames[str(38 + s - offset)] = {"t": rows[-1]["t"], "actors": {
            str(hero_id): [x, 0.0, 0.0, 0.0],
            str(other_id): [other_xy[0], other_xy[1], 0.0, 0.0]}}
    (d / "trajectory.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (d / "decisions.jsonl").write_text(json.dumps(
        {"i": 1, "step": 0, "cmd": "apply_plan", "why": "go",
         "args": {"waypoints": [[10, 0], [20, 0]], "ticks": plan_ticks, "target_speed": 8}}) + "\n")
    (d / "recorder_truth.json").write_text(json.dumps({
        "dt": 0.05, "hero_id": str(hero_id), "frame_offset": 0 if misalign else offset,
        "frames": frames,
        "actors": {str(hero_id): {"type": "vehicle.hero", "role": "hero"},
                   str(other_id): {"type": other_type, "role": "scenario"}}}))
    return d


def test_the_clearance_is_body_to_body_not_centre_to_centre(tmp_path):
    d = _session(tmp_path, other_xy=(20.0, 3.0))
    got = cl.session_clearances(str(d), TABLE, stride=1)
    assert len(got) == 1
    # centres pass 3.0 m apart; the bodies pass 3.0 - 0.92 - 0.92 = 1.16 m apart.
    assert got[0]["min_clearance_m"] == pytest.approx(1.16, abs=0.01)
    assert got[0]["nearest_type"] == "vehicle.other"
    # Two parallel flanks hold the same gap for as long as they overlap, so the minimum is a tie
    # over a range of ticks and `at_step` is where it is first reached -- the noses drawing level,
    # not the midpoint of the pass. 4.89 m of overlap at 0.4 m a tick puts that at step 38.
    assert got[0]["at_step"] == 38


def test_a_body_that_touches_reports_zero(tmp_path):
    d = _session(tmp_path, other_xy=(20.0, 1.5))
    got = cl.session_clearances(str(d), TABLE, stride=1)
    assert got[0]["min_clearance_m"] == 0.0


def test_a_narrower_body_leaves_more_room_at_the_same_separation(tmp_path):
    d = _session(tmp_path, other_xy=(20.0, 3.0), other_type="walker.pedestrian.0001")
    got = cl.session_clearances(str(d), TABLE, stride=1)
    assert got[0]["min_clearance_m"] == pytest.approx(3.0 - 0.92 - 0.25, abs=0.01)


def test_a_session_whose_recording_does_not_line_up_is_skipped_not_guessed(tmp_path):
    """A wrong frame offset returns poses from another moment that are still on the road, so the
    clearances would be plausible and wrong (docs/JOURNAL.md, tools/collisions.py)."""
    d = _session(tmp_path, misalign=True)
    assert cl.session_clearances(str(d), TABLE, stride=1) is None


def test_a_session_with_no_recording_is_skipped(tmp_path):
    d = _session(tmp_path)
    (d / "recorder_truth.json").unlink()
    assert cl.session_clearances(str(d), TABLE, stride=1) is None


def test_the_stride_does_not_move_the_answer_much(tmp_path):
    d = _session(tmp_path, other_xy=(20.0, 3.0))
    fine = cl.session_clearances(str(d), TABLE, stride=1)[0]["min_clearance_m"]
    coarse = cl.session_clearances(str(d), TABLE, stride=2)[0]["min_clearance_m"]
    assert coarse == pytest.approx(fine, abs=0.05)


def test_gather_tags_each_plan_with_its_arm_and_route(tmp_path):
    _session(tmp_path, "arm_r1111_sensors")
    _session(tmp_path, "arm_r2222_sensors")
    recs, skipped, unknown = cl.gather(["ARM=arm"], str(tmp_path / "sessions" / "{prefix}_r*_sensors"),
                                       table=TABLE)
    assert sorted(r["route"] for r in recs) == ["1111", "2222"]
    assert all(r["arm"] == "ARM" for r in recs) and not skipped["no_truth"] and not unknown


def test_an_unmeasured_actor_is_reported_so_the_number_is_not_trusted_silently(tmp_path):
    d = _session(tmp_path, other_type="vehicle.brand.new")
    seen = set()
    cl.session_clearances(str(d), TABLE, stride=1, unknown=seen)
    assert seen == {"vehicle.brand.new"}


def test_the_arms_print_in_the_order_they_were_named_not_alphabetically():
    """Sorting them printed AFTER before BEFORE, so the delta read as though the change had been
    undone. Direction is the one thing a reader takes straight off the output."""
    recs = [{"arm": "BEFORE", "min_clearance_m": 1.0}, {"arm": "AFTER", "min_clearance_m": 2.0}]
    assert cl.arm_order(recs, ["BEFORE=a,b", "AFTER=c,d"]) == ["BEFORE", "AFTER"]
    assert cl.arm_order(recs, None) == ["BEFORE", "AFTER"]          # else first-seen order
    assert cl.arm_order(recs, ["BEFORE=a", "AFTER=c", "THIRD=e"]) == ["BEFORE", "AFTER"]


def test_the_near_miss_rate_is_the_fraction_at_or_below_the_threshold():
    recs = [{"min_clearance_m": v} for v in (0.0, 0.4, 0.5, 0.51, 3.0)]
    assert cl.near_rate(recs, 0.5) == pytest.approx(3 / 5)
    assert cl.near_rate(recs, 0.0) == pytest.approx(1 / 5)


def test_the_paired_table_excludes_the_report_only_route(tmp_path, capsys, monkeypatch):
    """The pinned standard route is also a dev-set member; letting it steer a decision would be
    tuning on the headline route. Same rule as tools/pool_report.py."""
    monkeypatch.setattr(cl, "read_report_only", lambda: {"9999"})
    recs = []
    for route, before, after in (("1111", 4, 1), ("2222", 4, 2), ("9999", 0, 10)):
        for arm, n_near in (("BEFORE", before), ("AFTER", after)):
            for i in range(10):
                recs.append({"route": route, "arm": arm,
                             "min_clearance_m": 0.1 if i < n_near else 5.0})
    cl._paired(recs, ["BEFORE", "AFTER"], 0.5)
    out = capsys.readouterr().out
    assert "(report-only)" in out
    # 1111 -30 points, 2222 -20: mean -25 over the two steering routes, the report-only +100 left out
    assert "paired over 2 routes: mean -25.0" in out
    assert "2 of 2 improved" in out


def test_each_plan_records_how_far_the_ego_actually_went(tmp_path):
    """Without it a per-plan rate cannot be told apart from a change in plan length: shorter
    plans meet fewer hazards each, so the rate falls while the driving is no safer."""
    d = _session(tmp_path, plan_ticks=50)
    got = cl.session_clearances(str(d), TABLE, stride=1)[0]
    assert got["ticks"] == 50
    assert got["dist_m"] == pytest.approx(50 * 0.4, abs=0.01)     # 0.4 m a tick in the fixture
