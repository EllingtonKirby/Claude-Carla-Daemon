"""Offline tests for the collision reconstructor. No simulator.

Each of these pins something that fails into a plausible number rather than an error:

* the location in a collision message is the EGO's, so using it as the thing that was hit gives a
  bearing of roughly zero on every event -- "the driver always drives into things head-on";
* the recorder numbers frames from 1, so reading truth without the offset returns poses from a
  different moment that are still on the road and still near the route;
* a CARLA camera yaw is clockwise, so getting the sign wrong swaps left and right and reports the
  driver looking away from exactly the hazards it looked at.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import collisions as co  # noqa: E402

CAMS = [{"id": "CAM_FRONT", "yaw": 0.0, "fov": 100.0},
        {"id": "CAM_FRONT_LEFT", "yaw": -60.0, "fov": 90.0},
        {"id": "CAM_FRONT_RIGHT", "yaw": 60.0, "fov": 90.0},
        {"id": "CAM_BACK", "yaw": 180.0, "fov": 110.0}]

CONFIG_SNIPPET = """\
plan_interval_s: 1.5
cameras:
  - {id: CAM_FRONT,       x: 0.8,  y: 0.0,  z: 1.7, roll: 0.0, pitch: 0.0, yaw: 0.0,   width: 1024, height: 512, fov: 100}
  - {id: CAM_FRONT_LEFT,  x: 0.5,  y: -0.5, z: 1.7, roll: 0.0, pitch: 0.0, yaw: -60.0, width: 800,  height: 450, fov: 90}
  - {id: CAM_FRONT_RIGHT, x: 0.5,  y: 0.5,  z: 1.7, roll: 0.0, pitch: 0.0, yaw: 60.0,  width: 800,  height: 450, fov: 90}
  - {id: CAM_BACK,        x: -1.5, y: 0.0,  z: 1.7, roll: 0.0, pitch: 0.0, yaw: 180.0, width: 800,  height: 450, fov: 110}
steer_mode: adaptive
"""


def _session(tmp_path: Path, name: str = "arm_r1111_sensors", *, events=None, traj=None,
             cmds=None, reads=None, truth=None, config=True, commit="abc1234") -> Path:
    d = tmp_path / "sessions" / name
    (d / "frames").mkdir(parents=True)
    (d / "session.json").write_text(json.dumps({"repo_commit": commit}))
    (d / "score_events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in (events or [])))
    (d / "trajectory.jsonl").write_text("".join(json.dumps(r) + "\n" for r in (traj or [])))
    (d / "decisions.jsonl").write_text("".join(json.dumps(c) + "\n" for c in (cmds or [])))
    if reads is not None:
        (d / "driver_transcript.jsonl").write_text("".join(
            json.dumps({"message": {"content": [
                {"type": "tool_use", "name": "Read", "input": {"file_path": p}}]}}) + "\n"
            for p in reads))
    if truth is not None:
        (d / "recorder_truth.json").write_text(json.dumps(truth))
    if config:
        (d / "config.yaml").write_text(CONFIG_SNIPPET)
    return d


def _straight_track(n=200, v=6.0, dt=0.05, brake_from=None):
    """Ego driving down +x in the CARLA world frame at constant speed, yaw 0."""
    rows = []
    for s in range(n):
        rows.append({"step": s, "t": round(s * dt, 3), "frame": 38 + s,
                     "x": round(v * s * dt, 3), "y": 0.0, "yaw": 0.0, "speed": v,
                     "steer": 0.0, "throttle": 0.6,
                     "brake": 1.0 if brake_from is not None and s >= brake_from else 0.0})
    return rows


def _truth_from(rows, impact_step=100, rel=None, other_speed=0.0, other_yaw=0.0,
                other_id=4917, offset=27, hero_id=3695, dt=0.05):
    """A recorder cache matching the track.

    ``rel`` places the other actor at that WORLD offset from the ego at the impact step, and
    ``other_speed`` slides it along +x so its finite-difference speed is that. Anchoring on the
    impact makes the geometry the test asserts exact rather than approximate.
    """
    frames = {}
    for i, r in enumerate(rows):
        actors = {str(hero_id): [r["x"], r["y"], r["yaw"], 0.0]}
        if rel is not None:
            anchor = rows[impact_step]
            ox = anchor["x"] + rel[0] + other_speed * (i - impact_step) * dt
            oy = anchor["y"] + rel[1]
            actors[str(other_id)] = [ox, oy, other_yaw, 0.0]
        frames[str(r["frame"] - offset)] = {"t": r["t"], "actors": actors}
    return {"dt": dt, "hero_id": str(hero_id), "frame_offset": offset, "frames": frames,
            "actors": {str(hero_id): {"type": "vehicle.x", "role": "hero"},
                       str(other_id): {"type": "vehicle.y", "role": "scenario"}}}


def _event(step=100, frame=None, other_id=4917, ego_xy=(30.0, 0.0),
           kind="collisions_vehicle", other_type="vehicle.dodge.charger_2020"):
    return {"step": step, "t": round(step * 0.05, 2), "frame": frame if frame is not None else 38 + step,
            "type": kind,
            "message": "Agent collided against object with type=%s and id=%d at (x=%.3f, y=%.3f, z=1.0)"
                       % (other_type, other_id, ego_xy[0], ego_xy[1])}


# --------------------------------------------------------------------------- parsing

def test_the_message_location_is_the_ego_not_the_other_actor():
    """Measured over the first corpus: median 0.21 m from the driver's own pose, max 0.60 m. So
    it cannot be used as the struck actor's position, which is what it reads like."""
    rows = _straight_track()
    ego = rows[100]
    ev = co.parse_event(_event(step=100, ego_xy=(ego["x"], ego["y"])))
    assert ev["other_type"] == "vehicle.dodge.charger_2020" and ev["other_id"] == 4917
    assert math.hypot(ev["ego_xy_reported"][0] - ego["x"],
                      ev["ego_xy_reported"][1] - ego["y"]) == 0.0
    # and it is never fed to the geometry: with no recorder cache there is no bearing at all.
    assert co.truth_geometry({"frames": {}, "frame_offset": 0, "hero_id": "1"}, ev, 0.05) == {
        "truth": False}


def test_only_collision_rows_are_taken(tmp_path):
    d = _session(tmp_path, events=[
        {"step": 10, "t": 0.5, "frame": 48, "type": "min_speed_infractions", "message": "slow"},
        _event(step=100),
        {"step": 20, "t": 1.0, "frame": 58, "type": "outside_route_lanes", "message": "lane"},
    ])
    evs = co.collision_events(str(d))
    assert [e["type"] for e in evs] == ["collisions_vehicle"]
    assert evs[0]["other_id"] == 4917


def test_a_frame_read_is_counted_and_a_doc_read_is_not(tmp_path):
    d = _session(tmp_path, reads=[
        "/repo/docs/OPERATOR_GUIDE.md",
        "/repo/sessions/x/frames/frame_00030/CAM_FRONT.jpg",
        "/repo/sessions/x/frames/frame_00030/CAM_BACK.jpg",
        "/repo/sessions/x/frames/frame_00075/CAM_FRONT.jpg",
    ])
    looks = co.looks_by_step(str(d))
    assert looks == {30: {"CAM_FRONT", "CAM_BACK"}, 75: {"CAM_FRONT"}}


def test_the_rig_is_read_from_the_run_s_own_config(tmp_path):
    d = _session(tmp_path)
    cams = co.cameras(str(d))
    assert [c["id"] for c in cams] == [c["id"] for c in CAMS]
    assert cams[1]["yaw"] == -60.0 and cams[1]["fov"] == 90.0


# --------------------------------------------------------------------------- frames and bearings

def test_a_carla_camera_yaw_is_clockwise_so_the_left_camera_sees_the_left():
    assert co.covering_cameras(CAMS, 0.0) == ["CAM_FRONT"]
    assert "CAM_FRONT_LEFT" in co.covering_cameras(CAMS, +60.0)
    assert "CAM_FRONT_RIGHT" not in co.covering_cameras(CAMS, +60.0)
    assert "CAM_FRONT_RIGHT" in co.covering_cameras(CAMS, -60.0)
    assert co.covering_cameras(CAMS, 180.0) == ["CAM_BACK"]


def test_the_rig_has_a_blind_sector_over_each_shoulder():
    """Not a property of this code, a property of the rig: front-quarter cameras stop at 105 deg
    and CAM_BACK starts at 125. Pinned so a rig change that closes or widens it is visible."""
    assert co.covering_cameras(CAMS, 115.0) == []
    assert co.covering_cameras(CAMS, -115.0) == []


def test_bearing_is_in_the_ego_frame_with_left_positive():
    rows = _straight_track()
    # 3 m to the ego's LEFT: the ego frame's y is LEFT, the CARLA world's y is RIGHT.
    tr = _truth_from(rows, rel=(0.0, -3.0))
    g = co.truth_geometry(tr, co.parse_event(_event(step=100)), 0.05)
    assert g["truth"] and abs(g["bearing_deg"] - 90.0) < 1.0 and abs(g["range_m"] - 3.0) < 0.05


def test_a_hit_from_behind_reads_as_behind():
    tr = _truth_from(_straight_track(), rel=(-4.0, 0.0))
    g = co.truth_geometry(tr, co.parse_event(_event(step=100)), 0.05)
    assert abs(abs(g["bearing_deg"]) - 180.0) < 1.0


def test_the_recorder_frame_offset_is_applied_and_a_wrong_one_is_refused():
    rows = _straight_track()
    traj = {r["step"]: r for r in rows}
    good = _truth_from(rows, rel=(10.0, 0.0), offset=27)
    assert co.alignment_error(good, traj) < 0.01
    bad = dict(good, frame_offset=0)          # the recorder numbers its own frames from 1
    assert co.alignment_error(bad, traj) > co.ALIGN_TOLERANCE_M


def test_truth_is_dropped_rather_than_trusted_when_the_mapping_is_off(tmp_path):
    rows = _straight_track()
    bad = dict(_truth_from(rows, rel=(5.0, 0.0), offset=27), frame_offset=0)
    d = _session(tmp_path, events=[_event(step=100)], traj=rows, truth=bad)
    rec = co.reconstruct(str(d), "1111", "ARM")[0]
    assert rec["truth"] is False and "bearing_deg" not in rec
    assert rec["align_err_m"] > co.ALIGN_TOLERANCE_M


def test_an_actor_parked_below_the_map_is_not_a_pose():
    rows = _straight_track(n=5)
    tr = _truth_from(rows, impact_step=0, rel=(10.0, 0.0))
    first = str(rows[0]["frame"] - tr["frame_offset"])
    tr["frames"][first]["actors"]["4917"][3] = -3000.0
    assert co.pose(tr, rows[0]["frame"], "4917") is None
    assert co.pose(tr, rows[1]["frame"], "4917") is not None


# --------------------------------------------------------------------------- reaction

def test_the_brake_run_is_the_unbroken_one_that_reaches_the_impact():
    """Not the first brake tick in the window. The controller brakes for the rest of a burst the
    moment a plan runs out, so 'braked at some point in the last five seconds' is true of almost
    every collision and says nothing about whether the driver was trying to stop."""
    rows = _straight_track(brake_from=80)
    r = co.reaction(rows[:101], 0.05)
    assert r["brake_ticks"] == 21 and abs(r["brake_lead_s"] - 1.0) < 1e-9
    assert abs(r["brake_run_s"] - 1.05) < 1e-9


def test_an_early_brake_that_was_released_does_not_count_as_braking_at_impact():
    rows = _straight_track()
    for r in rows[10:30]:
        r["brake"] = 1.0                       # braked five seconds ago, then let it go
    got = co.reaction(rows[:101], 0.05)
    assert got["brake_ticks"] == 20 and got["brake_lead_s"] == 4.5
    assert got["brake_run_s"] == 0.0


def test_no_brake_at_all_is_distinguished_from_a_late_one():
    never = co.reaction(_straight_track()[:101], 0.05)
    assert never["brake_ticks"] == 0 and never["brake_run_s"] == 0.0
    late = co.reaction(_straight_track(brake_from=95)[:101], 0.05)
    assert late["brake_run_s"] == 0.3


def test_the_window_stops_at_the_impact_and_reaches_back_by_the_window(tmp_path):
    traj = {r["step"]: r for r in _straight_track()}
    rows = co._window(traj, 100, 0.05, 5.0)
    assert rows[0]["step"] == 0 and rows[-1]["step"] == 100 and len(rows) == 101


# --------------------------------------------------------------------------- decision and looks

def test_the_decision_in_flight_is_the_last_one_issued_at_or_before_the_impact(tmp_path):
    cmds = [{"i": 1, "step": 0, "cmd": "apply_plan", "why": "first",
             "args": {"waypoints": [[5, 0], [10, 0]], "target_speed": 8, "ticks": 30}},
            {"i": 2, "step": 60, "cmd": "apply_plan", "why": "second",
             "args": {"waypoints": [[5, 0]], "target_speed": 4, "ticks": 30}},
            {"i": 3, "step": 140, "cmd": "apply_plan", "why": "after the crash", "args": {}}]
    got = co.decision_in_flight(cmds, 100, 0.05)
    assert got["decision_i"] == 2 and got["why"] == "second" and got["prev_why"] == "first"
    assert got["decision_age_s"] == 2.0 and got["target_speed"] == 4 and got["n_wps"] == 1


def test_attention_reports_the_last_look_and_what_was_open_in_the_window():
    looks = {0: {"CAM_FRONT"}, 30: {"CAM_FRONT", "CAM_FRONT_LEFT"}, 90: {"CAM_FRONT"},
             130: {"CAM_BACK"}}
    a = co.attention(looks, 100, 0.05, 5.0)
    assert a["last_look_step"] == 90 and a["last_look_age_s"] == 0.5
    assert a["cams_last_look"] == ["CAM_FRONT"]
    assert a["cams_in_window"] == ["CAM_FRONT", "CAM_FRONT_LEFT"]   # step 130 is after the impact
    assert a["looks_in_window"] == 3


def test_a_route_with_no_looks_at_all_does_not_crash():
    assert co.attention({}, 100, 0.05, 5.0)["last_look_step"] is None


# --------------------------------------------------------------------------- flags

def _rec(tmp_path, **kw):
    rows = kw.pop("traj", _straight_track())
    tr = kw.pop("truth", None)
    d = _session(tmp_path, events=[kw.pop("event", _event(step=100))], traj=rows,
                 cmds=kw.pop("cmds", []), reads=kw.pop("reads", []), truth=tr)
    return co.reconstruct(str(d), "1111", "ARM")[0]


def test_a_stationary_ego_hit_from_behind_is_flagged_as_such(tmp_path):
    rows = _straight_track(v=0.0)
    tr = _truth_from(rows, rel=(-4.0, 0.0), other_speed=5.0)
    rec = _rec(tmp_path, traj=rows, truth=tr, event=_event(step=100, ego_xy=(0.0, 0.0)))
    assert "ego_stopped" in rec["flags"] and "rear_impact" in rec["flags"]
    assert "other_stationary" not in rec["flags"]


def test_a_hazard_in_a_camera_the_driver_never_opened_is_flagged(tmp_path):
    rows = _straight_track()
    tr = _truth_from(rows, rel=(0.0, -3.0))                          # 90 deg to the LEFT
    rec = _rec(tmp_path, traj=rows, truth=tr,
               reads=["/s/frames/frame_00090/CAM_FRONT.jpg"])
    assert "CAM_FRONT_LEFT" in rec["cams_covering"]
    assert "never_looked_that_way" in rec["flags"]


def test_looking_the_right_way_is_not_flagged(tmp_path):
    rows = _straight_track()
    tr = _truth_from(rows, rel=(0.0, -3.0))
    rec = _rec(tmp_path, traj=rows, truth=tr,
               reads=["/s/frames/frame_00090/CAM_FRONT_LEFT.jpg"])
    assert "never_looked_that_way" not in rec["flags"]


def test_a_hit_in_the_blind_sector_is_not_blamed_on_the_driver_s_attention(tmp_path):
    rows = _straight_track()
    # 115 deg to the left: past the front-left camera's edge, short of CAM_BACK's.
    tr = _truth_from(rows, rel=(math.cos(math.radians(115)) * 4.0,
                               -math.sin(math.radians(115)) * 4.0))
    rec = _rec(tmp_path, traj=rows, truth=tr, reads=["/s/frames/frame_00090/CAM_FRONT.jpg"])
    assert rec["cams_covering"] == []
    assert "blind_sector" in rec["flags"] and "never_looked_that_way" not in rec["flags"]


# --------------------------------------------------------------------------- end to end

def test_a_session_without_truth_still_yields_reaction_and_intent(tmp_path):
    rows = _straight_track(brake_from=90)
    d = _session(tmp_path, events=[_event(step=100)], traj=rows,
                 cmds=[{"i": 4, "step": 70, "cmd": "apply_plan", "why": "clear road",
                        "args": {"waypoints": [[5, 0], [10, 0]], "target_speed": 6}}],
                 reads=["/s/frames/frame_00070/CAM_FRONT.jpg"])
    rec = co.reconstruct(str(d), "1111", "ARM")[0]
    assert rec["truth"] is False and rec["align_err_m"] is None
    assert rec["why"] == "clear road" and rec["decision_age_s"] == 1.5
    assert rec["brake_lead_s"] == 0.5 and "braking_started_late" in rec["flags"]
    assert rec["commit"] == "abc1234" and rec["route"] == "1111" and rec["arm"] == "ARM"


def test_the_route_comes_from_the_glob_template_not_a_scan_of_the_path(tmp_path):
    """A parent directory containing '_r<something>_' silently becomes the route otherwise; that
    bug collapsed every attempt into one bogus route in the pooler (docs/JOURNAL.md 2026-09-18)."""
    parent = tmp_path / "test_arm_r9_sensors_scratch"
    rows = _straight_track()
    d = _session(parent, "arm_rFail2Drive_A1_sensors", events=[_event(step=100)], traj=rows)
    recs = co.gather(["ARM=arm"], str(parent / "sessions" / "{prefix}_r*_sensors"))
    assert len(recs) == 1 and recs[0]["route"] == "Fail2Drive_A1"


def test_a_session_with_no_collisions_contributes_nothing(tmp_path):
    d = _session(tmp_path, events=[], traj=_straight_track())
    assert co.reconstruct(str(d), "1111", "ARM") == []


# --------------------------------------------------------------------------- the loss ledger

def _scored(tmp_path: Path, name: str, *, rc=100.0, penalty=1.0, infractions=None,
            records_empty=False) -> Path:
    d = tmp_path / "sessions" / name
    d.mkdir(parents=True, exist_ok=True)
    records = [] if records_empty else [{
        "scores": {"score_composed": rc * penalty, "score_route": rc, "score_penalty": penalty},
        "infractions": infractions or {},
    }]
    (d / "result.json").write_text(json.dumps({"_checkpoint": {"records": records}}))
    return d


def test_a_started_route_is_not_a_scored_one(tmp_path):
    d = _scored(tmp_path, "arm_r1_sensors", records_empty=True)
    assert (d / "result.json").exists() and co._record(str(d)) is None
    assert co.loss_ledger([str(d)]) == {}


def test_one_collision_outranks_a_thousand_percentage_infractions(tmp_path):
    """The reason this tool exists. Ranking infraction classes by how often they fire points at
    exactly the wrong one: the percentage-based classes scale the penalty once however many
    events they log, while each collision multiplies it again."""
    d = _scored(tmp_path, "arm_r1_sensors", rc=100.0, penalty=0.6 * 0.95,
                infractions={"collisions_vehicle": ["hit"],
                             "min_speed_infractions": ["slow"] * 1000})
    led = co.loss_ledger([str(d)])
    loss = led["loss"]
    assert loss["collisions_vehicle"] > 10 * loss["speed and lane (percentage-based)"]
    assert abs(loss["collisions_vehicle"] - (100 * 0.95 - 57.0)) < 0.01


def test_distance_never_driven_is_attributed_to_the_route_not_to_an_infraction(tmp_path):
    d = _scored(tmp_path, "arm_r1_sensors", rc=40.0, penalty=1.0)
    loss = co.loss_ledger([str(d)])["loss"]
    assert abs(loss["route not completed"] - 60.0) < 1e-6
    assert set(loss) == {"route not completed"}


def test_the_ledger_flags_a_penalty_it_cannot_reconstruct(tmp_path, capsys):
    """If the leaderboard's coefficients change, the residual goes above 1 and the split becomes
    fiction. Say so rather than printing it."""
    d = _scored(tmp_path, "arm_r1_sensors", penalty=0.95,
                infractions={"collisions_vehicle": ["hit"]})   # 0.95 > the 0.60 one hit implies
    led = co.loss_ledger([str(d)])
    assert led["max_residual"] > 1.001
    co.print_ledger(led)
    assert "no longer matches the leaderboard" in capsys.readouterr().out


def test_the_ledger_averages_over_sessions_including_clean_ones(tmp_path):
    a = _scored(tmp_path, "arm_r1_sensors", penalty=0.6, infractions={"collisions_vehicle": ["x"]})
    b = _scored(tmp_path, "arm_r2_sensors", penalty=1.0)
    led = co.loss_ledger([str(a), str(b)])
    assert led["sessions"] == 2 and led["completed"] == 2
    assert abs(led["mean_ds"] - 80.0) < 1e-6
    assert abs(led["loss"]["collisions_vehicle"] - 20.0) < 1e-6   # 40 points, over 2 sessions


def test_driving_into_something_under_throttle_is_called_that(tmp_path):
    rows = _straight_track()                    # no brake anywhere, throttle 0.6 throughout
    rec = _rec(tmp_path, traj=rows)
    assert "not_braking_at_impact" in rec["flags"] and "on_the_throttle" in rec["flags"]


def test_reversing_into_something_is_called_that(tmp_path):
    rows = _straight_track(v=-3.0)
    rec = _rec(tmp_path, traj=rows)
    assert "reversing" in rec["flags"]
