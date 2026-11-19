"""The measured controller model: speed curves, path following, the error band, box distances."""
import math

import numpy as np
import pytest

from carla_probe.motion_model import MotionModel, box_corners, poly_distance

DT = 0.05


def table():
    t = np.arange(120) * DT
    launch = {str(v): list(np.minimum(v, 2.0 * t)) for v in (3.0, 5.0, 9.0)}      # 2 m/s^2 to the target
    brake = {str(v): list(np.maximum(0.0, v - 8.0 * t)) for v in (5.0, 9.0)}       # 8 m/s^2 to a stop
    rows = [{"speed": 3.0, "curvature": 0.0, "p95": 0.1}, {"speed": 9.0, "curvature": 0.0, "p95": 0.3},
            {"speed": 3.0, "curvature": 0.08, "p95": 0.6}, {"speed": 9.0, "curvature": 0.03, "p95": 0.9}]
    return {"configs": {"pid075": {"launch": launch, "brake": brake, "offset": [], "arc": rows}}}


@pytest.fixture
def mm():
    return MotionModel(table(), "pid075", brake_speed=0.4, brake_ratio=1.1)


def test_launch_follows_the_measured_curve_from_the_current_speed(mm):
    v = mm.speed_profile(0.0, 5.0, 60)
    assert v[19] == pytest.approx(2.0, abs=0.11)          # 1 s at 2 m/s^2
    assert v[-1] == pytest.approx(5.0)
    v2 = mm.speed_profile(4.0, 5.0, 5)                     # entered mid-curve, not from standstill
    assert v2[0] == pytest.approx(4.1, abs=0.11)


def test_braking_uses_the_braking_curve(mm):
    v = mm.speed_profile(9.0, 0.0, 30)
    assert v[9] == pytest.approx(9.0 - 8.0 * 0.55, abs=0.5) and v[-1] == 0.0


def test_positions_advance_along_the_plan_and_continue_straight(mm):
    plan = np.array([[5.0, 0.0], [10.0, 0.0]])
    p = mm.predict(plan, speed=5.0, target_speed=5.0, plan_interval_s=0.5, ticks=60)
    assert p["poses"][19]["fwd"] == pytest.approx(5.0, abs=0.01) and p["poses"][19]["left"] == 0.0
    assert p["reaches_plan_end"] and p["poses"][-1]["fwd"] == pytest.approx(15.0, abs=0.01)


def test_the_band_grows_with_speed_and_curvature(mm):
    assert mm.band(3.0, 0.0) < mm.band(9.0, 0.0) < mm.band(9.0, 0.03)


def test_target_from_spacing_when_none_is_given(mm):
    plan = np.array([[2.0, 0.0], [4.0, 0.0], [6.0, 0.0]])  # 2 m per 0.5 s = 4 m/s
    assert mm.predict(plan, 0.0, None, 0.5, 5)["target_speed"] == pytest.approx(4.0)


def test_box_distance():
    a = box_corners(0.0, 0.0, 0.0, 2.0, 1.0)
    assert poly_distance(a, box_corners(6.0, 0.0, 0.0, 2.0, 1.0)) == pytest.approx(2.0)
    assert poly_distance(a, box_corners(0.0, 3.0, 0.0, 2.0, 1.0)) == pytest.approx(1.0)
    assert poly_distance(a, box_corners(1.0, 0.5, 30.0, 2.0, 1.0)) == 0.0          # overlapping
    rot = box_corners(0.0, 0.0, 90.0, 2.0, 1.0)                                     # turned 90 deg: 2 m to each side
    assert poly_distance(rot, box_corners(0.0, 4.0, 90.0, 0.5, 0.5)) == pytest.approx(1.5)
    ego = box_corners(0.0, 0.0, 0.0, 2.44, 0.92, rear=2.45)                         # asymmetric ego body
    assert poly_distance(ego, box_corners(-5.0, 0.0, 0.0, 0.5, 0.5)) == pytest.approx(5.0 - 2.45 - 0.5)


def test_a_car_already_near_its_target_does_not_replay_the_launch_overshoot():
    t = np.arange(120) * DT
    over = np.minimum(2.0 * t, 6.6)                      # a launch curve that overshoots a 6 m/s target...
    over[over >= 6.6] = 6.0
    tab = {"configs": {"x": {"launch": {"6.0": list(over)}, "brake": {"6.0": list(np.maximum(0, 6 - 8 * t))},
                             "offset": [], "arc": []}}}
    v = MotionModel(tab, "x", 0.4, 1.1).speed_profile(5.9, 6.0, 40)
    assert v.max() <= 6.0 + 1e-6 and v[-1] == pytest.approx(6.0, abs=0.01)
