import pytest
import yaml
from pathlib import Path

from carla_probe.pid import PIDController

CFG = yaml.safe_load((Path(__file__).resolve().parents[1] / "carla_probe" / "config.yaml").read_text())


def _ctrl():
    return PIDController(CFG["pid"])


def test_straight_plan_accelerates_without_steering():
    steer, throttle, brake, meta = _ctrl().control([[3, 0], [6, 0], [9, 0], [12, 0]], 0.0, None, 0.5)
    assert meta["desired_speed"] == pytest.approx(6.0)
    assert steer == pytest.approx(0.0, abs=1e-9)
    assert throttle > 0.0 and brake == 0.0


@pytest.mark.parametrize("left, sign", [(-4.0, 1.0), (4.0, -1.0)])
def test_steer_sign(left, sign):
    # Plan bending to the ego's right (negative left) must steer right (positive CARLA steer).
    plan = [[3, 0.25 * left], [6, 0.5 * left], [9, 0.75 * left], [12, left]]
    steer, _t, _b, _m = _ctrl().control(plan, 5.0, None, 0.5)
    assert steer * sign > 0.0


def test_target_speed_overrides_spacing_and_brakes_when_fast():
    plan = [[1, 0], [2, 0], [3, 0]]
    _s, throttle, brake, meta = _ctrl().control(plan, 10.0, 3.0, 0.5)
    assert meta["desired_speed"] == pytest.approx(3.0)
    assert brake == 1.0 and throttle == 0.0


def test_zero_target_holds():
    _s, throttle, brake, _m = _ctrl().control([[1, 0], [2, 0]], 0.0, 0.0, 0.5)
    assert brake == 1.0 and throttle == 0.0
