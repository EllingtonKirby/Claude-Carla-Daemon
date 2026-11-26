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


# --------------------------------------------------------------------------- aim point / lookahead
def _ctrl_with(**over):
    cfg = dict(CFG["pid"])
    cfg.update(over)
    return PIDController(cfg)


def _straight(spacing, length=30.0):
    return [[i * spacing, 0.0] for i in range(int(length / spacing) + 1)]


@pytest.mark.parametrize("spacing", [1.0, 2.0, 3.0, 5.0, 6.0, 7.5, 10.0, 15.0])
def test_interpolating_modes_ignore_waypoint_spacing(spacing):
    """The whole point of `adaptive`: the lookahead is an arc-length, so how densely the driver
    happened to space its waypoints cannot change where the controller aims."""
    c = _ctrl_with(steer_mode="adaptive")
    _s, _t, _b, meta = c.control(_straight(spacing), 5.0, 5.0, 0.5)
    assert meta["aim_fwd"] == pytest.approx(meta["lookahead"], abs=1e-6)


def test_fixed_mode_lookahead_is_the_spacing_not_aim_dist():
    """`fixed` snaps to a waypoint, so the driver's spacing sets the real lookahead. Pinned because
    the tuned gains assume this behaviour, and so a change to it cannot pass unnoticed."""
    c = _ctrl_with(steer_mode="fixed", aim_dist=7.0)
    ahead = lambda sp: c.control([[(i + 1) * sp, 0.0] for i in range(int(30 / sp))], 5.0, 5.0, 0.5)[3]["aim_fwd"]
    assert ahead(4.0) == pytest.approx(4.0)     # nominal 7 m, actually 4
    assert ahead(15.0) == pytest.approx(15.0)   # nominal 7 m, actually 15


def test_fixed_mode_can_aim_at_the_ego_itself():
    """_compute_control passes the plan from the waypoint nearest the ego, so on reaching one that
    point sits at ~0 m. At spacings where its midpoint is the nearest to aim_dist, `fixed` steers
    on a zero-length vector -- the degenerate case the interpolating modes remove."""
    c = _ctrl_with(steer_mode="fixed", aim_dist=7.0)
    assert c.control(_straight(7.5), 5.0, 5.0, 0.5)[3]["aim_fwd"] == pytest.approx(0.0)
    assert _ctrl_with(steer_mode="adaptive").control(_straight(7.5), 5.0, 5.0, 0.5)[3]["aim_fwd"] > 1.0


def test_adaptive_lookahead_grows_with_speed_and_is_clipped():
    c = _ctrl_with(steer_mode="adaptive")
    slow = c.control(_straight(2.0), 0.0, 5.0, 0.5)[3]["lookahead"]
    fast = c.control(_straight(2.0), 20.0, 5.0, 0.5)[3]["lookahead"]
    assert slow < fast
    assert slow >= CFG["pid"]["lookahead_min"] and fast <= CFG["pid"]["lookahead_max"]


def test_aim_falls_back_to_the_last_waypoint_when_the_plan_is_short():
    c = _ctrl_with(steer_mode="adaptive")
    meta = c.control([[1.0, 0.0], [2.0, 0.0]], 20.0, 5.0, 0.5)[3]
    assert meta["aim_fwd"] == pytest.approx(2.0)


def test_lead_longitudinal_is_memoryless_and_stops_hard():
    c = _ctrl_with(long_mode="lead")
    _s, throttle, brake, _m = c.control(_straight(3.0), 0.0, 6.0, 0.5)
    assert throttle == pytest.approx(1.0) and brake == 0.0   # crisp launch, no integral to build
    assert c.control(_straight(3.0), 5.0, 0.0, 0.5)[2] == 1.0  # ~zero target is a hard stop


def test_brake_zero_steer_drains_the_turn_integral():
    """Braking on a bending plan must not accumulate steer, or the first ticks after a red light
    launch with a pre-loaded wheel."""
    plan = [[3, -1.0], [6, -2.0], [9, -3.5], [12, -5.0]]
    c = _ctrl_with(brake_zero_steer=True)
    for _ in range(10):
        steer, _t, brake, _m = c.control(plan, 5.0, 0.0, 0.5)   # target 0 -> braking
    assert brake == 1.0 and steer == pytest.approx(0.0, abs=1e-9)


# ----------------------------------------------------------------------------- Privileged+ controller knobs
def _cfg(**over):
    import yaml
    from pathlib import Path
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / "carla_probe" / "config.yaml").read_text())["pid"]
    cfg.update(over)
    return cfg


def test_defaults_are_the_stock_controller():
    from carla_probe.pid import PIDController
    c = PIDController(_cfg())
    assert c.gear(6.0, 6.0) is None                       # gear hold off by default
    _, brake, _ = c._longitudinal(6.7, 6.0)               # 12% over: stock brakes hard
    assert brake == 1.0


def test_gear_hold_only_at_cruise_when_enabled():
    from carla_probe.pid import PIDController
    c = PIDController(_cfg(cruise_gear_hold=True))
    assert c.gear(6.0, 2.0) is None                       # launching: automatic gearbox
    assert c.gear(6.0, 5.6) == 2 and c.gear(9.0, 8.5) == 3 and c.gear(4.0, 4.0) is None
    assert c.gear(0.0, 0.0) is None                       # a stop request never holds a gear


def test_graded_braking_coasts_then_brakes_gently_but_stops_hard():
    from carla_probe.pid import PIDController
    c = PIDController(_cfg(cruise_brake_ratio=1.25, cruise_brake=0.4))
    thr, brake, _ = c._longitudinal(6.9, 6.0)             # 15% over: coast
    assert brake == 0.0 and thr == 0.0
    _, brake, _ = c._longitudinal(8.0, 6.0)               # 33% over: gentle brake
    assert brake == 0.4
    _, brake, _ = c._longitudinal(5.0, 0.0)               # asked to stop: full brake
    assert brake == 1.0


def test_a_cruise_target_inside_the_hunting_band_moves_to_its_nearer_edge():
    from carla_probe.pid import PIDController
    c = PIDController(_cfg(avoid_speed_band=[5.0, 6.7]))
    assert c.cruise_target(5.5) == pytest.approx(4.9) and c.cruise_target(6.2) == pytest.approx(6.8)
    assert c.cruise_target(4.0) == 4.0 and c.cruise_target(8.0) == 8.0
    assert PIDController(_cfg()).cruise_target(6.0) == 6.0           # off by default
