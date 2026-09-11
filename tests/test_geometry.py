import math

import numpy as np
import pytest

from carla_probe import geometry as g


def test_yaw0_left_handed_axes():
    # CARLA world: +y is to the RIGHT of a car heading +x, so it must come out as NEGATIVE left.
    out = g.world_to_ego((0.0, 0.0), 0.0, np.array([[10.0, 5.0]]))
    assert out[0] == pytest.approx([10.0, -5.0])


def test_yaw90_heading_plus_y():
    # Heading +y: forward is +y, the right vector is (-1, 0), so world +x is on the LEFT.
    out = g.world_to_ego((0.0, 0.0), 90.0, np.array([[0.0, 10.0], [10.0, 0.0]]))
    assert out[0] == pytest.approx([10.0, 0.0], abs=1e-9)
    assert out[1] == pytest.approx([0.0, 10.0], abs=1e-9)


@pytest.mark.parametrize("yaw", [-170.0, -45.0, 0.0, 33.0, 90.0, 179.0])
def test_roundtrip(yaw):
    rng = np.random.default_rng(0)
    pts = rng.normal(scale=50.0, size=(20, 2))
    ego = (123.4, -56.7)
    back = g.ego_to_world(ego, yaw, g.world_to_ego(ego, yaw, pts))
    assert np.allclose(back, pts)


def test_vec_matches_points():
    yaw = 37.0
    v = (3.0, -4.0)
    assert g.vec_world_to_ego(yaw, v) == pytest.approx(tuple(g.world_to_ego((0, 0), yaw, np.array([v]))[0]))


def test_wrap_deg():
    assert g.wrap_deg(190.0) == pytest.approx(-170.0)
    assert g.wrap_deg(-180.0) == pytest.approx(180.0)


def test_project_on_route_straight():
    route = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
    seg, s, lat = g.project_on_route(route, (15.0, 2.0))
    assert seg == 1
    assert s == pytest.approx(15.0)
    # +y is right of a +x route in CARLA, so the lateral "left" offset is negative, consistent
    # with world_to_ego at yaw 0.
    assert lat == pytest.approx(-2.0)


def test_project_hint_window_avoids_loop_snap():
    # Route goes out along +x and comes back just 1 m away; the hint keeps us on the first pass.
    route = np.array([[0.0, 0.0], [50.0, 0.0], [50.0, 1.0], [0.0, 1.0]])
    seg, _s, _lat = g.project_on_route(route, (10.0, 0.6), hint_idx=0, window=1)
    assert seg == 0


def test_points_ahead_and_extrapolation():
    route = np.array([[0.0, 0.0], [10.0, 0.0]])
    pts = g.route_points_ahead(route, 2.0, [3.0, 20.0])
    assert pts[0] == pytest.approx([5.0, 0.0])
    assert pts[1] == pytest.approx([22.0, 0.0])


def test_route_heading():
    route = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    assert g.route_heading_deg(route, 0) == pytest.approx(0.0)
    assert g.route_heading_deg(route, 1) == pytest.approx(90.0)  # +y: CARLA yaw is clockwise


def test_route_box_entry():
    route = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 100.0]])  # sparse: boxes lie mid-segment
    assert g.route_box_entry_s(route, (50.0, 0.5), (2.0, 1.0), 0.0) == pytest.approx(48.0)
    assert g.route_box_entry_s(route, (100.5, 30.0), (1.0, 3.0), 0.0) == pytest.approx(127.0)
    assert g.route_box_entry_s(route, (50.0, 0.0), (2.0, 1.0), 49.0) == pytest.approx(49.0)  # inside now
    assert g.route_box_entry_s(route, (50.0, 0.0), (2.0, 1.0), 60.0) is None                 # passed it
    assert g.route_box_entry_s(route, (50.0, 3.5), (2.0, 1.5), 0.0) is None                  # next lane
    assert g.route_box_entry_s(route, (-68.0, 2.7), (2.0, 3.0), 0.0) is None                 # behind start


@pytest.mark.parametrize("ref", [(0.0, 0.0), (49.0, 8.4), (-33.9, 151.2)])
@pytest.mark.parametrize("xy", [(0.0, 0.0), (1234.5, -987.6), (-4000.0, 5000.0)])
def test_gnss_roundtrip_and_reference_solve(ref, xy):
    lat, lon = g.world_to_gps(xy[0], xy[1], *ref)
    back = g.gps_to_world(lat, lon, *ref)
    assert back == pytest.approx(xy, abs=1e-6)
    lat_ref, lon_ref = g.solve_geo_reference(xy[0], xy[1], lat, lon)
    assert (lat_ref, lon_ref) == (pytest.approx(ref[0], abs=1e-7), pytest.approx(ref[1], abs=1e-7))
    # A second point must then convert consistently through the solved reference.
    lat2, lon2 = g.world_to_gps(xy[0] + 30.0, xy[1] - 40.0, *ref)
    assert g.gps_to_world(lat2, lon2, lat_ref, lon_ref) == pytest.approx((xy[0] + 30.0, xy[1] - 40.0), abs=1e-3)


def test_gnss_axes_north_is_minus_y():
    lat0, _ = g.world_to_gps(0.0, 0.0, 0.0, 0.0)
    lat_south, _ = g.world_to_gps(0.0, 100.0, 0.0, 0.0)
    assert lat_south < lat0


@pytest.mark.parametrize("compass, yaw", [(0.0, -90.0), (math.pi / 2, 0.0), (math.pi, 90.0), (1.5 * math.pi, 180.0)])
def test_compass_to_yaw(compass, yaw):
    assert abs(g.wrap_deg(g.compass_to_yaw_deg(compass) - yaw)) < 1e-9


def test_predict_blend():
    assert g.predict_blend(None, 5.0, 0.0, 0.05, (3.0, 4.0), 0.2) == (3.0, 4.0)
    assert g.predict_blend((0.0, 0.0), 10.0, 0.0, 0.1, (5.0, 0.0), 1.0) == (5.0, 0.0)
    # Dead-reckoned to (1, 0), then pulled 20% toward (2, 0).
    assert g.predict_blend((0.0, 0.0), 10.0, 0.0, 0.1, (2.0, 0.0), 0.2) == pytest.approx((1.2, 0.0))


def test_order_footprint_rectangle():
    corners = np.array([[1, 1], [-1, -1], [1, -1], [-1, 1]] * 2, dtype=float)
    fp = g.order_footprint(corners)
    assert fp.shape == (4, 2)
    angles = np.arctan2(fp[:, 1], fp[:, 0])
    assert np.all(np.diff(angles) > 0)
    e1, e2 = fp[1] - fp[0], fp[2] - fp[1]
    assert math.isclose(abs(e1[0] * e2[1] - e1[1] * e2[0]), 4.0)
