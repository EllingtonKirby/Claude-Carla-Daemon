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


# --------------------------------------------------------------------------- the ego's own body
#
# These pin the arithmetic the driver was getting wrong: it read a distance off a camera image and
# spent all of it, because nothing told it how much of that distance its own bodywork occupies.

# The real Bench2Drive hero, and what config.yaml `ego:` says: a Lincoln MKZ 2020.
FRONT, REAR, HALF_W = 2.44, 2.45, 0.92


def test_a_distance_to_the_origin_is_not_a_clearance():
    """The whole point. A car 3 m from the ego origin is 0.55 m from the bumper, and contact
    registers at 0 -- so "3 m of room ahead" is most of a car's length short of the truth."""
    assert g.point_clearance(FRONT, REAR, HALF_W, (3.0, 0.0)) == pytest.approx(0.56)
    assert g.point_clearance(FRONT, REAR, HALF_W, (0.0, 1.5)) == pytest.approx(0.58)
    assert g.point_clearance(FRONT, REAR, HALF_W, (-3.0, 0.0)) == pytest.approx(0.55)


def test_touching_is_zero_and_inside_is_negative():
    assert g.point_clearance(FRONT, REAR, HALF_W, (FRONT, 0.0)) == pytest.approx(0.0)
    assert g.point_clearance(FRONT, REAR, HALF_W, (0.0, HALF_W)) == pytest.approx(0.0)
    assert g.point_clearance(FRONT, REAR, HALF_W, (0.0, 0.0)) == pytest.approx(-HALF_W)
    assert g.point_clearance(FRONT, REAR, HALF_W, (2.0, 0.0)) == pytest.approx(-0.44)


def test_a_corner_is_measured_diagonally_not_per_axis():
    """Off the corner both axes are outside, so the clearance is the hypotenuse. Taking the larger
    of the two would over-report it by up to 41%."""
    got = g.point_clearance(FRONT, REAR, HALF_W, (FRONT + 3.0, HALF_W + 4.0))
    assert got == pytest.approx(5.0)


def test_front_and_rear_are_separate_because_the_origin_is_not_the_middle():
    """A long-nosed body: 3 m of car ahead of the origin, 1 m behind. A point and its mirror image
    are not the same problem, which is why config splits front_m from rear_m."""
    assert g.point_clearance(3.0, 1.0, HALF_W, (2.5, 0.0)) == pytest.approx(-0.5)   # inside the nose
    assert g.point_clearance(3.0, 1.0, HALF_W, (-2.5, 0.0)) == pytest.approx(1.5)   # well behind


def test_the_body_frame_is_right_handed_unlike_the_carla_world():
    """`world_to_ego` must not be reused here: its left-handed convention would mirror the answer,
    and a mirrored clearance looks entirely plausible."""
    pose = (0.0, 0.0, 90.0)                                  # nose swung to the ego's LEFT
    fwd, left = g._into_body(pose, (0.0, 10.0))              # a point 10 m to the ego's left
    assert (fwd, left) == pytest.approx((10.0, 0.0), abs=1e-9)   # ...is now dead ahead of the body
    fwd, left = g._into_body(pose, (10.0, 0.0))
    assert (fwd, left) == pytest.approx((0.0, -10.0), abs=1e-9)  # ...and ahead is now to its right


def test_the_sweep_starts_at_the_ego_not_at_the_first_waypoint():
    """A plan's first point is metres ahead; the body covers that stretch too, and an obstacle
    inside it would be missed by testing the waypoints alone."""
    poses = g.path_poses([[5.0, 0.0], [10.0, 0.0]], step_m=0.5)
    assert poses[0] == pytest.approx([0.0, 0.0, 0.0])
    assert poses[-1][0] == pytest.approx(10.0)
    assert len(poses) == 21


def test_a_straight_sweep_finds_the_closest_approach_not_the_endpoint():
    got = g.sweep_clearance(FRONT, REAR, HALF_W, [[10.0, 0.0]], (5.0, 2.0), step_m=0.1)
    assert got["min_m"] == pytest.approx(2.0 - HALF_W, abs=0.02)
    # A flank holds the same clearance for the whole length of the car, so the minimum is a tie
    # across an interval; the station reported is where it is first reached, which is the nose
    # drawing level rather than the middle of the pass.
    assert got["at_station_m"] == pytest.approx(5.0 - FRONT, abs=0.2)


def test_the_rear_corner_swinging_out_is_caught():
    """The shape of several real contacts: the driver judges it is 'past' an object, turns, and
    hits it with the back of the car. Testing only the origin's path says it is clear."""
    path = [[4.0, 0.0], [8.0, 4.0]]                     # drive on, then bear left
    obstacle = (3.5, -1.4)                              # off the right flank, behind the nose
    origin_only = min(math.hypot(obstacle[0] - p[0], obstacle[1] - p[1])
                      for p in g.path_poses(path, step_m=0.1)[:, :2])
    body = g.sweep_clearance(FRONT, REAR, HALF_W, path, obstacle, step_m=0.1)
    assert origin_only == pytest.approx(1.40, abs=0.05)  # the origin never gets near it
    assert g.point_clearance(FRONT, REAR, HALF_W, obstacle) == pytest.approx(1.16, abs=0.02)
    assert body["min_m"] == pytest.approx(-0.28, abs=0.05)   # ...the bodywork goes 0.3 m into it


def test_a_bending_plan_needs_a_wider_corridor_than_the_car():
    straight = g.swept_bounds(FRONT, REAR, HALF_W, [[20.0, 0.0]])
    assert straight["corridor_width_m"] == pytest.approx(2 * HALF_W)
    bend = g.swept_bounds(FRONT, REAR, HALF_W, [[10.0, 0.0], [20.0, 6.0]])
    assert bend["corridor_width_m"] > 2 * HALF_W
    assert bend["max_left_m"] > 6.0                     # the outer corner overshoots the path
    assert bend["reach_m"] > 20.0                       # and the nose reaches past the last point


def test_a_degenerate_plan_does_not_divide_by_zero():
    poses = g.path_poses([[0.0, 0.0], [0.0, 0.0]])
    assert poses.shape == (1, 3)
    got = g.sweep_clearance(FRONT, REAR, HALF_W, [[0.0, 0.0]], (3.0, 0.0))
    assert got["min_m"] == pytest.approx(0.56)
