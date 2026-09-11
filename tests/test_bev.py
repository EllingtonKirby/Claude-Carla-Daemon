import cv2
import numpy as np

from carla_probe.bev import box_polygon, lanes_to_polys, render_bev


def test_render_synthetic_scene(tmp_path):
    centers = np.array([[float(x), 0.0] for x in range(-10, 40, 2)])
    lanes = lanes_to_polys(centers, np.zeros(len(centers)), np.full(len(centers), 3.5), 2.0,
                           np.zeros(len(centers), dtype=bool))
    path = render_bev(
        tmp_path / "bev.png",
        forward_m=50.0, back_m=20.0, side_m=35.0, px_per_m=8.0,
        lanes=lanes,
        route=np.array([[0.0, 0.0], [40.0, 0.0]]),
        actors=[("vehicle", box_polygon(15.0, 0.0, 0.0, 2.3, 1.0)), ("walker", box_polygon(10.0, 5.0, 0.0, 0.3, 0.3))],
        traffic_lights=[("Red", 30.0, -4.0)],
        stop_signs=[(25.0, -3.0)],
        route_points=np.array([[5.0, 0.0], [10.0, 0.0]]),
        plan=np.array([[3.0, 0.0], [6.0, 0.5]]),
        ego_poly=box_polygon(0.0, 0.0, 0.0, 2.4, 1.0),
        header="test",
    )
    img = cv2.imread(path)
    assert img.shape == (int(70 * 8), int(70 * 8), 3)
    # Ego is drawn red at (row = forward_m*ppm, col = side_m*ppm).
    b, g, r = img[50 * 8, 35 * 8]
    assert r > 200 and g < 50 and b < 50


def test_box_polygon_heading():
    poly = box_polygon(0.0, 0.0, np.pi / 2, 2.0, 1.0)  # rotated to point along +left
    assert np.isclose(poly[:, 1].max(), 2.0) and np.isclose(poly[:, 0].max(), 1.0)
