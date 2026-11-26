"""Decoding of CARLA's dense GT cameras, and the camera model a driver's script relies on."""
import math

import numpy as np
import pytest

from carla_probe.gt_dense import camera_model, decode_depth, decode_instance

CAMS = [{"id": "CAM_FRONT", "x": -1.5, "y": 0.0, "z": 2.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
         "width": 1024, "height": 512, "fov": 110}]


def test_depth_decoding_is_carla_s_24_bit_formula():
    # 1000 m * (R + 256 G + 65536 B) / (2^24 - 1); BGRA order in the buffer
    target = np.array([5.0, 12.5, 80.0, 999.0])
    code = np.round(target / 1000.0 * (256 ** 3 - 1)).astype(np.int64)
    bgra = np.zeros((1, 4, 4), np.uint8)
    bgra[0, :, 2] = code % 256
    bgra[0, :, 1] = (code // 256) % 256
    bgra[0, :, 0] = code // 65536
    assert decode_depth(bgra)[0] == pytest.approx(target, abs=1e-3)


def test_instance_decoding_puts_the_tag_first():
    bgra = np.zeros((1, 1, 4), np.uint8)
    bgra[0, 0] = [7, 3, 14, 255]          # B=7 G=3 R=14 (car)
    assert decode_instance(bgra)[0, 0].tolist() == [14, 3, 7]


def test_camera_model_intrinsics_scale_with_each_file():
    m = camera_model(CAMS, rgb_scale=0.5, dense_scale=0.5, full_res=True)["cameras"]["CAM_FRONT"]
    full, jpg = m["intrinsics"]["full_jpg"], m["intrinsics"]["jpg"]
    assert full["fx"] == pytest.approx(512 / math.tan(math.radians(55)), abs=1e-2)
    assert jpg["fx"] == pytest.approx(full["fx"] / 2, abs=1e-2)
    assert (jpg["width"], jpg["height"]) == (512, 256)
    assert m["intrinsics"]["depth_npy"] == m["intrinsics"]["inst_png"] == jpg
    assert m["mount"]["z"] == 2.0


def test_sensors_camera_model_lists_no_dense_files():
    m = camera_model(CAMS, rgb_scale=0.5, dense_scale=0.0, full_res=False)["cameras"]["CAM_FRONT"]
    assert set(m["intrinsics"]) == {"jpg"}
