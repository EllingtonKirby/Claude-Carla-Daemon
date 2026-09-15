"""tools/recorder_truth.py: parsing the CARLA recorder dump into ground truth.

This is the only ground truth a sensors-track run has, so a silent misparse here does not fail --
it produces confident, wrong answers downstream. Both cases below were live bugs: yaw was read from
the rotation tuple's second field (the pitch, ~0 on flat ground), which put every bearing 180 deg
out on a road running toward -x, and z was discarded, so an actor parked below the map read as
present and nearby.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path

_SPEC = importlib.util.spec_from_file_location(
    "recorder_truth", Path(__file__).resolve().parents[1] / "tools" / "recorder_truth.py")
recorder_truth = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(recorder_truth)

# Verbatim shape of `show_recorder_file_info(..., show_all=True)`. Centimetres, and rotation is
# printed (roll, pitch, yaw) -- the hero here faces -x, which is what the pitch/yaw mix-up hid.
DUMP = """Frame 1 at 0.05 seconds
 Create 109: vehicle.lincoln.mkz_2020 (1) at (11465.1, 19324.5, 187.3)
  role_name = hero
 Create 110: vehicle.ford.ambulance (1) at (13417.2, 19319.6, -49775.2)
  role_name = scenario
 Positions: 2
  Id: 109 Location: (11465.1, 19324.5, 187.312) Rotation: (0, -1.27475, 179.857)
  Id: 110 Location: (13417.2, 19319.6, -49775.2) Rotation: (0, -1.27475, 179.857)
Frame 2 at 0.10 seconds
 Positions: 2
  Id: 109 Location: (11365.1, 19324.5, 187.312) Rotation: (0, -1.27475, 179.857)
  Id: 110 Location: (13317.2, 19319.6, 180.0) Rotation: (0, -1.27475, 179.857)
"""


def _parsed():
    return recorder_truth.parse_dump(DUMP)


def test_actor_type_and_role_are_captured():
    actors = _parsed()["actors"]
    assert actors[109] == {"type": "vehicle.lincoln.mkz_2020", "role": "hero"}
    assert actors[110]["role"] == "scenario"


def test_positions_are_metres():
    x, y = _parsed()["frames"][1]["actors"][109][:2]
    assert (round(x, 1), round(y, 1)) == (114.7, 193.2)


def test_yaw_is_the_third_rotation_field_not_the_second():
    """(roll, pitch, yaw): reading the second field yields -1.27, and every bearing computed from
    it comes out 180 deg wrong."""
    assert _parsed()["frames"][1]["actors"][109][2] == 179.86


def test_yaw_matches_the_direction_the_actor_travels():
    """The check that catches this class of bug regardless of the recorder's field order: an actor
    moving in a straight line must be facing the way it moves."""
    f = _parsed()["frames"]
    a, b = f[1]["actors"][109], f[2]["actors"][109]
    travel = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    assert abs((travel - a[2] + 180) % 360 - 180) < 1.0


def test_z_is_kept_so_out_of_play_actors_can_be_spotted():
    """CARLA parks a not-yet-active scenario actor hundreds of metres underground while leaving a
    believable x/y, so without z it reads as present and nearby."""
    f = _parsed()["frames"]
    assert f[1]["actors"][110][3] == -497.75   # below the map: not in the scene
    assert f[2]["actors"][110][3] == 1.8       # in play
