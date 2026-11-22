#!/usr/bin/env python3
"""Fix OPERATOR_GUIDE §5.1's camera mount in ccd_priv2 (apply AFTER the ladder_plus batch finishes).
The guide said CAM_FRONT is 0.8 m forward of the bumper; config.yaml mounts it at x = -1.5 m (behind the
origin), so camera-derived distances were ~4.7 m too generous. Found from B0+ debriefs (2091, 26405)."""
import os, subprocess
p = os.path.expanduser("~/workspace/ccd_priv2/docs/OPERATOR_GUIDE.md")
s = open(p).read()
old = """`ego_body`, from `config.yaml`). The front camera is mounted a further 0.8 m forward still, so a
car you judge "3 m away" in `CAM_FRONT` is about **1.4 m from your bumper**."""
new = """`ego_body`, from `config.yaml`). `CAM_FRONT` is **not** at the bumper: it is mounted at x = **-1.5 m**
(1.5 m *behind* the origin, about 3.9 m behind your front bumper) and 2.0 m up (`get_state` ->
`ego_body.cameras` has every mount). A distance you measure from a `CAM_FRONT` image is from the
camera: subtract ~3.9 m to get the gap to your front bumper. Anything about 4 m from the camera is
already touching you."""
assert old in s, "guide text changed; fix by hand"
open(p + ".new", "w").write(s.replace(old, new)); os.replace(p + ".new", p)
subprocess.run(["git", "-C", os.path.dirname(os.path.dirname(p)), "commit", "-qam",
    "OPERATOR_GUIDE 5.1: CAM_FRONT is mounted 1.5 m behind the origin, not 0.8 m ahead of the bumper\n\n"
    "The old text made every camera-derived distance ~4.7 m too generous. Two B0+ drivers found it\n"
    "from get_state (ego_body.cameras) and blamed their collisions on it (2091 DS 4.1, 26405 DS 20).\n"
    "Every camera arm so far (B0-B4, B0+/B2+) ran with the wrong figure.\n\n"
    "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"], check=True)
print("applied and committed")
