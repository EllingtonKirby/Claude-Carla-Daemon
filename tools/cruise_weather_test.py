#!/usr/bin/env python3
"""Does the weather make the plan controller hunt at cruise? Cruise at a target speed on the test pad
in several weathers and report the speed wobble and how often the wheels are spinning.

    python tools/cruise_weather_test.py --port 20080 [--target 6]
"""
import argparse
import copy
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import carla  # noqa: E402
from characterize_controller import Rig, run_up, straight_plan  # noqa: E402

from carla_probe.pid import PIDController  # noqa: E402

WEATHERS = {"ClearNoon": carla.WeatherParameters.ClearNoon, "WetNoon": carla.WeatherParameters.WetNoon,
            "HardRainNoon": carla.WeatherParameters.HardRainNoon, "SoftRainSunset": carla.WeatherParameters.SoftRainSunset}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=20080)
    ap.add_argument("--target", type=float, default=6.0)
    a = ap.parse_args()
    repo = Path(__file__).resolve().parents[1]
    base = yaml.safe_load((repo / "carla_probe" / "config.yaml").read_text())["pid"]
    client = carla.Client("localhost", a.port)
    client.set_timeout(300)
    rig = Rig(client, "Town04")
    try:
        for wname, w in WEATHERS.items():
            rig.world.set_weather(w)
            for cname, over in (("stock", {}), ("boost", {"launch_throttle": 1.0})):
                cfg = copy.deepcopy(base)
                cfg.update(over)
                ctl = PIDController(cfg)
                plan = run_up(rig, ctl, a.target)
                rows = rig.drive(ctl, plan, a.target, 120, 0.5)
                sp = np.array([r[3] for r in rows])
                thr = np.array([r[4] for r in rows])
                jumps = np.abs(np.diff(sp))
                print(f"{wname:15s} {cname:6s} cruise mean {sp.mean():.2f} std {sp.std():.2f}  max 1-tick change {jumps.max():.2f} m/s"
                      f"  throttle mean {thr.mean():.2f}", flush=True)
    finally:
        rig.car.destroy()
        s = rig.world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = False, None
        rig.world.apply_settings(s)


if __name__ == "__main__":
    main()
