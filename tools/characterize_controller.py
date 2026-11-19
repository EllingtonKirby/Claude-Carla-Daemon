#!/usr/bin/env python3
"""Measure how the daemon's plan controller actually moves the Bench2Drive car.

    python tools/characterize_controller.py --port 20090 [--out tools/controller_model.json]
                                            [--configs pid075,pid100,lead]

The driver hands the daemon a path (`apply_plan`) and the controller (carla_probe/pid.py) turns it
into pedals and steering. On the privileged track the driver then knows where everything else is,
but not where ITS OWN car will be: how fast it launches, how fast it can brake, how far it strays
off the planned line at a given speed and curvature. The 2026-11-18 corpus lost points to exactly
that -- a 0.5 m margin against a ~0.4 m tracking error, a left turn committed on a launch slower
than the driver assumed.

This drives the hero blueprint (vehicle.lincoln.mkz_2020) through the SAME controller class and
config the daemon uses, at the leaderboard's 20 Hz synchronous step, on a long straight of the
Town04 highway, and records:

  launch    standstill -> target speed (3, 5, 7, 9, 12 m/s): speed every tick for 6 s
  brake     plan target 0 from 5, 9, 12 m/s: speed every tick until stopped
  offset    a cosine lane-change of 1.5 / 3.5 m over 15 / 30 m at 3, 6, 9 m/s: cross-track error
  arc       constant radius 12 / 20 / 35 m at 3, 6, 9 m/s for 3 s: cross-track error

for each longitudinal configuration named in --configs:
  pid075  the stock PID, max_throttle 0.75 (what every run so far used)
  pid100  the same PID with max_throttle 1.0
  lead    long_mode: lead (a memoryless expert regression that goes to full throttle at once)

The output is the table carla_probe/motion_model.py predicts from. It describes the car and the
controller, not any route, so it is safe to commit. Needs a CARLA 0.9.15 server on --port.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import carla  # noqa: E402

from carla_probe import geometry as geo  # noqa: E402
from carla_probe.pid import PIDController  # noqa: E402

DT = 0.05
REPO = Path(__file__).resolve().parents[1]


def find_straight(world, need_m=220.0):
    """A driving waypoint with `need_m` of almost-straight lane ahead and a lane on each side."""
    m = world.get_map()
    for wp in m.generate_waypoints(8.0):
        if wp.is_junction or wp.lane_type != carla.LaneType.Driving:
            continue
        if not (wp.get_left_lane() and wp.get_right_lane()):
            continue
        yaw0, cur, ok = wp.transform.rotation.yaw, wp, True
        for _ in range(int(need_m / 10)):
            nxt = cur.next(10.0)
            if not nxt:
                ok = False
                break
            cur = nxt[0]
            if cur.is_junction or abs(((cur.transform.rotation.yaw - yaw0 + 180) % 360) - 180) > 3.0:
                ok = False
                break
        if ok:
            return wp
    raise RuntimeError("no straight stretch found")


class Rig:
    def __init__(self, client, town: str):
        self.world = client.load_world(town)
        s = self.world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, DT
        self.world.apply_settings(s)
        self.world.tick()
        self.start_wp = find_straight(self.world)
        bp = self.world.get_blueprint_library().find("vehicle.lincoln.mkz_2020")
        bp.set_attribute("role_name", "hero")
        tf = self.start_wp.transform
        tf.location.z += 0.5
        self.car = self.world.spawn_actor(bp, tf)
        for _ in range(40):
            self.world.tick()

    def reset(self):
        tf = self.start_wp.transform
        tf.location.z += 0.3
        self.car.set_target_velocity(carla.Vector3D(0, 0, 0))
        self.car.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self.car.apply_control(carla.VehicleControl(brake=1.0))
        self.car.set_transform(tf)
        for _ in range(30):
            self.world.tick()
        self.car.set_target_velocity(carla.Vector3D(0, 0, 0))
        for _ in range(10):
            self.world.tick()

    def state(self):
        tf, v = self.car.get_transform(), self.car.get_velocity()
        return tf.location.x, tf.location.y, tf.rotation.yaw, math.hypot(v.x, v.y)

    def drive(self, ctl: PIDController, plan_world: np.ndarray, target: float, ticks: int, interval: float):
        """Run the controller on a WORLD-frame plan, re-projected into the ego frame every tick (as
        the daemon does). Returns per-tick (x, y, yaw, speed, throttle, brake, steer)."""
        rows = []
        for _ in range(ticks):
            x, y, yaw, v = self.state()
            plan_ego = geo.world_to_ego((x, y), yaw, plan_world)
            ahead = plan_ego[plan_ego[:, 0] > 0.5]
            if len(ahead) < 2:
                ahead = plan_ego[-2:]
            steer, thr, brk, meta = ctl.control(ahead, v, target, interval)
            control = carla.VehicleControl(steer=steer, throttle=thr, brake=brk)
            gear = ctl.gear(meta["desired_speed"], v)
            if gear is not None:
                control.manual_gear_shift, control.gear = True, gear
            self.car.apply_control(control)
            self.world.tick()
            rows.append((x, y, yaw, v, thr, brk, steer))
        return rows


def straight_plan(x, y, yaw, length=230.0, step=2.0, offset=None):
    """World points along the heading; `offset(s)` gives the LEFT offset at arclength s."""
    s = np.arange(step, length, step)
    left = np.array([offset(v) for v in s]) if offset else np.zeros_like(s)
    return geo.ego_to_world((x, y), yaw, np.stack([s, left], axis=1))


def arc_plan(x, y, yaw, radius, length, step=1.0, right=True):
    th = np.arange(step, length, step) / radius
    fwd, side = radius * np.sin(th), radius * (1 - np.cos(th))
    return geo.ego_to_world((x, y), yaw, np.stack([fwd, -side if right else side], axis=1))


def cross_track(rows, plan_world, start_xy):
    # the plan's first point is a step ahead of the car: measure against the path FROM the car
    pts = np.vstack([np.asarray(start_xy, float)[None, :], np.asarray(plan_world)])
    out = []
    for x, y, *_ in rows:
        d = np.linalg.norm(pts - np.array([x, y]), axis=1)
        i = int(np.argmin(d))
        best = d[i]
        for j in (i - 1, i):
            if 0 <= j < len(pts) - 1:
                a, b = pts[j], pts[j + 1]
                ab = b - a
                t = np.clip(np.dot(np.array([x, y]) - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
                best = min(best, float(np.linalg.norm(np.array([x, y]) - (a + t * ab))))
        out.append(best)
    return np.array(out)


def run_up(rig, ctl, v):
    """Accelerate on the straight to speed v and settle there. Returns the world plan in use."""
    rig.reset()
    x, y, yaw, _ = rig.state()
    plan = straight_plan(x, y, yaw)
    for _ in range(400):
        rig.drive(ctl, plan, v, 1, 0.5)
        if abs(rig.state()[3] - v) < 0.15:
            break
    rig.drive(ctl, plan, v, 20, 0.5)
    return plan


def make_ctl(base_cfg, name):
    cfg = copy.deepcopy(base_cfg)
    if name == "pid075":
        cfg["long_mode"], cfg["max_throttle"] = "pid", 0.75
    elif name == "pid100":
        cfg["long_mode"], cfg["max_throttle"] = "pid", 1.0
    elif name == "lead":
        cfg["long_mode"] = "lead"
    elif name == "plus":   # the Privileged+ preset: must match run_session.sh PID_PRESET=plus
        cfg.update({"long_mode": "pid", "max_throttle": 0.75, "launch_throttle": 1.0, "steer_mode": "adaptive",
                    "speed_scale": 0.5, "speed_offset": 1.2, "lookahead_min": 2.2,
                    "cruise_brake_ratio": 1.25, "cruise_brake": 0.4, "cruise_gear_hold": False,
                    "avoid_speed_band": [5.0, 6.7]})
    elif name == "boost":
        cfg["long_mode"], cfg["max_throttle"], cfg["launch_throttle"] = "pid", 0.75, 1.0
    else:
        raise ValueError(name)
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=20090)
    ap.add_argument("--town", default="Town04")
    ap.add_argument("--configs", default="pid075,pid100,lead")
    ap.add_argument("--out", type=Path, default=REPO / "tools" / "controller_model.json")
    ap.add_argument("--lateral-from", default=None,
                    help="copy the tracking-error rows from this already-measured config instead of measuring "
                         "(valid when only the longitudinal setting differs)")
    ap.add_argument("--merge", action="store_true", help="add to the existing --out file instead of replacing it")
    args = ap.parse_args()
    base = yaml.safe_load((REPO / "carla_probe" / "config.yaml").read_text())
    client = carla.Client("localhost", args.port)
    client.set_timeout(300)
    rig = Rig(client, args.town)
    prev = json.loads(args.out.read_text()) if args.merge and args.out.exists() else None
    out = prev or {"vehicle": "vehicle.lincoln.mkz_2020", "dt": DT, "town": args.town, "measured": time.strftime("%Y-%m-%d"),
           "pid_base": {k: base["pid"][k] for k in ("steer_mode", "lookahead_min", "lookahead_max", "speed_scale", "speed_offset")},
           "configs": {}}
    try:
        for name in args.configs.split(","):
            cfg = make_ctl(base["pid"], name)
            res = {"launch": {}, "brake": {}, "offset": [], "arc": []}
            for target in (3.0, 5.0, 7.0, 9.0, 12.0):
                rig.reset()
                x, y, yaw, _ = rig.state()
                rows = rig.drive(PIDController(cfg), straight_plan(x, y, yaw), target, 120, 0.5)
                sp = [round(r[3], 3) for r in rows]
                res["launch"][str(target)] = sp
                res.setdefault("cruise_std", {})[str(target)] = round(float(np.std(sp[-40:])), 3)
                print(f"{name} launch {target:>4} m/s: v@1s {sp[19]:.2f} v@2s {sp[39]:.2f} v@4s {sp[79]:.2f} max {max(sp):.2f}", flush=True)
            for v0 in (5.0, 9.0, 12.0):
                ctl = PIDController(cfg)
                plan = run_up(rig, ctl, v0)
                rows = rig.drive(ctl, plan, 0.0, 120, 0.5)
                sp = [round(r[3], 3) for r in rows]
                res["brake"][str(v0)] = sp
                stop = next((i for i, s in enumerate(sp) if s < 0.1), None)
                print(f"{name} brake from {v0:>4} (actual {sp[0]:.2f}): stopped after {stop and stop * DT} s", flush=True)
            if args.lateral_from:
                src = out["configs"][args.lateral_from]
                res["offset"], res["arc"] = src["offset"], src["arc"]
                res["lateral_copied_from"] = args.lateral_from
            for v in (() if args.lateral_from else (3.0, 6.0, 9.0)):
                for lat, dist in ((1.5, 15.0), (3.5, 15.0), (3.5, 30.0)):
                    ctl = PIDController(cfg)
                    run_up(rig, ctl, v)
                    x, y, yaw, _ = rig.state()
                    off = lambda s, lat=lat, dist=dist: lat * (0.5 - 0.5 * math.cos(math.pi * min(s, dist) / dist))
                    plan = straight_plan(x, y, yaw, length=dist + 60, step=1.0, offset=off)
                    ticks = int((dist + 15) / v / DT)
                    rows = rig.drive(ctl, plan, v, ticks, 0.5)
                    e = cross_track(rows, plan, (x, y))
                    peak_curv = lat * (math.pi / dist) ** 2 / 2
                    res["offset"].append({"speed": v, "lateral_m": lat, "over_m": dist, "peak_curvature": round(peak_curv, 4),
                                          "mean": round(float(e.mean()), 3), "p95": round(float(np.percentile(e, 95)), 3),
                                          "max": round(float(e.max()), 3)})
                    print(f"{name} offset {lat} m over {dist} m at {v} m/s: p95 {np.percentile(e, 95):.2f} max {e.max():.2f}", flush=True)
                for radius in (12.0, 20.0, 35.0):
                    if v * v / radius > 4.0:      # beyond ~0.4 g the car slides: not a regime a plan should ask for
                        continue
                    ctl = PIDController(cfg)
                    run_up(rig, ctl, v)
                    x, y, yaw, _ = rig.state()
                    plan = arc_plan(x, y, yaw, radius, length=v * 3.0 + 20)
                    rows = rig.drive(ctl, plan, v, int(3.0 / DT), 0.5)
                    e = cross_track(rows, plan, (x, y))
                    res["arc"].append({"speed": v, "radius": radius, "curvature": round(1 / radius, 4),
                                       "mean": round(float(e.mean()), 3), "p95": round(float(np.percentile(e, 95)), 3),
                                       "max": round(float(e.max()), 3)})
                    print(f"{name} arc r={radius} at {v} m/s: p95 {np.percentile(e, 95):.2f} max {e.max():.2f}", flush=True)
            out["configs"][name] = res
            args.out.write_text(json.dumps(out, indent=1))
    finally:
        rig.car.destroy()
        s = rig.world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = False, None
        rig.world.apply_settings(s)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
