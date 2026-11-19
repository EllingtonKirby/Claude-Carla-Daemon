"""Waypoint-tracking PID controller (TransFuser lineage), taking plans directly in the ego frame.

``control(plan_ego, speed, target_speed)``:

* **Lateral**: pick an aim point on the plan, then steer on its bearing (normalised by 90 deg)
  through a windowed PID. Positive steer = right, as in CARLA. ``steer_mode`` picks how:

  ``fixed``     the TransFuser-lineage rule: the waypoint starting the segment whose MIDPOINT is
                nearest ``aim_dist``. Beware -- the aim lands ON a waypoint, so the effective
                lookahead is quantised to the driver's spacing rather than being ``aim_dist``: with
                ``aim_dist`` 7 it measures 4 m on a 4 m-spaced plan and 15 m on a 15 m-spaced one.
                It also degenerates: ``_compute_control`` hands over the plan from the waypoint
                nearest the ego, so once the ego reaches a waypoint that point is at ~0 m, and at
                spacings where the first midpoint is nearer ``aim_dist`` than the second (7.5 m at
                ``aim_dist`` 7) the controller aims at the ego itself. Kept as the default because
                it is what the tuned gains were tuned against.
  ``adaptive``  aim at arc-length ``clip(speed_scale*speed + speed_offset, lookahead_min,
                lookahead_max)`` along the plan, INTERPOLATED, so the lookahead is what it says and
                does not depend on waypoint spacing: close when slow (tight junction tracking), far
                when fast (smooth).
  ``lead``      the same constants on speed in km/h with LEAD's clip arithmetic
                (``clip(., 24, 105)/10 - 2`` -> 0.4-8.5 m, pinned at 0.4 m below ~6.3 m/s), plus a
                curvature term looking up to +2 m farther through bends (``curvature_gain``).
* **Longitudinal**: desired speed = ``target_speed`` if given, else the plan's mean waypoint spacing
  divided by ``plan_interval_s`` (so waypoint spacing encodes speed), then capped at ``max_speed``.
  ``long_mode="pid"`` brakes when the desired speed is below ``brake_speed`` or the ego is more than
  ``brake_ratio`` x too fast, and otherwise drives throttle from a speed PID on the clipped error.
  ``long_mode="lead"`` uses LEAD's memoryless expert-tuned regression map instead: no integral to
  wind up, so launches are crisp and a ~zero target is a hard stop.

Ported from dynamo_nav123d/eval_carla/team_code/pid_controller.py, which carries the tuning these
gains come from. That file works in a [right, forward] frame; this one keeps the probe's
client-facing [forward, left] frame throughout, so only the aim SCHEDULES are shared, not the
trigonometry.

The PIDs are stateful (integral/derivative windows): keep ONE controller per session.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Dict, Optional, Tuple

import numpy as np


_STEER_MODES = ("fixed", "adaptive", "lead", "pursuit")
_LONG_MODES = ("pid", "lead")
# LEAD's expert-tuned longitudinal regression: the first six weight the (speed, speed_error)
# feature vector below (km/h, scaled /100), the last is its brake ratio. Fixed constants of the
# ported controller, not knobs -- select them with long_mode="lead".
_LEAD_LONG_PARAMS = (1.1990342347353184, -0.8057602384167799, 1.710818710950062, 0.921890257450335,
                     1.556497522998393, -0.7013479734904027, 1.031266635497984)
_LEAD_MAX_ACCEL_KPH = 1.89   # speed error above this -> full throttle
_LEAD_MIN_TARGET_MPS = 0.278  # tiny-but-nonzero targets are floored to 1 km/h


class PID:
    def __init__(self, k_p: float, k_i: float, k_d: float, n: int) -> None:
        self._k_p, self._k_i, self._k_d = k_p, k_i, k_d
        self._window: deque = deque([0.0] * n, maxlen=n)

    def step(self, error: float) -> float:
        self._window.append(error)
        if len(self._window) >= 2:
            integral = float(np.mean(self._window))
            derivative = self._window[-1] - self._window[-2]
        else:
            integral = derivative = 0.0
        return self._k_p * error + self._k_i * integral + self._k_d * derivative


class PIDController:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        """``cfg`` is the ``pid`` block of the probe config; every key is required."""
        self.turn = PID(float(cfg["turn_kp"]), float(cfg["turn_ki"]), float(cfg["turn_kd"]), int(cfg["turn_n"]))
        self.speed = PID(float(cfg["speed_kp"]), float(cfg["speed_ki"]), float(cfg["speed_kd"]), int(cfg["speed_n"]))
        self.max_throttle = float(cfg["max_throttle"])
        # Launch boost: allow `launch_throttle` while the car is more than `launch_margin` m/s below
        # its target, max_throttle otherwise. Measured 2026-11-19: max_throttle 1.0 everywhere
        # doubles the launch but turns cruise into a bang-bang limit cycle (+-1.3 m/s around 6 m/s),
        # because the gains were tuned against the 0.75 cap; boosting only the launch keeps both.
        self.launch_throttle = float(cfg["launch_throttle"])
        # Over-speed handling at cruise. Stock: FULL brake once speed > brake_ratio x target, which on a
        # live route turned a 10% overshoot into a 0.5 s stab of braking and a +-0.8 m/s limit cycle.
        # Between brake_ratio and cruise_brake_ratio the car coasts; beyond it, it brakes at
        # cruise_brake. cruise_brake_ratio == brake_ratio and cruise_brake == 1.0 is the stock behaviour.
        self.cruise_brake_ratio = float(cfg["cruise_brake_ratio"])
        self.cruise_brake = float(cfg["cruise_brake"])
        # Cruise gear hold. Measured 2026-11-19: cruising at 6 m/s the automatic gearbox hunts between
        # 1st, 2nd and neutral (speed std 0.58 m/s, 0.6 m/s single-tick jumps); holding 2nd/3rd gives
        # 0.17-0.21. Launch stays automatic; within launch_margin of the target a gear is held.
        self.cruise_gear_hold = bool(cfg["cruise_gear_hold"])
        # ...but the leaderboard forces manual_gear_shift = False on every control (autonomous_agent.py),
        # so a gear cannot be held in a benchmark run. What works within the rules: never HOLD a target
        # inside the band where the gearbox hunts (measured 5.0-6.7 m/s on the MKZ: speed std 0.3-0.7
        # inside, 0.15 outside). A target in the band is held at the nearer edge. [] = off.
        band = list(cfg["avoid_speed_band"] or [])
        self.avoid_band = (float(band[0]), float(band[1])) if band else None
        self.launch_margin = float(cfg["launch_margin"])
        self.clip_delta = float(cfg["clip_delta"])
        self.brake_speed = float(cfg["brake_speed"])
        self.brake_ratio = float(cfg["brake_ratio"])
        self.aim_dist = float(cfg["aim_dist"])
        self.max_speed = float(cfg["max_speed"])
        self.steer_mode = str(cfg["steer_mode"]).strip().lower()
        assert self.steer_mode in _STEER_MODES, f"steer_mode must be one of {_STEER_MODES}"
        self.lookahead_min = float(cfg["lookahead_min"])
        self.lookahead_max = float(cfg["lookahead_max"])
        self.speed_scale = float(cfg["speed_scale"])
        self.speed_offset = float(cfg["speed_offset"])
        self.curvature_gain = float(cfg["curvature_gain"])
        self.brake_zero_steer = bool(cfg["brake_zero_steer"])
        self.steer_rate_limit = float(cfg["steer_rate_limit"])
        # pursuit: geometric pure pursuit from the rear axle (measured on the MKZ 2020: wheelbase
        # 2.86 m, rear axle 1.39 m behind the origin, 70 deg max wheel angle). Steady state on an arc
        # is the arc itself, where the bearing-PID modes cut inside by roughly lookahead^2 / 2R.
        self.pursuit_gain = float(cfg["pursuit_gain"])
        self.pursuit_offset = float(cfg["pursuit_offset"])
        self.pursuit_min = float(cfg["pursuit_min"])
        self.pursuit_max = float(cfg["pursuit_max"])
        self.wheelbase = float(cfg["wheelbase_m"])
        self.rear_axle = float(cfg["rear_axle_m"])
        self.max_steer_rad = math.radians(float(cfg["max_steer_deg"]))
        # CARLA scales the steer command down with speed (the vehicle's steering_curve, km/h -> scale);
        # an exact wheel angle has to be divided back by it.
        curve = np.asarray(cfg["steering_curve"], dtype=np.float64).reshape(-1, 2)
        self._steer_curve_kph, self._steer_curve_scale = curve[:, 0], curve[:, 1]
        self._last_steer: Optional[float] = None
        self.long_mode = str(cfg["long_mode"]).strip().lower()
        assert self.long_mode in _LONG_MODES, f"long_mode must be one of {_LONG_MODES}"

    def control(self, plan_ego: np.ndarray, speed: float, target_speed: Optional[float],
                plan_interval_s: float) -> Tuple[float, float, float, Dict[str, float]]:
        """Returns ``(steer, throttle, brake, meta)``; ``plan_ego`` is (N>=2, 2) [forward, left]."""
        wps = np.asarray(plan_ego, dtype=np.float64).reshape(-1, 2)
        assert len(wps) >= 2, "plan needs >= 2 waypoints"
        num_pairs = len(wps) - 1

        spacing = float(np.linalg.norm(np.diff(wps, axis=0), axis=1).sum())
        aim = self._aim(wps, speed)

        if target_speed is None:
            desired = spacing / (plan_interval_s * num_pairs)
        else:
            desired = float(target_speed)
        desired = self.cruise_target(min(max(desired, 0.0), self.max_speed))

        throttle, brake, delta = self._longitudinal(speed, desired)

        if self.steer_mode == "pursuit":
            steer, la, aim = self._pursuit(wps, speed)
            steer = self._rate_limit(steer)
            meta = {"desired_speed": desired, "aim_fwd": float(aim[0]), "aim_left": float(aim[1]),
                    "angle": 0.0, "lookahead": la, "delta": delta}
            return steer, throttle, float(brake), meta

        # Bearing of the aim point, 0 = straight ahead, + = to the RIGHT (CARLA steer sign).
        right = -aim[1]
        angle = math.degrees(math.pi / 2.0 - math.atan2(aim[0], right)) / 90.0
        # Anti-windup: hold the steering error at zero while braking or stopped, so the turn
        # integral drains at a red light instead of pre-loading a saturated steer for the first
        # ticks after it goes green.
        if self.brake_zero_steer and (brake or speed < 0.01):
            angle = 0.0
        steer = float(np.clip(self.turn.step(angle), -1.0, 1.0))
        steer = self._rate_limit(steer)

        meta = {"desired_speed": desired, "aim_fwd": float(aim[0]), "aim_left": float(aim[1]),
                "angle": angle, "lookahead": self._lookahead(wps, speed), "delta": delta}
        return steer, throttle, float(brake), meta

    def _rate_limit(self, steer: float) -> float:
        """Cap how far the steer command may move in one tick. This treats the symptom rather than
        the cause -- it cannot tell controller chatter from a genuine swerve, and it delays every
        real correction by the same amount -- so leave it off (>= 1.0, the full range in one tick)
        unless a measurement says the loop is still chattering with its noise source fixed."""
        if self.steer_rate_limit < 1.0 and self._last_steer is not None:
            steer = float(np.clip(steer, self._last_steer - self.steer_rate_limit,
                                  self._last_steer + self.steer_rate_limit))
        self._last_steer = steer
        return steer

    def cruise_target(self, desired: float) -> float:
        """The target actually held: one inside avoid_speed_band moves to the nearer edge of it."""
        if self.avoid_band is None:
            return desired
        lo, hi = self.avoid_band
        if lo < desired < hi:
            return lo - 0.1 if desired - lo < hi - desired else hi + 0.1
        return desired

    def gear(self, desired: float, speed: float) -> Optional[int]:
        """The gear to hold at cruise, or None for the automatic gearbox."""
        if not self.cruise_gear_hold or desired < self.brake_speed or abs(desired - speed) > self.launch_margin:
            return None
        for top, g in ((4.5, None), (7.0, 2), (11.0, 3), (16.0, 4)):
            if desired < top:
                return g
        return None

    # ------------------------------------------------------------------ lateral
    def _pursuit(self, wps: np.ndarray, speed: float):
        la = float(np.clip(self.pursuit_gain * speed + self.pursuit_offset, self.pursuit_min, self.pursuit_max))
        rear = wps + np.array([self.rear_axle, 0.0])           # the plan as seen from the rear axle
        pts = np.vstack([np.zeros((1, 2)), rear])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.cumsum(seg)
        if cum[-1] <= la:
            aim = rear[-1]
        else:
            j = int(np.searchsorted(cum, la))
            prev = cum[j - 1] if j > 0 else 0.0
            aim = pts[j] + (la - prev) / max(float(seg[j]), 1e-6) * (pts[j + 1] - pts[j])
        ld = max(float(np.hypot(aim[0], aim[1])), 1e-3)
        alpha = math.atan2(aim[1], aim[0])                     # + = aim point to the LEFT
        wheel = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        scale = float(np.interp(speed * 3.6, self._steer_curve_kph, self._steer_curve_scale))
        steer = float(np.clip(-wheel / (self.max_steer_rad * max(scale, 0.1)), -1.0, 1.0))   # CARLA: + steers RIGHT
        return steer, la, aim - np.array([self.rear_axle, 0.0])


    def _lookahead(self, wps: np.ndarray, speed: float) -> float:
        """Arc-length the aim point should sit at, for the interpolating modes. `fixed` has no
        single answer (it snaps to a waypoint), so it reports its nominal aim_dist."""
        if self.steer_mode == "adaptive":
            return float(np.clip(self.speed_scale * speed + self.speed_offset,
                                 self.lookahead_min, self.lookahead_max))
        if self.steer_mode == "lead":
            la = float(np.clip(self.speed_scale * (speed * 3.6) + self.speed_offset, 24.0, 105.0)) / 10.0 - 2.0
            if self.curvature_gain > 0.0:
                # Mean bearing off straight-ahead; the magnitude is frame-independent.
                curvature = float(np.mean(np.abs(np.arctan2(wps[:, 1], wps[:, 0]))))
                la += float(np.clip(int(curvature * self.curvature_gain), 0, 2))
            return la
        return self.aim_dist

    def _aim(self, wps: np.ndarray, speed: float) -> np.ndarray:
        if self.steer_mode == "fixed":
            aim, best = wps[0], 1e5
            for i in range(len(wps) - 1):
                norm = float(np.linalg.norm((wps[i + 1] + wps[i]) / 2.0))
                if abs(self.aim_dist - best) > abs(self.aim_dist - norm):
                    aim, best = wps[i], norm
            return aim
        return self._interp_aim(wps, self._lookahead(wps, speed))

    @staticmethod
    def _interp_aim(wps: np.ndarray, lookahead: float) -> np.ndarray:
        """The point `lookahead` metres along the plan from the ego, linearly interpolated between
        the bracketing waypoints -- so the lookahead is independent of how the driver spaced them.
        Returns the last waypoint when the whole plan is shorter than the lookahead."""
        pts = np.vstack([np.zeros((1, 2)), wps])  # arc length is measured from the ego, not wps[0]
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        cum = np.cumsum(seg)
        if cum[-1] <= lookahead:
            return wps[-1]
        j = int(np.searchsorted(cum, lookahead))
        prev = cum[j - 1] if j > 0 else 0.0
        frac = (lookahead - prev) / max(float(seg[j]), 1e-6)
        return pts[j] + frac * (pts[j + 1] - pts[j])

    # ------------------------------------------------------------------ longitudinal
    def _longitudinal(self, speed: float, desired: float) -> Tuple[float, bool, float]:
        if self.long_mode == "lead":
            throttle, brake = self._lead_throttle(desired, speed)
            return throttle, brake, 0.0
        stop = desired < self.brake_speed
        over = speed / max(desired, 1e-6)
        if stop or over > max(self.cruise_brake_ratio, self.brake_ratio):
            brake = 1.0 if stop else self.cruise_brake
        else:
            brake = 0.0
        coast = (not stop) and brake == 0.0 and over > self.brake_ratio
        delta = float(np.clip(desired - speed, 0.0, self.clip_delta))
        cap = self.launch_throttle if (desired - speed) > self.launch_margin else self.max_throttle
        throttle = 0.0 if (brake or coast) else float(np.clip(self.speed.step(delta), 0.0, max(cap, self.max_throttle)))
        if not (brake or coast) and cap > self.max_throttle:
            throttle = cap          # well below target: the clipped-error PID would saturate anyway
        return throttle, brake, delta

    def _lead_throttle(self, desired: float, speed: float) -> Tuple[float, bool]:
        """LEAD's `get_throttle`: an expert-tuned linear regression, memoryless -- nothing winds up
        at a stop, and an error above ~1.9 km/h goes straight to full throttle."""
        if desired < 1e-5:
            return 0.0, True
        desired = max(desired, _LEAD_MIN_TARGET_MPS)
        v_kph, target_kph = speed * 3.6, desired * 3.6
        err_kph = target_kph - v_kph
        if err_kph > _LEAD_MAX_ACCEL_KPH:
            return 1.0, False
        if v_kph / target_kph > _LEAD_LONG_PARAMS[-1]:
            return 0.0, True
        err, v = max(err_kph, 0.0) / 100.0, v_kph / 100.0
        feats = np.array([v, v * v, 100.0 * err, err * err, v * err, v * v * err])
        return float(np.clip(feats @ np.asarray(_LEAD_LONG_PARAMS[:-1]), 0.0, 1.0)), False
