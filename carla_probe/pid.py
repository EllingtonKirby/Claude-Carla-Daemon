"""Waypoint-tracking PID controller (TransFuser lineage), taking plans directly in the ego frame.

``control(plan_ego, speed, target_speed)``:

* **Lateral**: aim at the plan waypoint whose segment midpoint is nearest ``aim_dist`` metres from
  the ego; steer error is that aim point's bearing (normalised by 90 deg) through a windowed PID.
  Positive steer = right, as in CARLA.
* **Longitudinal**: desired speed = ``target_speed`` if given, else the plan's mean waypoint spacing
  divided by ``plan_interval_s`` (so waypoint spacing encodes speed). Brake when the desired speed is
  below ``brake_speed`` or the ego is more than ``brake_ratio`` x too fast; otherwise a speed PID on
  the clipped speed error drives throttle.

The PIDs are stateful (integral/derivative windows): keep ONE controller per session.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Any, Dict, Optional, Tuple

import numpy as np


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
        self.clip_delta = float(cfg["clip_delta"])
        self.brake_speed = float(cfg["brake_speed"])
        self.brake_ratio = float(cfg["brake_ratio"])
        self.aim_dist = float(cfg["aim_dist"])
        self.max_speed = float(cfg["max_speed"])

    def control(self, plan_ego: np.ndarray, speed: float, target_speed: Optional[float],
                plan_interval_s: float) -> Tuple[float, float, float, Dict[str, float]]:
        """Returns ``(steer, throttle, brake, meta)``; ``plan_ego`` is (N>=2, 2) [forward, left]."""
        wps = np.asarray(plan_ego, dtype=np.float64).reshape(-1, 2)
        assert len(wps) >= 2, "plan needs >= 2 waypoints"
        num_pairs = len(wps) - 1

        aim, best = wps[0], 1e5
        spacing = 0.0
        for i in range(num_pairs):
            spacing += float(np.linalg.norm(wps[i + 1] - wps[i]))
            norm = float(np.linalg.norm((wps[i + 1] + wps[i]) / 2.0))
            if abs(self.aim_dist - best) > abs(self.aim_dist - norm):
                aim, best = wps[i], norm

        if target_speed is None:
            desired = spacing / (plan_interval_s * num_pairs)
        else:
            desired = float(target_speed)
        desired = min(max(desired, 0.0), self.max_speed)

        brake = desired < self.brake_speed or (speed / desired) > self.brake_ratio
        delta = float(np.clip(desired - speed, 0.0, self.clip_delta))
        throttle = 0.0 if brake else float(np.clip(self.speed.step(delta), 0.0, self.max_throttle))

        # Bearing of the aim point, 0 = straight ahead, + = to the RIGHT (CARLA steer sign).
        right = -aim[1]
        angle = math.degrees(math.pi / 2.0 - math.atan2(aim[0], right)) / 90.0
        steer = float(np.clip(self.turn.step(angle), -1.0, 1.0))

        meta = {"desired_speed": desired, "aim_fwd": float(aim[0]), "aim_left": float(aim[1]), "angle": angle}
        return steer, throttle, float(brake), meta
