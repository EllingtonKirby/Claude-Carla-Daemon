"""CARLA probe daemon: a Bench2Drive leaderboard agent driven over a JSON-lines socket.

The leaderboard runs CARLA in synchronous mode and calls ``run_step`` every tick; the world only
advances when it returns. This agent blocks inside ``run_step`` waiting for commands, so while it
waits the ENTIRE simulation is frozen: a client (Claude, or a human) can inspect the scene for
minutes at zero sim-time cost, then advance time in controlled steps. The route is scored exactly as
any other agent's.

There is no built-in driving policy. The client IS the driver. Control modes:
  hold  brake to a standstill (the initial mode)
  plan  track a client plan (ego-frame waypoints) with a PID controller   <- apply_plan
  raw   apply fixed steer/throttle/brake                                  <- set_control

Access levels (config ``access``), enforced here so a client cannot bypass them:
  privileged  ground-truth scene: actors, lanes, lights, stop signs, speed limit, live score
  sensors     Bench2Drive SENSORS-track inputs only: cameras, speedometer, IMU, GNSS and the
              leaderboard's route plan. Ego pose is estimated from GNSS + compass.

A time-advancing command gets ONE response, sent when the sim pauses again. If the route ends
mid-advance, the response comes from ``destroy`` with reason ``route_end``. Command reference:
``help`` and docs/OPERATOR_GUIDE.md. Launch with scripts/run_session.sh.
"""

# carla, leaderboard and srunner only exist in the simulation environment.
# pyright: reportMissingImports=false
from __future__ import annotations

import json
import math
import os
import sys
import time
import traceback
from collections import Counter, deque
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import carla
import cv2
import numpy as np
import yaml
from leaderboard.autoagents import autonomous_agent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carla_probe import geometry as geo  # noqa: E402
from carla_probe.bev import box_polygon, lanes_to_polys, render_bev  # noqa: E402
from carla_probe.pid import PIDController  # noqa: E402
from carla_probe.server import CommandServer  # noqa: E402

ACCESS_LEVELS = ("privileged", "sensors")
_ACTOR_PREFIXES = ("vehicle.", "walker.", "static.")
# Level (baked-into-the-map) vehicles: invisible to world.get_actors() but solid.
_PARKED_LABELS = ("Car", "Truck", "Bus", "Motorcycle", "Bicycle")
_LANE_OPTIONS = ("LANEFOLLOW", "VOID")
# The ego footprint lives in config.yaml under `ego:` -- it is reported to the driver, so it is a
# knob, not a constant. It used to be hardcoded here as (2.45, 1.05), which was the MKZ *2017*'s
# half-width against a MKZ 2020 hero: 0.13 m too wide per side, in every sensors-only BEV.
# RunningStopTest.point_inside_boundingbox scales stop trigger volumes by this: mirrored, not tuned.
_STOP_BOX_MULTIPLIER = 1.2

HELP: Dict[str, str] = {
    "status": "session summary (also attached to every reply)",
    "help": "this table",
    "tick n [stop_on_collision]": "advance n ticks in the current mode",
    "run_until_frame frame": "advance until step >= frame",
    "run_until_event event [max_ticks]": "event = stopped | plan_done | collision* | infraction*",
    "get_state": "full state (privileged: world + ego frame scene; sensors: raw ego sensors + pose estimate)",
    "get_score*": "live driving-score estimate with every infraction event",
    "percept [cams] [k]": "THE per-decision view, and the only one: the driving state plus a "
                                  "JPG from every camera for this tick, in "
                                  "ONE round trip. percept=true on a driving command returns the next "
                                  "one with its result. Open the JPGs you need with your file reader",
    "clearance points [waypoints]": "how close your BODY comes to points you believe are at "
        "ego-frame [[fwd,left],...] -- now, and over a path (the one given, else the plan in "
        "flight). Every distance in a reply is to the ego ORIGIN, not the bodywork; this does the "
        "subtraction. Also available as points= on percept, in the same round trip as looking",
    "render_bev": "top-down PNG (privileged: lanes, actors, lights; sensors: route + plan only)",
    "dump_frame": "state.json + cameras + bev.png into frames/frame_<step>/",
    "apply_plan waypoints ticks [target_speed] [stop_on_collision]":
        "track ego-frame [[fwd,left],...] (>=2 pts, plan_interval_s apart) for ticks",
    "set_control steer throttle brake ticks [stop_on_collision]": "raw control for ticks",
    "drive text ticks [mode] [stop_on_collision]":
        "let the learned policy (config vla) drive for ticks, told `text` -- a sentence in its "
        "phrasebook (docs/OPERATOR_GUIDE.md §8a). mode = commentary (default: a description of the "
        "scene and what to do) | instruction (an order it obeys) | safety (an order it may refuse) | "
        "plain (no text: the policy alone). The reply carries what it did",
    "hold [ticks]": "brake; with ticks, also advance",
    "record on [every]": "toggle camera frame logging to camera_log/ (for tools/make_video.py)",
    "shutdown": "abort the route (scored as an agent failure)",
    "why": "justification string, REQUIRED on every driving command (apply_plan, set_control, hold, tick, "
           "run_until_*, shutdown): what you observed and why this command follows from it",
}
_PRIVILEGED_EVENTS = ("collision", "infraction")
# Commands that decide what the ego does. With require_justification they must carry why="...", and
# every accepted one is logged to decisions.jsonl / decisions.txt with its outcome.
_DRIVING_COMMANDS = ("apply_plan", "set_control", "drive", "hold", "tick", "run_until_frame", "run_until_event", "shutdown")
_VLA_MODES = ("plain", "commentary", "command", "instruction", "safety", "cot")
_ADVANCE_EVENTS = ("stopped", "plan_done") + _PRIVILEGED_EVENTS


class _ShutdownRequested(Exception):
    pass


def get_entry_point() -> str:
    return "ProbeAgent"


def _vec(v) -> List[float]:
    return [round(float(v.x), 3), round(float(v.y), 3), round(float(v.z), 3)]


def _rot(r) -> List[float]:
    return [round(float(r.roll), 3), round(float(r.pitch), 3), round(float(r.yaw), 3)]


def _r(x: float, nd: int = 2) -> float:
    return round(float(x), nd)


def _event_key(ev: Dict[str, Any]) -> Tuple[Any, ...]:
    return ev["type"], ev["frame"], ev["message"]


def _new_events(events: List[Dict[str, Any]], already: "Counter[Tuple[Any, ...]]") -> List[Dict[str, Any]]:
    """Events not accounted for by ``already``. Criteria events are concatenated per criterion, so a
    new event can land BEFORE older ones: diff by identity, not by index."""
    seen = Counter(already)
    new = []
    for ev in events:
        key = _event_key(ev)
        if seen[key] > 0:
            seen[key] -= 1
        else:
            new.append(ev)
    return new


def _resolve_config_path(raw: str) -> str:
    """The leaderboard appends '+<save_name>' to --agent-config per route, and a path may itself
    contain '+', so take the longest '+'-joined prefix that is an existing file."""
    parts = (raw or "").split("+")
    for i in range(len(parts), 0, -1):
        candidate = "+".join(parts[:i])
        if candidate and os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(f"probe config not found in --agent-config {raw!r}")


_TICK_S = 0.05   # the leaderboard's fixed simulation step


class ProbeAgent(autonomous_agent.AutonomousAgent):
    # ------------------------------------------------------------------ lifecycle
    def setup(self, path_to_conf_file: str) -> None:
        self.track = autonomous_agent.Track.SENSORS
        with open(_resolve_config_path(path_to_conf_file)) as f:
            cfg = yaml.safe_load(f)
        # Every knob is read eagerly with cfg["key"]: a missing key fails HERE, not mid-session.
        self._access = str(cfg["access"])
        assert self._access in ACCESS_LEVELS, f"access must be one of {ACCESS_LEVELS}, got {self._access!r}"
        self._sensors_only = self._access == "sensors"
        self._out = Path(cfg["output_dir"]).resolve()
        self._out.mkdir(parents=True, exist_ok=True)
        self._state_radius = float(cfg["state_radius_m"])
        self._nearest_k = int(cfg["observe_nearest_actors"])
        self._lookahead = [float(d) for d in cfg["route_lookahead_m"]]
        self._route_window = int(cfg["route_search_window"])
        self._stopped_speed = float(cfg["stopped_speed_mps"])
        self._stopped_min_ticks = int(cfg["stopped_min_ticks"])
        # The collision sensor is not a Bench2Drive-eligible sensor: never attached in sensors access.
        self._want_collision_sensor = bool(cfg["collision_sensor"]) and not self._sensors_only
        self._gnss_gain = float(cfg["gnss_position_gain"])
        self._gnss_filter = str(cfg["gnss_filter"]).strip().lower()
        assert self._gnss_filter in ("blend", "dr_offset", "dr_trend"), \
            f"gnss_filter must be 'blend', 'dr_offset' or 'dr_trend', got {self._gnss_filter!r}"
        # Window in SAMPLES. The leaderboard runs a fixed 20 Hz, which is also what the sensor
        # specs below assume (`sensor_tick: 0.05`); a short window is harmless if that ever changes,
        # since the fit only needs the samples to be evenly spaced, not to be 50 ms apart.
        self._gnss_window = deque(maxlen=max(1, int(float(cfg["gnss_offset_window_s"]) / _TICK_S)))
        self._dr_xy: Optional[Tuple[float, float]] = None
        # How an active plan is tracked between re-plans (sensors access only; under privileged the
        # pose is ground truth, so re-projecting through it is exact and the knob is ignored).
        self._plan_tracking = str(cfg["plan_tracking"]).strip().lower()
        assert self._plan_tracking in ("gnss", "dead_reckon"), \
            f"plan_tracking must be 'gnss' or 'dead_reckon', got {self._plan_tracking!r}"
        self._dead_reckon_plan = self._plan_tracking == "dead_reckon" and self._sensors_only
        self._plan_interval_s = float(cfg["plan_interval_s"])
        self._bev = {k: cfg["bev"][k] for k in ("forward_m", "back_m", "side_m", "px_per_m", "draw_lanes",
                                                "lane_step_m", "lane_max_nodes")}
        self._record_traj = bool(cfg["record"]["trajectory"])
        self._record_cam_id = str(cfg["record"]["camera_id"])
        self._record_every = int(cfg["record"]["camera_every_n_ticks"])
        self._ego_body = {k: float(cfg["ego"][k]) for k in ("front_m", "rear_m", "half_width_m")}
        self._cameras = [dict(c) for c in cfg["cameras"]]
        self._percept_cams = list(cfg["percept"]["cams"])
        self._percept_scale = float(cfg["percept"]["scale"])
        self._percept_on_advance = bool(cfg["percept"]["on_advance"])
        self._truth_step = float(cfg["route_truth"]["step_m"])
        self._truth_reach = float(cfg["route_truth"]["reach_m"])
        self._require_why = bool(cfg["require_justification"])
        self._vla_enabled = bool(cfg["vla"]["enabled"])
        self._vla_addr = str(cfg["vla"]["addr"])
        self._vla_cam = str(cfg["vla"]["camera"])
        self._vla_every = int(cfg["vla"]["every_n_ticks"])
        self._vla_control = str(cfg["vla"]["control"])
        self._vla_tp_at = [float(d) for d in cfg["vla"]["target_points_at_m"]]
        self._vla_max_ticks = int(cfg["vla"]["max_ticks"])
        assert self._vla_control in ("simlingo", "probe_pid"), f"vla.control must be simlingo|probe_pid, got {self._vla_control!r}"
        assert self._vla_every >= 1 and len(self._vla_tp_at) == 2
        assert any(c["id"] == self._vla_cam for c in self._cameras), f"vla.camera {self._vla_cam!r} is not in cameras"
        self.pid = PIDController(cfg["pid"])
        # The route's tick budget, for `status.ticks_left` only. AGENT_MAX_TICKS is the analysis
        # override the leaderboard itself reads (docs/DESIGN.md); PROBE_TICK_BUDGET is the stock
        # budget of the benchmark being run, which run_session.sh knows and the agent does not
        # (Bench2Drive caps hard at 4000, Fail2Drive has NO tick cap). Neither set -> no budget,
        # and `ticks_left` is null rather than a 4000 that does not exist.
        _budget = os.environ.get("AGENT_MAX_TICKS") or os.environ.get("PROBE_TICK_BUDGET") or ""
        self._max_ticks = int(_budget) if _budget else None

        self.step = -1
        self._sim_time = 0.0
        self._frame = -1
        self._decision_i = 0
        self._pending_decision: Optional[int] = None
        self._logged_events: "Counter[Tuple[Any, ...]]" = Counter()
        self._ego: Optional[Dict[str, float]] = None
        self._last_input: Optional[dict] = None
        self._mode = "hold"
        self._plan_world: Optional[np.ndarray] = None
        # dead_reckon: the pose the plan is tracked from, seeded from the believed pose at the
        # re-plan and carried by speed + compass alone. None whenever `gnss` tracking is in use.
        self._track_pose: Optional[Dict[str, float]] = None
        self._plan_speed: Optional[float] = None
        self._plan_done = False
        self._raw = {"steer": 0.0, "throttle": 0.0, "brake": 1.0}
        self._last_control = {"steer": 0.0, "throttle": 0.0, "brake": 1.0}
        self._vla: Optional[Dict[str, Any]] = None       # the running `drive`: text, mode, last reply
        self._vla_ready = False                          # /reset sent once per route
        self._advance: Optional[Dict[str, Any]] = None
        self._collisions: List[Dict[str, Any]] = []
        self._collision_sensor = None
        self._stopped_run = 0
        self._scenario = None
        self._route: Optional[np.ndarray] = None
        self._route_options: List[str] = []
        self._route_cum: Optional[np.ndarray] = None
        self._route_idx, self._route_s, self._route_lat = 0, 0.0, 0.0
        self._lat_ref, self._lon_ref = 0.0, 0.0
        self._last_t: Optional[float] = None
        if getattr(self, "_dense_world_plan", None) is not None:
            self._build_route()

        self._cmd_log = open(self._out / "commands.jsonl", "a")
        self._traj_log = open(self._out / "trajectory.jsonl", "a") if self._record_traj else None
        self._decisions_jsonl = open(self._out / "decisions.jsonl", "a")
        self._decisions_txt = open(self._out / "decisions.txt", "a")
        # Analysis only, both access levels: never read back into a reply (see _record_score_events).
        self._score_log = open(self._out / "score_events.jsonl", "a")
        # Every policy query with its full reply, for reviewing what the driver's words did.
        self._vla_log = open(self._out / "vla.jsonl", "a") if self._vla_enabled else None
        host = os.environ.get("PROBE_HOST", str(cfg["host"]))
        port = int(os.environ.get("PROBE_PORT", cfg["port"]))
        self._server = CommandServer(host, port)
        print(f"[PROBE] session ready (access={self._access}): the sim pauses on the first tick until a "
              "command arrives", flush=True)

    def sensors(self) -> list:
        specs = [
            {"type": "sensor.camera.rgb", "id": c["id"], "x": c["x"], "y": c["y"], "z": c["z"],
             "roll": c["roll"], "pitch": c["pitch"], "yaw": c["yaw"],
             "width": c["width"], "height": c["height"], "fov": c["fov"]}
            for c in self._cameras
        ]
        specs += [
            {"type": "sensor.other.imu", "id": "IMU", "x": 0.0, "y": 0.0, "z": 0.0,
             "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "sensor_tick": 0.05},
            {"type": "sensor.other.gnss", "id": "GPS", "x": 0.0, "y": 0.0, "z": 0.0,
             "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "sensor_tick": 0.01},
            {"type": "sensor.speedometer", "id": "SPEED", "reading_frequency": 20},
        ]
        return specs

    def set_global_plan(self, global_plan_gps, global_plan_world_coord) -> None:
        # Both evaluators call this BEFORE setup (the access level is not known yet), so the
        # route is built by whichever of the two runs last.
        super().set_global_plan(global_plan_gps, global_plan_world_coord)
        self._dense_world_plan = global_plan_world_coord
        if hasattr(self, "_access"):
            self._build_route()

    def _build_route(self) -> None:
        pts: List[Tuple[float, float]] = []
        opts: List[str] = []
        if self._sensors_only:
            # Only what a SENSORS-track agent is given: the base class's downsampled GPS plan --
            # a point at most every `sample_factor` metres, which is the benchmark's own choice
            # (Bench2Drive 50 m, Fail2Drive 200 m: see docs/DESIGN.md). The
            # town geo reference comes from its first point, which the leaderboard provides in both
            # GPS and world form; the route is then projected through the SAME conversion as the
            # ego's GNSS so both live in one frame.
            first_loc = self._global_plan_world_coord[0][0].location
            first_gps = self._global_plan[0][0]
            self._lat_ref, self._lon_ref = geo.solve_geo_reference(first_loc.x, first_loc.y, first_gps["lat"], first_gps["lon"])
            source = [(geo.gps_to_world(g["lat"], g["lon"], self._lat_ref, self._lon_ref), o) for g, o in self._global_plan]
        else:
            # Privileged: the DENSE world-coordinate route, identical on both benchmarks (only the
            # base class's own copy is downsampled).
            source = [((float(t.location.x), float(t.location.y)), o) for t, o in self._dense_world_plan]
        for p, option in source:
            if pts and math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) < 0.05:
                continue
            pts.append((float(p[0]), float(p[1])))
            opts.append(getattr(option, "name", str(option)))
        if len(pts) >= 2:
            self._route = np.asarray(pts, dtype=np.float64)
            self._route_options = opts
            self._route_cum = geo.cumulative_arclength(self._route)

    def set_scenario(self, scenario) -> None:
        """Called by the patched evaluator before the first tick: enables the live score."""
        self._scenario = scenario

    def destroy(self, results=None) -> None:
        # Guarded: may run on a partially constructed agent if setup failed.
        server = getattr(self, "_server", None)
        if getattr(self, "_advance", None) is not None and server is not None:
            self._finish_advance("route_end")
        try:
            if getattr(self, "_score_log", None) is not None:
                self._record_score_events()  # events the criteria added while terminating
        except Exception as e:
            print(f"[PROBE] could not record final score events: {e!r}", flush=True)
        try:
            done = {"t_wall": time.time(), "status": self._status() if getattr(self, "_ego", None) else None,
                    "record": results.to_json() if hasattr(results, "to_json") else None}
            (self._out / "probe_done.json").write_text(json.dumps(done, indent=2, default=str))
        except Exception as e:  # never mask the evaluator's own teardown
            print(f"[PROBE] could not write probe_done.json: {e!r}", flush=True)
        sensor = getattr(self, "_collision_sensor", None)
        if sensor is not None and sensor.is_alive:  # the evaluator's cleanup may have destroyed it already
            try:
                sensor.stop()
                sensor.destroy()
            except RuntimeError:
                pass
            self._collision_sensor = None
        for f in (getattr(self, "_cmd_log", None), getattr(self, "_traj_log", None), getattr(self, "_decisions_jsonl", None),
                  getattr(self, "_decisions_txt", None), getattr(self, "_score_log", None), getattr(self, "_vla_log", None)):
            if f is not None:
                f.close()
        if server is not None:
            server.close()

    # ------------------------------------------------------------------ per tick
    def run_step(self, input_data: dict, timestamp) -> "carla.VehicleControl":
        self.step += 1
        self._sim_time = float(timestamp)
        self._last_input = input_data
        self._frame = int(input_data["SPEED"][0])  # the CARLA frame this tick's sensor data belongs to
        speed = float(input_data["SPEED"][1]["speed"])
        self._record_score_events()
        if self._sensors_only:
            self._ego = self._estimate_pose(input_data, speed)
        else:
            # getattr, not attribute access: Bench2Drive's base class resolves hero_actor in
            # __init__ so it always exists, Fail2Drive's never sets it at all.
            if getattr(self, "hero_actor", None) is None:
                self.get_hero()
            tf = self.hero_actor.get_transform()
            self._ego = {"x": float(tf.location.x), "y": float(tf.location.y), "yaw": float(tf.rotation.yaw), "speed": speed}
            self._ensure_collision_sensor()
            self._check_ego_body()
        self._advance_track_pose(speed)
        self._last_t = self._sim_time
        self._update_route_progress()
        self._stopped_run = self._stopped_run + 1 if speed < self._stopped_speed else 0

        if self._advance is not None:
            self._advance["ticks"] += 1
            reason = self._advance_reason()
            if reason is not None:
                self._finish_advance(reason)

        # Paused: serve commands until one advances time. THIS is the daemon.
        if self._advance is None and self._traj_log is not None:
            self._traj_log.flush()
        while self._advance is None:
            cmd = self._server.recv()
            try:
                reply = self._dispatch(cmd)
            except _ShutdownRequested:
                self._log_decision(cmd, {"reason": "shutdown"})
                self._log({**cmd, "advancing": False, "ok": True})
                raise RuntimeError("probe shutdown requested by client (route scored as an agent failure)")
            except Exception:
                reply = {"ok": False, "error": traceback.format_exc(limit=5)}
            if cmd.get("cmd") in _DRIVING_COMMANDS and (reply is None or reply.get("ok")):
                self._log_decision(cmd, None if reply is None else reply.get("result", {"mode": self._mode}))
            self._log({**cmd, "advancing": reply is None, "ok": None if reply is None else bool(reply.get("ok"))})
            if reply is not None:
                reply.setdefault("status", self._status())
                self._server.send(reply)

        control = self._compute_control()
        self._last_control = {"steer": _r(control.steer, 3), "throttle": _r(control.throttle, 3), "brake": _r(control.brake, 3)}
        self._record_tick()
        return control

    def _estimate_pose(self, input_data: dict, speed: float) -> Dict[str, float]:
        """Ego pose from GNSS + IMU compass only (sensors access), in the CARLA world frame."""
        lat, lon = float(input_data["GPS"][1][0]), float(input_data["GPS"][1][1])
        meas = geo.gps_to_world(lat, lon, self._lat_ref, self._lon_ref)
        compass = float(input_data["IMU"][1][-1])
        prev = self._ego
        yaw = prev["yaw"] if (math.isnan(compass) and prev) else geo.compass_to_yaw_deg(0.0 if math.isnan(compass) else compass)
        dt = 0.0 if self._last_t is None else self._sim_time - self._last_t
        if self._gnss_filter in ("dr_offset", "dr_trend"):
            x, y = self._dr_anchor_estimate(meas, speed, yaw, dt)
        else:
            x, y = geo.predict_blend(None if prev is None else (prev["x"], prev["y"]),
                                     speed, yaw, dt, meas, self._gnss_gain)
        return {"x": x, "y": y, "yaw": yaw, "speed": speed}

    def _dr_anchor_estimate(self, meas: Tuple[float, float], speed: float, yaw: float,
                            dt: float) -> Tuple[float, float]:
        """Dead-reckon the path, and use GNSS only for the one thing dead reckoning cannot supply.

        The leaderboard adds noise to the GNSS but not to the compass or the speedometer, so
        integrating those two reproduces the true path's SHAPE essentially exactly. All GNSS has to
        supply is where that shape sits -- two numbers for the whole run, not two numbers every
        tick. Fitting that offset over a window therefore drives the position error down as
        1/sqrt(N), instead of the complementary filter's sqrt(gain/(2-gain)) floor: at 20 Hz a 5 s
        window is 100 samples, so ~0.56 m per-sample noise becomes ~6 cm.

        Dead reckoning is not drift-free, though, which is what the two modes differ over. The
        speedometer reports the FORWARD component of the velocity, so along-track integration is
        exact; what it cannot see is the sideways component, and in a turn with a slip angle the
        dead-reckoned path slides laterally at v*sin(beta).

          dr_offset  fit a constant over the window. Cheapest, lowest noise (sigma/sqrt(N)) -- but a
                     steady drift is only ever half-corrected, leaving a lag of about rate*window/2.
          dr_trend   fit a line and read it at the current sample, so a steady drift is removed
                     rather than lagged. The endpoint of a line fit costs 2x the noise of its mean,
                     which a longer window pays back.
        """
        if self._dr_xy is None:
            self._dr_xy = (0.0, 0.0)
        else:
            r = math.radians(yaw)
            self._dr_xy = (self._dr_xy[0] + speed * dt * math.cos(r),
                           self._dr_xy[1] + speed * dt * math.sin(r))
        self._gnss_window.append((meas[0] - self._dr_xy[0], meas[1] - self._dr_xy[1]))
        res = np.asarray(self._gnss_window, dtype=np.float64)   # (n, 2) GNSS minus dead reckoning
        n = len(res)
        if self._gnss_filter == "dr_trend" and n >= 4:
            tc = np.arange(n, dtype=np.float64)
            tc -= tc.mean()
            off = res.mean(axis=0) + (tc @ res) / float(tc @ tc) * tc[-1]
        else:
            off = res.mean(axis=0)
        return self._dr_xy[0] + float(off[0]), self._dr_xy[1] + float(off[1])

    def _advance_track_pose(self, speed: float) -> None:
        """Carry the plan-tracking pose forward by one tick (``plan_tracking: dead_reckon``).

        Tracking a plan means converting the driver's ego-frame waypoints to world with the pose
        believed when the plan was issued, then back to the ego frame with the pose believed now.
        Under `gnss` both poses are GNSS-blended estimates, so the estimator's wander between those
        two instants lands in the controller as tracked lateral error -- and because the loop closes
        on the estimate, the car steers that wander into the road. Dead reckoning carries the anchor
        forward with the speedometer and the compass instead, neither of which the leaderboard adds
        noise to, so nothing GNSS-borne reaches the controller within a plan. GNSS still fixes the
        anchor at every re-plan, so this cannot drift: it is strictly less information, not more.
        """
        if self._track_pose is None or self._ego is None:
            return
        dt = 0.0 if self._last_t is None else self._sim_time - self._last_t
        yaw = math.radians(self._ego["yaw"])
        self._track_pose = {"x": self._track_pose["x"] + speed * dt * math.cos(yaw),
                            "y": self._track_pose["y"] + speed * dt * math.sin(yaw),
                            "yaw": self._ego["yaw"]}

    # ------------------------------------------------------------------ advancing
    def _start_advance(self, kind: str, cmd: Dict[str, Any], **extra: Any) -> None:
        self._advance = {
            "kind": kind,
            "ticks": 0,
            "coll0": len(self._collisions),
            "inf0": Counter(_event_key(ev) for ev in self._infraction_events()),
            "stop_on_collision": bool(cmd.get("stop_on_collision", False)),
            # percept.on_advance decides the default; percept=true/false on the command overrides.
            "percept": dict(cmd) if cmd.get("percept", self._percept_on_advance) else None,
            **extra,
        }

    def _advance_reason(self) -> Optional[str]:
        a = self._advance
        assert a is not None
        if a["stop_on_collision"] and len(self._collisions) > a["coll0"]:
            return "collision"
        if a["kind"] == "ticks":
            return "ticks" if a["ticks"] >= a["n"] else None
        if a["kind"] == "frame":
            return "frame" if self.step >= a["target"] else None
        if a.get("max_ticks") and a["ticks"] >= a["max_ticks"]:
            return "max_ticks"
        event = a["event"]
        if event == "collision" and len(self._collisions) > a["coll0"]:
            return "collision"
        if event == "stopped" and self._stopped_run >= self._stopped_min_ticks:
            return "stopped"
        if event == "infraction" and len(self._infraction_events()) > sum(a["inf0"].values()):
            return "infraction"
        if event == "plan_done" and (self._mode != "plan" or self._plan_done):
            return "plan_done"
        return None

    def _finish_advance(self, reason: str) -> None:
        a = self._advance
        assert a is not None
        result: Dict[str, Any] = {"reason": reason, "ticks_advanced": a["ticks"]}
        if self._mode == "vla":
            result["vla"] = self._vla_summary()
        if not self._sensors_only:
            score = self._live_score(detail=True)
            result["collisions_during"] = self._collisions[a["coll0"]:]
            result["new_infractions"] = _new_events(score["events"] if score else [], a["inf0"])
        reply: Dict[str, Any] = {"ok": True, "result": result, "status": self._status()}
        # Not on route_end: that call comes from `destroy`, mid-teardown, where a privileged
        # `_observe` would touch hero_actor / the map / level bounding boxes the evaluator may
        # already have torn down -- and the percept would be useless anyway, the route is over.
        if a["percept"] is not None and reason != "route_end":
            reply.update(self._percept(a["percept"]))
        self._server.send(reply)
        if self._pending_decision is not None:
            self._write_decision_result(self._pending_decision, {"reason": reason, "ticks_advanced": a["ticks"]})
            self._pending_decision = None
        self._log({"event": "advance_done", "reason": reason, "ticks": a["ticks"]})
        self._advance = None

    # ------------------------------------------------------------------ dispatch
    def _denied(self, name: str, cmd: Dict[str, Any]) -> Optional[str]:
        """What a sensors-only session may not request. Checked BEFORE a command changes any state."""
        if not self._sensors_only:
            return None
        if name == "get_score":
            return "get_score"
        if cmd.get("stop_on_collision"):
            return "stop_on_collision"
        if name == "run_until_event" and cmd.get("event") in _PRIVILEGED_EVENTS:
            return f"event={cmd['event']}"
        return None

    def _dispatch(self, cmd: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Execute one command. Returns the reply, or None when the reply is deferred to the tick
        where the sim pauses again (time-advancing commands)."""
        name = str(cmd["cmd"])
        denied = self._denied(name, cmd)
        if denied is not None:
            return {"ok": False, "error": f"{denied} needs privileged access; this session is access=sensors"}
        if name in _DRIVING_COMMANDS and self._require_why and not str(cmd.get("why") or "").strip():
            return {"ok": False, "error": f'{name} needs why="...": what you observed and why this command follows '
                                          "from it (every driving command is logged with its justification)"}
        if name == "status":
            return {"ok": True}
        if name == "help":
            return {"ok": True, "access": self._access, "commands": HELP,
                    "privileged_only": "items marked * and the stop_on_collision flag",
                    "advance_events": [e for e in _ADVANCE_EVENTS if not (self._sensors_only and e in _PRIVILEGED_EVENTS)]}
        if name == "tick":
            n = int(cmd["n"])
            assert n >= 1, "tick: n must be >= 1"
            self._start_advance("ticks", cmd, n=n)
            return None
        if name == "run_until_frame":
            target = int(cmd["frame"])
            if self.step >= target:
                return {"ok": True, "result": {"reason": "already_at_frame"}}
            self._start_advance("frame", cmd, target=target)
            return None
        if name == "run_until_event":
            event = str(cmd["event"])
            assert event in _ADVANCE_EVENTS, f"event must be one of {_ADVANCE_EVENTS}"
            assert not (event == "collision" and not self._want_collision_sensor), "collision_sensor is off in config"
            max_ticks = cmd.get("max_ticks")
            self._start_advance("event", cmd, event=event, max_ticks=int(max_ticks) if max_ticks else None)
            return None
        if name == "get_state":
            return {"ok": True, "state": self._sensor_state() if self._sensors_only else self._extract_state()}
        if name == "get_score":
            return {"ok": True, "score": self._live_score(detail=True)}
        if name == "percept":
            return {"ok": True, **self._percept(cmd)}
        if name == "clearance":
            return {"ok": True, "clearance": self._clearance(cmd)}
        if name == "render_bev":
            return {"ok": True, "file": self._render_bev()}
        if name == "dump_frame":
            return {"ok": True, "dump": self._dump_frame()}
        if name == "apply_plan":
            wps = np.asarray(cmd["waypoints"], dtype=np.float64)
            assert wps.ndim == 2 and wps.shape[1] == 2 and len(wps) >= 2, "waypoints must be [[fwd,left], ...] with >= 2 points"
            ticks = int(cmd["ticks"])
            assert ticks >= 1, "apply_plan: ticks must be >= 1"
            e = self._ego
            assert e is not None
            self._plan_world = geo.ego_to_world((e["x"], e["y"]), e["yaw"], wps)
            self._track_pose = {"x": e["x"], "y": e["y"], "yaw": e["yaw"]} if self._dead_reckon_plan else None
            ts = cmd.get("target_speed")
            self._plan_speed = None if ts is None else float(ts)
            self._plan_done = False
            self._mode = "plan"
            self._start_advance("ticks", cmd, n=ticks)
            return None
        if name == "drive":
            assert self._vla_enabled, "drive: the learned policy is off in this session (config vla.enabled)"
            mode = str(cmd.get("mode", "commentary"))
            assert mode in _VLA_MODES, f"drive: mode must be one of {_VLA_MODES}"
            text = str(cmd.get("text") or "").strip()
            assert text or mode in ("plain", "cot"), f"drive: mode={mode} needs text"
            ticks = int(cmd["ticks"])
            assert 1 <= ticks <= self._vla_max_ticks, f"drive: ticks must be 1..{self._vla_max_ticks} (config vla.max_ticks)"
            if not self._vla_ready:
                self._vla_post("/reset", {})   # fails loudly here, before anything moves
                self._vla_ready = True
            self._vla = {"text": text, "mode": mode, "last": None, "queries": 0, "ms": 0.0}
            self._mode, self._plan_world, self._plan_speed, self._plan_done = "vla", None, None, False
            self._start_advance("ticks", cmd, n=ticks)
            return None
        if name == "set_control":
            raw = {k: float(cmd[k]) for k in ("steer", "throttle", "brake")}
            ticks = int(cmd["ticks"])
            assert ticks >= 1, "set_control: ticks must be >= 1"
            self._raw, self._mode = raw, "raw"
            self._start_advance("ticks", cmd, n=ticks)
            return None
        if name == "hold":
            self._mode, self._plan_world, self._plan_speed = "hold", None, None
            self._track_pose = None
            if cmd.get("ticks"):
                self._start_advance("ticks", cmd, n=int(cmd["ticks"]))
                return None
            return {"ok": True}
        if name == "record":
            on = bool(cmd["on"])
            self._record_every = max(1, int(cmd.get("every", self._record_every or 2))) if on else 0
            return {"ok": True, "camera_every_n_ticks": self._record_every, "camera_id": self._record_cam_id}
        if name == "shutdown":
            self._server.send({"ok": True, "result": {"reason": "shutdown"}, "status": self._status()})
            raise _ShutdownRequested()
        return {"ok": False, "error": f"unknown command {name!r} (try 'help')"}

    # ------------------------------------------------------------------ control
    def _compute_control(self) -> "carla.VehicleControl":
        control = carla.VehicleControl()
        e = self._ego
        assert e is not None
        if self._mode == "raw":
            control.steer = float(np.clip(self._raw["steer"], -1.0, 1.0))
            control.throttle = float(np.clip(self._raw["throttle"], 0.0, 1.0))
            control.brake = float(np.clip(self._raw["brake"], 0.0, 1.0))
            return control
        if self._mode == "vla":
            return self._vla_control_tick(control)
        if self._mode != "plan" or self._plan_world is None:
            control.brake = 1.0
            return control

        p = self._track_pose or e
        plan_ego = geo.world_to_ego((p["x"], p["y"]), p["yaw"], self._plan_world)
        last = plan_ego[-1]
        if last[0] < 0.0 or float(np.hypot(last[0], last[1])) < 1.5:
            self._plan_done = True
        if self._plan_done:
            control.brake = 1.0
            return control
        # Drop the driven prefix: points behind the ego have unsigned norms and would hijack the aim.
        i0 = min(int(np.argmin(np.linalg.norm(plan_ego, axis=1))), len(plan_ego) - 2)
        steer, throttle, brake, _meta = self.pid.control(plan_ego[i0:], e["speed"], self._plan_speed, self._plan_interval_s)
        control.steer, control.throttle, control.brake = steer, throttle, brake
        return control

    # ------------------------------------------------------------------ learned policy (`drive`)
    def _vla_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        import urllib.request
        req = urllib.request.Request(f"http://{self._vla_addr}{path}", data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                reply = json.loads(r.read())
        except Exception as e:
            raise RuntimeError(f"vla service at {self._vla_addr} failed on {path}: {e!r} "
                               "(is scripts/run_vla_server.sh running?)") from e
        assert reply.get("ok"), f"vla service error on {path}: {reply.get('error')}"
        return reply

    def _vla_query(self) -> Dict[str, Any]:
        """One policy step from what BOTH tracks may see: the vla camera, the speedometer and two
        points of the route plan (the same plan `nav.route_points_ego` comes from). Nothing here
        touches hero_actor, the world or the map, so it needs no access branch."""
        assert self._last_input is not None and self._ego is not None and self._route is not None
        v = self._vla
        assert v is not None
        img = np.ascontiguousarray(self._last_input[self._vla_cam][1][:, :, :3])
        ok, jpg = cv2.imencode(".jpg", img)
        assert ok
        e = self._ego
        pts = geo.route_points_ahead(self._route, self._route_s, self._vla_tp_at)
        tps = geo.world_to_ego((e["x"], e["y"]), e["yaw"], pts)
        import base64
        t0 = time.time()
        reply = self._vla_post("/act", {
            "image_jpeg_b64": base64.b64encode(jpg.tobytes()).decode(), "speed_mps": float(e["speed"]),
            "target_points": [[float(p[0]), float(p[1])] for p in tps], "text": v["text"], "mode": v["mode"]})
        v["queries"] += 1
        v["ms"] += (time.time() - t0) * 1000.0
        if self._vla_log is not None:
            self._vla_log.write(json.dumps({"step": self.step, "text": v["text"], "mode": v["mode"],
                                            "speed_mps": _r(e["speed"]), "target_points": [[_r(p[0]), _r(p[1])] for p in tps],
                                            **{k: reply[k] for k in ("route", "speed_wps", "desired_speed_mps", "language", "control", "ms")}}) + "\n")
            self._vla_log.flush()
        return reply

    def _vla_control_tick(self, control: "carla.VehicleControl") -> "carla.VehicleControl":
        v = self._vla
        assert v is not None
        a = self._advance
        # Query on the first tick of the advance and every every_n_ticks after; hold in between.
        if v["last"] is None or (a is not None and a["ticks"] % self._vla_every == 0):
            v["last"] = self._vla_query()
            if self._vla_control == "probe_pid":
                e = self._ego
                self._plan_world = geo.ego_to_world((e["x"], e["y"]), e["yaw"], np.asarray(v["last"]["route"], dtype=np.float64))
                self._plan_speed = float(v["last"]["desired_speed_mps"])
        if self._vla_control == "simlingo":
            c = v["last"]["control"]
            control.steer, control.throttle, control.brake = float(c["steer"]), float(c["throttle"]), float(c["brake"])
            return control
        e = self._ego
        plan_ego = geo.world_to_ego((e["x"], e["y"]), e["yaw"], self._plan_world)
        i0 = min(int(np.argmin(np.linalg.norm(plan_ego, axis=1))), len(plan_ego) - 2)
        steer, throttle, brake, _meta = self.pid.control(plan_ego[i0:], e["speed"], self._plan_speed, self._plan_interval_s)
        control.steer, control.throttle, control.brake = steer, throttle, brake
        return control

    def _vla_summary(self) -> Optional[Dict[str, Any]]:
        v = self._vla
        if v is None or v["last"] is None:
            return None
        last = v["last"]
        return {"text": v["text"], "mode": v["mode"], "language": last["language"],
                "desired_speed_mps": last["desired_speed_mps"], "queries": v["queries"],
                "ms_per_query": _r(v["ms"] / max(v["queries"], 1), 0)}

    # ------------------------------------------------------------------ route + score
    def _update_route_progress(self) -> None:
        if self._route is None or self._ego is None:
            return
        self._route_idx, self._route_s, self._route_lat = geo.project_on_route(
            self._route, (self._ego["x"], self._ego["y"]), hint_idx=self._route_idx, window=self._route_window
        )

    def _route_ahead_distance(self, xy: Tuple[float, float], yaw_deg: float, max_offset_m: float) -> Optional[float]:
        """Arclength from the ego to ``xy`` along the route, if ``xy`` lies on the route ahead within
        ``max_offset_m`` (true distance, so points beyond either route end don't count) and its lane
        heading ``yaw_deg`` runs the same way as the route."""
        if self._route is None:
            return None
        seg, s, _lat = geo.project_on_route(self._route, xy, hint_idx=self._route_idx, window=self._route_window)
        ahead = s - self._route_s
        on_route = geo.point_at_arclength(self._route, s)
        if math.hypot(on_route[0] - xy[0], on_route[1] - xy[1]) > max_offset_m or ahead < -1.0:
            return None
        if abs(geo.wrap_deg(yaw_deg - geo.route_heading_deg(self._route, seg))) > 90.0:
            return None
        return _r(ahead, 1)

    def _route_stop_ahead(self, center: Tuple[float, float], extent: Tuple[float, float]) -> Optional[float]:
        """Arclength from the ego to where the route enters a stop sign's trigger box, the way
        RunningStopTest decides a stop applies (world axis-aligned box, extent x1.2)."""
        if self._route is None:
            return None
        half = (_STOP_BOX_MULTIPLIER * extent[0], _STOP_BOX_MULTIPLIER * extent[1])
        s = geo.route_box_entry_s(self._route, center, half, self._route_s - 1.0,
                                  hint_idx=self._route_idx, window=self._route_window)
        return None if s is None else _r(max(s - self._route_s, 0.0), 1)

    def _scenario_events(self) -> Optional[Tuple[float, float, List[Dict[str, Any]]]]:
        """(route_completed, penalty, events) from the running scenario's criteria, mirroring the
        leaderboard's compute_route_statistics; None before set_scenario. Access-agnostic ON PURPOSE:
        in sensors sessions its output may only go to the write-only score_events.jsonl."""
        if self._scenario is None:
            return None
        try:
            from leaderboard.utils.statistics_manager import PENALTY_NAME_DICT, PENALTY_PERC_DICT, PENALTY_VALUE_DICT
            from srunner.scenariomanager.traffic_events import TrafficEventType
        except ImportError:
            return None
        penalty, route_completed = 1.0, 0.0
        events: List[Dict[str, Any]] = []
        for node in self._scenario.get_criteria():
            for ev in node.events:
                etype = ev.get_type()
                data = ev.get_dict() or {}
                if etype == TrafficEventType.ROUTE_COMPLETION:
                    route_completed = float(data.get("route_completed", 0.0))
                    continue
                if etype in PENALTY_VALUE_DICT:
                    penalty *= PENALTY_VALUE_DICT[etype]
                elif etype in PENALTY_PERC_DICT:
                    value, kind = PENALTY_PERC_DICT[etype]
                    pct = float(data.get("percentage", 0.0))
                    if kind == "decreases":
                        penalty *= 1 - (1 - value) * (1 - pct / 100)
                    elif kind == "increases":
                        penalty *= 1 - (1 - value) * pct / 100
                events.append({"type": PENALTY_NAME_DICT.get(etype, getattr(etype, "name", str(etype))),
                               "frame": ev.get_frame(), "message": ev.get_message()})
        return route_completed, penalty, events

    def _live_score(self, detail: bool) -> Optional[Dict[str, Any]]:
        """The live score estimate for replies; result.json (written at route end) is authoritative.
        Never available with sensors access."""
        if self._sensors_only:
            return None
        scenario_events = self._scenario_events()
        if scenario_events is None:
            return None
        route_completed, penalty, events = scenario_events
        counts: Dict[str, int] = {}
        for ev in events:
            counts[ev["type"]] = counts.get(ev["type"], 0) + 1
        out: Dict[str, Any] = {
            "driving_score": _r(max(route_completed * penalty, 0.0), 2),
            "route_completion": _r(route_completed, 2),
            "infraction_penalty": _r(penalty, 4),
            "infractions": counts,
        }
        if detail:
            out["events"] = events
        return out

    def _record_score_events(self) -> None:
        """Append criteria events that appeared since the last tick to score_events.jsonl, with the
        step and CARLA frame at which the daemon saw them (the event's own ``frame`` is exact).
        Both access levels; write-only, never read back into a reply."""
        scenario_events = self._scenario_events()
        if scenario_events is None:
            return
        for ev in _new_events(scenario_events[2], self._logged_events):
            self._logged_events[_event_key(ev)] += 1
            self._score_log.write(json.dumps({"step": self.step, "t": _r(self._sim_time, 2), "frame_seen": self._frame,
                                              **ev}, default=str) + "\n")
        self._score_log.flush()

    def _log_decision(self, cmd: Dict[str, Any], result: Optional[Dict[str, Any]]) -> None:
        """Log an accepted driving command with its justification; ``result`` None = pending advance."""
        self._decision_i += 1
        i = self._decision_i
        name = str(cmd.get("cmd"))
        args = {k: v for k, v in cmd.items() if k not in ("cmd", "why")}
        why = str(cmd.get("why") or "").strip()
        self._decisions_jsonl.write(json.dumps({"i": i, "step": self.step, "t": _r(self._sim_time, 2), "frame": self._frame,
                                                "cmd": name, "args": args, "why": why}) + "\n")
        arg_text = " ".join(f"{k}={json.dumps(v)}" for k, v in args.items())
        self._decisions_txt.write(f"[{i}] step {self.step} (t={self._sim_time:.2f} s, frame {self._frame}): {name} {arg_text}\n"
                                  f"    why: {why or '(none given)'}\n")
        if result is None:
            self._pending_decision = i
        else:
            self._write_decision_result(i, result)
        self._decisions_jsonl.flush()
        self._decisions_txt.flush()

    def _write_decision_result(self, i: int, result: Dict[str, Any]) -> None:
        e = self._ego or {}
        self._decisions_jsonl.write(json.dumps({"i": i, "result": result, "step": self.step, "t": _r(self._sim_time, 2),
                                                "frame": self._frame, "speed_mps": _r(e.get("speed", 0.0))}) + "\n")
        outcome = ", ".join(f"{k}={v}" for k, v in result.items())
        self._decisions_txt.write(f"    -> {outcome}; now step {self.step}, speed {e.get('speed', 0.0):.2f} m/s\n")
        self._decisions_jsonl.flush()
        self._decisions_txt.flush()

    def _infraction_events(self) -> List[Dict[str, Any]]:
        score = self._live_score(detail=True)
        return score["events"] if score else []

    # ------------------------------------------------------------------ inspection (both levels)
    def _status(self) -> Dict[str, Any]:
        e = self._ego or {}
        route = None
        if self._route_cum is not None:
            total = float(self._route_cum[-1])
            route = {
                "progress_pct": _r(100.0 * self._route_s / max(total, 1e-6), 1),
                "distance_remaining_m": _r(total - self._route_s, 1),
                "lateral_offset_m": _r(self._route_lat, 2),
            }
        status: Dict[str, Any] = {
            "step": self.step,
            "sim_time": _r(self._sim_time, 2),
            "ticks_left": None if self._max_ticks is None else self._max_ticks - max(self.step, 0),
            "access": self._access,
            "mode": self._mode,
            "plan_done": self._plan_done if self._mode == "plan" else None,
            "speed_mps": _r(e.get("speed", 0.0), 2),
            "control": self._last_control,
            "route": route,
            "stopped_run_ticks": self._stopped_run,
        }
        if self._mode == "vla":
            status["vla"] = self._vla_summary()
        if not self._sensors_only:
            hero = self.hero_actor
            status["speed_limit_mps"] = _r(hero.get_speed_limit() / 3.6, 1) if hero is not None else None
            status["collision_contacts"] = len(self._collisions)
            status["score"] = self._live_score(detail=False)
        return status

    def _ego_frame(self, x: float, y: float) -> Tuple[float, float]:
        e = self._ego
        assert e is not None
        p = geo.world_to_ego((e["x"], e["y"]), e["yaw"], np.array([[x, y]]))[0]
        return float(p[0]), float(p[1])

    def _yaw_rel_ccw(self, yaw_deg: float) -> float:
        """World yaw -> heading relative to the ego, degrees, positive toward the ego's LEFT."""
        return geo.wrap_deg(-(yaw_deg - self._ego["yaw"]))

    def _nav(self) -> Optional[Dict[str, Any]]:
        if self._route is None or self._route_cum is None:
            return None
        e = self._ego
        pts = geo.route_points_ahead(self._route, self._route_s, self._lookahead)
        pts_ego = geo.world_to_ego((e["x"], e["y"]), e["yaw"], pts)
        next_cmd = None
        for i in range(self._route_idx, len(self._route_options)):
            ahead = float(self._route_cum[i]) - self._route_s
            if ahead > 120.0:
                break
            if ahead >= 0.0 and self._route_options[i] not in _LANE_OPTIONS:
                next_cmd = {"command": self._route_options[i], "distance_m": _r(ahead, 1)}
                break
        total = float(self._route_cum[-1])
        return {
            "route_points_ego": [[_r(p[0], 1), _r(p[1], 1)] for p in pts_ego],
            "route_points_at_m": self._lookahead,
            "next_command": next_cmd,
            "progress_pct": _r(100.0 * self._route_s / max(total, 1e-6), 1),
            "distance_remaining_m": _r(total - self._route_s, 1),
            "lateral_offset_m": _r(self._route_lat, 2),
        }

    def _write_cameras(self, out_dir: Path, cams: Optional[List[str]], scale: float) -> List[str]:
        assert self._last_input is not None
        ids = [c["id"] for c in self._cameras]
        cams = list(cams) if cams else ids
        if isinstance(cams, str):
            cams = [cams]
        out_dir.mkdir(parents=True, exist_ok=True)
        files = []
        for cam in cams:
            assert cam in ids, f"unknown camera {cam!r} (have {ids})"
            img = np.ascontiguousarray(self._last_input[cam][1][:, :, :3])
            if scale != 1.0:
                img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            path = out_dir / f"{cam}.jpg"
            cv2.imwrite(str(path), img)
            files.append(str(path.resolve()))
        return files

    # ------------------------------------------------------------------ the ego's own body
    def _check_ego_body(self) -> None:
        """Once per privileged run, confirm config's `ego:` is the car actually spawned.

        The footprint is now quoted to the driver and used for clearance arithmetic, so a stale
        config is worse than no config: it would be confidently wrong. Only a privileged session
        can read the real bounding box, and a sensors session inherits the guarantee because the
        leaderboard spawns the same hero at both levels. Warn rather than fail -- losing a scored
        route to a 2 cm discrepancy is the worse trade -- and never let this reach a reply.
        """
        if getattr(self, "_body_checked", False):
            return
        self._body_checked = True
        bb = self.hero_actor.bounding_box
        real = {"front_m": bb.location.x + bb.extent.x, "rear_m": bb.extent.x - bb.location.x,
                "half_width_m": bb.extent.y}
        off = {k: round(real[k] - self._ego_body[k], 3) for k in real}
        if max(abs(v) for v in off.values()) > 0.05:
            msg = (f"config ego: {self._ego_body} does not match the spawned "
                   f"{self.hero_actor.type_id} {({k: round(v, 3) for k, v in real.items()})} "
                   f"(off by {off}); clearances reported to the driver will be wrong by that much")
            print(f"WARNING: {msg}", flush=True)
        self._log({"event": "ego_body_check", "type_id": self.hero_actor.type_id,
                   "config": self._ego_body, "measured": {k: round(v, 4) for k, v in real.items()},
                   "off_m": off})

    def _footprint(self) -> Dict[str, Any]:
        """How big the car is and where its cameras sit. Identical at both access levels.

        This is not world state: it is the vehicle the driver is driving, known before the route
        starts, and a human driver would be told it by sitting in the car. Withholding it is what
        made every distance in a reply ambiguous -- they are all measured from the ego ORIGIN, and
        a driver that believes it is a point spends its own bodywork as clearance.
        """
        b = self._ego_body
        return {
            **{k: _r(v, 2) for k, v in b.items()},
            "corners_ego": [[_r(f, 2), _r(l, 2)] for f, l in
                            geo.footprint_corners(b["front_m"], b["rear_m"], b["half_width_m"])],
            "note": ("every ego-frame distance in a reply is to the ego ORIGIN, not to the "
                     "bodywork: subtract the extent on that side before calling it clearance"),
            "cameras": [{"id": c["id"], "x": c["x"], "y": c["y"], "yaw_deg": c["yaw"], "fov_deg": c["fov"]}
                        for c in self._cameras],
            "camera_frame": "CARLA vehicle frame: x forward, y RIGHT, yaw clockwise (ego bearing = -yaw)",
        }

    def _plan_ego(self) -> Optional[np.ndarray]:
        """The plan in flight, expressed in the CURRENT ego frame, or None if nothing is running."""
        if self._mode != "plan" or self._plan_world is None or self._ego is None:
            return None
        e = self._ego
        return geo.world_to_ego((e["x"], e["y"]), e["yaw"], self._plan_world)

    def _clearance(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Body-to-point clearance for points the DRIVER supplies, now and over a path.

        Access-safe by construction, and it must stay that way: every input is either the driver's
        own estimate of where something is or the ego's own dimensions from config. Nothing here
        reads `hero_actor`, the world, the map or the scenario, so it tells a sensors session
        nothing it did not already believe -- it only does the arithmetic correctly. Do NOT be
        tempted to snap the points to real actors on a privileged run: the two levels would then
        answer differently and the number would stop being comparable across a benchmark.
        """
        b = self._ego_body
        f, r, w = b["front_m"], b["rear_m"], b["half_width_m"]
        pts = np.asarray(cmd.get("points") or [], dtype=np.float64).reshape(-1, 2)
        given = cmd.get("waypoints")
        if given is not None:
            path, source = np.asarray(given, dtype=np.float64).reshape(-1, 2), "given"
        else:
            path, source = self._plan_ego(), "in_flight"
        if path is None or len(path) < 1:
            path, source = None, None

        out: Dict[str, Any] = {"ego_body": {k: _r(v, 2) for k, v in b.items()}, "plan": source,
                               "points": []}
        for pt in pts:
            entry: Dict[str, Any] = {"point_ego": [_r(pt[0], 2), _r(pt[1], 2)],
                                     "now_m": _r(geo.point_clearance(f, r, w, pt), 2)}
            if path is not None:
                sw = geo.sweep_clearance(f, r, w, path, pt)
                entry["along_plan"] = {"min_m": _r(sw["min_m"], 2),
                                       "at_station_m": _r(sw["at_station_m"], 1)}
            out["points"].append(entry)
        if path is not None:
            out["swept"] = {k: _r(v, 2) for k, v in geo.swept_bounds(f, r, w, path).items()}
        out["legend"] = ("clearance is from the BODYWORK: 0 is touching and negative means the "
                         "point is inside the car. `now_m` is at the current pose; `along_plan` is "
                         "the closest the body comes while following the path, which is what "
                         "catches a rear corner swinging into something the nose has passed. The "
                         "path is the one asked for, not the one the controller will track (it "
                         "lags ~0.2 m), so leave margin.")
        return out

    def _percept(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """One round trip's worth of perception: the observation plus this tick's camera images.

        Purely a composition of the existing access-correct builders -- `_sensor_observe` vs
        `_observe`, and cameras, which are Bench2Drive-eligible at both levels -- so it cannot widen
        what a sensors session sees. Keep it that way: anything added here must be routed through a
        builder that already branches on `_sensors_only`, never read off `hero_actor` or the world.
        """
        # With the learned policy on, the driver sees exactly the policy's camera and nothing else:
        # a disagreement between them must be a disagreement about the same image.
        cams = [self._vla_cam] if self._vla_enabled else cmd.get("cams", self._percept_cams)
        # Image scale is NOT a driver argument: it is the harness's cost/detail control, and a
        # driver-supplied value silently overrode whatever the config asked for, so it could not be
        # varied experimentally at all. Set it in config.yaml; a `scale` in the command is ignored.
        scale = self._percept_scale
        k = int(cmd.get("k", self._nearest_k))
        out_dir = self._out / "frames" / f"frame_{self.step:05d}"
        files = self._write_cameras(out_dir, cams, scale)
        # The directory once, then bare filenames. Absolute paths repeated per camera cost ~115
        # tokens a reply and, on a track where nothing is evicted, are re-read on every later turn;
        # this is ~45. The driver joins dir + name, which its file reader needs anyway because its
        # working directory is not the session.
        out = {
            "observe": self._sensor_observe() if self._sensors_only else self._observe(k),
            "frames_dir": str(out_dir.resolve()),
            "files": [Path(f).name for f in files],
        }
        # Looking IS the moment a clearance belief is formed, so answering it here costs no extra
        # round trip. Same builder, same access guarantees; it reads only driver-supplied points.
        if cmd.get("points"):
            out["clearance"] = self._clearance(cmd)
        return out

    # ------------------------------------------------------------------ sensors-only views
    def _sensor_state(self) -> Dict[str, Any]:
        """Raw Bench2Drive-eligible ego sensors plus what is derived from them. Nothing else."""
        inp = self._last_input
        assert inp is not None and self._ego is not None
        lat, lon, alt = (float(v) for v in inp["GPS"][1][:3])
        imu = [float(v) for v in inp["IMU"][1]]
        return {
            "step": self.step,
            "sim_time": _r(self._sim_time, 2),
            "ego_status": {
                "speed_mps": _r(self._ego["speed"], 3),
                "accel_fwd_mps2": _r(imu[0], 3),
                "accel_left_mps2": _r(-imu[1], 3),  # IMU frame is CARLA's: y points RIGHT
                "imu_raw": {"accelerometer_xyz": [_r(v, 4) for v in imu[0:3]],
                            "gyroscope_xyz_rad_s": [_r(v, 4) for v in imu[3:6]],
                            "compass_rad": _r(imu[6], 5),
                            "frame": "CARLA sensor frame: x forward, y right, z up"},
                "gnss": {"lat": lat, "lon": lon, "alt": _r(alt, 2)},
            },
            "pose_estimate": {
                "x": _r(self._ego["x"], 2), "y": _r(self._ego["y"], 2), "yaw_deg": _r(self._ego["yaw"], 2),
                "source": "GNSS + compass (noisy), CARLA world frame: left-handed, yaw clockwise",
            },
            "nav": self._nav(),
            "ego_body": self._footprint(),
            "cameras": [c["id"] for c in self._cameras],
        }

    def _sensor_observe(self) -> Dict[str, Any]:
        st = self._sensor_state()
        es = st["ego_status"]
        return {
            "step": st["step"],
            "ego": {"speed_mps": es["speed_mps"], "accel_fwd_mps2": es["accel_fwd_mps2"],
                    "accel_left_mps2": es["accel_left_mps2"]},
            "nav": st["nav"],
            "cameras": st["cameras"],
            "legend": ("sensors access: the scene is only in the cameras (percept). Route points are in the "
                       "ego frame (fwd ahead, left to the LEFT), from the leaderboard's sparse GPS plan and a "
                       "noisy GNSS pose estimate, so treat them as approximate. Distances are to the ego "
                       f"ORIGIN: your body reaches {self._ego_body['front_m']:.2f} m ahead of it, "
                       f"{self._ego_body['rear_m']:.2f} m behind and {self._ego_body['half_width_m']:.2f} m "
                       "each side, so subtract that before calling a distance clearance "
                       "(`clearance points=[[fwd,left],...]`, or points= on percept)"),
        }

    # ------------------------------------------------------------------ privileged views
    def _lane_info(self) -> Optional[Dict[str, Any]]:
        world_map = CarlaDataProvider.get_map()
        wp = world_map.get_waypoint(self.hero_actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return None

        def same_direction_driving(other) -> bool:
            return other is not None and other.lane_type == carla.LaneType.Driving and other.lane_id * wp.lane_id > 0

        wtf = wp.transform
        e = self._ego
        offset = geo.world_to_ego((wtf.location.x, wtf.location.y), wtf.rotation.yaw, np.array([[e["x"], e["y"]]]))[0]
        return {
            "road_id": wp.road_id,
            "lane_id": wp.lane_id,
            "is_junction": bool(wp.is_junction),
            "lane_width_m": _r(wp.lane_width),
            "offset_from_center_m": _r(offset[1]),  # + = ego is left of the lane centre
            "heading_error_deg": _r(self._yaw_rel_ccw(wtf.rotation.yaw) * -1.0, 1),  # + = ego rotated left of lane
            "lane_change_allowed": str(wp.lane_change),
            "left_lane_same_direction": same_direction_driving(wp.get_left_lane()),
            "right_lane_same_direction": same_direction_driving(wp.get_right_lane()),
        }

    def _actors(self, radius: float) -> List[Dict[str, Any]]:
        world = CarlaDataProvider.get_world()
        hero = self.hero_actor
        hero_loc = hero.get_location()
        e = self._ego
        out = []
        for actor in world.get_actors():
            tid = actor.type_id
            if actor.id == hero.id or not tid.startswith(_ACTOR_PREFIXES):
                continue
            tf = actor.get_transform()
            dist = tf.location.distance(hero_loc)
            if dist > radius:
                continue
            fwd, left = self._ego_frame(tf.location.x, tf.location.y)
            vel = actor.get_velocity()
            v_fwd, v_left = geo.vec_world_to_ego(e["yaw"], (vel.x, vel.y))
            ext = actor.bounding_box.extent
            out.append({
                "id": int(actor.id),
                "type_id": tid,
                "kind": "vehicle" if tid.startswith("vehicle.") else "walker" if tid.startswith("walker.") else "static",
                "distance_m": _r(dist, 1),
                "ego": {"fwd": _r(fwd, 1), "left": _r(left, 1), "v_fwd": _r(v_fwd, 1), "v_left": _r(v_left, 1),
                        "yaw_rel_deg": _r(self._yaw_rel_ccw(tf.rotation.yaw), 0)},
                "speed_mps": _r(math.hypot(vel.x, vel.y), 1),
                "length_m": _r(2 * ext.x, 1),
                "width_m": _r(2 * ext.y, 1),
                "loc": _vec(tf.location),
                "rot": _rot(tf.rotation),
                "velocity": _vec(vel),
            })
        return sorted(out, key=lambda a: a["distance_m"])

    def _parked_vehicles(self, radius: float, actors: List[Dict[str, Any]]) -> List[Tuple[float, float, float, float, float]]:
        """Level (map-baked) vehicle boxes near the ego as (fwd, left, yaw_rel_ccw_deg, half_len, half_wid).
        get_level_bbs also returns spawned actors, so boxes that coincide with a live actor (or the
        ego) are dropped. The radius is 3D like _actors', which drops scenario actors parked
        underground (z ~ -500) before they trigger."""
        world = CarlaDataProvider.get_world()
        hero_loc = self.hero_actor.get_location()
        live = np.array([[a["loc"][0], a["loc"][1]] for a in actors] + [[hero_loc.x, hero_loc.y]])
        out = []
        for name in _PARKED_LABELS:
            label = getattr(carla.CityObjectLabel, name, None)
            if label is None:
                continue
            try:
                boxes = world.get_level_bbs(label)
            except RuntimeError:
                continue
            for bb in boxes:
                loc = bb.location
                if loc.distance(hero_loc) > radius:
                    continue
                if np.min(np.hypot(live[:, 0] - loc.x, live[:, 1] - loc.y)) < 1.5:
                    continue
                fwd, left = self._ego_frame(loc.x, loc.y)
                out.append((fwd, left, self._yaw_rel_ccw(bb.rotation.yaw), float(bb.extent.x), float(bb.extent.y)))
        return out

    def _traffic_lights(self, radius: float) -> List[Dict[str, Any]]:
        world = CarlaDataProvider.get_world()
        hero = self.hero_actor
        hero_loc = hero.get_location()
        affecting = hero.get_traffic_light()
        out = []
        for tl in world.get_actors().filter("*traffic_light*"):
            if not isinstance(tl, carla.TrafficLight):
                continue
            loc = tl.get_transform().location
            dist = loc.distance(hero_loc)
            if dist > radius:
                continue
            fwd, left = self._ego_frame(loc.x, loc.y)
            stop_line, on_route = None, None
            for swp in tl.get_stop_waypoints():
                sl = swp.transform.location
                ahead = self._route_ahead_distance((sl.x, sl.y), swp.transform.rotation.yaw, max_offset_m=2.5)
                if ahead is not None and (on_route is None or ahead < on_route):
                    on_route = ahead
                    stop_line = [_r(v, 1) for v in self._ego_frame(sl.x, sl.y)]
            out.append({
                "id": int(tl.id),
                "state": str(tl.state),
                "distance_m": _r(dist, 1),
                "ego": {"fwd": _r(fwd, 1), "left": _r(left, 1)},
                "affects_ego": affecting is not None and affecting.id == tl.id,
                "stop_line_on_route_ahead_m": on_route,
                "stop_line_ego": stop_line,
            })
        return sorted(out, key=lambda t: t["distance_m"])

    def _stop_signs(self, radius: float) -> List[Dict[str, Any]]:
        world = CarlaDataProvider.get_world()
        hero_loc = self.hero_actor.get_location()
        out = []
        for sign in world.get_actors().filter("*stop*"):
            tf = sign.get_transform()
            dist = tf.location.distance(hero_loc)
            if dist > radius:
                continue
            trigger = tf.transform(sign.trigger_volume.location)
            ext = sign.trigger_volume.extent
            fwd, left = self._ego_frame(tf.location.x, tf.location.y)
            out.append({
                "id": int(sign.id),
                "distance_m": _r(dist, 1),
                "ego": {"fwd": _r(fwd, 1), "left": _r(left, 1)},
                "trigger_ego": [_r(v, 1) for v in self._ego_frame(trigger.x, trigger.y)],
                "trigger_on_route_ahead_m": self._route_stop_ahead((trigger.x, trigger.y), (ext.x, ext.y)),
            })
        return sorted(out, key=lambda s: s["distance_m"])

    def _extract_state(self) -> Dict[str, Any]:
        hero = self.hero_actor
        tf = hero.get_transform()
        actors = self._actors(self._state_radius)
        parked = self._parked_vehicles(self._state_radius, actors)
        return {
            "step": self.step,
            "sim_time": _r(self._sim_time, 2),
            "ego": {
                "loc": _vec(tf.location),
                "rot": _rot(tf.rotation),
                "velocity": _vec(hero.get_velocity()),
                "acceleration": _vec(hero.get_acceleration()),
                "speed_mps": _r(self._ego["speed"]),
                "speed_limit_mps": _r(hero.get_speed_limit() / 3.6, 1),
                "length_m": _r(2 * hero.bounding_box.extent.x, 1),
                "width_m": _r(2 * hero.bounding_box.extent.y, 1),
            },
            "ego_body": self._footprint(),
            "lane": self._lane_info(),
            "nav": self._nav(),
            "traffic_lights": self._traffic_lights(self._state_radius),
            "stop_signs": self._stop_signs(self._state_radius),
            "actors": actors,
            "parked_vehicles": [
                {"ego": {"fwd": _r(f, 1), "left": _r(l, 1), "yaw_rel_deg": _r(y, 0)}, "length_m": _r(2 * hl, 1), "width_m": _r(2 * hw, 1)}
                for f, l, y, hl, hw in parked
            ],
            "collisions": self._collisions[-5:],
            "score": self._live_score(detail=False),
            "route_truth": self._route_truth(),
        }

    def _route_truth(self) -> Optional[List[List[float]]]:
        """The DENSE route ahead in the ego frame, sampled every `route_truth.step_m` out to
        `reach_m`: [[fwd, left], ...].

        Privileged only, and deliberately not part of the driving view -- it exists to LABEL what
        the road actually does, so a run can be scored on whether it saw the geometry. The sparse
        `nav.route_points_ego` is what a driver reasons from on either track; this is the answer
        sheet. `_build_route` already keeps the dense plan when privileged (the sensors track only
        ever has the downsampled GPS one), so there is nothing to compute beyond resampling it.
        """
        if self._sensors_only or self._route is None or self._route_cum is None:
            return None
        e = self._ego
        assert e is not None
        n = int(self._truth_reach / self._truth_step)
        dists = [self._truth_step * (i + 1) for i in range(n)]
        pts = geo.route_points_ahead(self._route, self._route_s, dists)
        pts_ego = geo.world_to_ego((e["x"], e["y"]), e["yaw"], pts)
        return [[_r(float(p[0]), 2), _r(float(p[1]), 2)] for p in pts_ego]

    def _observe(self, k: int) -> Dict[str, Any]:
        """The compact per-decision view: everything needed for the next plan, nothing else."""
        st = self._extract_state()
        lights = [
            {key: t[key] for key in ("state", "affects_ego", "stop_line_on_route_ahead_m", "stop_line_ego")}
            for t in st["traffic_lights"] if t["affects_ego"] or t["stop_line_on_route_ahead_m"] is not None
        ]
        signs = [
            {"trigger_on_route_ahead_m": s["trigger_on_route_ahead_m"], "trigger_ego": s["trigger_ego"]}
            for s in st["stop_signs"] if s["trigger_on_route_ahead_m"] is not None
        ]
        actors = [
            {"id": a["id"], "kind": a["kind"], **a["ego"], "len": a["length_m"], "wid": a["width_m"]}
            for a in st["actors"][:k]
        ]
        parked = sorted(st["parked_vehicles"], key=lambda p: math.hypot(p["ego"]["fwd"], p["ego"]["left"]))[:k]
        return {
            "step": st["step"],
            "ego": {key: st["ego"][key] for key in ("speed_mps", "speed_limit_mps", "length_m", "width_m")},
            "lane": st["lane"],
            "nav": st["nav"],
            "traffic_lights_relevant": lights,
            "stop_signs_on_route": signs,
            "actors_nearest": actors,
            "parked_nearest": [p["ego"] | {"len": p["length_m"], "wid": p["width_m"]} for p in parked],
            "legend": "ego frame: fwd = metres ahead, left = metres to the LEFT, v_* in m/s, yaw_rel_deg + = turned left",
        }

    # ------------------------------------------------------------------ BEV + frame dump
    def _local_lanes(self, reach: float):
        """Driving-lane rectangles around the ego, by bounded graph expansion from the ego's lane
        (next/previous/left/right), so large maps never need a full waypoint generation."""
        world_map = CarlaDataProvider.get_map()
        hero_loc = self.hero_actor.get_location()
        step = float(self._bev["lane_step_m"])
        start = world_map.get_waypoint(hero_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        if start is None:
            return []
        seen = set()
        queue = deque([start])
        centers, yaws, widths, junction = [], [], [], []
        while queue and len(centers) < int(self._bev["lane_max_nodes"]):
            w = queue.popleft()
            key = (w.road_id, w.section_id, w.lane_id, int(round(w.s / step)))
            if key in seen:
                continue
            seen.add(key)
            loc = w.transform.location
            if loc.distance(hero_loc) > reach:
                continue
            centers.append((loc.x, loc.y))
            yaws.append(w.transform.rotation.yaw)
            widths.append(w.lane_width)
            junction.append(w.is_junction)
            queue.extend(w.next(step))
            queue.extend(w.previous(step))
            for side in (w.get_left_lane(), w.get_right_lane()):
                if side is not None and side.lane_type == carla.LaneType.Driving:
                    queue.append(side)
        if not centers:
            return []
        e = self._ego
        centers_ego = geo.world_to_ego((e["x"], e["y"]), e["yaw"], np.asarray(centers))
        yaws_rel = np.radians([self._yaw_rel_ccw(y) for y in yaws])
        return lanes_to_polys(centers_ego, yaws_rel, np.asarray(widths), step, np.asarray(junction))

    def _render_bev(self, out_path: Optional[Path] = None) -> str:
        e = self._ego
        assert e is not None, "advance at least one tick before render_bev"
        b = self._bev
        reach = math.hypot(max(b["forward_m"], b["back_m"]), b["side_m"]) + 5.0
        ego_xy = (e["x"], e["y"])

        route_ego, route_pts = None, None
        if self._route is not None:
            route_ego = geo.world_to_ego(ego_xy, e["yaw"], self._route)
            route_pts = geo.world_to_ego(ego_xy, e["yaw"], geo.route_points_ahead(self._route, self._route_s, self._lookahead))
        plan = None
        if self._mode == "plan" and self._plan_world is not None:
            plan = geo.world_to_ego(ego_xy, e["yaw"], self._plan_world)

        lanes, footprints, lights, signs = [], [], [], []
        header = f"step {self.step} t={self._sim_time:.1f}s {self._mode} v={e['speed']:.1f}m/s access={self._access}"
        if self._sensors_only:
            # NOT `b`: that is the bev config above and is read again below.
            body = self._ego_body
            # box_polygon takes a symmetric half-length about its centre; the origin sits 6 mm off
            # the body's middle on this vehicle, which is invisible at BEV resolution.
            ego_poly = box_polygon((body["front_m"] - body["rear_m"]) / 2.0, 0.0, 0.0,
                                   (body["front_m"] + body["rear_m"]) / 2.0, body["half_width_m"])
        else:
            hero = self.hero_actor
            actors = self._actors(reach)
            world = CarlaDataProvider.get_world()
            for a in actors:
                actor = world.get_actor(a["id"])
                if actor is None:
                    continue
                verts = actor.bounding_box.get_world_vertices(actor.get_transform())
                poly = geo.world_to_ego(ego_xy, e["yaw"], np.array([[v.x, v.y] for v in verts]))
                footprints.append((a["kind"], geo.order_footprint(poly)))
            for fwd, left, yaw, hl, hw in self._parked_vehicles(reach, actors):
                footprints.append(("parked", box_polygon(fwd, left, math.radians(yaw), hl, hw)))
            ego_verts = hero.bounding_box.get_world_vertices(hero.get_transform())
            ego_poly = geo.order_footprint(geo.world_to_ego(ego_xy, e["yaw"], np.array([[v.x, v.y] for v in ego_verts])))
            lanes = self._local_lanes(reach) if b["draw_lanes"] else []
            lights = [(t["state"], t["ego"]["fwd"], t["ego"]["left"]) for t in self._traffic_lights(reach)]
            signs = [tuple(s["trigger_ego"]) for s in self._stop_signs(reach)]
            score = self._live_score(detail=False)
            header += f" lim={hero.get_speed_limit() / 3.6:.1f}" + (f" DS~{score['driving_score']}" if score else "")

        return render_bev(
            out_path or (self._out / "frames" / f"frame_{self.step:05d}" / "bev.png"),
            forward_m=float(b["forward_m"]), back_m=float(b["back_m"]), side_m=float(b["side_m"]),
            px_per_m=float(b["px_per_m"]),
            lanes=lanes,
            route=route_ego,
            actors=footprints,
            traffic_lights=lights,
            stop_signs=signs,
            route_points=route_pts,
            plan=plan,
            ego_poly=ego_poly,
            header=header,
        )

    def _dump_frame(self) -> Dict[str, Any]:
        frame_dir = self._out / "frames" / f"frame_{self.step:05d}"
        frame_dir.mkdir(parents=True, exist_ok=True)
        state_path = frame_dir / "state.json"
        state = self._sensor_state() if self._sensors_only else self._extract_state()
        state_path.write_text(json.dumps({"status": self._status(), "state": state}, indent=2))
        manifest = {
            "dir": str(frame_dir.resolve()),
            "step": self.step,
            "state": str(state_path.resolve()),
            "cameras": self._write_cameras(frame_dir / "cameras", None, 1.0),
            "bev": self._render_bev(frame_dir / "bev.png"),
        }
        (frame_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
        return manifest

    # ------------------------------------------------------------------ logging + sensors
    def _log(self, row: Dict[str, Any]) -> None:
        self._cmd_log.write(json.dumps({"t_wall": _r(time.time(), 3), "step": self.step, **row}) + "\n")
        self._cmd_log.flush()

    def _record_tick(self) -> None:
        e = self._ego
        if self._traj_log is not None:
            self._traj_log.write(json.dumps({
                "step": self.step, "t": _r(self._sim_time, 2), "frame": self._frame, "x": _r(e["x"], 3), "y": _r(e["y"], 3),
                "yaw": _r(e["yaw"], 2), "speed": _r(e["speed"], 3), "mode": self._mode,
                "pose": "gnss_estimate" if self._sensors_only else "ground_truth", **self._last_control,
            }) + "\n")
        if self._record_every > 0 and self.step % self._record_every == 0 and self._last_input is not None:
            cam_dir = self._out / "camera_log"
            cam_dir.mkdir(exist_ok=True)
            img = self._last_input[self._record_cam_id][1][:, :, :3]
            cv2.imwrite(str(cam_dir / f"step_{self.step:05d}.jpg"), np.ascontiguousarray(img))

    def _ensure_collision_sensor(self) -> None:
        if not self._want_collision_sensor or self._collision_sensor is not None:
            return
        world = CarlaDataProvider.get_world()
        bp = world.get_blueprint_library().find("sensor.other.collision")
        sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self.hero_actor)

        def _on_collision(event) -> None:
            other = event.other_actor
            imp = event.normal_impulse
            self._collisions.append({
                "frame": int(event.frame),
                "step": self.step,
                "other_type": str(other.type_id) if other is not None else "unknown",
                "other_id": int(other.id) if other is not None else None,
                "impulse": _r(math.sqrt(imp.x ** 2 + imp.y ** 2 + imp.z ** 2), 1),
            })

        sensor.listen(_on_collision)
        self._collision_sensor = sensor
