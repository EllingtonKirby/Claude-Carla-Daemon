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
# Nominal ego footprint (Bench2Drive's ego is always a Lincoln MKZ 2020): drawn in sensors-only BEVs.
_EGO_HALF_LEN, _EGO_HALF_WID = 2.45, 1.05
# RunningStopTest.point_inside_boundingbox scales stop trigger volumes by this: mirrored, not tuned.
_STOP_BOX_MULTIPLIER = 1.2

HELP: Dict[str, str] = {
    "status": "session summary (also attached to every reply)",
    "help": "this table",
    "tick n [stop_on_collision]": "advance n ticks in the current mode",
    "run_until_frame frame": "advance until step >= frame",
    "run_until_event event [max_ticks]": "event = stopped | plan_done | collision* | infraction*",
    "observe [k]": "compact driving view (privileged: scene around the ego; sensors: ego status + route)",
    "get_state": "full state (privileged: world + ego frame scene; sensors: raw ego sensors + pose estimate)",
    "get_score*": "live driving-score estimate with every infraction event",
    "get_camera [cams] [scale]": "write camera JPGs, return paths",
    "percept [cams] [scale] [k]": "observe + the camera JPGs for this tick in ONE round trip; add "
                                  "percept=true to any driving command to get the next percept in its reply",
    "render_bev": "top-down PNG (privileged: lanes, actors, lights; sensors: route + plan only)",
    "dump_frame": "state.json + cameras + bev.png into frames/frame_<step>/",
    "apply_plan waypoints ticks [target_speed] [stop_on_collision]":
        "track ego-frame [[fwd,left],...] (>=2 pts, plan_interval_s apart) for ticks",
    "set_control steer throttle brake ticks [stop_on_collision]": "raw control for ticks",
    "hold [ticks]": "brake; with ticks, also advance",
    "record on [every]": "toggle camera frame logging to camera_log/ (for tools/make_video.py)",
    "shutdown": "abort the route (scored as an agent failure)",
    "why": "justification string, REQUIRED on every driving command (apply_plan, set_control, hold, tick, "
           "run_until_*, shutdown): what you observed and why this command follows from it",
}
_PRIVILEGED_EVENTS = ("collision", "infraction")
# Commands that decide what the ego does. With require_justification they must carry why="...", and
# every accepted one is logged to decisions.jsonl / decisions.txt with its outcome.
_DRIVING_COMMANDS = ("apply_plan", "set_control", "hold", "tick", "run_until_frame", "run_until_event", "shutdown")
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
        self._plan_interval_s = float(cfg["plan_interval_s"])
        self._bev = {k: cfg["bev"][k] for k in ("forward_m", "back_m", "side_m", "px_per_m", "draw_lanes",
                                                "lane_step_m", "lane_max_nodes")}
        self._record_traj = bool(cfg["record"]["trajectory"])
        self._record_cam_id = str(cfg["record"]["camera_id"])
        self._record_every = int(cfg["record"]["camera_every_n_ticks"])
        self._cameras = [dict(c) for c in cfg["cameras"]]
        self._percept_cams = list(cfg["percept"]["cams"])
        self._percept_scale = float(cfg["percept"]["scale"])
        self._require_why = bool(cfg["require_justification"])
        self.pid = PIDController(cfg["pid"])
        # The stock route budget; the patched leaderboard reads the same variable (docs/DESIGN.md).
        self._max_ticks = int(os.environ.get("AGENT_MAX_TICKS", "4000"))

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
        self._plan_speed: Optional[float] = None
        self._plan_done = False
        self._raw = {"steer": 0.0, "throttle": 0.0, "brake": 1.0}
        self._last_control = {"steer": 0.0, "throttle": 0.0, "brake": 1.0}
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
        # Bench2Drive's evaluator calls this BEFORE setup (the access level is not known yet), so the
        # route is built by whichever of the two runs last.
        super().set_global_plan(global_plan_gps, global_plan_world_coord)
        self._dense_world_plan = global_plan_world_coord
        if hasattr(self, "_access"):
            self._build_route()

    def _build_route(self) -> None:
        pts: List[Tuple[float, float]] = []
        opts: List[str] = []
        if self._sensors_only:
            # Only what a SENSORS-track agent is given: the base class's downsampled GPS plan. The
            # town geo reference comes from its first point, which the leaderboard provides in both
            # GPS and world form; the route is then projected through the SAME conversion as the
            # ego's GNSS so both live in one frame.
            first_loc = self._global_plan_world_coord[0][0].location
            first_gps = self._global_plan[0][0]
            self._lat_ref, self._lon_ref = geo.solve_geo_reference(first_loc.x, first_loc.y, first_gps["lat"], first_gps["lon"])
            source = [(geo.gps_to_world(g["lat"], g["lon"], self._lat_ref, self._lon_ref), o) for g, o in self._global_plan]
        else:
            # Privileged: the DENSE world-coordinate route (the base class keeps a 50 m downsample).
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
                  getattr(self, "_decisions_txt", None), getattr(self, "_score_log", None)):
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
            if self.hero_actor is None:
                self.get_hero()
            tf = self.hero_actor.get_transform()
            self._ego = {"x": float(tf.location.x), "y": float(tf.location.y), "yaw": float(tf.rotation.yaw), "speed": speed}
            self._ensure_collision_sensor()
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
        x, y = geo.predict_blend(None if prev is None else (prev["x"], prev["y"]),
                                 speed, yaw, dt, meas, self._gnss_gain)
        return {"x": x, "y": y, "yaw": yaw, "speed": speed}

    # ------------------------------------------------------------------ advancing
    def _start_advance(self, kind: str, cmd: Dict[str, Any], **extra: Any) -> None:
        self._advance = {
            "kind": kind,
            "ticks": 0,
            "coll0": len(self._collisions),
            "inf0": Counter(_event_key(ev) for ev in self._infraction_events()),
            "stop_on_collision": bool(cmd.get("stop_on_collision", False)),
            "percept": dict(cmd) if cmd.get("percept") else None,
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
        if name == "observe":
            k = int(cmd.get("k", self._nearest_k))
            return {"ok": True, "observe": self._sensor_observe() if self._sensors_only else self._observe(k)}
        if name == "get_state":
            return {"ok": True, "state": self._sensor_state() if self._sensors_only else self._extract_state()}
        if name == "get_score":
            return {"ok": True, "score": self._live_score(detail=True)}
        if name == "get_camera":
            out_dir = self._out / "frames" / f"frame_{self.step:05d}"
            return {"ok": True, "files": self._write_cameras(out_dir, cmd.get("cams"), float(cmd.get("scale", 1.0)))}
        if name == "percept":
            return {"ok": True, **self._percept(cmd)}
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
            ts = cmd.get("target_speed")
            self._plan_speed = None if ts is None else float(ts)
            self._plan_done = False
            self._mode = "plan"
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
        if self._mode != "plan" or self._plan_world is None:
            control.brake = 1.0
            return control

        plan_ego = geo.world_to_ego((e["x"], e["y"]), e["yaw"], self._plan_world)
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
            "ticks_left": self._max_ticks - max(self.step, 0),
            "access": self._access,
            "mode": self._mode,
            "plan_done": self._plan_done if self._mode == "plan" else None,
            "speed_mps": _r(e.get("speed", 0.0), 2),
            "control": self._last_control,
            "route": route,
            "stopped_run_ticks": self._stopped_run,
        }
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

    def _percept(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """One round trip's worth of perception: the observation plus this tick's camera images.

        Purely a composition of the existing access-correct builders -- `_sensor_observe` vs
        `_observe`, and cameras, which are Bench2Drive-eligible at both levels -- so it cannot widen
        what a sensors session sees. Keep it that way: anything added here must be routed through a
        builder that already branches on `_sensors_only`, never read off `hero_actor` or the world.
        """
        cams = cmd.get("cams", self._percept_cams)
        scale = float(cmd.get("scale", self._percept_scale))
        k = int(cmd.get("k", self._nearest_k))
        out_dir = self._out / "frames" / f"frame_{self.step:05d}"
        return {
            "observe": self._sensor_observe() if self._sensors_only else self._observe(k),
            "files": self._write_cameras(out_dir, cams, scale),
        }

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
            "legend": ("sensors access: the scene is only in the cameras (get_camera). Route points are in the "
                       "ego frame (fwd ahead, left to the LEFT), from the leaderboard's sparse GPS plan and a "
                       "noisy GNSS pose estimate, so treat them as approximate"),
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
        }

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
            ego_poly = box_polygon(0.0, 0.0, 0.0, _EGO_HALF_LEN, _EGO_HALF_WID)
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
