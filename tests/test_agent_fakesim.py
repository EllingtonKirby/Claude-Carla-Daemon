"""End-to-end tests of the probe agent against a FAKE simulator (no CARLA needed).

``carla``, ``leaderboard`` and ``srunner`` are replaced by minimal stubs, a toy kinematic vehicle
stands in for physics, and the real agent is driven over its real socket by the real client, once per
access level. This covers dispatch, pause/advance semantics, plan tracking, access enforcement,
inspection outputs, recording and shutdown. It says nothing about CARLA API fidelity: that needs a
live smoke run (benchmark/routes/smoke.txt).
"""

import fnmatch
import json
import math
import socket
import sys
import threading
import types
from pathlib import Path

import cv2

import numpy as np
import pytest
import yaml

from carla_probe import geometry as geo

REPO = Path(__file__).resolve().parents[1]
_STUBBED = ("carla", "leaderboard", "leaderboard.autoagents", "leaderboard.autoagents.autonomous_agent",
            "srunner", "srunner.scenariomanager", "srunner.scenariomanager.carla_data_provider", "carla_probe.agent",
            "leaderboard.utils", "leaderboard.utils.statistics_manager", "srunner.scenariomanager.traffic_events")
GEO_REF = (1.5, 2.5)
_DRIVING = ("apply_plan", "set_control", "hold", "tick", "run_until_frame", "run_until_event", "shutdown")  # a non-zero town geo reference, to exercise the solver


# ----------------------------------------------------------------------------- fake world
class V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def distance(self, o):
        return math.dist((self.x, self.y, self.z), (o.x, o.y, o.z))


class Rot:
    def __init__(self, yaw=0.0):
        self.roll = self.pitch = 0.0
        self.yaw = float(yaw)


class TF:
    def __init__(self, loc, rot):
        self.location, self.rotation = loc, rot

    def transform(self, local):
        return V(self.location.x + local.x, self.location.y + local.y, self.location.z + local.z)


class BB:
    def __init__(self, ex, ey):
        self.extent, self.location = V(ex, ey, 0.8), V()

    def get_world_vertices(self, tf):
        c, s = math.cos(math.radians(tf.rotation.yaw)), math.sin(math.radians(tf.rotation.yaw))
        out = []
        for lx in (-self.extent.x, self.extent.x):
            for ly in (-self.extent.y, self.extent.y):
                for z in (0.0, 1.6):  # forward (c, s), right (-s, c): CARLA left-handed
                    out.append(V(tf.location.x + lx * c - ly * s, tf.location.y + lx * s + ly * c, z))
        return out


class Actor:
    def __init__(self, aid, type_id, x, y, yaw, ex=2.3, ey=1.0):
        self.id, self.type_id = aid, type_id
        self.tf = TF(V(x, y), Rot(yaw))
        self.vel = V()
        self.bounding_box = BB(ex, ey)
        self.speed = 0.0

    def get_transform(self):
        return self.tf

    def get_location(self):
        return self.tf.location

    def get_velocity(self):
        return self.vel

    def get_acceleration(self):
        return V()

    def get_speed_limit(self):
        return 36.0  # km/h

    def get_traffic_light(self):
        return None


class ActorList(list):
    def filter(self, pattern):
        return ActorList(a for a in self if fnmatch.fnmatch(a.type_id, pattern))


class FakeSensor:
    is_alive = True

    def listen(self, cb):
        self.cb = cb

    def stop(self):
        pass

    def destroy(self):
        pass


class World:
    def __init__(self, hero, others):
        self.hero, self.actors = hero, [hero] + others
        self.spawned = 0
        self.level_bbs = {}

    def get_actors(self):
        return ActorList(self.actors)

    def get_actor(self, aid):
        return next((a for a in self.actors if a.id == aid), None)

    def get_level_bbs(self, label):
        return self.level_bbs.get(label, [])

    def get_blueprint_library(self):
        return types.SimpleNamespace(find=lambda _name: None)

    def spawn_actor(self, *_a, **_k):
        self.spawned += 1
        return FakeSensor()


class RoadOption:
    def __init__(self, name):
        self.name = name


class TrafficEvent:
    def __init__(self, etype, frame, message, data=None):
        self.etype, self.frame, self.message, self.data = etype, frame, message, data

    def get_type(self):
        return self.etype

    def get_dict(self):
        return self.data

    def get_frame(self):
        return self.frame

    def get_message(self):
        return self.message


class Scenario:
    """Live criteria in the leaderboard's order: each node keeps its own event list."""

    def __init__(self):
        self.route = types.SimpleNamespace(events=[TrafficEvent("ROUTE_COMPLETION", 0, "", {"route_completed": 0.0})])
        self.min_speed = types.SimpleNamespace(events=[])
        self.collisions = types.SimpleNamespace(events=[])
        self.scheduled = []  # (step, node, event): criteria only record events while ticking

    def fire(self, step):
        for item in [it for it in self.scheduled if it[0] == step]:
            item[1].events.append(item[2])
            self.scheduled.remove(item)

    def get_criteria(self):
        return [self.route, self.min_speed, self.collisions]


# ----------------------------------------------------------------------------- stubs + session
def _install_stubs(world):
    carla = types.ModuleType("carla")

    class VehicleControl:
        def __init__(self):
            self.steer = self.throttle = self.brake = 0.0

    class TrafficLight:
        pass

    carla.VehicleControl = VehicleControl
    carla.TrafficLight = TrafficLight
    carla.LaneType = types.SimpleNamespace(Driving=2)
    carla.CityObjectLabel = types.SimpleNamespace(Car="Car", Truck="Truck")
    carla.Transform = lambda *a, **k: None

    aa = types.ModuleType("leaderboard.autoagents.autonomous_agent")

    class AutonomousAgent:
        def __init__(self, carla_host, carla_port, debug=False):
            self.hero_actor = None

        def set_global_plan(self, gps, world_coord):  # the real base class also downsamples
            self._global_plan = gps
            self._global_plan_world_coord = world_coord

        def get_hero(self):
            self.hero_actor = world.hero

    aa.AutonomousAgent = AutonomousAgent
    aa.Track = types.SimpleNamespace(SENSORS="SENSORS")

    cdp = types.ModuleType("srunner.scenariomanager.carla_data_provider")
    cdp.CarlaDataProvider = types.SimpleNamespace(get_world=lambda: world,
                                                  get_map=lambda: types.SimpleNamespace(get_waypoint=lambda *a, **k: None))
    stats = types.ModuleType("leaderboard.utils.statistics_manager")
    stats.PENALTY_VALUE_DICT = {"COLLISION": 0.6}
    stats.PENALTY_PERC_DICT = {"MIN_SPEED": (0.7, "unused")}
    stats.PENALTY_NAME_DICT = {"COLLISION": "collisions_vehicle", "MIN_SPEED": "min_speed_infractions"}
    events = types.ModuleType("srunner.scenariomanager.traffic_events")
    events.TrafficEventType = types.SimpleNamespace(ROUTE_COMPLETION="ROUTE_COMPLETION")
    sys.modules.update({
        "leaderboard.utils": types.ModuleType("leaderboard.utils"),
        "leaderboard.utils.statistics_manager": stats,
        "srunner.scenariomanager.traffic_events": events,
        "carla": carla,
        "leaderboard": types.ModuleType("leaderboard"),
        "leaderboard.autoagents": types.ModuleType("leaderboard.autoagents"),
        "leaderboard.autoagents.autonomous_agent": aa,
        "srunner": types.ModuleType("srunner"),
        "srunner.scenariomanager": types.ModuleType("srunner.scenariomanager"),
        "srunner.scenariomanager.carla_data_provider": cdp,
    })
    return TrafficLight


@pytest.fixture
def make_session(tmp_path, monkeypatch):
    saved = {k: sys.modules.get(k) for k in _STUBBED}
    opened = []

    def _make(access):
        hero = Actor(1, "vehicle.lincoln.mkz", 0.0, 0.0, 90.0)  # heading world +y
        lead = Actor(2, "vehicle.audi.tt", -5.0, 20.0, 90.0)     # 20 m ahead, 5 m to the ego's RIGHT
        world = World(hero, [lead])
        # Like CARLA 0.9.15, get_level_bbs also returns SPAWNED actors: the ego's own box, and a
        # scenario actor parked 500 m underground until triggered. Only the map-baked car is parked.
        level_bb = lambda x, y, z=0.0: types.SimpleNamespace(location=V(x, y, z), extent=V(2.2, 0.9, 0.8), rotation=Rot(90.0))
        world.level_bbs = {"Car": [level_bb(0.0, 0.0), level_bb(4.0, 30.0)], "Truck": [level_bb(0.0, -20.0, -500.0)]}
        TrafficLight = _install_stubs(world)

        class FakeTL(TrafficLight, Actor):
            def __init__(self, aid, x, y, stop_x, stop_y, stop_yaw):
                Actor.__init__(self, aid, "traffic.traffic_light", x, y, 0.0)
                self.state = "Red"
                self.stop = TF(V(stop_x, stop_y), Rot(stop_yaw))

            def get_stop_waypoints(self):
                return [types.SimpleNamespace(transform=self.stop)]

        def stop_sign(aid, x, y, ex, ey):
            sign = Actor(aid, "traffic.stop", x, y, 0.0)
            sign.trigger_volume = types.SimpleNamespace(location=V(), extent=V(ex, ey, 1.0))
            return sign

        # The route runs x=0, y=0..200 (+y). Only the first light and the first sign apply to it; the
        # others are behind the route start or govern the opposite lane (x ~ +3, heading -y).
        world.actors += [FakeTL(3, 4.0, 62.0, 0.0, 60.0, 90.0), FakeTL(4, 4.0, -28.0, 0.0, -30.0, 90.0),
                         FakeTL(5, -4.0, 72.0, 0.5, 70.0, -90.0),
                         stop_sign(6, 0.0, 40.0, 1.5, 1.0), stop_sign(7, 0.5, -68.0, 2.0, 3.0),
                         stop_sign(8, 3.0, 50.0, 1.0, 1.5)]
        sys.modules.pop("carla_probe.agent", None)
        from carla_probe.agent import ProbeAgent

        cfg = yaml.safe_load((REPO / "carla_probe" / "config.yaml").read_text())
        cfg["output_dir"] = str(tmp_path)
        cfg["access"] = access
        for cam in cfg["cameras"]:
            cam["width"], cam["height"] = 64, 32
        cfg_path = tmp_path / "config.yaml"
        cfg_path.write_text(yaml.safe_dump(cfg))
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        monkeypatch.setenv("PROBE_PORT", str(port))

        agent = ProbeAgent("localhost", 20000, False)
        ys = list(range(0, 201))
        world_plan = [(TF(V(0.0, float(y)), Rot(90.0)), RoadOption("LEFT" if y == 100 else "LANEFOLLOW")) for y in ys]
        gps_plan = [({"lat": ll[0], "lon": ll[1], "z": 0.0}, o)
                    for (t, o), ll in zip(world_plan, (geo.world_to_gps(0.0, float(y), *GEO_REF) for y in ys))]
        # Bench2Drive's evaluator order: set_global_plan BEFORE setup, which gets '+<save_name>' appended.
        agent.set_global_plan(gps_plan, world_plan)
        agent.setup(f"{cfg_path}+RouteScenario_0_rep0")
        scenario = Scenario()
        agent.set_scenario(scenario)  # the patched evaluator does this before the first tick

        state = {"error": None}

        def sim():
            t = 0.0
            try:
                for i in range(5000):
                    scenario.fire(i)
                    lat, lon = geo.world_to_gps(hero.tf.location.x, hero.tf.location.y, *GEO_REF)
                    compass = math.radians(hero.tf.rotation.yaw + 90.0) % (2 * math.pi)
                    inputs = {"SPEED": (0, {"speed": hero.speed}), "GPS": (0, [lat, lon, 0.0]),
                              "IMU": (0, [0.0, 0.0, 9.81, 0.0, 0.0, 0.0, compass])}
                    for cam in cfg["cameras"]:
                        inputs[cam["id"]] = (0, np.zeros((32, 64, 4), dtype=np.uint8))
                    ctl = agent.run_step(inputs, t)
                    dt = 0.05
                    hero.speed = max(0.0, hero.speed + (4.0 * ctl.throttle - 8.0 * ctl.brake) * dt)
                    if hero.speed > 0.0:
                        hero.tf.rotation.yaw += 40.0 * ctl.steer * dt
                    yaw = math.radians(hero.tf.rotation.yaw)
                    hero.tf.location.x += hero.speed * math.cos(yaw) * dt
                    hero.tf.location.y += hero.speed * math.sin(yaw) * dt
                    hero.vel = V(hero.speed * math.cos(yaw), hero.speed * math.sin(yaw))
                    t += dt
            except RuntimeError as e:
                state["error"] = str(e)
            finally:
                agent.destroy(None)

        thread = threading.Thread(target=sim, daemon=True)
        thread.start()

        from carla_probe.client import ProbeClient
        client = ProbeClient(f"127.0.0.1:{port}", wait_s=10)
        client._sock.settimeout(60)
        raw_call = client.call

        def call(cmd, **kw):
            # Every driving command must be justified: supply a why unless the test passes why=None
            # to check the rejection.
            if cmd in _DRIVING and "why" not in kw:
                kw["why"] = f"test: {cmd}"
            if kw.get("why", "") is None:
                kw.pop("why")
            return raw_call(cmd, **kw)

        client.call = call
        opened.append(client)
        return types.SimpleNamespace(c=client, hero=hero, world=world, out=tmp_path, thread=thread, state=state,
                                     scenario=scenario)

    yield _make

    for client in opened:
        client.close()
    for k, mod in saved.items():
        if mod is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = mod


def _finish(s):
    assert s.c.call("shutdown")["result"]["reason"] == "shutdown"
    s.thread.join(timeout=10)
    assert "shutdown" in (s.state["error"] or "")
    assert (s.out / "probe_done.json").is_file()


# ----------------------------------------------------------------------------- privileged
def test_privileged_session(make_session):
    s = make_session("privileged")
    c, hero, out = s.c, s.hero, s.out

    r = c.call("status")
    assert r["ok"] and r["status"]["step"] == 0 and r["status"]["mode"] == "hold"
    assert r["status"]["access"] == "privileged"
    assert r["status"]["speed_limit_mps"] == pytest.approx(10.0)
    assert "apply_plan waypoints ticks [target_speed] [stop_on_collision]" in c.call("help")["commands"]

    obs = c.call("percept")["observe"]
    lead = next(a for a in obs["actors_nearest"] if a["id"] == 2)
    assert [(p["fwd"], p["left"]) for p in obs["parked_nearest"]] == [(pytest.approx(30.0), pytest.approx(4.0))]
    assert (lead["fwd"], lead["left"]) == (pytest.approx(20.0), pytest.approx(-5.0))
    assert obs["nav"]["next_command"] == {"command": "LEFT", "distance_m": pytest.approx(100.0)}
    assert obs["nav"]["route_points_ego"][0] == [pytest.approx(5.0), pytest.approx(0.0)]
    assert len(obs["traffic_lights_relevant"]) == 1
    light = obs["traffic_lights_relevant"][0]
    assert obs["stop_signs_on_route"] == [{"trigger_on_route_ahead_m": pytest.approx(38.8), "trigger_ego": [40.0, 0.0]}]
    assert light["state"] == "Red" and light["stop_line_on_route_ahead_m"] == pytest.approx(60.0)

    r = c.call("apply_plan", waypoints=[[3, 0], [6, 0], [9, 0], [12, 0]], target_speed=4, ticks=20)
    assert r["ok"] and r["result"]["reason"] == "ticks" and r["result"]["ticks_advanced"] == 20
    assert "collisions_during" in r["result"] and "new_infractions" in r["result"]
    assert r["status"]["step"] == 20 and r["status"]["mode"] == "plan" and r["status"]["speed_mps"] > 1.0
    assert abs(hero.tf.location.x) < 0.2 and hero.tf.location.y > 0.5  # went straight along +y
    assert r["status"]["route"]["progress_pct"] > 0.0
    assert s.world.spawned == 1  # the collision sensor

    assert Path(c.call("render_bev")["file"]).is_file()
    dump = c.call("dump_frame")["dump"]
    assert Path(dump["state"]).is_file() and Path(dump["bev"]).is_file() and len(dump["cameras"]) == 4
    # percept is the ONLY view: observe and get_camera are gone, so there is one way to see.
    assert c.call("get_camera", cams=["CAM_FRONT"])["ok"] is False
    assert c.call("observe")["ok"] is False
    assert len(c.call("percept", cams=["CAM_FRONT"])["files"]) == 1
    # scale is the harness's control: a driver-supplied value is ignored, not honoured.
    def _front(**kw):
        r = c.call("percept", cams=["CAM_FRONT"], **kw)
        return cv2.imread(str(Path(r["frames_dir"]) / r["files"][0])).shape[:2]
    assert _front(scale=1.0) == _front()

    r = c.call("percept", cams=["CAM_FRONT", "CAM_BACK"])
    # The directory once, bare names after: cheaper to repeat, and still absolute when joined.
    assert r["ok"] and r["files"] == ["CAM_FRONT.jpg", "CAM_BACK.jpg"]
    assert Path(r["frames_dir"]).is_absolute() and (Path(r["frames_dir"]) / r["files"][0]).is_file()
    assert r["observe"].keys() == obs.keys() and r["status"]["step"] == c.call("status")["status"]["step"]
    assert len(c.call("percept")["files"]) == 4  # config default (cams: []): every camera
    # ... and attached to a driving command on request, so a decision costs ONE round trip, not
    # three. Off unless asked for: an unwanted percept is tokens the driver cannot evict.
    r = c.call("tick", n=2, percept=True, why="test: percept rides along")
    assert r["result"]["reason"] == "ticks" and r["observe"]["step"] == r["status"]["step"]
    assert Path(r["frames_dir"]).name == f"frame_{r['status']['step']:05d}"
    assert "observe" not in c.call("tick", n=1, why="test: no percept asked for")

    # Live score: criteria events are concatenated per criterion, so a new event can land BEFORE an
    # already-reported one. new_infractions must report each event exactly once.
    step = c.call("status")["status"]["step"]
    s.scenario.scheduled.append((step + 1, s.scenario.collisions, TrafficEvent("COLLISION", 30, "collided")))
    r = c.call("tick", n=1)
    assert [e["type"] for e in r["result"]["new_infractions"]] == ["collisions_vehicle"]
    assert r["status"]["score"]["infraction_penalty"] == pytest.approx(0.6)
    s.scenario.scheduled.append((step + 3, s.scenario.min_speed, TrafficEvent("MIN_SPEED", 31, "slow")))
    r = c.call("run_until_event", event="infraction", max_ticks=5)
    assert r["result"]["reason"] == "infraction"
    assert [e["type"] for e in r["result"]["new_infractions"]] == ["min_speed_infractions"]
    assert c.call("tick", n=1)["result"]["new_infractions"] == []

    r = c.call("run_until_event", event="plan_done", max_ticks=400)
    assert r["result"]["reason"] == "plan_done" and r["status"]["plan_done"] is True

    r = c.call("run_until_event", event="stopped", max_ticks=400)
    assert r["result"]["reason"] == "stopped" and r["status"]["speed_mps"] < 0.1

    r = c.call("set_control", steer=0.0, throttle=0.5, brake=0.0, ticks=5)
    assert r["status"]["mode"] == "raw" and r["status"]["control"]["throttle"] == 0.5

    assert c.call("bogus")["ok"] is False
    assert c.call("tick", n=0)["ok"] is False
    assert c.call("run_until_frame", frame=0)["result"]["reason"] == "already_at_frame"

    assert c.call("record", on=True, every=1)["camera_every_n_ticks"] == 1
    c.call("hold", ticks=3)
    assert len(list((out / "camera_log").glob("step_*.jpg"))) >= 3

    # Unjustified driving commands are refused before they change anything.
    r = c.call("tick", n=5, why=None)
    assert r["ok"] is False and "why" in r["error"]
    before = r["status"]["step"]
    assert c.call("hold", ticks=2, why="  ")["ok"] is False and c.call("status")["status"]["step"] == before
    r = c.call("hold", ticks=2, why="light ahead is red; stopping")
    assert r["result"]["reason"] == "ticks"

    _finish(s)
    decisions = (out / "decisions.txt").read_text()
    assert "why: light ahead is red; stopping" in decisions and "-> reason=ticks, ticks_advanced=2" in decisions
    assert "why: test: shutdown" in decisions
    rows = [json.loads(line) for line in (out / "decisions.jsonl").read_text().splitlines()]
    hold = next(r for r in rows if r.get("why") == "light ahead is red; stopping")
    assert hold["cmd"] == "hold" and hold["args"] == {"ticks": 2}
    assert next(r for r in rows if r.get("i") == hold["i"] and "result" in r)["step"] == hold["step"] + 2
    assert not any(r.get("why", "x").strip() == "" for r in rows if "cmd" in r)  # refused ones are not logged
    events = [json.loads(line) for line in (out / "score_events.jsonl").read_text().splitlines()]
    assert [e["type"] for e in events] == ["collisions_vehicle", "min_speed_infractions"]
    assert all(isinstance(e["step"], int) and "frame_seen" in e for e in events)
    assert len((out / "commands.jsonl").read_text().splitlines()) >= 15
    assert len((out / "trajectory.jsonl").read_text().splitlines()) > 100


# ----------------------------------------------------------------------------- sensors only
_PRIVILEGED_WORDS = ("actors", "traffic_lights", "stop_signs", "lane", "parked", "speed_limit", "score",
                     "collision", "infraction", "loc", "rot", "velocity")


def _keys(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield k
            yield from _keys(v)
    elif isinstance(obj, list):
        for v in obj:
            yield from _keys(v)


def test_sensors_session(make_session):
    s = make_session("sensors")
    c, hero = s.c, s.hero

    r = c.call("status")
    st = r["status"]
    assert st["access"] == "sensors"
    assert not {"speed_limit_mps", "collision_contacts", "score"} & set(st)
    assert s.world.spawned == 0  # no collision sensor

    obs = c.call("percept")["observe"]
    state = c.call("get_state")["state"]
    for payload in (obs, state):
        leaked = [k for k in _keys(payload) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]
        assert not leaked, leaked
    # Route and pose come from GNSS through the solved geo reference: they match ground truth here
    # because the fake GNSS is noise-free.
    assert state["pose_estimate"]["x"] == pytest.approx(0.0, abs=1e-3)
    assert state["pose_estimate"]["y"] == pytest.approx(0.0, abs=1e-3)
    assert state["pose_estimate"]["yaw_deg"] == pytest.approx(90.0, abs=1e-6)
    assert obs["nav"]["next_command"] == {"command": "LEFT", "distance_m": pytest.approx(100.0)}
    assert obs["nav"]["route_points_ego"][1] == [pytest.approx(10.0), pytest.approx(0.0)]

    # Privileged requests are refused BEFORE they change anything.
    for bad in (
        {"cmd": "get_score"},
        {"cmd": "run_until_event", "event": "collision"},
        {"cmd": "run_until_event", "event": "infraction"},
        {"cmd": "apply_plan", "waypoints": [[3, 0], [6, 0]], "ticks": 5, "stop_on_collision": True},
        {"cmd": "tick", "n": 5, "stop_on_collision": True},
    ):
        name = bad.pop("cmd")
        resp = c.call(name, **bad)
        assert resp["ok"] is False and "access=sensors" in resp["error"], (name, resp)
        assert resp["status"]["mode"] == "hold" and resp["status"]["step"] == 0
    assert "event=collision" not in str(c.call("help")["advance_events"])

    # percept composes the same access-correct builders, so it must leak nothing either -- checked
    # at both entry points, standalone and attached to a driving command.
    standalone = c.call("percept")
    attached = c.call("tick", n=2, percept=True)
    for payload in (standalone, attached):
        assert payload["ok"] and payload["observe"].keys() == obs.keys()
        assert payload["files"] == [
            "CAM_FRONT.jpg", "CAM_FRONT_LEFT.jpg", "CAM_FRONT_RIGHT.jpg", "CAM_BACK.jpg"]
        leaked = [k for k in _keys(payload) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]
        assert not leaked, leaked

    r = c.call("apply_plan", waypoints=[[3, 0], [6, 0], [9, 0], [12, 0]], target_speed=4, ticks=20)
    assert r["result"] == {"reason": "ticks", "ticks_advanced": 20}
    assert abs(hero.tf.location.x) < 0.2 and hero.tf.location.y > 0.5
    est = c.call("get_state")["state"]["pose_estimate"]
    assert (est["x"], est["y"]) == (pytest.approx(hero.tf.location.x, abs=0.2), pytest.approx(hero.tf.location.y, abs=0.2))

    assert Path(c.call("render_bev")["file"]).is_file()
    dump = c.call("dump_frame")["dump"]
    assert "ego_status" in Path(dump["state"]).read_text()
    assert c.call("run_until_event", event="plan_done", max_ticks=400)["result"]["reason"] == "plan_done"

    # Scoring events are logged for analysis on the sensors track too, but never reach a reply.
    step = c.call("status")["status"]["step"]
    s.scenario.scheduled.append((step + 1, s.scenario.collisions, TrafficEvent("COLLISION", 99, "collided")))
    r = c.call("tick", n=3)
    assert not [k for k in _keys(r) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)], r
    assert "collided" not in json.dumps(r) and "collided" not in json.dumps(c.call("get_state"))

    _finish(s)
    events = [json.loads(line) for line in (s.out / "score_events.jsonl").read_text().splitlines()]
    assert [(e["type"], e["step"]) for e in events] == [("collisions_vehicle", step + 1)]
    assert "why: test: apply_plan" in (s.out / "decisions.txt").read_text()
