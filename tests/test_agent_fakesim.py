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
_DRIVING = ("apply_plan", "set_control", "drive", "hold", "tick", "run_until_frame", "run_until_event", "shutdown")  # a non-zero town geo reference, to exercise the solver


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
    def __init__(self, ex, ey, lx=0.0):
        # `location` is the box centre relative to the ACTOR origin, and it is not zero on a real
        # vehicle -- which is why config's ego: splits front_m from rear_m instead of a half-length.
        self.extent, self.location = V(ex, ey, 0.8), V(lx, 0.0, 0.0)

    def get_world_vertices(self, tf):
        c, s = math.cos(math.radians(tf.rotation.yaw)), math.sin(math.radians(tf.rotation.yaw))
        out = []
        for lx in (-self.extent.x, self.extent.x):
            for ly in (-self.extent.y, self.extent.y):
                for z in (0.0, 1.6):  # forward (c, s), right (-s, c): CARLA left-handed
                    out.append(V(tf.location.x + lx * c - ly * s, tf.location.y + lx * s + ly * c, z))
        return out


class Actor:
    def __init__(self, aid, type_id, x, y, yaw, ex=2.3, ey=1.0, lx=0.0):
        self.id, self.type_id = aid, type_id
        self.tf = TF(V(x, y), Rot(yaw))
        self.vel = V()
        self.bounding_box = BB(ex, ey, lx)
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

    def _make(access, gnss_noise=0.0, hero_extent=None, vla_addr=None, **cfg_overrides):
        """``gnss_noise`` (m, per axis, seeded) makes the fake GNSS behave like the leaderboard's,
        which is the only way to tell the position filters apart: with a clean fix they all return
        the truth. ``cfg_overrides`` sets top-level config keys. ``hero_extent`` is
        ``(extent_x, extent_y, bbox_offset_x)`` for the ego's bounding box, for the footprint check.
        ``vla_addr`` enables the learned policy (config `vla`) against a fake service.
        """
        # The real Bench2Drive hero, measured off the blueprint: a Lincoln MKZ 2020. Using its
        # actual box means config's `ego:` is exercised against the vehicle it claims to describe.
        ex, ey, lx = hero_extent or (2.4462, 0.9184, -0.0063)
        hero = Actor(1, "vehicle.lincoln.mkz", 0.0, 0.0, 90.0, ex=ex, ey=ey, lx=lx)  # heading world +y
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
        cfg.update(cfg_overrides)
        for cam in cfg["cameras"]:
            cam["width"], cam["height"] = 64, 32
        if vla_addr is not None:
            cfg["vla"]["enabled"], cfg["vla"]["addr"] = True, vla_addr
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
            rng = np.random.default_rng(0)
            try:
                for i in range(5000):
                    scenario.fire(i)
                    gx, gy = hero.tf.location.x, hero.tf.location.y
                    if gnss_noise:
                        gx += rng.normal(0.0, gnss_noise)
                        gy += rng.normal(0.0, gnss_noise)
                    lat, lon = geo.world_to_gps(gx, gy, *GEO_REF)
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

    _bev = c.call("render_bev")
    assert "file" in _bev, _bev
    assert Path(_bev["file"]).is_file()
    dump = c.call("dump_frame")["dump"]
    # route_truth: the dense route ahead in the ego frame -- the label a route-perception score
    # needs, and deliberately NOT part of the driving view the sparse nav points give.
    truth = c.call("get_state")["state"]["route_truth"]
    cfg = yaml.safe_load((REPO / "carla_probe" / "config.yaml").read_text())["route_truth"]
    assert len(truth) == int(cfg["reach_m"] / cfg["step_m"])
    assert truth[0][0] == pytest.approx(cfg["step_m"], abs=0.5)   # first sample one step ahead
    assert truth[-1][0] > truth[0][0]                             # ordered by distance ahead

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
                     "collision", "infraction", "loc", "rot", "velocity", "route_truth")


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

    # The ego is told how big it is. Not world state -- it is the car it is driving -- so it is
    # reported at both levels, and it is the thing whose absence made every distance ambiguous.
    body = state["ego_body"]
    assert (body["front_m"], body["rear_m"], body["half_width_m"]) == (2.44, 2.45, 0.92)
    assert body["corners_ego"] == [[2.44, 0.92], [2.44, -0.92], [-2.45, -0.92], [-2.45, 0.92]]
    assert [cam["id"] for cam in body["cameras"]] == [
        "CAM_FRONT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK"]
    # The mount comes from config (CAM_FRONT sits where the learned policy's camera was trained).
    front = yaml.safe_load((REPO / "carla_probe" / "config.yaml").read_text())["cameras"][0]
    assert body["cameras"][0] == {"id": "CAM_FRONT", "x": front["x"], "y": front["y"],
                                  "yaw_deg": front["yaw"], "fov_deg": front["fov"]}
    assert "ORIGIN" in obs["legend"] and "2.44" in obs["legend"]

    # clearance is from the BODYWORK. A point 3 m from the origin is 0.56 m from the bumper, and
    # spending the 3 m is how the driver hit things it had correctly located.
    cl = c.call("clearance", points=[[3.0, 0.0], [0.0, 1.0], [0.0, 0.0]])["clearance"]
    assert [pt["now_m"] for pt in cl["points"]] == [pytest.approx(0.56), pytest.approx(0.08),
                                                    pytest.approx(-0.92)]
    assert cl["plan"] is None and "along_plan" not in cl["points"][0]
    assert not [k for k in _keys(cl) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]

    # ...and over a path, which is what catches a corner swinging into something already passed.
    swept = c.call("clearance", points=[[3.5, -1.4]], waypoints=[[4.0, 0.0], [8.0, 4.0]])["clearance"]
    assert swept["plan"] == "given"
    assert swept["points"][0]["now_m"] == pytest.approx(1.16, abs=0.02)      # clear right now
    assert swept["points"][0]["along_plan"]["min_m"] == pytest.approx(-0.28, abs=0.05)  # not after
    assert swept["swept"]["corridor_width_m"] > 2 * body["half_width_m"]

    # Asking while looking costs no extra round trip, and must not widen what percept returns.
    with_points = c.call("percept", points=[[3.0, 0.0]])
    assert with_points["clearance"]["points"][0]["now_m"] == pytest.approx(0.56)
    assert not [k for k in _keys(with_points) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]

    # percept composes the same access-correct builders, so it must leak nothing either -- checked
    # at both entry points, standalone and attached to a driving command.
    standalone = c.call("percept")
    attached = c.call("tick", n=2, percept=True)
    for payload in (standalone, attached):
        assert payload["ok"] and payload["observe"].keys() == obs.keys()
        assert payload["files"] == [
            "CAM_FRONT.jpg", "CAM_FRONT_LEFT.jpg", "CAM_FRONT_RIGHT.jpg", "CAM_BACK.jpg"]
        assert Path(payload["camera_model"]) == Path(payload["frames_dir"]).parent / "cameras.json"
        intrinsics = json.loads(Path(payload["camera_model"]).read_text())["cameras"]["CAM_FRONT"]["intrinsics"]
        assert set(intrinsics) == {"jpg"}
        assert intrinsics["jpg"]["fx"] > 0
        leaked = [k for k in _keys(payload) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]
        assert not leaked, leaked

    r = c.call("apply_plan", waypoints=[[3, 0], [6, 0], [9, 0], [12, 0]], target_speed=4, ticks=20)
    assert r["result"] == {"reason": "ticks", "ticks_advanced": 20}
    assert abs(hero.tf.location.x) < 0.2 and hero.tf.location.y > 0.5
    est = c.call("get_state")["state"]["pose_estimate"]
    assert (est["x"], est["y"]) == (pytest.approx(hero.tf.location.x, abs=0.2), pytest.approx(hero.tf.location.y, abs=0.2))

    _bev = c.call("render_bev")
    assert "file" in _bev, _bev
    assert Path(_bev["file"]).is_file()
    dump = c.call("dump_frame")["dump"]
    # The dense plan does not exist on this track, so the label is absent rather than approximated.
    assert "route_truth" not in c.call("get_state")["state"]

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


# --------------------------------------------------------------- position filters and plan tracking
# The leaderboard noises the GNSS but not the compass or the speedometer (agent_wrapper.py sets
# noise_lat/lon/alt on the GNSS, accel/gyro on the IMU, and CARLA has no compass noise attribute at
# all). Everything below exists because of that asymmetry, so every test here runs with a noisy fix.
GNSS_SD = 0.56   # m per axis per sample, the leaderboard's 5e-6 deg at this latitude


def _traj(s, since):
    """Trajectory rows this session appended. The fixture reuses one tmp_path and the daemon opens
    trajectory.jsonl in APPEND mode, so two sessions in one test share the file -- read the whole
    thing and you are scoring the previous variant as well as this one."""
    rows = [json.loads(l) for l in (s.out / "trajectory.jsonl").read_text().splitlines() if l.strip()]
    return rows[since:]


def _traj_len(s):
    f = s.out / "trajectory.jsonl"
    return len(f.read_text().splitlines()) if f.exists() else 0


def _drive_straight(s, plans=14, ticks=20):
    """Drive the fake route, sampling the believed pose against the truth at each re-plan."""
    errs = []
    for _ in range(plans):
        s.c.call("apply_plan", waypoints=[[5, 0], [10, 0], [20, 0], [40, 0]], ticks=ticks, target_speed=6.0)
        est = s.c.call("get_state")["state"]["pose_estimate"]
        errs.append(math.hypot(est["x"] - s.hero.tf.location.x, est["y"] - s.hero.tf.location.y))
    return errs[len(errs) // 2:]   # drop the warm-up: every filter starts at the raw fix


def test_dead_reckoning_filters_beat_the_blend(make_session):
    """dr_offset / dr_trend exist to stop discarding old fixes, so they must localise better than
    the complementary filter on the same three sensors. Relative, not absolute: the fake bicycle is
    not CARLA, but the ordering is the whole point of the change."""
    rms = {}
    for mode in ("blend", "dr_offset", "dr_trend"):
        s = make_session("sensors", gnss_noise=GNSS_SD, gnss_filter=mode)
        e = _drive_straight(s)
        rms[mode] = math.sqrt(sum(v * v for v in e) / len(e))
    assert rms["dr_offset"] < 0.6 * rms["blend"], rms
    assert rms["dr_trend"] < rms["blend"], rms


def test_dead_reckon_plan_tracking_steadies_the_steering(make_session):
    """plan_tracking: dead_reckon keeps GNSS noise out of the tracking transform, so the steer
    command must stop chattering. Measured the way the wheel is felt: mean |change| per tick."""
    def chatter(**over):
        s = make_session("sensors", gnss_noise=GNSS_SD, **over)
        n0 = _traj_len(s)
        _drive_straight(s, plans=10)
        st = [r["steer"] for r in _traj(s, n0) if r.get("speed", 0) > 1.0]
        return sum(abs(b - a) for a, b in zip(st, st[1:])) / max(len(st) - 1, 1)

    assert chatter(plan_tracking="dead_reckon") < 0.5 * chatter(plan_tracking="gnss")


def test_plan_tracking_is_ignored_under_privileged(make_session):
    """The privileged pose is ground truth, so the world round trip is already exact and dead
    reckoning could only add error. The knob must be inert there -- identical steering, tick for
    tick, not merely similar."""
    def steers(tracking):
        s = make_session("privileged", plan_tracking=tracking)
        n0 = _traj_len(s)
        for _ in range(6):
            s.c.call("apply_plan", waypoints=[[5, 0], [10, 1], [20, 2], [40, 2]], ticks=20, target_speed=6.0)
        return [r["steer"] for r in _traj(s, n0)]

    assert steers("dead_reckon") == steers("gnss")


def test_the_footprint_is_checked_against_the_car_that_was_actually_spawned(make_session, capsys):
    """The footprint is quoted to the driver and used for clearance arithmetic, so a stale config
    is worse than none: it would be confidently wrong. Only a privileged session can read the real
    bounding box; a sensors session inherits the guarantee because the same hero is spawned."""
    s = make_session("privileged", hero_extent=(2.0, 1.4, 0.0))     # not the MKZ 2020
    s.c.call("tick", n=1, why="settle")
    assert "does not match the spawned" in capsys.readouterr().out
    logged = [json.loads(l) for l in (s.out / "commands.jsonl").read_text().splitlines()
              if '"ego_body_check"' in l]
    assert logged and logged[-1]["off_m"]["half_width_m"] == pytest.approx(0.48)  # measured - config
    # ...and it warns without changing what is reported: config stays the source of truth.
    assert s.c.call("get_state")["state"]["ego_body"]["half_width_m"] == 0.92


def test_the_real_vehicle_passes_the_footprint_check(make_session, capsys):
    s = make_session("privileged")                                   # the measured MKZ 2020 box
    s.c.call("tick", n=1, why="settle")
    assert "does not match the spawned" not in capsys.readouterr().out


def test_clearance_reads_the_plan_in_flight_when_none_is_given(make_session):
    s = make_session("sensors")
    s.c.call("apply_plan", waypoints=[[4.0, 0.0], [8.0, 4.0]], ticks=1, why="turn out")
    cl = s.c.call("clearance", points=[[3.5, -1.4]])["clearance"]
    assert cl["plan"] == "in_flight"
    assert cl["points"][0]["along_plan"]["min_m"] < 0.0


# ----------------------------------------------------------------------------- learned policy (`drive`)
@pytest.fixture
def fake_vla():
    """A stand-in for carla_probe/vla_server.py: records every request, answers a straight route."""
    import base64
    from http.server import BaseHTTPRequestHandler, HTTPServer

    requests = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            requests.append((self.path, body))
            if self.path == "/act":
                img = cv2.imdecode(np.frombuffer(base64.b64decode(body["image_jpeg_b64"]), np.uint8), cv2.IMREAD_COLOR)
                reply = {"ok": True, "route": [[0.6 * i, 0.0] for i in range(1, 21)],
                         "speed_wps": [[0.5 * i, 0.0] for i in range(1, 11)], "desired_speed_mps": 4.0,
                         "language": f"echo: {body.get('text')}", "prompt": "", "ms": 1.0,
                         "control": {"steer": 0.0, "throttle": 0.6, "brake": 0.0}, "image_hw": list(img.shape[:2])}
            else:
                reply = {"ok": True}
            data = json.dumps(reply).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass

    httpd = HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield types.SimpleNamespace(addr=f"127.0.0.1:{httpd.server_port}", requests=requests)
    httpd.shutdown()


@pytest.mark.parametrize("access", ["privileged", "sensors"])
def test_drive_learned_policy(make_session, fake_vla, access):
    s = make_session(access, vla_addr=fake_vla.addr)
    c, hero = s.c, s.hero

    # A driving command: justified, and only in a mode that makes sense.
    assert c.call("drive", text="Follow the route.", ticks=5, why=None)["ok"] is False
    assert "needs text" in c.call("drive", mode="instruction", ticks=5)["error"]
    assert "1..200" in c.call("drive", text="x", ticks=500)["error"]

    # With the policy on, the driver sees exactly the policy's camera, at both entry points.
    assert c.call("percept")["files"] == ["CAM_FRONT.jpg"]

    r = c.call("drive", text="Follow the route. Accelerate to drive with the target speed.", ticks=10)
    assert r["ok"] and r["result"]["reason"] == "ticks" and r["result"]["ticks_advanced"] == 10
    assert r["status"]["mode"] == "vla"
    v = r["result"]["vla"]
    assert v["queries"] == 5                      # every_n_ticks 2: ticks 0, 2, 4, 6, 8
    assert v["language"].startswith("echo: Follow the route.") and v["desired_speed_mps"] == 4.0
    assert hero.tf.location.y > 0.3 and abs(hero.tf.location.x) < 0.1   # its throttle moved the car
    # Reset once per route, then only queries: each from the sensors alone.
    assert [p for p, _ in fake_vla.requests][:2] == ["/reset", "/act"]
    acts = [b for p, b in fake_vla.requests if p == "/act"]
    assert len(acts) == 5
    q = acts[0]
    assert q["mode"] == "commentary" and q["text"].startswith("Follow the route.")
    assert set(q) == {"image_jpeg_b64", "speed_mps", "target_points", "text", "mode"}
    # The route runs straight ahead (+y, the ego's heading): the two points are dead ahead.
    assert q["target_points"] == [[pytest.approx(10.0, abs=0.05), pytest.approx(0.0, abs=0.05)],
                                  [pytest.approx(20.0, abs=0.05), pytest.approx(0.0, abs=0.05)]]
    if access == "sensors":
        leaked = [k for k in _keys(r) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]
        assert not leaked, leaked

    # Ticking on keeps the policy in charge (the count runs over the whole `drive` episode);
    # `hold` takes it away.
    r = c.call("tick", n=4)
    assert r["status"]["mode"] == "vla" and r["result"]["vla"]["queries"] == 7
    assert c.call("hold")["status"]["mode"] == "hold"
    # Every query is on record, for reviewing what the words did.
    rows = [json.loads(l) for l in (s.out / "vla.jsonl").read_text().splitlines()]
    assert len(rows) == 7 and rows[0]["language"].startswith("echo:") and len(rows[0]["route"]) == 20
    _finish(s)


# ----------------------------------------------------------------------------- dense
def test_dense_session(make_session, monkeypatch):
    """dense = the sensors view, exactly, plus GT depth + instance arrays written into the frame dir.
    Everything sensors refuses, dense refuses; nothing privileged reaches a reply."""
    import carla_probe.gt_dense as gd
    started = []

    class FakeRig:
        def __init__(self, world, hero, cameras, scale):
            started.append((len(cameras), scale))

        def write(self, out_dir, cams, frame):
            for cam in cams:
                np.save(out_dir / f"{cam}_depth.npy", np.zeros((2, 2), np.float16))
            return [f"{cam}_depth.npy" for cam in cams]

        def destroy(self):
            pass

    monkeypatch.setattr(gd, "DenseRig", FakeRig)
    s = make_session("dense")
    c = s.c
    st = c.call("status")["status"]
    assert st["access"] == "dense"
    assert not {"speed_limit_mps", "collision_contacts", "score"} & set(st)

    first = c.call("percept")
    assert started == [(4, 0.5)]
    assert not any(f.endswith("_depth.npy") for f in first["files"]), "the rig spawned this tick has no images yet"
    for payload in (first["observe"], c.call("get_state")["state"]):
        leaked = [k for k in _keys(payload) if any(k.startswith(w) for w in _PRIVILEGED_WORDS)]
        assert not leaked, leaked
    for bad in ({"cmd": "get_score"}, {"cmd": "run_until_event", "event": "collision"},
                {"cmd": "tick", "n": 5, "stop_on_collision": True}):
        name = bad.pop("cmd")
        resp = c.call(name, **bad)
        assert resp["ok"] is False and "access=dense" in resp["error"], (name, resp)

    c.call("hold", ticks=1)
    later = c.call("percept")
    assert "CAM_FRONT_depth.npy" in later["files"]
    frames = Path(later["frames_dir"])
    assert (frames / "CAM_FRONT_depth.npy").is_file()
    model = json.loads((frames.parent / "cameras.json").read_text())
    assert set(model["cameras"]["CAM_FRONT"]["intrinsics"]) == {"jpg", "depth_npy", "inst_png"}
    _finish(s)


def test_sensors_session_writes_a_camera_model_but_no_dense_files(make_session):
    s = make_session("sensors", code={"full_res_frames": True})
    r = s.c.call("percept")
    assert "CAM_FRONT_full.jpg" in r["files"] and not any("depth" in f for f in r["files"])
    model = json.loads((Path(r["frames_dir"]).parent / "cameras.json").read_text())
    assert set(model["cameras"]["CAM_BACK"]["intrinsics"]) == {"jpg", "full_jpg"}
    _finish(s)


# ----------------------------------------------------------------------------- privileged extras
def _extras_cfg(tmp_path):
    import json as _json
    t = np.arange(120) * 0.05
    table = {"configs": {"pid075": {"launch": {str(v): list(np.minimum(v, 2.0 * t)) for v in (3.0, 6.0, 9.0)},
                                    "brake": {str(v): list(np.maximum(0.0, v - 8.0 * t)) for v in (5.0, 9.0)},
                                    "offset": [], "arc": [{"speed": 3.0, "curvature": 0.0, "p95": 0.2},
                                                          {"speed": 9.0, "curvature": 0.03, "p95": 0.8}]}}}
    p = tmp_path / "controller_model.json"
    p.write_text(_json.dumps(table))
    return {"enabled": True, "scenario_lookahead_m": 150.0, "controller_model": str(p)}


def test_privileged_extras(make_session, tmp_path):
    s = make_session("privileged", privileged_extras=_extras_cfg(tmp_path))
    c = s.c
    obs = c.call("percept")["observe"]
    a = obs["actors_nearest"][0]
    assert {"type", "role", "emergency"} <= set(a), a
    assert {"stop_sign", "traffic_light", "emergency_vehicle", "collisions"} <= set(obs["rules"])
    assert "4 m" in obs["rules"]["stop_sign"]["rule"]
    assert isinstance(obs["scenarios"], list)

    pred = c.call("predict", waypoints=[[5, 0], [10, 0], [20, 0]], target_speed=6, ticks=40)
    assert pred["ok"], pred
    p = pred["predict"]
    assert p["poses"] and p["poses"][-1]["fwd"] > 0 and "band_m" in p["poses"][-1] and "clearances" not in p

    chk = c.call("plan_check", waypoints=[[5, 0], [10, 0], [20, 0], [30, 0]], target_speed=6, ticks=60)
    assert chk["ok"], chk
    cl = chk["plan_check"]["clearances"]
    lead = next(r for r in cl if r["id"] == 2)          # the fake lead car: 20 m ahead, 5 m to the right
    assert lead["min_clearance_m"] > 0 and lead["worst_case_m"] <= lead["min_clearance_m"]

    c.call("apply_plan", waypoints=[[5, 0], [10, 0], [20, 0]], ticks=5, target_speed=4)
    rows = [json.loads(l) for l in (s.out / "predictions.jsonl").read_text().splitlines()]
    assert len(rows) == 1 and len(rows[0]["poses"]) == 5
    _finish(s)


def test_extras_are_off_by_default_and_never_reach_sensors(make_session, tmp_path):
    s = make_session("privileged")
    obs = s.c.call("percept")["observe"]
    assert "rules" not in obs and "type" not in obs["actors_nearest"][0]
    assert s.c.call("predict", waypoints=[[5, 0], [10, 0]])["ok"] is False
    _finish(s)
    s = make_session("sensors", privileged_extras=_extras_cfg(tmp_path))
    obs = s.c.call("percept")["observe"]
    assert "rules" not in obs and "scenarios" not in obs
    r = s.c.call("plan_check", waypoints=[[5, 0], [10, 0]])
    assert r["ok"] is False and "access=sensors" in r["error"]
    assert s.c.call("predict", waypoints=[[5, 0], [10, 0]])["ok"] is False
    _finish(s)


def test_info_extras_on_camera_tracks(make_session, tmp_path):
    ex = _extras_cfg(tmp_path)
    s = make_session("sensors", privileged_extras={**ex, "enabled": False}, info_extras={"enabled": True})
    c = s.c
    obs = c.call("percept")["observe"]
    assert {"stop_sign", "traffic_light", "min_speed"} <= set(obs["rules"]) and isinstance(obs["scenarios"], list)
    assert isinstance(obs["rules"]["stop_sign"], str)            # text only: no live scorer state
    assert c.call("predict", waypoints=[[5, 0], [10, 0], [20, 0]], target_speed=6, ticks=40)["ok"]
    r = c.call("plan_check", waypoints=[[5, 0], [10, 0], [20, 0]], target_speed=6, ticks=40)
    assert r["ok"] is False and "obstacles" in r["error"]       # nothing from ground truth
    # an obstacle 2 m to the left of the path binds; the advice is to move right
    chk = c.call("plan_check", waypoints=[[5, 0], [10, 0], [20, 0]], target_speed=6, ticks=120,
                 obstacles=[[12, 2.0, 4.0, 1.8]])
    assert chk["ok"], chk
    pc = chk["plan_check"]
    row = pc["clearances"][0]
    assert row["side"] == "left" and row["trend"] == "closing"
    assert pc["binding"] and "RIGHT" in pc["binding"], pc["clearances"]
    pinch = c.call("plan_check", waypoints=[[5, 0], [10, 0], [20, 0]], target_speed=6, ticks=120,
                   obstacles=[[12, 2.0, 4.0, 1.8], [12, -2.0, 4.0, 1.8]])["plan_check"]
    assert pinch["binding"].startswith("pinched")
    _finish(s)


def test_notices_for_speed_band_and_negative_throttle(make_session, tmp_path):
    s = make_session("privileged", pid={**yaml.safe_load((REPO / "carla_probe" / "config.yaml").read_text())["pid"], "avoid_speed_band": [5.0, 6.7]})
    c = s.c
    r = c.call("apply_plan", waypoints=[[5, 0], [10, 0], [20, 0]], ticks=2, target_speed=6.0, why="t")
    assert any("gearbox" in n for n in r["result"].get("notices", [])), r["result"]
    r = c.call("set_control", steer=0.0, throttle=-0.5, brake=0.0, ticks=1, why="t")
    assert any("no reverse" in n for n in r["result"].get("notices", [])), r["result"]
    r = c.call("set_control", steer=0.0, throttle=0.2, brake=0.0, ticks=1, why="t")
    assert "notices" not in r["result"]
    _finish(s)


def test_behaviour_reader_survives_nested_actor_lists():
    import types as _t
    from carla_probe import privileged_extras as px
    a1 = _t.SimpleNamespace(id=11, type_id="vehicle.audi.tt")
    a2 = _t.SimpleNamespace(id=12, type_id="static.prop.constructioncone")

    class ActorFlow:                         # name matters: the reader keys on the node class
        def __init__(self):
            self._actor_list = [[a1], [a2, None]]
            self._is_constant_velocity_active = True

    class Tree:
        def iterate(self):
            yield ActorFlow()

    sc = _t.SimpleNamespace(config=_t.SimpleNamespace(type="ConstructionObstacleTwoWays"), behavior_tree=Tree(),
                            other_actors=[[a2]])
    beh = px.actor_behaviours(_t.SimpleNamespace(list_scenarios=[sc]))
    assert beh[11]["reacts_to_you"].startswith("no") and 12 in beh
    assert px.actor_behaviours(_t.SimpleNamespace(list_scenarios=[_t.SimpleNamespace(behavior_tree=1 / 1)])) == {}
