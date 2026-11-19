"""What the privileged driver is told beyond positions: who each actor is, how it will behave, the
rules as the scorer applies them, and the scenarios on the route. (config `privileged_extras`)

Why: the 2026-11-18 privileged corpus lost every point it lost with the other road user in view.
It held a left turn against an oncoming flow that, by construction, never brakes for anyone; it
took an ambulance for a speeding car because the view said only "vehicle"; it stopped 7.4 m short of
the zone where the scorer checks a stop. All of that is in the scenario and the scorer, and none of
it reached the driver. This module reads it from the running RouteScenario -- the behaviour tree,
the criteria and the scenario configs -- and never guesses.

Privileged only. Nothing here may reach a sensors or dense session.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

# What the ego has to DO in each scenario type. Written from the scenario classes in
# third_party/Bench2Drive/scenario_runner/srunner/scenarios (their docstrings are quoted alongside).
SCENARIO_TASKS = {
    "NonSignalizedJunctionLeftTurn": "turn left at a junction without lights across an oncoming flow of vehicles",
    "SignalizedJunctionLeftTurn": "turn left at a traffic light across an oncoming flow of vehicles",
    "SignalizedJunctionLeftTurnEnterFlow": "turn left at a traffic light into a road with a flow of vehicles",
    "NonSignalizedJunctionRightTurn": "turn right at a junction without lights into a flow of vehicles",
    "SignalizedJunctionRightTurn": "turn right at a traffic light into a flow of vehicles",
    "ConstructionObstacleTwoWays": "pass a construction zone blocking your lane by using the opposite lane, which has oncoming traffic",
    "ConstructionObstacle": "pass a construction zone blocking your lane",
    "YieldToEmergencyVehicle": "an emergency vehicle comes up behind you: leave its lane so it can pass",
    "HazardAtSideLane": "bicycles ride at the edge of your lane ahead: pass them with clearance",
    "HazardAtSideLaneTwoWays": "bicycles ride at the edge of your lane ahead, with oncoming traffic: pass them with clearance",
    "ParkingExit": "you start parked: pull out into the lane when it is clear",
    "ParkingCrossingPedestrian": "a pedestrian steps out from between parked cars and crosses your lane",
    "StaticCutIn": "a vehicle stopped in the next lane cuts into your lane ahead of you",
    "BlockedIntersection": "a vehicle stops in the junction across your path after the turn: wait behind it",
    "SequentialLaneChange": "the route needs several lane changes in a row",
    "Accident": "an accident blocks your lane ahead: go around it",
    "AccidentTwoWays": "an accident blocks your lane ahead: go around it through the oncoming lane",
    "ParkedObstacle": "a parked vehicle blocks part of your lane ahead: go around it",
    "ParkedObstacleTwoWays": "a parked vehicle blocks part of your lane ahead: go around it through the oncoming lane",
    "VehicleOpensDoorTwoWays": "a parked vehicle opens its door into your lane: go around it",
    "DynamicObjectCrossing": "a pedestrian or cyclist crosses in front of you from behind an obstacle",
    "PedestrianCrossing": "pedestrians cross at a junction you drive through",
    "VehicleTurningRoute": "a cyclist crosses your path as you turn",
    "VehicleTurningRoutePedestrian": "a pedestrian crosses your path as you turn",
    "OppositeVehicleRunningRedLight": "a vehicle runs a red light across your path at a junction",
    "OppositeVehicleTakingPriority": "a vehicle crosses your path at a junction without yielding",
    "EnterActorFlow": "merge into a flow of vehicles",
    "EnterActorFlowV2": "merge into a flow of vehicles",
    "MergerIntoSlowTraffic": "merge into a lane of slow traffic",
    "MergerIntoSlowTrafficV2": "merge into a lane of slow traffic",
    "HighwayExit": "leave the highway across a flow of vehicles",
    "HighwayCutIn": "a vehicle on an on-ramp cuts into your lane",
    "InterurbanActorFlow": "turn across a flow of vehicles on an interurban road",
    "InterurbanAdvancedActorFlow": "merge across a flow of vehicles on an interurban road",
    "InvadingTurn": "oncoming vehicles cut into your lane on a bend: keep to the edge of your lane",
    "ControlLoss": "your vehicle may lose grip on debris",
    "HardBreakRoute": "the traffic ahead brakes hard",
    "BackgroundActivityParametrizer": "the background traffic changes density",
    "PriorityAtJunction": "you have priority at the next junction",
    "CrossingBicycleFlow": "a flow of bicycles crosses your turn",
    "T_Junction": "turn at a T-junction",
}

# How each atomic behaviour that moves a scenario actor treats the ego. Read from
# srunner/scenariomanager/scenarioatomics/atomic_behaviors.py: ActorFlow-type flows set
# ignore_lights/ignore_signs to 100% and, with constant velocity on, ignore_vehicles 100% too.
_BEHAVIOURS = {
    "AdaptiveConstantVelocityAgentBehavior": ("holds a set speed but slows for vehicles ahead of it", "yes, from behind: it slows when you are ahead of it"),
    "ConstantVelocityAgentBehavior": ("drives a fixed route at a set speed", "unknown: check before relying on it"),
    "BasicAgentBehavior": ("drives a route, obeys lights, brakes for vehicles ahead", "yes, if you are ahead of it in its lane"),
    "KeepVelocity": ("holds its speed whatever happens", "no"),
    "WaypointFollower": ("follows fixed waypoints at a set speed", "no"),
    "ActorTransformSetter": ("placed by the scenario", "n/a"),
    "HandBrakeVehicle": ("held stationary", "no: it does not move"),
}
_FLOWS = ("ActorFlow", "OppositeActorFlow", "BicycleFlow", "InvadingActorFlow", "EnterActorFlow")


def _iterate(node):
    try:
        yield from node.iterate()
    except Exception:
        return


def _flat_actors(x) -> List[Any]:
    """Actors held by a node, flattened: some scenarios keep lists of lists (ConstructionObstacleTwoWays
    crashed the daemon on 2026-11-19 with `'list' object has no attribute 'id'`)."""
    if x is None:
        return []
    if isinstance(x, (list, tuple, set)):
        return [a for item in x for a in _flat_actors(item)]
    return [x] if hasattr(x, "id") and hasattr(x, "type_id") else []


def actor_behaviours(scenario) -> Dict[int, Dict[str, str]]:
    """actor id -> {scenario, behaviour, reacts}, read from the scenarios' behaviour trees. Never
    raises: this only enriches the driver's view, and a reporting error must not end a route."""
    try:
        return _actor_behaviours(scenario)
    except Exception as exc:  # noqa: BLE001
        print(f"[PROBE] actor_behaviours failed: {exc!r}", flush=True)
        return {}


def _actor_behaviours(scenario) -> Dict[int, Dict[str, str]]:
    out: Dict[int, Dict[str, str]] = {}
    if scenario is None:
        return out
    for sc in list(getattr(scenario, "list_scenarios", []) or []):
        name = getattr(getattr(sc, "config", None), "type", None) or type(sc).__name__
        tree = getattr(sc, "behavior_tree", None) or getattr(sc, "scenario_tree", None)
        for node in _iterate(tree) if tree is not None else []:
            cls = type(node).__name__
            actors: List[Any] = []
            if hasattr(node, "_actor_list"):
                actors = _flat_actors(node._actor_list)
            elif getattr(node, "_actor", None) is not None:
                actors = _flat_actors(node._actor)
            if not actors:
                continue
            if cls in _FLOWS or hasattr(node, "_actor_list"):
                if getattr(node, "_is_constant_velocity_active", False):
                    desc = ("flow vehicle at constant speed: ignores other vehicles, traffic lights and signs",
                            "no: it will NOT brake for you")
                else:
                    desc = ("flow vehicle on autopilot: ignores traffic lights and signs, brakes for vehicles in its lane",
                            "partly: it brakes for a vehicle in its lane ahead, not for one crossing")
            elif cls in _BEHAVIOURS:
                desc = _BEHAVIOURS[cls]
            else:
                # Conditions (InTriggerDistanceToVehicle, DriveDistance, ...) also hold an actor; they
                # say nothing about how it drives, so they only fill in when nothing better is known.
                desc = None
            for a in actors:
                known = out.get(a.id)
                if desc is not None and (known is None or known.get("_weak") or known["behaviour"].startswith("placed")):
                    out[a.id] = {"scenario": name, "behaviour": desc[0], "reacts_to_you": desc[1]}
                elif desc is None and known is None:
                    out[a.id] = {"scenario": name, "behaviour": f"scripted by the scenario ({cls})",
                                 "reacts_to_you": "unknown", "_weak": True}
        for a in _flat_actors(getattr(sc, "other_actors", [])):
            if a.id not in out:
                out[a.id] = {"scenario": name, "behaviour": "scenario actor with no driving behaviour (parked, static or waiting for a trigger)",
                             "reacts_to_you": "n/a"}
    return out


def describe_actor(actor, behaviours: Dict[int, Dict[str, str]]) -> Dict[str, Any]:
    tid = actor.type_id
    role = actor.attributes.get("role_name", "") if hasattr(actor, "attributes") else ""
    # Only the scenario's own emergency vehicle is one the scorer expects you to yield to; a police
    # car in the background traffic has no lights on and no right of way.
    emergency = role == "scenario" and any(w in tid for w in ("ambulance", "firetruck", "police"))
    info = {"type": tid, "role": role or "unknown", "emergency": emergency}
    if actor.id in behaviours:
        info.update({k: v for k, v in behaviours[actor.id].items() if not k.startswith("_")})
    elif role == "background":
        info.update({"behaviour": "background traffic (traffic manager): keeps its lane, brakes for vehicles ahead of it",
                     "reacts_to_you": "partly: it brakes if you are ahead of it in its lane; it does not yield to you elsewhere"})
    elif tid.startswith("walker."):
        info.update({"behaviour": "pedestrian", "reacts_to_you": "unknown"})
    elif tid.startswith("static."):
        info.update({"behaviour": "static object: does not move", "reacts_to_you": "n/a"})
    return info


def stop_check(scenario) -> Optional[Dict[str, Any]]:
    """The scorer's own stop-sign state (RunningStopTest), read live. Never raises."""
    try:
        return _stop_check(scenario)
    except Exception as exc:  # noqa: BLE001
        return {"error": repr(exc)[:200]}


def _stop_check(scenario) -> Optional[Dict[str, Any]]:
    tree = getattr(scenario, "criteria_tree", None) if scenario is not None else None
    for node in _iterate(tree) if tree is not None else []:
        if type(node).__name__ == "RunningStopTest":
            target = getattr(node, "_target_stop_sign", None)
            return {"engaged": target is not None, "satisfied": bool(getattr(node, "_stop_completed", False)),
                    "stop_id": int(target.id) if target is not None else None}
    return None


def scenarios_on_route(scenario, ahead_fn, lookahead_m: float) -> List[Dict[str, Any]]:
    """Each scenario of the route with its trigger distance along the route ahead (None = behind
    or far off the route), whether it has been built yet, its parameters and what to do."""
    if scenario is None:
        return []
    built = {getattr(getattr(s, "config", None), "name", None): s for s in getattr(scenario, "list_scenarios", []) or []}
    cfgs = list(getattr(getattr(scenario, "config", None), "scenario_configs", None) or [])
    if not cfgs:
        cfgs = [getattr(s, "config", None) for s in built.values() if getattr(s, "config", None) is not None]
    out = []
    for c in cfgs:
        loc = c.trigger_points[0].location if getattr(c, "trigger_points", None) else None
        ahead = ahead_fn((loc.x, loc.y)) if loc is not None else None
        if ahead is not None and ahead > lookahead_m:
            continue
        inst = built.get(c.name)
        params = {k: (v.get("value") if isinstance(v, dict) and "value" in v else v)
                  for k, v in (getattr(c, "other_parameters", {}) or {}).items()}
        out.append({"type": c.type, "name": c.name, "trigger_ahead_m": None if ahead is None else round(ahead, 1),
                    "started": inst is not None, "task": SCENARIO_TASKS.get(c.type, ""),
                    "about": ((type(inst).__doc__ or "").strip().split("\n\n")[0].replace("\n", " ")[:300] if inst is not None else ""),
                    "parameters": params})
    return out


RULES_TEXT = {
    "stop_sign": ("a stop counts only if your speed drops below 0.1 m/s while your origin is within 4 m of the "
                  "stop's trigger centre (stop_signs_on_route.trigger_ego) and on its lane; stopping earlier "
                  "does not count, and leaving the zone unstopped is a stop infraction"),
    "traffic_light": "do not cross the stop line (stop_line_ego) while your light is red or yellow",
    "emergency_vehicle": ("yielding to an emergency vehicle means leaving the lane it is in so it can pass; "
                          "slowing or holding your lane in front of it is a failure to yield"),
    "collisions": "any contact with a vehicle, pedestrian or object is scored, including at walking speed",
}
