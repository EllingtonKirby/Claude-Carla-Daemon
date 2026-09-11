"""CARLA probe daemon: drive a Bench2Drive route from an LLM (or a human) over a JSON-lines socket.

``carla_probe.client`` is pure stdlib. ``carla_probe.agent`` needs the simulation environment
(carla, leaderboard, srunner), so it is deliberately not imported here.
"""

__version__ = "0.1.0"
