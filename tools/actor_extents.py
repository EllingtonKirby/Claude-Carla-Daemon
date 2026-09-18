#!/usr/bin/env python3
"""Body extents per CARLA blueprint, cached so clearance analysis can stay offline.

    python tools/actor_extents.py --rebuild --port 20401 --gpu 0      # needs a CARLA server
    python tools/actor_extents.py                                     # print the cache

Why this exists: ``recorder_truth.json`` records where every actor WAS on every frame, but not how
big it is, and a distance between two centres is not a clearance -- which is the whole finding this
analysis exists to measure (docs/JOURNAL.md). A blueprint's bounding box is a constant, so it is
measured once against a live server and committed as ``tools/actor_extents.json``; everything
downstream then runs with no simulator.

Extents are metres from the ACTOR ORIGIN, split front/rear because the box centre is not the origin
(``bounding_box.location``). That asymmetry is small on a car and large on nothing we measure, but
carrying it costs nothing and dropping it would be a silent bias.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Optional

CACHE = Path(__file__).resolve().parent / "actor_extents.json"

FALLBACK = {"front_m": 2.4, "rear_m": 2.4, "half_width_m": 1.0}
"""Used, with a warning, for a type the cache does not know. A mid-size car: wrong for a cone and
wrong for a truck, so an analysis that leans on it is reporting the fallback, not the actor."""


def load(path: Optional[Path] = None) -> Dict[str, Dict[str, float]]:
    path = path or CACHE
    if not path.exists():
        return {}
    return json.loads(path.read_text())


def extent_of(table: Dict[str, Dict[str, float]], type_id: str,
              unknown: Optional[set] = None) -> Dict[str, float]:
    got = table.get(type_id)
    if got is None:
        if unknown is not None:
            unknown.add(type_id)
        return dict(FALLBACK)
    return got


def rebuild(type_ids, port: int, gpu: int, path: Optional[Path] = None) -> Dict[str, Dict[str, float]]:
    """Spawn each blueprint once and read its bounding box. Needs a CARLA server."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from replay_sim import connect  # noqa: E402  (imports carla)

    client, _proc = connect(port, gpu)
    client.set_timeout(60.0)
    world = client.get_world()
    library = world.get_blueprint_library()
    spawns = world.get_map().get_spawn_points()
    out = load(path)
    for i, type_id in enumerate(sorted(set(type_ids))):
        try:
            bp = library.find(type_id)
        except (IndexError, RuntimeError):
            print(f"  no blueprint: {type_id}", flush=True)
            continue
        actor = None
        for tf in spawns[i % len(spawns):] + spawns[:i % len(spawns)]:
            actor = world.try_spawn_actor(bp, tf)
            if actor is not None:
                break
        if actor is None:
            print(f"  could not spawn: {type_id}", flush=True)
            continue
        bb = actor.bounding_box
        out[type_id] = {"front_m": round(bb.location.x + bb.extent.x, 4),
                        "rear_m": round(bb.extent.x - bb.location.x, 4),
                        "half_width_m": round(bb.extent.y, 4)}
        print(f"  {type_id}: {out[type_id]}", flush=True)
        actor.destroy()
    (path or CACHE).write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rebuild", action="store_true", help="measure against a live CARLA server")
    ap.add_argument("--types", nargs="*", default=None, metavar="TYPE_ID",
                    help="blueprints to measure (default: every type already in the cache)")
    ap.add_argument("--port", type=int, default=20401)
    ap.add_argument("--gpu", type=int, default=0)
    args = ap.parse_args(argv)
    if args.rebuild:
        types = args.types or list(load())
        if not types:
            print("nothing to measure: pass --types", file=sys.stderr)
            return 2
        rebuild(types, args.port, args.gpu)
    table = load()
    print(f"{len(table)} blueprints in {CACHE}")
    for k in sorted(table):
        v = table[k]
        print("  %-38s front %5.2f  rear %5.2f  half_width %5.2f"
              % (k, v["front_m"], v["rear_m"], v["half_width_m"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
