"""Client CLI for the CARLA probe daemon.

One-shot:
    python -m carla_probe.client [HOST:PORT] CMD [key=value ...]
REPL:
    python -m carla_probe.client [HOST:PORT] repl
Wait until a freshly launched session answers (CARLA + route load takes 1-3 min):
    python -m carla_probe.client [HOST:PORT] --wait 600 status

HOST:PORT defaults to $CARLA_PROBE_ADDR, else 127.0.0.1:8790. Values are parsed as JSON when
possible (numbers, lists, null, booleans), else kept as strings:
    python -m carla_probe.client tick n=40 why="junction clear both ways; keep the current plan"
    python -m carla_probe.client run_until_event event=stopped why="car stopped ahead; brake to a stop"
    python -m carla_probe.client apply_plan 'waypoints=[[3,0],[6,0],[9,0],[12,0]]' target_speed=5 ticks=40 \
        why="lane ahead clear for 40 m; follow it at 5 m/s"

Driving commands (apply_plan, set_control, hold, tick, run_until_*, shutdown) need why="..." when the
session's config has require_justification: true (the default).

Each command prints exactly one JSON response. Time-advancing commands block until the sim pauses
again: that wait IS the sim running. Interrupting abandons the reply, not the advance (the daemon
keeps going); reconnect and `status` to catch up, do NOT re-issue the advance.

Pure stdlib on purpose: runs from any Python >= 3.8 with no dependencies.
"""

from __future__ import annotations

import json
import os
import shlex
import socket
import sys
import time
from typing import Any, Dict, List, Tuple

DEFAULT_ADDR = "127.0.0.1:8790"


def parse_kv(args: List[str]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for arg in args:
        key, sep, raw = arg.partition("=")
        if not sep:
            raise ValueError(f"expected key=value, got {arg!r}")
        try:
            out[key] = json.loads(raw)
        except json.JSONDecodeError:
            out[key] = raw
    return out


def parse_addr(addr: str) -> Tuple[str, int]:
    host, _, port = addr.rpartition(":")
    if not host or not port.isdigit():
        raise ValueError(f"expected HOST:PORT, got {addr!r}")
    return host, int(port)


class ProbeClient:
    """Minimal programmatic client: ``ProbeClient("127.0.0.1:8790").call("tick", n=20)``."""

    def __init__(self, addr: str = DEFAULT_ADDR, wait_s: float = 0.0) -> None:
        host, port = parse_addr(addr)
        deadline = time.monotonic() + wait_s
        while True:
            try:
                self._sock = socket.create_connection((host, port), timeout=5.0)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(2.0)
        self._sock.settimeout(None)  # advances legitimately take minutes
        self._rfile = self._sock.makefile("r")

    def call(self, cmd: str, **kwargs: Any) -> Dict[str, Any]:
        self._sock.sendall((json.dumps({"cmd": cmd, **kwargs}) + "\n").encode())
        line = self._rfile.readline()
        if not line:
            raise ConnectionError("daemon closed the connection without a reply; the route outcome is unknown")
        return json.loads(line)

    def close(self) -> None:
        self._rfile.close()
        self._sock.close()


def main(argv: List[str]) -> int:
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 2
    addr = os.environ.get("CARLA_PROBE_ADDR", DEFAULT_ADDR)
    if ":" in argv[0] and "=" not in argv[0]:
        addr, argv = argv[0], argv[1:]
    wait_s = 0.0
    if len(argv) >= 2 and argv[0] == "--wait":
        wait_s, argv = float(argv[1]), argv[2:]
    if not argv:
        print(__doc__)
        return 2

    try:
        client = ProbeClient(addr, wait_s=wait_s)
    except OSError as e:
        print(f"cannot connect to probe daemon at {addr}: {e}", file=sys.stderr)
        return 3

    if argv[0] == "repl":
        print("probe repl: '<cmd> [key=value ...]', 'quit' to exit (the daemon keeps running)")
        while True:
            try:
                line = input("probe> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if not line:
                continue
            if line in ("quit", "exit"):
                return 0
            parts = shlex.split(line)
            try:
                kwargs = parse_kv(parts[1:])
            except ValueError as e:
                print(e)
                continue
            print(json.dumps(client.call(parts[0], **kwargs), indent=2))

    try:
        kwargs = parse_kv(argv[1:])
    except ValueError as e:
        print(e, file=sys.stderr)
        return 2
    resp = client.call(argv[0], **kwargs)
    print(json.dumps(resp, indent=2))
    return 0 if resp.get("ok") else 1


def cli() -> None:
    sys.exit(main(sys.argv[1:]))


if __name__ == "__main__":
    cli()
