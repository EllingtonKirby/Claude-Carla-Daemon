"""JSON-lines TCP command server for the CARLA probe daemon.

Runs INSIDE the leaderboard agent process (see ``agent.py``): the agent blocks in ``recv()`` while
paused, which freezes the whole synchronous-mode simulation. That blocking is the daemon's core
mechanism, not a bug. One client at a time; a dropped client just puts the server back into
``accept`` (the sim stays paused, nothing is lost).

Protocol: one JSON object per line in, one JSON object per line out. Non-advancing commands are
answered immediately; time-advancing commands are answered with a single message once the sim
pauses again.
"""

from __future__ import annotations

import json
import socket
from typing import Any, Dict, Optional


class CommandServer:
    def __init__(self, host: str, port: int) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(1)
        self._conn: Optional[socket.socket] = None
        self._rfile = None
        print(f"[PROBE] command server listening on {host}:{port}", flush=True)

    @property
    def port(self) -> int:
        return int(self._listener.getsockname()[1])

    def _drop_client(self) -> None:
        if self._rfile is not None:
            try:
                self._rfile.close()
            except OSError:
                pass
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
        self._conn, self._rfile = None, None

    def recv(self) -> Dict[str, Any]:
        """Block until the next parsed command arrives (accepting a client first if needed)."""
        while True:
            if self._conn is None:
                conn, addr = self._listener.accept()
                self._conn = conn
                self._rfile = conn.makefile("r")
                print(f"[PROBE] client connected from {addr[0]}:{addr[1]}", flush=True)
            assert self._rfile is not None
            try:
                line = self._rfile.readline()
            except OSError:
                line = ""
            if not line:  # client hung up: back to accept, sim stays paused
                print("[PROBE] client disconnected", flush=True)
                self._drop_client()
                continue
            line = line.strip()
            if not line:
                continue
            try:
                cmd = json.loads(line)
            except json.JSONDecodeError as e:
                self.send({"ok": False, "error": f"bad json: {e}"})
                continue
            if not isinstance(cmd, dict) or "cmd" not in cmd:
                self.send({"ok": False, "error": "expected a json object with a 'cmd' key"})
                continue
            return cmd

    def send(self, obj: Dict[str, Any]) -> None:
        """Best-effort reply to the current client (silently dropped if it went away)."""
        if self._conn is None:
            return
        try:
            self._conn.sendall((json.dumps(obj) + "\n").encode())
        except OSError:
            self._drop_client()

    def close(self) -> None:
        self._drop_client()
        try:
            self._listener.close()
        except OSError:
            pass
