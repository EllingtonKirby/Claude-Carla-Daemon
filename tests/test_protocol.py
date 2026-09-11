import json
import socket
import threading

from carla_probe.client import ProbeClient, parse_kv
from carla_probe.server import CommandServer


def _echo_loop(server: CommandServer, n: int) -> None:
    for _ in range(n):
        cmd = server.recv()
        server.send({"ok": True, "echo": cmd})


def test_roundtrip_and_reconnect():
    server = CommandServer("127.0.0.1", 0)
    t = threading.Thread(target=_echo_loop, args=(server, 3), daemon=True)
    t.start()
    addr = f"127.0.0.1:{server.port}"

    c = ProbeClient(addr)
    assert c.call("tick", n=5) == {"ok": True, "echo": {"cmd": "tick", "n": 5}}
    c.close()

    # A dropped client puts the server back into accept; the next client carries on.
    c2 = ProbeClient(addr)
    assert c2.call("status")["echo"] == {"cmd": "status"}
    assert c2.call("apply_plan", waypoints=[[1, 0], [2, 0]], ticks=3)["echo"]["waypoints"] == [[1, 0], [2, 0]]
    c2.close()
    t.join(timeout=5)
    server.close()


def test_bad_json_gets_error_then_recovers():
    server = CommandServer("127.0.0.1", 0)
    t = threading.Thread(target=_echo_loop, args=(server, 1), daemon=True)
    t.start()
    sock = socket.create_connection(("127.0.0.1", server.port))
    rfile = sock.makefile("r")
    sock.sendall(b"{not json\n")
    assert json.loads(rfile.readline())["ok"] is False
    sock.sendall(b'{"nocmd": 1}\n')
    assert "cmd" in json.loads(rfile.readline())["error"]
    sock.sendall(b'{"cmd": "status"}\n')
    assert json.loads(rfile.readline())["ok"] is True
    sock.close()
    t.join(timeout=5)
    server.close()


def test_parse_kv_json_and_strings():
    assert parse_kv(["n=3", "event=stopped", "w=[[1,2]]", "x=null"]) == {
        "n": 3, "event": "stopped", "w": [[1, 2]], "x": None,
    }
