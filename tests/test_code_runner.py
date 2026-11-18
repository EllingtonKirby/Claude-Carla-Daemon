"""The code tool's confinement, probed the way a driver's script would reach a privileged file.

Each case runs a real script through `python -m carla_probe.code_runner` in a mock session: a
frames/ dir the driver may read, privileged files beside it that it may not, and a code dir. A case
that starts passing a read it should refuse is a leak into a benchmark run.
"""
import json
import os
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]


@pytest.fixture
def sess(tmp_path):
    s = tmp_path / "session"
    (s / "frames" / "frame_00001").mkdir(parents=True)
    (s / "session.json").write_text(json.dumps({"tm_seed": 0, "access": "sensors"}))
    (s / "result.json").write_text("{}")
    cv2.imwrite(str(s / "frames" / "frame_00001" / "CAM_FRONT.jpg"), np.full((8, 16, 3), 200, np.uint8))
    cv2.imwrite(str(s / "secret.jpg"), np.zeros((8, 16, 3), np.uint8))
    np.save(s / "secret.npy", np.ones(3))
    code = tmp_path / "code"
    code.mkdir()
    return s, code


def run(sess, body, extra_env=None):
    s, code = sess
    (code / "t.py").write_text(body)
    env = {**os.environ, "CARLA_PROBE_FRAMES": str(s / "frames"), "CARLA_PROBE_CODE_DIR": str(code),
           "PYTHONPATH": str(REPO), **(extra_env or {})}
    env.pop("CARLA_PROBE_PERCEPTION_ADDR", None) if not extra_env else None
    p = subprocess.run([sys.executable, "-m", "carla_probe.code_runner", str(code / "t.py")],
                       capture_output=True, text=True, env=env, cwd="/", timeout=60)
    return p.returncode, p.stdout


ALLOWED = [
    "import cv2; print(cv2.imread(F + '/frame_00001/CAM_FRONT.jpg').shape)",
    "import os; print(os.listdir(F))",
    "open('mine.txt', 'w').write('x'); print(open('mine.txt').read())",
    "import numpy as np; np.save('a.npy', np.ones(2)); print(np.load('a.npy').sum())",
    "print(len(open(F + '/frame_00001/../frame_00001/CAM_FRONT.jpg', 'rb').read()) > 0)",
]
DENIED = [
    "print(open(S + '/session.json').read())",
    "print(open(S + '/result.json').read())",
    "import os; print(os.listdir(S))",
    "import os; print(os.open(S + '/session.json', os.O_RDONLY))",
    "import pathlib; print(pathlib.Path(S + '/session.json').read_text())",
    "import io; print(io.FileIO(S + '/session.json').read())",
    "import numpy as np; print(np.load(S + '/secret.npy'))",
    "import numpy as np; print(np.fromfile(S + '/session.json', dtype=np.uint8))",
    "import numpy as np; print(np.memmap(S + '/secret.npy', mode='r'))",
    "import cv2; print(cv2.imread(S + '/secret.jpg').shape)",
    "import cv2; print(cv2.VideoCapture(S + '/secret.jpg').isOpened())",
    "print(open(F + '/../session.json').read())",          # traversal out of frames/
    "open('/tmp/leak_' + str(id(0)), 'w').write('x')",     # writing outside the code dir
    "open(F + '/frame_00001/planted.txt', 'w').write('x')",  # frames are read-only
    "import os; os.system('cat ' + S + '/session.json')",
    "import subprocess; subprocess.run(['ls'])",
    "import os; print(os.popen('ls').read())",
    "import socket; socket.create_connection(('127.0.0.1', 8790))",
    "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8790')",
    "import ctypes; ctypes.CDLL('libc.so.6')",
    "import ctypes; ctypes.CDLL('/usr/lib/x86_64-linux-gnu/libc.so.6')",
    "print(open('/etc/hostname').read())",
]


@pytest.mark.parametrize("stmt", ALLOWED)
def test_allowed(sess, stmt):
    s, _ = sess
    rc, out = run(sess, f"F = {str(s / 'frames')!r}\nS = {str(s)!r}\n{stmt}\n")
    assert rc == 0 and "denied" not in out, out


@pytest.mark.parametrize("stmt", DENIED)
def test_denied(sess, stmt):
    s, _ = sess
    rc, out = run(sess, f"F = {str(s / 'frames')!r}\nS = {str(s)!r}\n{stmt}\n")
    assert rc != 0, out
    assert "tm_seed" not in out and "denied" in out, out


def test_output_is_truncated(sess):
    rc, out = run(sess, "print('x' * 100000)\n")
    assert rc == 0 and len(out) < 8000 and "chars cut" in out


def test_script_must_live_in_the_code_dir(sess, tmp_path):
    s, code = sess
    other = tmp_path / "elsewhere.py"
    other.write_text("print('ran')")
    env = {**os.environ, "CARLA_PROBE_FRAMES": str(s / "frames"), "CARLA_PROBE_CODE_DIR": str(code), "PYTHONPATH": str(REPO)}
    p = subprocess.run([sys.executable, "-m", "carla_probe.code_runner", str(other)], capture_output=True, text=True, env=env)
    assert p.returncode == 2 and "ran" not in p.stdout


def test_perception_address_is_the_only_one_allowed(sess):
    """With a service address configured, connecting there passes the hook (and fails only because
    nothing listens), while any other address is still refused."""
    s, _ = sess
    body = ("import socket\n"
            "try:\n    socket.create_connection(('127.0.0.1', 1), timeout=1)\nexcept ConnectionRefusedError:\n    print('reached')\n"
            "socket.create_connection(('127.0.0.1', 8790), timeout=1)\n")
    rc, out = run(sess, body, {"CARLA_PROBE_PERCEPTION_ADDR": "127.0.0.1:1"})
    assert "reached" in out and "denied" in out and rc != 0, out
