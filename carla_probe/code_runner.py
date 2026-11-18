"""Run a driver-written Python script inside a confined interpreter: the driver's code tool.

    python -m carla_probe.code_runner SCRIPT.py [ARGS...]

Why it exists: the driver is good at writing code, and most of what it gets wrong is metric
(docs/JOURNAL.md 2026-09-18: it reads a distance off a camera and spends the whole of it). A script
can do the arithmetic -- measure pixels, project through the camera model, threshold a depth map --
and hand back a few numbers instead of an image the driver has to eyeball.

What confines it. A benchmark driver must not read the session's privileged files (result.json,
evaluator.log, the recorder log, the config snapshot; benchmark/PROTOCOL.md rule 7). Claude Code's
permissions only gate the driver's own tool calls; a script it runs could open anything this uid
can. This container has no user namespaces and no bubblewrap, so the confinement is a PEP 578 audit
hook installed before the script runs. A hook cannot be removed once added, and it sees every
`open`, directory listing, process spawn, socket connect and native-library load the interpreter
makes on the script's behalf.

    reads     only under: the frames dir, the code dir, the Python installation, carla_probe/
    writes    only under: the code dir
    spawn     never (subprocess, os.system, exec*, fork, posix_spawn, pty)
    network   only to the perception service address, if one is configured
    native    ctypes may only load libraries from inside site-packages

The rules come from environment variables that run_eval.sh sets in the driver's settings.json:
CARLA_PROBE_FRAMES, CARLA_PROBE_CODE_DIR and optionally CARLA_PROBE_PERCEPTION_ADDR.

Limits, stated plainly: this stops an honest driver from reading a privileged file by accident or
from following a path it stumbled on. It is not a security boundary against an adversarial one --
C extensions already loaded can make syscalls the hook never sees. run_eval.sh's isolation check
asks a model to try the obvious routes through this tool and fails the run if any succeeds.

Output is the script's stdout and stderr, truncated to `max_output_chars`, so a script that prints
an image as text cannot blow up the driver's context.
"""
from __future__ import annotations

import io
import os
import runpy
import signal
import sys
import traceback
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

MAX_OUTPUT_CHARS = int(os.environ.get("CARLA_PROBE_CODE_MAX_OUTPUT", "6000"))
TIMEOUT_S = int(os.environ.get("CARLA_PROBE_CODE_TIMEOUT_S", "120"))

_SPAWN_EVENTS = {
    "subprocess.Popen", "os.system", "os.exec", "os.posix_spawn", "os.spawn", "os.fork",
    "os.forkpty", "pty.spawn", "os.startfile", "_posixsubprocess.fork_exec",
}
# Modules that only exist to spawn processes or poke at memory; importing them is refused so a
# script cannot reach the unaudited primitives underneath (_posixsubprocess.fork_exec predates the
# audit event on some builds).
_FORBIDDEN_IMPORTS = {"_posixsubprocess", "subprocess", "multiprocessing", "pty", "ctypes.util"}
_PATH_READ_EVENTS = {"os.listdir", "os.scandir", "glob.glob", "glob.glob/2", "os.walk", "shutil.copyfile",
                     "shutil.copytree", "shutil.make_archive"}
_PATH_WRITE_EVENTS = {"os.remove", "os.rmdir", "os.rename", "os.mkdir", "os.chmod", "os.chown",
                      "os.link", "os.symlink", "os.truncate", "os.utime", "shutil.rmtree", "shutil.move"}


class Confinement:
    """The access rules, separated from the hook so they can be unit-tested without installing it."""

    def __init__(self, read_roots: Iterable[str], write_roots: Iterable[str],
                 allowed_addr: Optional[Tuple[str, int]] = None, native_roots: Iterable[str] = ()) -> None:
        self.write_roots = [self._norm(p) for p in write_roots if p]
        self.native_roots = [self._norm(p) for p in native_roots if p]
        self.read_roots = [self._norm(p) for p in read_roots if p] + self.write_roots
        self.allowed_addr = allowed_addr

    @staticmethod
    def _norm(p: str) -> str:
        return os.path.realpath(os.path.abspath(os.fsdecode(p)))

    @staticmethod
    def _under(path: str, roots: List[str]) -> bool:
        return any(path == r or path.startswith(r.rstrip(os.sep) + os.sep) for r in roots)

    def may_read(self, path) -> bool:
        if isinstance(path, int):          # an fd the interpreter already holds
            return True
        p = self._norm(path)
        # Process and CPU introspection that numpy/opencv do at import; nothing about the session.
        if p.startswith(("/proc/self/", "/sys/devices/system/cpu", "/dev/null", "/dev/urandom")):
            return True
        return self._under(p, self.read_roots)

    def may_write(self, path) -> bool:
        if isinstance(path, int):
            return True
        p = self._norm(path)
        return p == "/dev/null" or self._under(p, self.write_roots)

    def check(self, event: str, args: tuple) -> None:
        if event == "open":
            path, mode = args[0], args[1]
            if path is None:
                return
            writing = isinstance(mode, str) and any(c in mode for c in "wax+")
            if isinstance(args[2], int) and not isinstance(mode, str):
                # os.open(path, flags): flags is args[2]
                writing = bool(args[2] & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC))
            ok = self.may_write(path) if writing else self.may_read(path)
            if not ok:
                raise PermissionError(f"code tool: denied: {'writing' if writing else 'reading'} {path!r} is outside "
                                      "the frames dir / your code dir")
        elif event in _PATH_READ_EVENTS:
            if args and isinstance(args[0], (str, bytes, os.PathLike)) and not self.may_read(args[0]):
                raise PermissionError(f"code tool: denied: listing {args[0]!r} is outside the frames dir / your code dir")
        elif event in _PATH_WRITE_EVENTS:
            for a in args[:2]:
                if isinstance(a, (str, bytes, os.PathLike)) and not self.may_write(a):
                    raise PermissionError(f"code tool: denied: modifying {a!r} is outside your code dir")
        elif event in _SPAWN_EVENTS or event.startswith("os.exec") or event.startswith("os.spawn"):
            raise PermissionError("code tool: denied: starting processes")
        elif event == "import":
            name = args[0]
            if name in _FORBIDDEN_IMPORTS or name.split(".")[0] in ("_posixsubprocess", "pty"):
                raise ImportError(f"code tool: denied: importing {name!r}")
        elif event == "socket.connect":
            addr = args[1]
            ok = (self.allowed_addr is not None and isinstance(addr, tuple)
                  and (addr[0], int(addr[1])) in {self.allowed_addr, ("localhost", self.allowed_addr[1])})
            if not ok:
                raise PermissionError(f"code tool: denied: network access to {addr!r}")
        elif event in ("socket.bind", "socket.sendto", "socket.getaddrinfo") :
            if event == "socket.getaddrinfo" and self.allowed_addr and args[0] in (self.allowed_addr[0], "localhost"):
                return
            raise PermissionError(f"code tool: denied: {event}")
        elif event == "ctypes.dlopen":
            # Absolute paths inside the Python installation only. A bare name ("libc.so.6") is
            # resolved by the dynamic loader, not against any root, and the code dir is writable.
            lib = args[0]
            name = os.fsdecode(lib) if isinstance(lib, (str, bytes)) else None
            if (name is None or not os.path.isabs(name) or self._under(self._norm(name), self.write_roots)
                    or not self._under(self._norm(name), self.native_roots)):
                raise PermissionError(f"code tool: denied: loading native library {lib!r}")
        elif event in ("sys.setprofile", "sys.settrace") :
            return
        elif event == "sys.addaudithook":
            return   # adding another hook cannot remove this one


def confinement_from_env() -> Confinement:
    frames = os.environ.get("CARLA_PROBE_FRAMES", "")
    code_dir = os.environ.get("CARLA_PROBE_CODE_DIR", "")
    if not frames or not code_dir:
        raise SystemExit("code tool: CARLA_PROBE_FRAMES and CARLA_PROBE_CODE_DIR must be set (run_eval.sh does)")
    pkg = str(Path(__file__).resolve().parent)
    prefixes = {sys.prefix, sys.base_prefix, sys.exec_prefix, sys.base_exec_prefix}
    # site-packages may live outside the prefixes (uv venvs symlink the interpreter in)
    prefixes.update(p for p in sys.path if p and "site-packages" in p)
    prefixes.update(os.path.dirname(os.path.realpath(sys.executable)) for _ in [0])
    real_base = os.path.dirname(os.path.dirname(os.path.realpath(sys.executable)))
    prefixes.add(real_base)
    addr = None
    raw = os.environ.get("CARLA_PROBE_PERCEPTION_ADDR", "").strip()
    if raw:
        host, port = raw.rsplit(":", 1)
        addr = (host, int(port))
    site = [p for p in sys.path if p and "site-packages" in p]
    return Confinement(read_roots=[frames, pkg, *prefixes], write_roots=[code_dir], allowed_addr=addr,
                       native_roots=site)


def _guard_native_readers(conf: Confinement) -> None:
    """Wrap the file readers that open paths in C, where the audit hook cannot see them.

    OpenCV's imread family and VideoCapture call fopen directly. Each is replaced by a wrapper that
    applies the same read rule first. Measured, not assumed: tests/test_code_runner.py probes each
    of these against a file outside the allowed roots.
    """
    try:
        import cv2
    except Exception:
        return

    def guarded(fn):
        def wrapper(path, *a, **k):
            if isinstance(path, (str, bytes, os.PathLike)) and not conf.may_read(path):
                raise PermissionError(f"code tool: denied: reading {path!r} is outside the frames dir / your code dir")
            return fn(path, *a, **k)
        wrapper.__doc__ = fn.__doc__
        return wrapper

    for name in ("imread", "imreadmulti", "imcount", "haveImageReader", "VideoCapture"):
        if hasattr(cv2, name):
            setattr(cv2, name, guarded(getattr(cv2, name)))


def _truncate(text: str) -> str:
    if len(text) <= MAX_OUTPUT_CHARS:
        return text
    half = MAX_OUTPUT_CHARS // 2
    return f"{text[:half]}\n... [{len(text) - MAX_OUTPUT_CHARS} chars cut: print less] ...\n{text[-half:]}"


def main(argv: Optional[List[str]] = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    script = os.path.realpath(argv[0])
    conf = confinement_from_env()
    if not conf.may_read(script) or not script.endswith(".py"):
        print(f"code tool: the script must be a .py file in your code dir ({os.environ.get('CARLA_PROBE_CODE_DIR')})")
        return 2
    # Pre-import what scripts commonly need BEFORE the hook, so their own import-time file access
    # (config files, plugin scans) is not misread as the script's. The script's own reads are what
    # the hook is for.
    for mod in ("numpy", "cv2", "json", "math", "urllib.request", "base64"):
        try:
            __import__(mod)
        except Exception:
            pass
    _guard_native_readers(conf)
    os.chdir(os.environ["CARLA_PROBE_CODE_DIR"])
    sys.path[:0] = [os.environ["CARLA_PROBE_CODE_DIR"]]
    sys.argv = [script, *argv[1:]]

    out = io.StringIO()
    real_stdout, real_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = out

    def _timeout(signum, frame):
        raise TimeoutError(f"code tool: script exceeded {TIMEOUT_S} s")

    signal.signal(signal.SIGALRM, _timeout)
    signal.alarm(TIMEOUT_S)
    sys.addaudithook(lambda event, args: conf.check(event, args))
    rc = 0
    try:
        runpy.run_path(script, run_name="__main__")
    except SystemExit as e:
        rc = int(e.code) if isinstance(e.code, int) else (0 if e.code is None else 1)
    except BaseException:
        traceback.print_exc(limit=6, file=out)
        rc = 1
    finally:
        signal.alarm(0)
        sys.stdout, sys.stderr = real_stdout, real_stderr
    real_stdout.write(_truncate(out.getvalue()))
    real_stdout.flush()
    return rc


if __name__ == "__main__":
    sys.exit(main())
