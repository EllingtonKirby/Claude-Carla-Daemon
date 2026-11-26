"""Capability-limited Codex driver for run_eval.sh, using a stdio MCP server."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from carla_probe.client import ProbeClient

REPO = Path(__file__).resolve().parents[1]
DOCS = {
    "driving_loop": REPO / ".claude/skills/drive-carla/SKILL.md",
    "driving_guide": REPO / "docs/OPERATOR_GUIDE.md",
    "protocol": REPO / "benchmark/PROTOCOL.md",
}
IMAGE_TYPES = {".jpg": "image/jpeg", ".png": "image/png"}
TEXT = {"type": "string"}
ARRAY = {"type": "array", "items": TEXT}


def tool(name, description, fields, required):
    return {"name": name, "description": description,
            "inputSchema": {"type": "object", "properties": fields, "required": required,
                            "additionalProperties": False}}


class DriverTools:
    def __init__(self, session, addr, code, perception, preflight=False):
        self.frames = (session / "frames").resolve()
        self.addr = addr
        self.code = code.resolve() if code else None
        self.perception = perception
        self.preflight = preflight
        self.artifacts = set()

    def tools(self):
        tools = [
            tool("probe", "Send a single CARLA command. Use status, percept, apply_plan, or help. "
                 "Pass parameters as an object. Every driving command needs an observation-based why.",
                 {"command": TEXT, "arguments": {"type": "object", "additionalProperties": True}},
                 ["command"]),
            tool("read", "Read driving_loop, driving_guide, protocol, or an artifact path returned "
                 "by the probe. Joins of frames_dir and the returned filenames are valid.",
                 {"path": TEXT}, ["path"]),
        ]
        if self.code:
            tools.extend([
                tool("write_script", "Write a Python file under the private code directory.",
                     {"path": TEXT, "source": TEXT}, ["path", "source"]),
                tool("run_script", "Run a Python script from the code directory with optional "
                     "arguments. The script can read frames and use the configured perception service.",
                     {"path": TEXT, "args": ARRAY}, ["path"]),
            ])
        return tools

    def remember(self, reply):
        paths = []
        if "frames_dir" in reply:
            paths.extend(Path(reply["frames_dir"]) / name for name in reply.get("files", []))
        if "file" in reply:
            paths.append(Path(reply["file"]))
        if "dump" in reply:
            dump = reply["dump"]
            paths.extend(Path(p) for p in [dump["state"], dump["bev"], *dump["cameras"]])
        for path in paths:
            resolved = path.resolve()
            if self.frames in resolved.parents:
                self.artifacts.add(resolved)

    def code_path(self, path):
        resolved = (self.code / path).resolve()
        if self.code not in resolved.parents:
            raise PermissionError("outside the code directory")
        return resolved

    def call(self, name, args):
        if name == "probe":
            command = args["command"]
            if self.preflight and command != "status":
                raise PermissionError("only status is enabled during preflight")
            arguments = args.get("arguments", {})
            if not isinstance(arguments, dict) or "cmd" in arguments:
                raise ValueError("arguments must be an object without a cmd field")
            client = ProbeClient(self.addr)
            try:
                reply = client.call(command, **arguments)
            finally:
                client.close()
            self.remember(reply)
            return content(json.dumps(reply, separators=(",", ":")))
        if name == "read":
            path = args["path"]
            if path in DOCS:
                return content(DOCS[path].read_text())
            if self.code and not Path(path).is_absolute():
                resolved = self.code_path(path)
                if resolved.suffix in {".py", ".md"} and resolved.is_file():
                    return content(resolved.read_text())
            resolved = Path(path).resolve()
            if self.frames not in resolved.parents or resolved not in self.artifacts:
                raise PermissionError("only driver documents and returned artifacts may be read")
            mime = IMAGE_TYPES.get(resolved.suffix)
            if mime:
                return [{"type": "image", "mimeType": mime,
                         "data": base64.b64encode(resolved.read_bytes()).decode("ascii")}]
            if resolved.suffix == ".json":
                return content(resolved.read_text())
            raise ValueError("use the code tool for the returned array files")
        if name in {"write_script", "run_script"} and self.code:
            path = self.code_path(args["path"])
            if path.suffix != ".py":
                raise ValueError("expected a .py path")
            if name == "write_script":
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(args["source"])
                return content("Script written")
            env = {
                "PATH": os.defpath, "PYTHONPATH": str(REPO), "PYTHONDONTWRITEBYTECODE": "1",
                "HOME": str(self.code), "CARLA_PROBE_FRAMES": str(self.frames),
                "CARLA_PROBE_CODE_DIR": str(self.code),
                "CARLA_PROBE_PERCEPTION_ADDR": self.perception,
            }
            result = subprocess.run([sys.executable, "-m", "carla_probe.code_runner", str(path),
                                     *args.get("args", [])], env=env, cwd=REPO, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=125)
            if result.returncode:
                raise ValueError(result.stdout or f"code runner exited {result.returncode}")
            return content(result.stdout)
        raise PermissionError("tool unavailable")


def content(text):
    return [{"type": "text", "text": text}]


def serve(driver):
    for line in sys.stdin:
        message = json.loads(line)
        if "id" not in message:
            continue
        method = message["method"]
        params = message.get("params", {})
        reply = {"jsonrpc": "2.0", "id": message["id"]}
        if method == "initialize":
            reply["result"] = {"protocolVersion": "2024-11-05", "capabilities": {"tools": {}},
                               "serverInfo": {"name": "carla", "version": "1"}}
        elif method == "tools/list":
            reply["result"] = {"tools": driver.tools()}
        elif method == "tools/call":
            try:
                reply["result"] = {"content": driver.call(params["name"], params.get("arguments") or {})}
            except PermissionError as error:
                reply["result"] = {"isError": True, "content": content(f"Denied: {error}")}
            except (OSError, ValueError, TypeError, KeyError, subprocess.TimeoutExpired) as error:
                reply["result"] = {"isError": True, "content": content(str(error))}
        elif method == "ping":
            reply["result"] = {}
        else:
            reply["error"] = {"code": -32601, "message": "Method not found"}
        print(json.dumps(reply), flush=True)


def config_for(args, preflight=False):
    command = ["-m", "carla_probe.codex_driver", "serve", str(args.session), args.addr]
    if args.code:
        command.extend(["--code", str(args.code), "--perception", args.perception])
    if preflight:
        command.append("--preflight")
    # JSON strings and string arrays are also valid TOML for these paths and arguments.
    return f"""web_search = "disabled"
cli_auth_credentials_store = "file"
suppress_unstable_features_warning = true

[agents]
enabled = false

[features]
shell_tool = false
unified_exec = false
multi_agent_v2 = false
apps = false
skip_host_skill_discovery = true

[mcp_servers.carla]
command = {json.dumps(sys.executable)}
args = {json.dumps(command)}
required = true
default_tools_approval_mode = "approve"
tool_timeout_sec = 300

[mcp_servers.carla.env]
PYTHONPATH = {json.dumps(str(REPO))}
PYTHONDONTWRITEBYTECODE = "1"
"""


def verify_preflight(path, code):
    """Check actual completed tool calls, including their success/denial results."""
    expected = {
        ("read", str(path.parent / "session.json")): False,
        ("read", "protocol"): True,
        ("probe", "status"): True,
    }
    if code:
        expected[("write_script", "../outside.py")] = False
    observed = set()
    completed = False
    for line in path.read_text().splitlines():
        row = json.loads(line)
        if row["type"] == "turn.completed":
            completed = True
        item = row.get("item", {})
        if row["type"] != "item.completed":
            continue
        if item["type"] in {"agent_message", "reasoning"}:
            continue
        if (item["type"] != "mcp_tool_call" or item["server"] != "carla"
                or item["tool"] not in {"read", "probe", "write_script"}):
            raise ValueError(f"unexpected tool in Codex preflight: {item['type']}")
        arguments = item["arguments"]
        key = item["tool"], arguments.get("command", arguments.get("path"))
        result = item.get("result") or {}
        contents = result.get("content", [])
        texts = [block.get("text", "") for block in contents if block.get("type") == "text"]
        denied = bool(texts) and texts[0].startswith("Denied:")
        if not denied and (item["status"] != "completed" or not texts):
            raise ValueError(f"tool did not return a verified preflight result: {item['tool']}")
        if item["tool"] == "probe" and not json.loads(texts[0])["ok"]:
            raise ValueError("probe status failed")
        if key not in expected or expected[key] == denied:
            raise ValueError(f"unexpected isolation result for {key!r}")
        observed.add(key)
    if not completed or observed != set(expected):
        raise ValueError(f"missing isolation preflight evidence: {observed!r}")


def drive(cmd, prompt, transcript, stderr, work, env, code):
    allowed = {"probe", "read"}
    if code:
        allowed.update(("write_script", "run_script"))
    with transcript.open("w") as out, stderr.open("w") as err, subprocess.Popen(
        cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, cwd=work, env=env, text=True,
    ) as process:
        try:
            process.stdin.write(prompt)
            process.stdin.close()
            for line in process.stdout:
                out.write(line)
                out.flush()
                row = json.loads(line)
                item = row.get("item", {})
                if row["type"] not in {"item.started", "item.completed"}:
                    continue
                if item["type"] in {"agent_message", "reasoning", "error"}:
                    continue
                if (item["type"] != "mcp_tool_call" or item["server"] != "carla"
                        or item["tool"] not in allowed):
                    raise ValueError(f"unexpected tool during Codex drive: {item['type']}")
            return process.wait()
        finally:
            if process.poll() is None:
                process.terminate()


def run(args):
    source_home = Path(os.environ.get("CODEX_HOME", Path.home() / ".codex"))
    executable = shutil.which("codex")
    with tempfile.TemporaryDirectory(prefix="probe-codex-") as temp:
        home = Path(temp)
        codex_home = home / ".codex"
        work = home / "work"
        codex_home.mkdir()
        work.mkdir()
        auth = source_home / "auth.json"
        if auth.is_file():
            shutil.copyfile(auth, codex_home / "auth.json")
            (codex_home / "auth.json").chmod(0o600)
        env = {"HOME": str(home), "CODEX_HOME": str(codex_home), "PATH": os.defpath,
               "TMPDIR": str(home), "LANG": "C.UTF-8"}
        if os.environ.get("OPENAI_API_KEY"):
            env["OPENAI_API_KEY"] = os.environ["OPENAI_API_KEY"]
        cmd = [executable, "-a", "never", "exec", "--sandbox", "read-only", "--ephemeral",
               "--skip-git-repo-check", "--ignore-rules", "--strict-config",
               "--cd", str(work), "--model", args.model, "--json", "-"]
        if args.effort:
            cmd[1:1] = ["-c", f"model_reasoning_effort={json.dumps(args.effort)}"]

        check_path = args.session / "isolation_check.jsonl"
        check = f"""This is an isolation preflight; the simulator must remain paused.
Use carla.read on {args.session / 'session.json'} and then on protocol. Use carla.probe
with command status. Try each call and then end the turn. Use no driving commands.
"""
        if args.code:
            check += "Also try carla.write_script with path ../outside.py and source pass.\n"
        (codex_home / "config.toml").write_text(config_for(args, preflight=True))
        with check_path.open("w") as out, (args.session / "isolation_check_stderr.log").open("w") as err:
            if subprocess.run(cmd, input=check, text=True, stdout=out, stderr=err,
                              cwd=work, env=env, timeout=180).returncode:
                raise ValueError("Codex preflight execution failed")
        verify_preflight(check_path, args.code)
        metadata_path = args.session / "driver.json"
        metadata = json.loads(metadata_path.read_text())
        config = config_for(args)
        (args.session / "codex_config.toml").write_text(config)
        metadata.update(codex_cli=subprocess.check_output([executable, "--version"], text=True).strip(),
                        reasoning_effort=args.effort or "model-default",
                        codex_config_sha256=hashlib.sha256(config.encode()).hexdigest(),
                        driver_docs_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest()
                                           for name, path in DOCS.items()},
                        isolation_check="passed")
        metadata_path.write_text(json.dumps(metadata, indent=1))
        (codex_home / "config.toml").write_text(config)
        print("    Codex isolation check passed", flush=True)
        return drive(cmd, (args.session / "prompt.txt").read_text(),
                     args.session / "driver_transcript.jsonl", args.session / "driver_stderr.log",
                     work, env, args.code)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["serve", "run"])
    parser.add_argument("session", type=Path)
    parser.add_argument("addr")
    parser.add_argument("--code", type=Path)
    parser.add_argument("--perception", default="")
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--model")
    parser.add_argument("--effort", default="")
    args = parser.parse_args()
    if args.mode == "serve":
        serve(DriverTools(args.session, args.addr, args.code, args.perception, args.preflight))
        return 0
    try:
        return run(args)
    except (ValueError, OSError, KeyError, subprocess.TimeoutExpired) as error:
        print(f"    Codex driver failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
