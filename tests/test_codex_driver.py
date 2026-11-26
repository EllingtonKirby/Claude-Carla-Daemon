import json
import os
import sys

import pytest

from carla_probe.codex_driver import DriverTools, drive, verify_preflight


def test_driver_reads_only_returned_artifacts(tmp_path):
    frame = tmp_path / "frames" / "frame_00000"
    frame.mkdir(parents=True)
    camera = frame / "camera.jpg"
    camera.write_bytes(b"camera pixels")
    score = tmp_path / "result.json"
    score.write_text('{"private":"score"}')
    alias = frame / "alias.jpg"
    alias.symlink_to(score)
    driver = DriverTools(tmp_path, "", None, "")
    with pytest.raises(PermissionError):
        driver.call("read", {"path": str(camera)})
    driver.remember({"frames_dir": str(frame), "files": [camera.name, alias.name]})
    assert driver.call("read", {"path": str(camera)})[0]["mimeType"] == "image/jpeg"
    for path in [alias, score]:
        with pytest.raises(PermissionError):
            driver.call("read", {"path": str(path)})


def test_driver_confines_code_paths_and_preflight_commands(tmp_path):
    code = tmp_path / "code"
    code.mkdir()
    outside = tmp_path / "outside.py"
    (code / "alias.py").symlink_to(outside)
    driver = DriverTools(tmp_path, "", code, "", preflight=True)
    with pytest.raises(PermissionError):
        driver.call("probe", {"command": "apply_plan"})
    for path in ("../outside.py", "alias.py", str(outside)):
        with pytest.raises(PermissionError):
            driver.call("write_script", {"path": path, "source": "pass"})
        with pytest.raises(PermissionError):
            driver.call("run_script", {"path": path})
    assert not outside.exists()
    driver.call("write_script", {"path": "drive.py", "source": "print('ready')"})
    assert driver.call("read", {"path": "drive.py"})[0]["text"] == "print('ready')"


def test_preflight_requires_carla_tool_evidence(tmp_path):
    def event(tool, args, reply):
        return {"type": "item.completed", "item": {
            "type": "mcp_tool_call", "server": "carla", "tool": tool,
            "arguments": args, "status": "completed",
            "result": {"content": [{"type": "text", "text": reply}]},
        }}

    transcript = tmp_path / "isolation_check.jsonl"
    rows = [
        event("read", {"path": str(tmp_path / "session.json")}, "Denied: private"),
        event("read", {"path": "protocol"}, "# Protocol"),
        event("probe", {"command": "status"}, '{"ok":true}'),
        {"type": "turn.completed"},
    ]
    transcript.write_text("\n".join(map(json.dumps, rows)))
    verify_preflight(transcript, code=False)
    rows.append({"type": "item.completed", "item": {"type": "command_execution"}})
    transcript.write_text("\n".join(map(json.dumps, rows)))
    with pytest.raises(ValueError, match="unexpected tool"):
        verify_preflight(transcript, code=False)


def test_drive_rejects_unexpected_tool(tmp_path):
    row = {"type": "item.started", "item": {"type": "command_execution"}}
    stub = f"import sys; sys.stdin.read(); print({json.dumps(row)!r})"
    transcript = tmp_path / "transcript.jsonl"
    with pytest.raises(ValueError, match="unexpected tool"):
        drive([sys.executable, "-c", stub], "test", transcript, tmp_path / "stderr.log",
              tmp_path, os.environ, code=False)
    assert json.loads(transcript.read_text()) == row
