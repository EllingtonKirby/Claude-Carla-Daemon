"""carla_probe/drive_loop.py: the benchmark driver loop, with the API and the daemon stubbed out.

No network and no simulator. What these cover is the wiring that only shows up at runtime and
would otherwise cost a CARLA session and real tokens to discover: that driving tools get
``percept=true``, that camera files become image blocks rather than paths, that the loop stops on
``route_end``, that thinking is shaped per model, and that the benchmark definition is snapshotted
and hashed.
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

from carla_probe import drive_loop


class _Block:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Usage:
    input_tokens = 100
    output_tokens = 10
    cache_read_input_tokens = 90
    cache_creation_input_tokens = 0


class _Response:
    def __init__(self, content):
        self.content, self.usage, self.stop_reason = content, _Usage(), "tool_use"


def _tool_use(name, tool_id, **inp):
    return _Block(type="tool_use", id=tool_id, name=name, input=inp)


class _FakeProbe:
    """Stands in for the daemon: records what it was asked, replies with a percept-shaped payload."""

    def __init__(self, jpg: Path, end_after: int = 2):
        self.calls, self.jpg, self.end_after = [], jpg, end_after

    def call(self, cmd, **kwargs):
        self.calls.append((cmd, kwargs))
        driving = len([c for c, _ in self.calls if c != "percept"])
        reason = "route_end" if driving >= self.end_after else "ticks"
        reply = {"ok": True, "files": [str(self.jpg)],
                 "observe": {"step": len(self.calls), "nav": {"progress_pct": 10.0}},
                 "status": {"step": len(self.calls), "route": {"progress_pct": 10.0}}}
        if cmd != "percept":
            reply["result"] = {"reason": reason, "ticks_advanced": 20}
        return reply


@pytest.fixture
def session(tmp_path):
    jpg = tmp_path / "CAM_FRONT.jpg"
    jpg.write_bytes(b"\xff\xd8\xff\xe0fake-jpeg")
    return tmp_path, jpg


def _run(monkeypatch, session, responses, argv_extra=()):
    out, jpg = session
    probe = _FakeProbe(jpg)
    requests = []

    class _Messages:
        def create(self, **kw):
            requests.append(kw)
            return responses[len(requests) - 1]

    fake_anthropic = types.ModuleType("anthropic")
    fake_anthropic.Anthropic = lambda **kw: types.SimpleNamespace(beta=types.SimpleNamespace(messages=_Messages()))
    monkeypatch.setitem(sys.modules, "anthropic", fake_anthropic)
    monkeypatch.setattr(drive_loop, "ProbeClient", lambda addr, wait_s=0.0: probe)
    monkeypatch.setattr(sys, "argv", ["drive_loop.py", "--session", str(out), *argv_extra])
    assert drive_loop.main() == 0
    rows = [json.loads(line) for line in (out / "driver_loop.jsonl").read_text().splitlines()]
    return probe, requests, rows


def test_driving_tool_batches_the_next_look_and_stops_at_route_end(monkeypatch, session):
    responses = [
        _Response([_tool_use("apply_plan", "t1", waypoints=[[4, 0], [8, 0]], ticks=20, why="clear road")]),
        _Response([_tool_use("apply_plan", "t2", waypoints=[[4, 0], [8, 0]], ticks=20, why="still clear")]),
    ]
    probe, requests, rows = _run(monkeypatch, session, responses)

    # §1.2: the opening look, then every driving command carries percept=true so its own reply
    # brings the next one. No separate observe/get_camera round trips.
    assert [c for c, _ in probe.calls] == ["percept", "apply_plan", "apply_plan"]
    assert all(kw["percept"] is True for c, kw in probe.calls if c == "apply_plan")
    assert probe.calls[1][1]["why"] == "clear road"

    # Images arrive as image blocks, not as paths for the driver to open.
    opening = requests[0]["messages"][0]["content"]
    assert [b["type"] for b in opening] == ["text", "image", "text"]
    assert opening[1]["source"]["media_type"] == "image/jpeg"
    assert "files" not in json.loads(opening[-1]["text"])  # popped out, not duplicated as text

    assert next(r for r in rows if r["event"] == "end")["reason"] == "route_end"
    assert len(requests) == 2  # stopped on route_end rather than asking the model again


def test_context_editing_is_configured_and_history_is_append_only(monkeypatch, session):
    responses = [_Response([_tool_use("hold", "t1", ticks=5, why="stop")]),
                 _Response([_tool_use("hold", "t2", ticks=5, why="stop again")])]
    _, requests, _ = _run(monkeypatch, session, responses, ("--keep-images", "2"))

    edit = requests[0]["context_management"]["edits"][0]
    assert edit["type"] == "clear_tool_uses_20250919"
    # Both must be set: the default trigger is 100k input tokens, which would let context grow
    # that far before anything cleared.
    assert edit["trigger"] == {"type": "tool_uses", "value": 2}
    assert edit["keep"] == {"type": "tool_uses", "value": 2}
    assert edit["clear_tool_inputs"] is False  # the driver's own `why` must survive the clear
    assert drive_loop.CONTEXT_EDITING_BETA in requests[0]["betas"]

    # Append-only: the second request must extend the first, byte for byte, never rewrite it.
    assert requests[1]["messages"][:1] == requests[0]["messages"][:1]
    assert requests[1]["messages"][1]["content"] is responses[0].content  # verbatim, thinking included


def test_percept_tool_does_not_advance_time(monkeypatch, session):
    responses = [_Response([_tool_use("percept", "t1")]),
                 _Response([_tool_use("hold", "t2", ticks=5, why="done looking")]),
                 _Response([_tool_use("hold", "t3", ticks=5, why="stop")])]
    probe, _, _ = _run(monkeypatch, session, responses)
    assert "percept" not in probe.calls[1][1]  # the look-only tool is not given percept=true


def test_stops_when_the_model_stops_calling_tools(monkeypatch, session):
    response = _Response([_Block(type="text", text="I have finished the route.")])
    response.stop_reason = "end_turn"
    _, requests, rows = _run(monkeypatch, session, [response])
    stop = next(r for r in rows if r["event"] == "stop")
    assert "end_turn" in stop["reason"] and "finished" in stop["text"]
    assert len(requests) == 1


@pytest.mark.parametrize("model,expect_effort,expect_budget", [
    ("claude-opus-5", True, False),
    ("claude-sonnet-5", True, False),
    ("claude-haiku-4-5", False, True),  # pre-4.6 shape: effort would be a 400, not a degradation
])
def test_thinking_is_shaped_per_model(model, expect_effort, expect_budget):
    params = drive_loop._thinking_params(model, "high", 16000)
    assert ("output_config" in params) is expect_effort
    assert ("budget_tokens" in params["thinking"]) is expect_budget
    if expect_budget:
        assert params["thinking"]["budget_tokens"] < 16000  # must stay under max_tokens


def test_definition_is_snapshotted_and_hashed(monkeypatch, session):
    out, _ = session
    responses = [_Response([_tool_use("hold", "t1", ticks=5, why="stop")]),
                 _Response([_tool_use("hold", "t2", ticks=5, why="stop")])]
    _, requests, rows = _run(monkeypatch, session, responses, ("--prompt", "guided"))

    assert (out / "prompt.txt").read_text().strip() == drive_loop.PROMPTS["guided"]
    assert json.loads((out / "tools.json").read_text()) == json.loads(json.dumps(drive_loop.TOOLS))
    assert requests[0]["system"] == drive_loop.PROMPTS["guided"]
    digest = next(r for r in rows if r["event"] == "end")["definition_sha256"]
    assert len(digest) == 64 and rows[0]["definition_sha256"] == digest


def test_baseline_prompt_carries_no_driving_guidance():
    """The headline configuration must not smuggle in the interventions that are under test."""
    baseline = drive_loop.PROMPTS["baseline"].lower()
    for phrase in ("driving blind", "yield", "red light", "lane discipline", "distances"):
        assert phrase not in baseline, phrase
    assert drive_loop.PROMPTS["guided"].startswith(drive_loop.BASELINE_PROMPT)
