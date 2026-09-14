#!/usr/bin/env python3
"""A purpose-built driver loop for the probe daemon, on the Claude Messages API.

    python -m carla_probe.drive_loop --addr 127.0.0.1:8790 --session SESSION_DIR [--model claude-opus-5]

Why this exists (sessions/plans/driver-loop-plan.md, phase 1). Driving a route with `claude -p`
costs turns quadratically: context grows linearly because nothing is ever evicted, and every turn
re-reads all of it, so ~99% of billed input is cache reads and cost goes as turns**2. Two levers,
both of which need a loop we own:

  1.1 Context control. Old camera images are cleared from the transcript once they are stale, so
      average context stops growing with route length. See "Context control" below.
  1.2 One round trip per decision. Each driving tool sets the daemon's `percept=true`, so the reply
      that reports what the manoeuvre did already carries the next observation and its images. The
      `claude -p` loop spent ~10 turns per decision (observe, get_camera, N image reads, plan);
      here a decision is one tool call, and the images arrive inline rather than as Read calls.

This IS the benchmark driver: `scripts/run_eval.sh` runs it by default (`DRIVER=claude_p` still
selects the old `claude -p` path, so the runs made with it stay reproducible). Numbers from the two
drivers are different configurations and must not be averaged or compared directly -- switching
harness is a deliberate re-baseline (§6 of the plan, benchmark/PROTOCOL.md).

It also makes PROTOCOL rule 4 -- the driver uses only the probe protocol -- true by construction
rather than by enforcement. `claude -p` is a general-purpose coding agent with Bash and Read, so it
needs a sandbox and an isolation probe to stop it reading result.json or the route XML. This loop
has five probe tools and no filesystem access at all: there is nothing to isolate.

Context control
---------------
Eviction is done by the API's server-side context editing (`clear_tool_uses_20250919`), not by
rewriting our own `messages`. That matters: under "preserved thinking", editing an earlier turn
invalidates every later thinking block (and restarts the prompt cache), while server-side editing
does not count as an edit -- the check compares the conversation as we SENT it, not the server's
edited copy. So this loop is strictly append-only.

The plan asked for stale looks to "degrade to their text description" before dropping out. That
falls out for free here rather than needing to be built: what gets cleared is the tool RESULT (the
images and the observation JSON), while the assistant turn that asked for it survives -- and every
driving command already has to carry `why="..."`, the driver's written account of what it saw. The
picture goes; the description the driver wrote of it stays.

Cost accounting for §0.3 of the plan is written to `driver_loop.jsonl` in the session directory:
per turn, the token usage split by cache reads / writes, the tool called, and the step it left the
route at: one `turn` row per API response carrying that turn's usage, and one `tool` row per call
carrying the decision. Usage lives on the turn row, not the tool row, because a turn may call more
than one tool. `tools/cost_report.py` reads it (and the `claude -p` transcript) into the cost model.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from carla_probe.client import ProbeClient

MODEL = "claude-opus-5"
# Server-side context editing is beta; so is the beta messages endpoint it lives on.
CONTEXT_EDITING_BETA = "context-management-2025-06-27"
# Thinking is configured differently per model generation, and getting it wrong is a 400, not a
# degradation: `output_config.effort` errors on Haiku 4.5, and `budget_tokens` is rejected by
# Opus 5 / Sonnet 5. The benchmark set spans all three, so shape the request per model.
_BUDGET_THINKING_MODELS = ("claude-haiku-4-5",)


def _thinking_params(model: str, effort: str, max_tokens: int) -> Dict[str, Any]:
    if any(model.startswith(m) for m in _BUDGET_THINKING_MODELS):
        # Pre-4.6 shape: a fixed budget, which must be < max_tokens. No `effort` at all.
        return {"thinking": {"type": "enabled", "budget_tokens": max(1024, max_tokens // 2)}}
    return {"thinking": {"type": "adaptive"}, "output_config": {"effort": effort}}

# ---------------------------------------------------------------- benchmark definition (prompts)
# The system prompt AND the tool descriptions below are part of the benchmark definition: reword
# either and results stop being comparable, which is why both are hashed into `definition_sha256`
# (written to the session as prompt.txt + tools.json). Pick one with --prompt.
#
# BASELINE states only mechanical facts about the harness -- the same level as the `claude -p`
# driver's prompt.txt, which points at the docs and gives no driving advice. It is the headline
# configuration. GUIDED adds driving guidance, which the plan treats as interventions to be
# MEASURED (§2.3 control guide, §2.4 prompt) rather than assumed: keeping them in a named variant
# is what lets their effect be attributed instead of silently folded into the baseline.
#
# Note the plan's validity boundary (§5): road rules the scorer enforces are in scope, scorer
# mechanics are not. GUIDED names the rules; it must never explain how the scoring can be gamed.

BASELINE_PROMPT = """You are driving a car in the CARLA simulator along a fixed route, and are \
scored by the Bench2Drive leaderboard: route completion multiplied by an infraction penalty.

The simulation is FROZEN while you think. Time advances only through your driving tools.

Each driving tool returns the next observation and camera images in its own result, so one tool \
call is one complete decision.

Every driving command needs `why`: what you observed and why the command follows from it. It is \
logged and reviewed.

Older tool results, including their camera images, are cleared from your context as the drive goes \
on. Your own messages are not.

Drive to the end of the route. Stop when a tool reports the route has ended."""

_GUIDANCE = """

The route is scored against the rules of the road: obey traffic lights and stop signs, yield to \
emergency vehicles, stay in your lane and on the route, and do not block traffic.

Look before you commit. Every tick you buy with `ticks` is a tick you are driving blind, so size \
the advance to how far ahead you can actually see and how fast the scene is changing.

Waypoints are in the ego frame: +forward is ahead, +left is to your LEFT, in metres.

Say what you actually see in `why` -- positions, distances, which lane, what the other cars are \
doing. Once an image is cleared, that account is what you still have."""

PROMPTS = {"baseline": BASELINE_PROMPT, "guided": BASELINE_PROMPT + _GUIDANCE}

# One tool per daemon command the driver needs. Kept deliberately close to the wire protocol
# (docs/OPERATOR_GUIDE.md) so behaviour stays comparable with the `claude -p` driver.
_WHY = {"type": "string", "description": "What you observe right now and why this command follows from "
                                         "it. Required, logged, and reviewed as part of the result."}
TOOLS: List[Dict[str, Any]] = [
    {
        "name": "percept",
        "description": "Look without moving: the current observation plus camera images. Free in "
                       "simulated time. You do not need this after a driving tool -- that already "
                       "returned one.",
        "input_schema": {"type": "object", "properties": {
            "cams": {"type": "array", "items": {"type": "string"},
                     "description": "Camera ids; omit for the configured default set."}},
            "required": []},
    },
    {
        "name": "apply_plan",
        "description": "Drive a path: ego-frame waypoints [[forward, left], ...] tracked by a PID "
                       "controller for `ticks` ticks (20 ticks = 1 second). The main driving tool.",
        "input_schema": {"type": "object", "properties": {
            "waypoints": {"type": "array", "description": "At least 2 points [[fwd, left], ...], metres, "
                                                          "ego frame: +fwd ahead, +left to your LEFT.",
                          "items": {"type": "array", "items": {"type": "number"}}},
            "ticks": {"type": "integer", "description": "Ticks to drive this plan before you look again."},
            "target_speed": {"type": "number", "description": "Target speed m/s; omit to take it from "
                                                              "waypoint spacing."},
            "why": _WHY},
            "required": ["waypoints", "ticks", "why"]},
    },
    {
        "name": "set_control",
        "description": "Raw actuators for `ticks`, bypassing the PID. For recovery, not for normal "
                       "driving. throttle/brake in [0,1]; steer in [-1,1], + is RIGHT.",
        "input_schema": {"type": "object", "properties": {
            "steer": {"type": "number"}, "throttle": {"type": "number"}, "brake": {"type": "number"},
            "ticks": {"type": "integer"}, "why": _WHY},
            "required": ["steer", "throttle", "brake", "ticks", "why"]},
    },
    {
        "name": "hold",
        "description": "Brake to a standstill; with `ticks`, hold for that long.",
        "input_schema": {"type": "object", "properties": {
            "ticks": {"type": "integer"}, "why": _WHY}, "required": ["why"]},
    },
    {
        "name": "run_until_event",
        "description": "Keep the current control until an event fires or max_ticks runs out. "
                       "event: stopped | plan_done.",
        "input_schema": {"type": "object", "properties": {
            "event": {"type": "string", "enum": ["stopped", "plan_done"]},
            "max_ticks": {"type": "integer"}, "why": _WHY},
            "required": ["event", "max_ticks", "why"]},
    },
]
# Tools that advance time. Each gets percept=true, so its reply carries the next look (§1.2).
DRIVING_TOOLS = {"apply_plan", "set_control", "hold", "run_until_event"}


def _image_block(path: str) -> Optional[Dict[str, Any]]:
    p = Path(path)
    if not p.is_file():
        return None
    return {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg",
                                        "data": base64.standard_b64encode(p.read_bytes()).decode()}}


def _result_content(reply: Dict[str, Any]) -> List[Dict[str, Any]]:
    """A daemon reply as tool-result content: the images as image blocks, the rest as JSON text.

    The images are pulled OUT of the JSON and sent as real image blocks -- the point of §1.2. Under
    `claude -p` the daemon returned paths and the driver spent one Read turn per image.
    """
    # The daemon sends the directory once and bare filenames; join them back up.
    frames_dir = reply.pop("frames_dir", "") or ""
    files = [str(Path(frames_dir) / n) for n in (reply.pop("files", []) or [])]
    blocks: List[Dict[str, Any]] = []
    for path in files:
        block = _image_block(path)
        if block is not None:
            blocks.append({"type": "text", "text": Path(path).stem})
            blocks.append(block)
    blocks.append({"type": "text", "text": json.dumps(reply, separators=(",", ":"))})
    return blocks


def _route_over(reply: Dict[str, Any]) -> bool:
    return str((reply.get("result") or {}).get("reason", "")) in ("route_end", "shutdown")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--addr", default="127.0.0.1:8790", help="daemon address (default %(default)s)")
    ap.add_argument("--session", required=True, type=Path, help="session directory, for driver_loop.jsonl")
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--prompt", default="baseline", choices=sorted(PROMPTS),
                    help="which benchmark prompt to run (default %(default)s)")
    ap.add_argument("--effort", default="high", choices=["low", "medium", "high", "xhigh", "max"])
    ap.add_argument("--max-turns", type=int, default=400, help="hard stop (default %(default)s)")
    ap.add_argument("--keep-images", type=int, default=3,
                    help="camera looks kept as images before the API clears them (default %(default)s)")
    ap.add_argument("--wait", type=float, default=600.0, help="seconds to wait for the daemon")
    args = ap.parse_args()

    import anthropic  # imported late so --help works without the SDK installed

    system = PROMPTS[args.prompt]
    # Snapshot the exact definition next to the run, and hash it, so a reworded tool description
    # can never change results without leaving a record (`claude -p` hashes prompt.txt the same way).
    tools_json = json.dumps(TOOLS, indent=2, sort_keys=True)
    (args.session / "prompt.txt").write_text(system + "\n")
    (args.session / "tools.json").write_text(tools_json + "\n")
    definition_sha256 = hashlib.sha256((system + "\n" + tools_json).encode()).hexdigest()

    client = anthropic.Anthropic()
    probe = ProbeClient(args.addr, wait_s=args.wait)
    log = open(args.session / "driver_loop.jsonl", "a")

    def record(row: Dict[str, Any]) -> None:
        log.write(json.dumps(row) + "\n")
        log.flush()

    opening = probe.call("percept")
    record({"event": "start", "t_wall": time.time(), "model": args.model, "effort": args.effort,
            "prompt": args.prompt, "definition_sha256": definition_sha256,
            "keep_images": args.keep_images, "status": opening.get("status")})
    messages: List[Dict[str, Any]] = [{"role": "user", "content": _result_content(dict(opening))}]

    turns = 0
    done: Optional[str] = None
    while turns < args.max_turns and done is None:
        turns += 1
        response = client.beta.messages.create(
            model=args.model,
            max_tokens=16000,
            system=system,
            tools=TOOLS,
            messages=messages,
            # Caching is opt-in: without this the growing transcript is billed at the full input
            # rate on EVERY turn (~10x a cache read), which would swamp the saving the loop is for.
            # Top-level auto-caching puts the breakpoint at the last cacheable block, which is what
            # a transcript that only ever grows at the end needs. Context editing does invalidate
            # the cache from each clear point, but it is clearing the run of stale images that
            # would otherwise be re-read for the rest of the route.
            cache_control={"type": "ephemeral"},
            **_thinking_params(args.model, args.effort, 16000),
            betas=[CONTEXT_EDITING_BETA],
            # §1.1. Every driving tool returns exactly one look, so a tool use IS a look and `keep`
            # reads as "the last N looks stay as pictures". `trigger` must be set too: it defaults
            # to 100k input tokens, which would let context grow to that before anything cleared.
            # `clear_tool_inputs` stays false, so only the RESULT (images + observation JSON) is
            # cleared and the tool_use that asked for it -- carrying the driver's own `why` account
            # of what it saw -- stays. That is the text the pictures degrade into.
            context_management={"edits": [{
                "type": "clear_tool_uses_20250919",
                "trigger": {"type": "tool_uses", "value": args.keep_images},
                "keep": {"type": "tool_uses", "value": args.keep_images},
                "clear_tool_inputs": False,
            }]},
        )
        u = response.usage
        record({"event": "turn", "turn": turns, "stop_reason": response.stop_reason,
                "usage": {"input": u.input_tokens, "output": u.output_tokens,
                          "cache_read": getattr(u, "cache_read_input_tokens", None),
                          "cache_write": getattr(u, "cache_creation_input_tokens", None)}})
        # Append the response CONTENT verbatim -- thinking blocks included and unmodified. Anything
        # else here would be a history edit (see the module docstring).
        messages.append({"role": "assistant", "content": response.content})

        calls = [b for b in response.content if getattr(b, "type", None) == "tool_use"]
        if not calls:
            done = f"model stopped calling tools (stop_reason={response.stop_reason})"
            record({"event": "stop", "turn": turns, "reason": done,
                    "text": "".join(b.text for b in response.content if getattr(b, "type", None) == "text")})
            break

        results: List[Dict[str, Any]] = []
        for call in calls:
            cmd = dict(call.input)
            if call.name in DRIVING_TOOLS:
                cmd["percept"] = True
            t0 = time.time()
            try:
                reply = probe.call(call.name, **cmd)
            except Exception as exc:  # a dead daemon ends the drive; report it as a tool error first
                reply, done = {"ok": False, "error": repr(exc)}, f"daemon unreachable: {exc!r}"
            results.append({"type": "tool_result", "tool_use_id": call.id,
                            "content": _result_content(dict(reply)),
                            **({"is_error": True} if not reply.get("ok", False) else {})})
            status = reply.get("status") or {}
            record({"event": "tool", "turn": turns, "tool": call.name,
                    "why": cmd.get("why"), "ok": reply.get("ok"),
                    "result": reply.get("result"), "step": status.get("step"),
                    "progress_pct": (status.get("route") or {}).get("progress_pct"),
                    "wall_s": round(time.time() - t0, 2)})
            if done is None and _route_over(reply):
                done = str((reply.get("result") or {}).get("reason"))
        messages.append({"role": "user", "content": results})

    if done is None:
        done = f"max turns ({args.max_turns})"
    record({"event": "end", "t_wall": time.time(), "turns": turns, "reason": done,
            "definition_sha256": definition_sha256})
    print(f"drive_loop: {done} after {turns} turn(s); definition {definition_sha256[:12]}; "
          f"log: {args.session / 'driver_loop.jsonl'}")
    log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
