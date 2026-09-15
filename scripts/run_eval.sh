#!/usr/bin/env bash
# Run one benchmark eval: a probe session driven by an isolated, headless Claude Code driver.
# See docs/HANDOFF.md §4 for the rules and §5 for what gets recorded.
#
#   bash scripts/run_eval.sh <ROUTE> <sensors|privileged> <MODEL> [SESSION_DIR]
#   bash scripts/run_eval.sh 2091 sensors claude-opus-5
#   BENCHMARK=fail2drive bash scripts/run_eval.sh Base_Animals_0075 sensors claude-opus-5
#
# Blocks until the route is scored (run it in the background). Steps:
#   1. launch scripts/run_session.sh and wait until the route is loaded and paused;
#   2. run the driver, streaming its transcript to the session;
#   3. if the driver exits before the route ends, abort the session (scored as an agent failure).
#
# Two drivers, selected with DRIVER. They are different configurations: never average or directly
# compare their scores (docs/HANDOFF.md §4.1, benchmark/PROTOCOL.md).
#   drive_loop (default)  carla_probe/drive_loop.py: a purpose-built loop on the Messages API whose
#                         whole action space is the probe tools. PROTOCOL rule 4 -- the driver uses
#                         only the probe protocol -- holds by construction: it has no filesystem
#                         access to sandbox. Needs Anthropic API credentials.
#   claude_p              the original isolated `claude -p` driver, kept so the runs made with it
#                         stay reproducible. Being a general-purpose agent with Bash and Read, it
#                         additionally needs a sandbox and a pre-flight isolation check in which a
#                         cheap model must be DENIED reading the session directory.
#
# Env: BENCHMARK (bench2drive | fail2drive): which benchmark to score on. The two are not
#        comparable -- different routes, CARLA build, route budget and success rule. Fail2Drive is
#        scored in PAIRS (tools/f2d_score.py); one route on its own is not an F2D result.
#      GPU_RANK (0)  CARLA_PORT (20000)  PROBE_PORT (8790): stride CARLA_PORT by >= 10 per parallel run
#      DRIVER (drive_loop)  LOAD_TIMEOUT_S (900)
#      drive_loop only: PROMPT_VARIANT (baseline|guided)  EFFORT (high)  KEEP_IMAGES (3)
#      claude_p only:   ISOLATION_CHECK (1)  CHECK_MODEL (haiku)
#      SIM_RECORD (1)  RECORD_CAMERA_EVERY (config value): recording knobs, passed to run_session.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USAGE="usage: run_eval.sh <ROUTE_ID> <sensors|privileged> <MODEL> [SESSION_DIR]"
ROUTE="${1:?$USAGE}"
TRACK="${2:?$USAGE}"
MODEL="${3:?$USAGE}"
case "$TRACK" in sensors | privileged) ;; *) echo "ERROR: track must be sensors or privileged" >&2; exit 1 ;; esac
: "${DRIVER:=drive_loop}"
case "$DRIVER" in drive_loop | claude_p) ;; *) echo "ERROR: DRIVER must be drive_loop or claude_p" >&2; exit 1 ;; esac
# Fail before CARLA spends two minutes loading a town.
if [ "$DRIVER" = claude_p ] || [ "${ISOLATION_CHECK:-1}" = 1 ]; then
    command -v claude >/dev/null || { echo "ERROR: claude (Claude Code CLI) not on PATH" >&2; exit 1; }
fi

export BENCHMARK="${BENCHMARK:-bench2drive}"
case "$BENCHMARK" in
    bench2drive) LEADERBOARD_NAME="Bench2Drive" ;;
    fail2drive)  LEADERBOARD_NAME="Fail2Drive" ;;
    *) echo "ERROR: BENCHMARK must be bench2drive or fail2drive" >&2; exit 1 ;;
esac
if [ -z "${LEADERBOARD_ROOT:-}" ] || [ "${PROBE_BENCHMARK_ACTIVE:-}" != "$BENCHMARK" ]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/scripts/env.sh"
fi

: "${GPU_RANK:=0}"
: "${CARLA_PORT:=20000}"
: "${PROBE_PORT:=8790}"
: "${LOAD_TIMEOUT_S:=900}"
: "${ISOLATION_CHECK:=1}"
: "${CHECK_MODEL:=haiku}"
: "${PROMPT_VARIANT:=baseline}"
: "${EFFORT:=high}"
: "${KEEP_IMAGES:=3}"
export GPU_RANK CARLA_PORT PROBE_PORT

SESSION_DIR="$(realpath -m "${4:-$REPO_ROOT/sessions/$(date +%Y%m%d_%H%M%S)_route_${ROUTE}_${TRACK}}")"
[ -e "$SESSION_DIR/result.json" ] && { echo "ERROR: $SESSION_DIR already has a result.json" >&2; exit 1; }
mkdir -p "$SESSION_DIR"
PY="$REPO_ROOT/.venv/bin/python"
ADDR="127.0.0.1:$PROBE_PORT"
if [ "$DRIVER" = drive_loop ]; then
    "$PY" -c 'import anthropic' 2>/dev/null || {
        echo "ERROR: the anthropic SDK is not installed in .venv (uv pip install --python .venv/bin/python anthropic)" >&2
        exit 1; }
fi

echo "=== eval: $BENCHMARK route $ROUTE, track $TRACK, model $MODEL, driver $DRIVER"
echo "    session: $SESSION_DIR"

# ---- 1. session --------------------------------------------------------------------------------
ACCESS="$TRACK" bash "$REPO_ROOT/scripts/run_session.sh" "$ROUTE" "$SESSION_DIR" > "$SESSION_DIR/launcher.log" 2>&1 &
LAUNCH_PID=$!
abort() {
    echo "    aborting: $1" >&2
    "$PY" -m carla_probe.client "$ADDR" shutdown why="operator abort: $1" > /dev/null 2>&1 || true
    sleep 10
    kill "$LAUNCH_PID" 2>/dev/null || true
    bash "$REPO_ROOT/scripts/clean_carla.sh" "$CARLA_PORT" > /dev/null
    exit 1
}
echo "    waiting for the route to load (log: $SESSION_DIR/launcher.log)"
if ! access="$("$PY" -m carla_probe.client "$ADDR" --wait "$LOAD_TIMEOUT_S" status 2>/dev/null \
        | "$PY" -c 'import json,sys; print(json.load(sys.stdin)["status"]["access"])')"; then
    abort "session did not come up; see $SESSION_DIR/evaluator.log and carla.log"
fi
[ "$access" = "$TRACK" ] || abort "session reports access=$access, expected $TRACK"

# ---- 2. driver sandbox (claude_p only: drive_loop has no filesystem access to sandbox) -----------
if [ "$DRIVER" = claude_p ]; then
    DRIVER_WS="$(mktemp -d "${TMPDIR:-/tmp}/probe_driver.XXXXXX")"
    cat > "$DRIVER_WS/settings.json" <<EOF
{
  "sandbox": {"enabled": false},
  "env": {"PYTHONPATH": "$REPO_ROOT", "CARLA_PROBE_ADDR": "$ADDR"},
  "permissions": {
    "allow": ["Bash($PY -m carla_probe.client:*)",
              "Read(/$SESSION_DIR/frames/**)",
              "Read(/$REPO_ROOT/docs/OPERATOR_GUIDE.md)",
              "Read(/$REPO_ROOT/benchmark/PROTOCOL.md)",
              "Read(/$REPO_ROOT/.claude/skills/drive-carla/SKILL.md)"],
    "deny": ["Read(/$SESSION_DIR/*.*)", "Read(/$REPO_ROOT/routes/**)", "Read(/$REPO_ROOT/third_party/**)",
             "Read(/$REPO_ROOT/benchmark/FAIL2DRIVE.md)"]
  }
}
EOF
    # The canonical driver prompt. Changing it makes results non-comparable: note it next to results.
    cat > "$DRIVER_WS/prompt.txt" <<EOF
You are the driver in a CARLA $LEADERBOARD_NAME benchmark run on the $TRACK track. A probe session for
one route is already running and paused, waiting for your commands.

Send commands only as: $PY -m carla_probe.client CMD key=value ...
(one command per Bash call, without pipes, redirection or other commands). Camera and BEV
commands return image paths; open them with the Read tool. Every driving command must include
why="..." saying what you observed and why this command follows from it; it is logged for review.

Before your first command, read $REPO_ROOT/.claude/skills/drive-carla/SKILL.md (skip its
section 1 "Launch": the session is already up; also skip the summarize step in section 4), then
$REPO_ROOT/docs/OPERATOR_GUIDE.md and $REPO_ROOT/benchmark/PROTOCOL.md, and follow them.

Drive the route to the end. Stop when an advance returns route_end or the client reports that
the daemon closed the connection. Then reply with a short account of the drive.
EOF
    cp "$DRIVER_WS/prompt.txt" "$DRIVER_WS/settings.json" "$SESSION_DIR/"
    DRIVER_FLAGS=(--permission-mode dontAsk --tools "Bash,Read" --setting-sources "" --settings "$DRIVER_WS/settings.json")

    # ---- 3. isolation check --------------------------------------------------------------------------
    if [ "$ISOLATION_CHECK" = "1" ]; then
        echo "    isolation check ($CHECK_MODEL)"
        (cd "$DRIVER_WS" && claude -p "This is a permissions test. Try each, one tool call at a time, even if you expect a denial: (1) Bash: cat $SESSION_DIR/session.json (2) Bash: ls $SESSION_DIR (3) Read tool on $SESSION_DIR/config.yaml. Then say done." \
            --model "$CHECK_MODEL" --output-format stream-json --verbose "${DRIVER_FLAGS[@]}" \
            < /dev/null > "$DRIVER_WS/isolation_check.jsonl" 2>&1) || true
        if ! "$PY" - "$DRIVER_WS/isolation_check.jsonl" <<'PY'
import json, sys
calls, leaked = 0, []
for line in open(sys.argv[1]):
    try:
        d = json.loads(line)
    except json.JSONDecodeError:
        continue
    message = d.get("message")
    if not isinstance(message, dict):  # system events carry a plain-text message
        continue
    for c in message.get("content") or []:
        if not isinstance(c, dict):
            continue
        if c.get("type") == "tool_use":
            calls += 1
        if c.get("type") == "tool_result":
            text = json.dumps(c.get("content"))
            if any(s in text for s in ("tm_seed", "config.yaml", "session.json", "access:")) and "denied" not in text:
                leaked.append(text[:120])
if calls < 3 or leaked:
    print(f"    isolation check FAILED: {calls} tool calls, leaks: {leaked}", file=sys.stderr)
    sys.exit(1)
PY
        then
            abort "driver sandbox is not isolated (transcript: $DRIVER_WS/isolation_check.jsonl)"
        fi
    fi

fi

# ---- 4. driver -----------------------------------------------------------------------------------
if [ "$DRIVER" = claude_p ]; then
    cat > "$SESSION_DIR/driver.json" <<EOF
{"driver": "$DRIVER", "model": "$MODEL", "claude_code": "$(claude --version)", "benchmark": "$BENCHMARK", "track": "$TRACK", "route": "$ROUTE",
 "prompt_sha256": "$(sha256sum "$DRIVER_WS/prompt.txt" | cut -d' ' -f1)",
 "driver_ws": "$DRIVER_WS", "started": "$(date -Iseconds)"}
EOF
    echo "    driver running (transcript: $SESSION_DIR/driver_transcript.jsonl)"
    set +e
    (cd "$DRIVER_WS" && claude -p "$(cat prompt.txt)" --model "$MODEL" --output-format stream-json --verbose \
        "${DRIVER_FLAGS[@]}" < /dev/null > "$SESSION_DIR/driver_transcript.jsonl" 2> "$SESSION_DIR/driver_stderr.log")
    driver_rc=$?
    set -e
else
    # drive_loop writes prompt.txt + tools.json into the session itself: together they ARE the
    # benchmark definition, and their hash lands in driver.json below (from driver_loop.jsonl).
    cat > "$SESSION_DIR/driver.json" <<EOF
{"driver": "$DRIVER", "model": "$MODEL", "benchmark": "$BENCHMARK", "track": "$TRACK", "route": "$ROUTE",
 "prompt_variant": "$PROMPT_VARIANT", "effort": "$EFFORT", "keep_images": $KEEP_IMAGES,
 "started": "$(date -Iseconds)"}
EOF
    echo "    driver running (log: $SESSION_DIR/driver_loop.jsonl)"
    set +e
    "$PY" -m carla_probe.drive_loop --addr "$ADDR" --session "$SESSION_DIR" \
        --model "$MODEL" --benchmark "$BENCHMARK" --prompt "$PROMPT_VARIANT" --effort "$EFFORT" \
        --keep-images "$KEEP_IMAGES" \
        < /dev/null > "$SESSION_DIR/driver_stdout.log" 2> "$SESSION_DIR/driver_stderr.log"
    driver_rc=$?
    set -e
fi

# ---- 5. finish -----------------------------------------------------------------------------------
stopped_early=false
for _ in $(seq 1 12); do
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    if "$PY" -m carla_probe.client "$ADDR" status > /dev/null 2>&1; then
        stopped_early=true   # the daemon still answers: the route is not over
        break
    fi
    sleep 5
done
"$PY" - "$SESSION_DIR/driver.json" "$driver_rc" "$stopped_early" "$SESSION_DIR/driver_loop.jsonl" <<'PY'
import datetime, json, pathlib, sys
path, rc, early, loop_log = sys.argv[1], int(sys.argv[2]), sys.argv[3] == "true", pathlib.Path(sys.argv[4])
d = json.load(open(path))
d.update(ended=datetime.datetime.now().astimezone().isoformat(timespec="seconds"), driver_exit_code=rc,
         driver_stopped_before_route_end=early)
if loop_log.is_file():  # drive_loop: the definition hash and turn count live in its own log
    rows = [json.loads(line) for line in loop_log.read_text().splitlines() if line.strip()]
    end = next((r for r in reversed(rows) if r.get("event") == "end"), {})
    d.update(definition_sha256=end.get("definition_sha256"), turns=end.get("turns"),
             loop_stop_reason=end.get("reason"))
json.dump(d, open(path, "w"), indent=1)
PY
if [ "$stopped_early" = true ]; then
    echo "    driver exited (rc=$driver_rc) before the route ended: aborting the session (agent failure)"
    "$PY" -m carla_probe.client "$ADDR" shutdown why="operator abort: the driver exited before the route ended" \
        > /dev/null 2>&1 || true
fi
wait "$LAUNCH_PID" || true
echo "=== eval done -> $SESSION_DIR"
"$PY" "$REPO_ROOT/tools/summarize_results.py" "$SESSION_DIR" || true
