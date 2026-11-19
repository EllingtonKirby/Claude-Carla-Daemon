#!/usr/bin/env bash
# Run one benchmark eval: a probe session driven by an isolated, headless Claude Code driver.
# See docs/HANDOFF.md §4 for the rules and §5 for what gets recorded.
#
#   bash scripts/run_eval.sh <ROUTE> <sensors|privileged|dense> <MODEL> [SESSION_DIR]
#   bash scripts/run_eval.sh 2091 sensors claude-opus-5
#   BENCHMARK=fail2drive bash scripts/run_eval.sh Base_Animals_0075 sensors claude-opus-5
#
# Blocks until the route is scored (run it in the background). Steps:
#   1. launch scripts/run_session.sh and wait until the route is loaded and paused;
#   2. build the driver's sandbox: an empty working directory outside the repo, no inherited Claude
#      Code settings, and permissions limited to the probe client, the session's frames/ and the
#      three driver documents;
#   3. pre-flight isolation check: a cheap model must be DENIED reading the session directory;
#   4. run the driver (`claude -p`, canonical prompt below), streaming its transcript to the session;
#   5. if the driver exits before the route ends, abort the session (scored as an agent failure).
#
# The driver is `claude -p`, so a run is billed to whatever the `claude` CLI is logged in to --
# normally a Claude subscription, in which case the reported total_cost_usd is an API-equivalent
# figure rather than a bill.
#
# Env: BENCHMARK (bench2drive | fail2drive): which benchmark to score on. The two are not
#        comparable -- different routes, CARLA build, route budget and success rule. Fail2Drive is
#        scored in PAIRS (tools/f2d_score.py); one route on its own is not an F2D result.
#      GPU_RANK (0)  CARLA_PORT (20000)  PROBE_PORT (8790): stride CARLA_PORT by >= 10 per parallel run
#      LOAD_TIMEOUT_S (900)  ISOLATION_CHECK (1; 0 skips step 3)  CHECK_MODEL (haiku)
#      AUTOCOMPACT (auto): Claude Code's auto-compact window, `auto` or 100k-1M. A THRESHOLD, not a
#           cadence: it fires when the window reaches that size, so 100k is the tightest cap
#           available. Lossy -- compaction summarizes, and a summarized camera frame is gone -- but
#           the daemon re-supplies the scene on every percept, so what it destroys is looks the
#           driver has already acted on.
#      SIM_RECORD (1)  RECORD_CAMERA_EVERY (config value): recording knobs, passed to run_session.sh
#      VLA (config value): on | off -- the learned policy behind the daemon's `drive` command; `on`
#           needs scripts/run_vla_server.sh running, and adds one sentence to the driver prompt
#           saying the command exists (recorded in driver.json)
#      CODE_TOOL (off): on gives the driver a code tool -- it may write Python into its own code dir
#           and run it with carla_probe/code_runner.py, confined to the session's frames/ (see that
#           file for what the confinement does and does not stop). Turns on full-resolution frames.
#           Changes the harness: results with and without it are different measurements.
#      PERCEPTION_ADDR (unset): host:port of carla_probe/perception_server.py; needs CODE_TOOL=on.
#           The driver's scripts may call it (and nothing else on the network). Sensors-eligible.
#      CODE_LIB (unset): a directory copied into the driver's code dir as lib/ before the drive --
#           e.g. a perception library written offline (tools/teach_perception.py). Hashed into
#           driver.json; the prompt says it is there.
#      DRIVER (claude): claude | baseline_vla | baseline_route -- the two baselines need no LLM
#           (carla_probe/baseline_driver.py) and skip the sandbox; MODEL is then just a label
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USAGE="usage: run_eval.sh <ROUTE_ID> <sensors|privileged|dense> <MODEL> [SESSION_DIR]"
ROUTE="${1:?$USAGE}"
TRACK="${2:?$USAGE}"
MODEL="${3:?$USAGE}"
case "$TRACK" in sensors | privileged | dense) ;; *) echo "ERROR: track must be sensors, privileged or dense" >&2; exit 1 ;; esac
: "${CODE_TOOL:=off}"
: "${PERCEPTION_ADDR:=}"
: "${CODE_LIB:=}"
case "$CODE_TOOL" in on | off) ;; *) echo "ERROR: CODE_TOOL must be on or off" >&2; exit 1 ;; esac
if [ "$CODE_TOOL" = off ] && { [ -n "$PERCEPTION_ADDR" ] || [ -n "$CODE_LIB" ]; }; then
    echo "ERROR: PERCEPTION_ADDR and CODE_LIB need CODE_TOOL=on" >&2; exit 1
fi
[ -z "$CODE_LIB" ] || [ -d "$CODE_LIB" ] || { echo "ERROR: CODE_LIB=$CODE_LIB is not a directory" >&2; exit 1; }
: "${DRIVER:=claude}"
: "${VLA:=}"
case "$DRIVER" in
    claude) command -v claude >/dev/null || { echo "ERROR: claude (Claude Code CLI) not on PATH" >&2; exit 1; } ;;
    baseline_vla) VLA=on ;;
    baseline_route) ;;
    *) echo "ERROR: DRIVER must be claude, baseline_vla or baseline_route" >&2; exit 1 ;;
esac
export VLA

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
: "${AUTOCOMPACT:=auto}"
export GPU_RANK CARLA_PORT PROBE_PORT

SESSION_DIR="$(realpath -m "${4:-$REPO_ROOT/sessions/$(date +%Y%m%d_%H%M%S)_route_${ROUTE}_${TRACK}}")"
[ -e "$SESSION_DIR/result.json" ] && { echo "ERROR: $SESSION_DIR already has a result.json" >&2; exit 1; }
mkdir -p "$SESSION_DIR"
PY="$REPO_ROOT/.venv/bin/python"
ADDR="127.0.0.1:$PROBE_PORT"

echo "=== eval: $BENCHMARK route $ROUTE, track $TRACK, model $MODEL, driver $DRIVER${VLA:+, vla $VLA}"
echo "    session: $SESSION_DIR"
if [ -n "$PERCEPTION_ADDR" ]; then
    curl -sf "http://$PERCEPTION_ADDR/health" > /dev/null \
        || { echo "ERROR: no perception service at $PERCEPTION_ADDR (start scripts/run_perception_server.sh)" >&2; exit 1; }
fi
if [ "$VLA" = on ]; then
    VLA_ADDR="$(sed -n '/^vla:/,/^[a-z]/p' "$REPO_ROOT/carla_probe/config.yaml" | awk '/addr:/{print $2}')"
    curl -sf "http://$VLA_ADDR/health" > /dev/null \
        || { echo "ERROR: no VLA service at $VLA_ADDR (start scripts/run_vla_server.sh)" >&2; exit 1; }
fi

# ---- 1. session --------------------------------------------------------------------------------
CODE_FULL_RES="$([ "$CODE_TOOL" = on ] && echo 1 || echo 0)" ACCESS="$TRACK" bash "$REPO_ROOT/scripts/run_session.sh" "$ROUTE" "$SESSION_DIR" > "$SESSION_DIR/launcher.log" 2>&1 &
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
# timeout: the client's --wait bounds CONNECTING, not the reply; a daemon that accepts and never
# answers (a CARLA that died mid-load) otherwise hangs this script forever (11 h, 2026-11-18).
if ! access="$(timeout $((LOAD_TIMEOUT_S + 60)) "$PY" -m carla_probe.client "$ADDR" --wait "$LOAD_TIMEOUT_S" status 2>/dev/null \
        | "$PY" -c 'import json,sys; print(json.load(sys.stdin)["status"]["access"])')"; then
    abort "session did not come up; see $SESSION_DIR/evaluator.log and carla.log"
fi
[ "$access" = "$TRACK" ] || abort "session reports access=$access, expected $TRACK"

# ---- 2. driver sandbox ---------------------------------------------------------------------------
DRIVER_WS="$(mktemp -d "${TMPDIR:-/tmp}/probe_driver.XXXXXX")"
if [ "$DRIVER" != claude ]; then
    # A scripted baseline: no sandbox, no isolation check, no prompt. Same session, same scoring.
    cat > "$SESSION_DIR/driver.json" <<EOF
{"model": "$MODEL", "driver": "$DRIVER", "vla": "${VLA:-off}", "benchmark": "$BENCHMARK", "track": "$TRACK", "route": "$ROUTE",
 "started": "$(date -Iseconds)"}
EOF
    echo "    baseline driver running ($DRIVER; log: $SESSION_DIR/driver_stdout.log)"
    "$PY" -m carla_probe.baseline_driver "$ADDR" --policy "${DRIVER#baseline_}" \
        > "$SESSION_DIR/driver_stdout.log" 2>&1 || echo "    baseline driver exited with an error (see driver_stdout.log)"
    wait "$LAUNCH_PID" || true
    echo "=== eval done -> $SESSION_DIR"
    "$PY" "$REPO_ROOT/tools/summarize_results.py" "$SESSION_DIR" || true
    exit 0
fi
CODE_DIR="$DRIVER_WS/code"
CODE_ALLOW=""
CODE_ENV=""
DRIVER_TOOLS="Bash,Read"
if [ "$CODE_TOOL" = on ]; then
    mkdir -p "$CODE_DIR"
    [ -z "$CODE_LIB" ] || cp -r "$CODE_LIB" "$CODE_DIR/lib"
    CODE_ALLOW='"Bash('"$PY"' -m carla_probe.code_runner:*)", "Write(/'"$CODE_DIR"'/**)", "Edit(/'"$CODE_DIR"'/**)", "Read(/'"$CODE_DIR"'/**)",'
    CODE_ENV=', "CARLA_PROBE_FRAMES": "'"$SESSION_DIR/frames"'", "CARLA_PROBE_CODE_DIR": "'"$CODE_DIR"'", "CARLA_PROBE_PERCEPTION_ADDR": "'"$PERCEPTION_ADDR"'"'
    DRIVER_TOOLS="Bash,Read,Write,Edit"
fi
cat > "$DRIVER_WS/settings.json" <<EOF
{
  "sandbox": {"enabled": false},
  "env": {"PYTHONPATH": "$REPO_ROOT", "CARLA_PROBE_ADDR": "$ADDR"$CODE_ENV},
  "permissions": {
    "allow": [$CODE_ALLOW
              "Bash($PY -m carla_probe.client:*)",
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
if [ "$VLA" = on ]; then
    cat >> "$DRIVER_WS/prompt.txt" <<EOF

This session has the learned driving policy on: the \`drive\` command (OPERATOR_GUIDE.md §8a) is
your primary driving primitive. You tell it what you see and what to do, in its phrasebook; it
steers and controls speed between your decisions.
EOF
fi
if [ "$CODE_TOOL" = on ]; then
    cat >> "$DRIVER_WS/prompt.txt" <<EOF

This session has the code tool (OPERATOR_GUIDE.md §8b). You may write Python files into
$CODE_DIR with the Write and Edit tools and run them, one per Bash call, as:
$PY -m carla_probe.code_runner $CODE_DIR/<file>.py [args]
A script may read the frames a percept returns and the camera model at <frames_dir>/../cameras.json,
and nothing else in the session. Use it for anything measurable: distances, gaps, clearances.
EOF
    if [ -n "$PERCEPTION_ADDR" ]; then
        cat >> "$DRIVER_WS/prompt.txt" <<EOF
Your scripts may also call the perception service at http://$PERCEPTION_ADDR (metric depth and
segmentation from a camera image; OPERATOR_GUIDE.md §8c).
EOF
    fi
    if [ -n "$CODE_LIB" ]; then
        cat >> "$DRIVER_WS/prompt.txt" <<EOF
A perception library is already installed at $CODE_DIR/lib: read its README.md before driving.
EOF
    fi
fi
cp "$DRIVER_WS/prompt.txt" "$DRIVER_WS/settings.json" "$SESSION_DIR/"
# --setting-sources "" is load-bearing for isolation, not just a settings.json switch: it also stops
# CLAUDE.md / memory being loaded, from the sandbox AND from ~/.claude. Verified -- with the flag a
# CLAUDE.md in the working directory is ignored; without it, its contents reach the driver. Dropping
# it would silently add an instruction channel outside the canonical prompt, and one that nothing
# hashes into driver.json. Leave it empty.
DRIVER_FLAGS=(--permission-mode dontAsk --tools "$DRIVER_TOOLS" --setting-sources "" --settings "$DRIVER_WS/settings.json"
              --autocompact "$AUTOCOMPACT")

# ---- 3. isolation check --------------------------------------------------------------------------
if [ "$ISOLATION_CHECK" = "1" ]; then
    echo "    isolation check ($CHECK_MODEL)"
    CHECK_PROMPT="This is a permissions test. Try each, one tool call at a time, even if you expect a denial: (1) Bash: cat $SESSION_DIR/session.json (2) Bash: ls $SESSION_DIR (3) Read tool on $SESSION_DIR/config.yaml."
    MIN_CALLS=3
    if [ "$CODE_TOOL" = on ]; then
        # The tool permissions are probed by the model; the code tool's own confinement is probed
        # below, deterministically -- a checker model may (reasonably) refuse to run a script it
        # recognises as a way around the denials it just got, and a refusal is not a pass.
        CHECK_PROMPT="$CHECK_PROMPT (4) Write tool: create $SESSION_DIR/../leak_outside.py containing print(1)."
        MIN_CALLS=4
    fi
    (cd "$DRIVER_WS" && claude -p "$CHECK_PROMPT Then say done." \
        --model "$CHECK_MODEL" --output-format stream-json --verbose "${DRIVER_FLAGS[@]}" \
        < /dev/null > "$DRIVER_WS/isolation_check.jsonl" 2>&1) || true
    if ! "$PY" - "$DRIVER_WS/isolation_check.jsonl" "$MIN_CALLS" <<'PY'
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
            if any(s in text for s in ("tm_seed", "config.yaml", "session.json", "access:", "result.json", "evaluator.log")) and "denied" not in text:
                leaked.append(text[:120])
if calls < int(sys.argv[2]) or leaked:
    print(f"    isolation check FAILED: {calls} tool calls, leaks: {leaked}", file=sys.stderr)
    sys.exit(1)
PY
    then
        abort "driver sandbox is not isolated (transcript: $DRIVER_WS/isolation_check.jsonl)"
    fi
    [ ! -e "$SESSION_DIR/../leak_outside.py" ] || abort "the driver could write outside its code dir"
fi
if [ "$CODE_TOOL" = on ]; then
    # The confinement itself: a script that tries to list the session and read session.json, run
    # through the code tool with exactly the driver's environment. It must be refused.
    printf '%s\n' "import os" "print(os.listdir('$SESSION_DIR'))" "print(open('$SESSION_DIR/session.json').read())" > "$CODE_DIR/leak.py"
    leak_out="$(cd "$DRIVER_WS" && PYTHONPATH="$REPO_ROOT" CARLA_PROBE_FRAMES="$SESSION_DIR/frames" CARLA_PROBE_CODE_DIR="$CODE_DIR" \
        CARLA_PROBE_PERCEPTION_ADDR="$PERCEPTION_ADDR" "$PY" -m carla_probe.code_runner "$CODE_DIR/leak.py" 2>&1 || true)"
    rm -f "$CODE_DIR/leak.py"
    if ! printf '%s' "$leak_out" | grep -q "denied" || printf '%s' "$leak_out" | grep -q -e tm_seed -e evaluator.log -e carla.log; then
        abort "the code tool is not confined: $(printf '%s' "$leak_out" | head -c 300)"
    fi
    echo "    code tool confinement check: refused, as it must be"
fi


# ---- 4. driver -----------------------------------------------------------------------------------
cat > "$SESSION_DIR/driver.json" <<EOF
{"model": "$MODEL", "claude_code": "$(claude --version)", "benchmark": "$BENCHMARK", "track": "$TRACK", "route": "$ROUTE",
 "driver": "claude", "vla": "${VLA:-off}", "autocompact": "$AUTOCOMPACT",
 "code_tool": "$CODE_TOOL", "perception_addr": "$PERCEPTION_ADDR", "code_lib": "$CODE_LIB",
 "code_lib_sha256": "$([ -n "$CODE_LIB" ] && (cd "$CODE_LIB" && find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1))",
 "prompt_sha256": "$(sha256sum "$DRIVER_WS/prompt.txt" | cut -d' ' -f1)",
 "driver_ws": "$DRIVER_WS", "started": "$(date -Iseconds)"}
EOF
echo "    driver running (transcript: $SESSION_DIR/driver_transcript.jsonl)"
set +e
(cd "$DRIVER_WS" && claude -p "$(cat prompt.txt)" --model "$MODEL" --output-format stream-json --verbose \
    "${DRIVER_FLAGS[@]}" < /dev/null > "$SESSION_DIR/driver_transcript.jsonl" 2> "$SESSION_DIR/driver_stderr.log")
driver_rc=$?
set -e

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
"$PY" - "$SESSION_DIR/driver.json" "$driver_rc" "$stopped_early" <<'PY'
import datetime, json, sys
path, rc, early = sys.argv[1], int(sys.argv[2]), sys.argv[3] == "true"
d = json.load(open(path))
d.update(ended=datetime.datetime.now().astimezone().isoformat(timespec="seconds"), driver_exit_code=rc,
         driver_stopped_before_route_end=early)
json.dump(d, open(path, "w"), indent=1)
PY
if [ "$stopped_early" = true ]; then
    echo "    driver exited (rc=$driver_rc) before the route ended: aborting the session (agent failure)"
    "$PY" -m carla_probe.client "$ADDR" shutdown why="operator abort: the driver exited before the route ended" \
        > /dev/null 2>&1 || true
fi
wait "$LAUNCH_PID" || true
# What the driver wrote is part of the result: keep it beside the transcript.
[ "$CODE_TOOL" != on ] || cp -r "$CODE_DIR" "$SESSION_DIR/driver_code" 2>/dev/null || true
echo "=== eval done -> $SESSION_DIR"
"$PY" "$REPO_ROOT/tools/summarize_results.py" "$SESSION_DIR" || true
