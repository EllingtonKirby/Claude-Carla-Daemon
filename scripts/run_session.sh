#!/usr/bin/env bash
# Launch one probe session: a CARLA server + the Bench2Drive leaderboard running the probe agent on
# ONE route. The sim pauses on its first tick and waits for commands on the probe socket.
#
#   bash scripts/run_session.sh <ROUTE_ID | ROUTE_XML> [SESSION_DIR]
#   bash scripts/run_session.sh 2091                         # routes/bench2drive220/route_2091.xml
#   ACCESS=sensors bash scripts/run_session.sh 2091          # Bench2Drive-eligible sensors only
#
# Then drive it (after ~1-3 min of CARLA + route loading):
#   python -m carla_probe.client --wait 600 status
#
# Blocks until the route ends. Artifacts land in SESSION_DIR (default sessions/<stamp>_route<id>/):
#   result.json (leaderboard scores), commands.jsonl, trajectory.jsonl, evaluator.log, carla.log,
#   frames/, camera_log/, config.yaml (snapshot), session.json (launch metadata).
#
# Env: ACCESS (privileged | sensors; default = the config's `access`)
#      CARLA_ROOT ($HOME/carla)  GPU_RANK (0)  CARLA_PORT (20000)  TM_PORT (CARLA_PORT+20000)
#      TM_SEED (0)  PROBE_CONFIG (carla_probe/config.yaml)  PROBE_PORT / PROBE_HOST (from config)
#      CARLA_EXTERNAL=1 to reuse an already-running CARLA server on CARLA_PORT
#      AGENT_MAX_TICKS (4000 = the stock 200 s route budget; changing it makes scores non-standard)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUTE_ARG="${1:?usage: run_session.sh <ROUTE_ID | ROUTE_XML> [SESSION_DIR]}"

if [ -z "${LEADERBOARD_ROOT:-}" ]; then
    # shellcheck disable=SC1091
    source "$REPO_ROOT/scripts/env.sh"
fi

if [ -f "$ROUTE_ARG" ]; then
    ROUTES="$(realpath "$ROUTE_ARG")"   # the leaderboard crashes on relative route paths
elif [[ "$ROUTE_ARG" =~ ^[0-9]+$ ]]; then
    ROUTES="$REPO_ROOT/routes/bench2drive220/route_${ROUTE_ARG}.xml"
    [ -f "$ROUTES" ] || { echo "ERROR: no route file $ROUTES (run scripts/setup_bench2drive.sh)" >&2; exit 1; }
else
    echo "ERROR: $ROUTE_ARG is neither a route id nor an XML file" >&2
    exit 1
fi
ROUTE_STEM="$(basename "$ROUTES" .xml)"
SESSION_DIR="$(realpath -m "${2:-$REPO_ROOT/sessions/$(date +%Y%m%d_%H%M%S)_${ROUTE_STEM}}")"

if [ -e "$SESSION_DIR/result.json" ]; then
    # The leaderboard resume logic would silently SKIP driving a route already recorded here.
    echo "ERROR: $SESSION_DIR already has a result.json; use a fresh SESSION_DIR" >&2
    exit 1
fi

: "${GPU_RANK:=0}"
: "${CARLA_PORT:=20000}"
: "${TM_PORT:=$((CARLA_PORT + 20000))}"
: "${TM_SEED:=0}"
: "${PROBE_CONFIG:=$REPO_ROOT/carla_probe/config.yaml}"
export PROBE_PORT PROBE_HOST AGENT_MAX_TICKS 2>/dev/null || true
# Effectively disables the leaderboard's wall-clock watchdogs (agent, sim and client timeouts all
# ride --timeout): they exist to kill hung agents, and this agent waits for commands on purpose.
PROBE_TIMEOUT=604800

mkdir -p "$SESSION_DIR"
cp "$PROBE_CONFIG" "$SESSION_DIR/config.yaml"
if [ -n "${ACCESS:-}" ]; then
    case "$ACCESS" in
        privileged | sensors) sed -i -E "s/^access:.*/access: $ACCESS/" "$SESSION_DIR/config.yaml" ;;
        *) echo "ERROR: ACCESS must be privileged or sensors, got '$ACCESS'" >&2; exit 1 ;;
    esac
fi
ACCESS_EFFECTIVE="$(awk '/^access:/{print $2}' "$SESSION_DIR/config.yaml")"
PROBE_ADDR="${PROBE_HOST:-$(awk '/^host:/{print $2}' "$PROBE_CONFIG")}:${PROBE_PORT:-$(awk '/^port:/{print $2}' "$PROBE_CONFIG")}"
cat > "$SESSION_DIR/session.json" <<EOF
{"routes": "$ROUTES", "access": "$ACCESS_EFFECTIVE", "tm_seed": $TM_SEED, "carla_port": $CARLA_PORT, "probe_addr": "$PROBE_ADDR",
 "max_ticks": ${AGENT_MAX_TICKS:-4000}, "started": "$(date -Iseconds)",
 "repo_commit": "$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"}
EOF

echo "=== CARLA probe session ==="
echo "    route:    $ROUTES"
echo "    access:   $ACCESS_EFFECTIVE"
echo "    session:  $SESSION_DIR"
echo "    connect:  python -m carla_probe.client $PROBE_ADDR --wait 600 status"

CARLA_PID=""
EVAL_PID=""
cleanup() {
    [ -n "$EVAL_PID" ] && kill -9 "$EVAL_PID" 2>/dev/null
    if [ -z "${CARLA_EXTERNAL:-}" ]; then
        # CarlaUE4.sh is a wrapper: kill the real binary by its unique RPC port too, or it keeps the GPU.
        pkill -9 -f "carla-rpc-port=$CARLA_PORT" 2>/dev/null
        [ -n "$CARLA_PID" ] && kill -9 "$CARLA_PID" 2>/dev/null
    fi
    return 0
}
trap cleanup EXIT

if [ -z "${CARLA_EXTERNAL:-}" ]; then
    "$CARLA_ROOT/CarlaUE4.sh" -RenderOffScreen -nosound -graphicsadapter="$GPU_RANK" \
        -carla-rpc-port="$CARLA_PORT" >"$SESSION_DIR/carla.log" 2>&1 &
    CARLA_PID=$!
fi

echo -n "    waiting for CARLA"
for _ in $(seq 1 90); do
    if python - "$CARLA_PORT" <<'PY' 2>/dev/null
import sys, carla
client = carla.Client("localhost", int(sys.argv[1]))
client.set_timeout(2.0)
client.get_server_version()
PY
    then echo " ready."; break; fi
    if [ -n "$CARLA_PID" ] && ! kill -0 "$CARLA_PID" 2>/dev/null; then
        echo " CARLA died (see $SESSION_DIR/carla.log)"; exit 1
    fi
    echo -n "."; sleep 2
done

cd "$SESSION_DIR"
CUDA_VISIBLE_DEVICES="$GPU_RANK" python "$LEADERBOARD_ROOT/leaderboard/leaderboard_evaluator.py" \
    --routes="$ROUTES" \
    --repetitions=1 \
    --track=SENSORS \
    --checkpoint="$SESSION_DIR/result.json" \
    --agent="$REPO_ROOT/carla_probe/agent.py" \
    --agent-config="$SESSION_DIR/config.yaml" \
    --resume=False \
    --timeout="$PROBE_TIMEOUT" \
    --port="$CARLA_PORT" \
    --traffic-manager-port="$TM_PORT" \
    --traffic-manager-seed="$TM_SEED" \
    --gpu-rank="$GPU_RANK" >"$SESSION_DIR/evaluator.log" 2>&1 &
EVAL_PID=$!
echo "    evaluator running (log: $SESSION_DIR/evaluator.log)"

# The evaluator can hang in teardown AFTER scores are written (it then ignores SIGINT). Once the
# agent has been destroyed, give it up to 120 s to write the global record, then kill it.
done_since=""
while kill -0 "$EVAL_PID" 2>/dev/null; do
    if [ -f "$SESSION_DIR/probe_done.json" ]; then
        [ -z "$done_since" ] && done_since=$(date +%s)
        if python3 -c "import json,sys; d=json.load(open('result.json')); sys.exit(0 if d.get('_checkpoint',{}).get('global_record') else 1)" 2>/dev/null \
            || [ $(( $(date +%s) - done_since )) -ge 120 ]; then
            sleep 5
            kill -9 "$EVAL_PID" 2>/dev/null || true
            break
        fi
    fi
    sleep 5
done
wait "$EVAL_PID" 2>/dev/null || true
EVAL_PID=""

echo "=== probe session ended -> $SESSION_DIR"
python3 "$REPO_ROOT/scripts/summarize_results.py" "$SESSION_DIR" || true
