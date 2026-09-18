#!/usr/bin/env bash
# Run BLUE's own agent through BLUE's own leaderboard on one route: the reference number nothing of
# ours touches (no probe daemon, no vla_server.py). Mirrors gate/evaluation/eval_blue_full.sh's
# run_single_route with its published settings (trained_gate, threshold 0.66, seed 1) on one route.
#
#   bash scripts/run_blue_eval.sh <ROUTE_ID> [SESSION_DIR]
#
# Env: GPU_RANK (0)  CARLA_PORT (20000)  TM_PORT (CARLA_PORT+20000)  TM_SEED (1, BLUE's)
#      CARLA_ROOT ($HOME/carla)  TIMEOUT_S (4000, BLUE's)
#      MODE (blue): blue = SimLingo + BLUE's gate, the published pipeline; simlingo = the gate off,
#           commentary generated on every frame (SimLingo's own paper setting)
#      DTYPE (bf16, the agent's own): fp32 | fp16 run the stock agent with the model in that format
#           (carla_probe/blue_dtype_agent.py) -- a numerics diagnostic for GPUs without bf16
#      AGENT (stock): stock = BLUE's agent untouched; oracle = carla_probe/blue_oracle_agent.py,
#           the same agent with the LLM in config `blue_oracle` writing the commentary when the
#           gate fires (MODE=blue only)
# Needs scripts/setup_vla.sh (the venv, the SimLingo checkpoint) and third_party/BLUE.
# Output: SESSION_DIR/res.json (BLUE's leaderboard checkpoint), viz/ (its SAVE_PATH: images,
# metric_info.json, blue_gate_decisions.jsonl), evaluator.log. Summarize with
# tools/summarize_results.py after copying res.json to result.json, or read it directly.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUTE_ID="${1:?usage: run_blue_eval.sh <ROUTE_ID> [SESSION_DIR]}"
BLUE="$REPO_ROOT/third_party/BLUE"
PY="$REPO_ROOT/third_party/vla_venv/bin/python"
CKPT="$REPO_ROOT/third_party/models/simlingo/checkpoints/epoch=013.ckpt/pytorch_model.pt"
: "${GPU_RANK:=0}"
: "${CARLA_PORT:=20000}"
: "${TM_PORT:=$((CARLA_PORT + 20000))}"
: "${TM_SEED:=1}"
: "${CARLA_ROOT:=$HOME/carla}"
: "${TIMEOUT_S:=4000}"
: "${MODE:=blue}"
: "${AGENT:=stock}"
: "${DTYPE:=bf16}"
case "$MODE" in blue | simlingo) ;; *) echo "ERROR: MODE must be blue or simlingo" >&2; exit 1 ;; esac
case "$AGENT" in
    stock) AGENT_FILE="$BLUE/team_code/agent_simlingo.py"
           if [ "$DTYPE" != bf16 ]; then AGENT_FILE="$REPO_ROOT/carla_probe/blue_dtype_agent.py"; export BLUE_DTYPE="$DTYPE"; fi ;;
    oracle) AGENT_FILE="$REPO_ROOT/carla_probe/blue_oracle_agent.py"
            [ "$MODE" = blue ] || { echo "ERROR: AGENT=oracle needs MODE=blue (the gate decides when the oracle speaks)" >&2; exit 1; }
            command -v claude >/dev/null || { echo "ERROR: claude (Claude Code CLI) not on PATH" >&2; exit 1; } ;;
    *) echo "ERROR: AGENT must be stock or oracle" >&2; exit 1 ;;
esac
[[ -x "$PY" && -f "$CKPT" ]] || { echo "ERROR: run scripts/setup_vla.sh first" >&2; exit 1; }
[[ -f "$CARLA_ROOT/CarlaUE4.sh" ]] || { echo "ERROR: no CARLA at CARLA_ROOT=$CARLA_ROOT" >&2; exit 1; }

# BLUE ships the 220 routes as bench2drive_NN.xml; find the one holding this route id.
ROUTE_XML="$(grep -l "route id=\"$ROUTE_ID\"" "$BLUE"/data/routes/bench2drive_split/*.xml | head -1)"
[[ -n "$ROUTE_XML" ]] || { echo "ERROR: route $ROUTE_ID not in $BLUE/data/routes/bench2drive_split" >&2; exit 1; }

SESSION_DIR="$(realpath -m "${2:-$REPO_ROOT/sessions/$(date +%Y%m%d_%H%M%S)_route_${ROUTE_ID}_${MODE}_${AGENT}${DTYPE:+_$DTYPE}}")"
mkdir -p "$SESSION_DIR/viz"
export CARLA_ROOT
export PYTHONPATH="$BLUE/Bench2Drive/leaderboard:$BLUE/Bench2Drive/scenario_runner:$BLUE:$CARLA_ROOT/PythonAPI/carla"
export SCENARIO_RUNNER_ROOT="$BLUE/Bench2Drive/scenario_runner"
export HF_HOME="$REPO_ROOT/third_party/hf_cache"
export ROUTES="$ROUTE_XML"                     # the agent reads its route id from this
export SAVE_PATH="$SESSION_DIR/viz"
if [ "$MODE" = blue ]; then
    export BLUE_MODE=trained_gate BLUE_GATE_CKPT="$BLUE/gate/weights/blue_simlingo_gate.pt" BLUE_GATE_THRESHOLD=0.66
else
    unset BLUE_MODE BLUE_GATE_CKPT BLUE_GATE_THRESHOLD
fi
export BLUE_OUTPUT_DIR="$SESSION_DIR"
export SIMLINGO_CKPT="$CKPT"
export NCCL_NVLS_ENABLE=0
cat > "$SESSION_DIR/session.json" <<EOF
{"driver": "blue_${AGENT}_agent", "mode": "$MODE", "dtype": "$DTYPE", "route": "$ROUTE_ID", "route_xml": "$ROUTE_XML", "tm_seed": $TM_SEED,
 "carla_port": $CARLA_PORT, "gpu_rank": $GPU_RANK, "gate_threshold": 0.66, "started": "$(date -Iseconds)"}
EOF
echo "=== BLUE: route $ROUTE_ID, mode $MODE, agent $AGENT ($ROUTE_XML)"
echo "    session: $SESSION_DIR"
# BLUE's evaluator would start CARLA itself, but with no graphics adapter, and the agent takes
# `cuda` = device 0: everything lands on GPU 0 whatever --gpu-rank says. Start CARLA on GPU_RANK
# here and confine the evaluator (the model) to the same GPU. `--start-carla=` (empty) is the only
# spelling that argparse's type=bool reads as False.
"$CARLA_ROOT/CarlaUE4.sh" -RenderOffScreen -nosound -graphicsadapter="$GPU_RANK" \
    -carla-rpc-port="$CARLA_PORT" > "$SESSION_DIR/carla.log" 2>&1 &
for _ in $(seq 1 60); do
    sleep 3
    "$PY" -c "import carla,sys; c=carla.Client('127.0.0.1',$CARLA_PORT); c.set_timeout(5.0); c.get_server_version()" 2>/dev/null && break
done
export CUDA_VISIBLE_DEVICES="$GPU_RANK"
cd "$BLUE"
"$PY" -u Bench2Drive/leaderboard/leaderboard/leaderboard_evaluator.py --start-carla= \
    --routes="$ROUTE_XML" --repetitions=1 --track=SENSORS \
    --checkpoint="$SESSION_DIR/res.json" --debug-checkpoint="$SESSION_DIR/live_results.txt" \
    --timeout="$TIMEOUT_S" --agent="$AGENT_FILE" --agent-config="$CKPT" \
    --traffic-manager-seed="$TM_SEED" --port="$CARLA_PORT" --traffic-manager-port="$TM_PORT" \
    --gpu-rank="$GPU_RANK" > "$SESSION_DIR/evaluator.log" 2>&1 || echo "    evaluator exited non-zero (see evaluator.log)"
pkill -f "CarlaUE4.*-carla-rpc-port=$CARLA_PORT" 2>/dev/null || true
echo "=== done -> $SESSION_DIR"
"$PY" - "$SESSION_DIR/res.json" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception as e:
    print(f"no result: {e}"); sys.exit(0)
for r in d.get("_checkpoint", {}).get("records", []):
    s = r["scores"]
    print(f"| {r['route_id']} | {r['status']} | DS {s['score_composed']:.2f} | RC {s['score_route']:.2f} | "
          f"IS {s['score_penalty']:.3f} | " + ", ".join(f"{k}={len(v)}" for k, v in r["infractions"].items() if v) + " |")
PY
