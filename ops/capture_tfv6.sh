#!/usr/bin/env bash
# Capture labelled scenes (privileged + GT dense + full-res frames + TransFuser camera ring) for
# validating the TransFuser perception service (carla_probe/tfv6_server.py).
#   bash capture_tfv6.sh <OUT_ROOT> <SLOT> <ROUTE> [ROUTE ...]
# Operator infrastructure. Scripted driving (no model): costs only wall-clock.
OUT="$1"; SLOT="$2"; shift 2
source "$HOME/workspace/hornet_envrc"
REPO="$HOME/workspace/ccd_b5"
cd "$REPO" && source scripts/env.sh > /dev/null 2>&1
export CARLA_PORT=$((20000 + 10 * SLOT)) PROBE_PORT=$((8790 + SLOT)) GPU_RANK=0
for r in "$@"; do
    S="$OUT/capture_route_$r"
    [ -e "$S/route_scenes/scenes.json" ] && { echo "skip $r"; continue; }
    rm -rf "$S"; mkdir -p "$S"
    echo "$(date -Is) capture $r"
    ACCESS=privileged GT_DENSE=1 CODE_FULL_RES=1 TFV6_CAMERAS=1 bash scripts/run_session.sh "$r" "$S" > "$S.launcher.log" 2>&1 &
    LP=$!
    python tools/capture_route_scenes.py "$S" --addr "127.0.0.1:$PROBE_PORT" --every 40 --max 60 --wait 1200 \
        > "$S.capture.log" 2>&1
    python -m carla_probe.client "127.0.0.1:$PROBE_PORT" shutdown why="capture finished" > /dev/null 2>&1
    sleep 10; kill $LP 2>/dev/null; bash scripts/clean_carla.sh "$CARLA_PORT" > /dev/null 2>&1
    echo "$(date -Is) done $r: $(tail -1 "$S.capture.log")"
done
