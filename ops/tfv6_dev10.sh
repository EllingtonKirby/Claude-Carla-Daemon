#!/usr/bin/env bash
# TransFuser v6 (LEAD) alone on the dev10 routes, for reference. No Claude usage.
# Usage: bash tfv6_dev10.sh CHECKPOINT_NAME [route ...]
CK=${1:-visiononly_resnet34}; shift
ROUTES=${@:-3514 3255 26405 25381 25378 25424 2091 27494 17569 28198}
source "$HOME/workspace/hornet_envrc" > /dev/null 2>&1
export SDL_VIDEODRIVER=dummy LEAD_PROJECT_ROOT=$HOME/workspace/lead
unset LD_LIBRARY_PATH
PORT=20300; TM=20350
OUT=$HOME/workspace/lead_runs/$CK
mkdir -p $OUT
cd $LEAD_PROJECT_ROOT
for r in $ROUTES; do
  python3 -c "import json,sys; sys.exit(0 if json.load(open('$OUT/$r/checkpoint_endpoint.json'))['_checkpoint']['records'] else 1)" 2>/dev/null && { echo "$(date -Is) $r done already"; continue; }
  rm -rf $OUT/$r
  echo "$(date -Is) route $r"
  $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Epic -carla-rpc-port=$PORT -carla-streaming-port=0 > $OUT/carla_$r.log 2>&1 &
  CP=$!
  for i in $(seq 60); do (exec 3<>/dev/tcp/127.0.0.1/$PORT) 2>/dev/null && break; sleep 5; done
  sleep 20
  timeout 3600 $HOME/workspace/venv_lead/bin/python -m lead --checkpoint outputs/checkpoints/$CK \
      --routes data/benchmark_routes/bench2drive/$r.xml --bench2drive --port $PORT --traffic-manager-port $TM \
      --output-dir $OUT/$r > $OUT/lead_$r.log 2>&1
  echo "  exit $?"
  kill $CP 2>/dev/null; sleep 3; pkill -f "carla-rpc-port=$PORT"
  for i in $(seq 30); do pgrep -f "carla-rpc-port=$PORT" > /dev/null || break; sleep 2; done
  pkill -9 -f "carla-rpc-port=$PORT"; sleep 10
done
echo "$(date -Is) all done"
