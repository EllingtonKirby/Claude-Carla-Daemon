#!/usr/bin/env bash
# Kill CARLA servers + leaderboard evaluators left behind by probe sessions.
#
#   bash scripts/clean_carla.sh            # every CarlaUE4 / leaderboard_evaluator owned by you
#   bash scripts/clean_carla.sh 20000      # only the session using CARLA RPC port 20000
#
# Needed because the evaluator can hang in teardown after a route ends (data is already written;
# it ignores SIGINT). Deliberately targeted: never a blanket GPU-process kill, which would also take
# out unrelated jobs sharing the machine.
set -u

if [ $# -ge 1 ]; then
    pkill -9 -u "$(id -u)" -f "carla-rpc-port=$1" 2>/dev/null && echo "killed CARLA on port $1"
    pkill -9 -u "$(id -u)" -f "leaderboard_evaluator.py.*--port=$1" 2>/dev/null && echo "killed evaluator on port $1"
else
    pkill -9 -u "$(id -u)" -f "CarlaUE4" 2>/dev/null && echo "killed CarlaUE4 servers"
    pkill -9 -u "$(id -u)" -f "leaderboard_evaluator" 2>/dev/null && echo "killed leaderboard evaluators"
fi
echo "clean_carla: done"
exit 0
