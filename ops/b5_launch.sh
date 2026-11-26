#!/usr/bin/env bash
# B5: cameras + code + strong perception + driver library + TransFuser v6 perception (scene tool),
# 10 routes. Starts the TransFuser service (LEAD's interpreter) and the strong perception service
# forwarding /scene to it, runs the lanes, stops both. Needs ccd_priv2 on a branch with B5
# (b5-scene). Usage: bash b5_launch.sh [START_UTC "YYYY-MM-DD HH:MM"] [CAP]
START=${1:-}; CAP=${2:-}
if [ -n "$START" ]; then T=$(date -u -d "$START" +%s); while [ "$(date +%s)" -lt "$T" ]; do sleep 60; done; fi
[ -n "$CAP" ] && echo "$CAP $(date -u -d '+20 hours' +%s)" > "$HOME/workspace/quota_cap"
R=$HOME/workspace/ccd_priv2
cd "$R" || exit 1
git -C "$R" merge-base --is-ancestor b5-scene HEAD 2>/dev/null || { echo "ccd_priv2 is not on b5-scene"; exit 1; }
uv python install 3.10 3.11 3.12 > /dev/null 2>&1
env -u LD_LIBRARY_PATH PYTHONPATH=$HOME/workspace/lead nohup $HOME/workspace/venv_lead/bin/python carla_probe/tfv6_server.py \
    --checkpoint $HOME/workspace/lead/outputs/checkpoints/visiononly_resnet34 --port 8785 > sessions/ladder_plus/tfv6_b5.log 2>&1 &
TF=$!
for i in $(seq 60); do curl -sf http://127.0.0.1:8785/health > /dev/null && break; sleep 5; done
BACKEND=strong TFV6_ADDR=127.0.0.1:8785 PERCEPTION_PY="$HOME/workspace/venv_perception2/bin/python" \
    nohup bash scripts/run_perception_server.sh > sessions/ladder_plus/perception_b5.log 2>&1 &
SRV=$!
for i in $(seq 60); do curl -sf http://127.0.0.1:8795/health > /dev/null && break; sleep 10; done
curl -sf http://127.0.0.1:8795/health > /dev/null || { echo "perception service did not start"; kill $TF $SRV; exit 1; }
pgrep -f "[c]arla_watchdog.sh" > /dev/null || (nohup setsid bash "$HOME/workspace/carla_watchdog.sh" > /dev/null 2>&1 < /dev/null &)
echo "$(date -Is) services up; starting B5 lanes"
bash "$HOME/workspace/lanes3.sh" "$R" "$R/sessions/ladder_plus/jobs_b5.txt" 3 6
echo "$(date -Is) B5 lanes done; stopping services"
kill $SRV $TF; sleep 2; kill $(pgrep -f "perception_server.py --port 8795") 2>/dev/null
echo "0.0 $(date -u -d '+7 days' +%s)" > "$HOME/workspace/quota_cap"
echo "$(date -Is) cap 0.0"
