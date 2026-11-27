#!/usr/bin/env bash
# B3s+E: B3s+ (cameras + code tool + strong perception, no driver library) with EGO_STATE=1
# (ego.json next to every frame), 10 routes. Starts the strong perception service, runs the lanes,
# stops the service and sets the cap to 0.0. Needs ccd_priv2 on a commit with EGO_STATE (PR #10).
# Usage: bash b3se_launch.sh [START_UTC "YYYY-MM-DD HH:MM"] [CAP]
START=${1:-}; CAP=${2:-}
if [ -n "$START" ]; then T=$(date -u -d "$START" +%s); while [ "$(date +%s)" -lt "$T" ]; do sleep 60; done; fi
R=$HOME/workspace/ccd_priv2
cd "$R" || exit 1
grep -q 'EGO_STATE' scripts/run_eval.sh && grep -q '_write_ego_state' carla_probe/agent.py \
    || { echo "$(date -Is) ccd_priv2 has no EGO_STATE support (merge PR #10 and update ccd_priv2 first)"; exit 1; }
[ -n "$CAP" ] && echo "$CAP $(date -u -d '+20 hours' +%s)" > "$HOME/workspace/quota_cap"
uv python install 3.11 3.12 > /dev/null 2>&1
BACKEND=strong PERCEPTION_PY="$HOME/workspace/venv_perception2/bin/python" \
    nohup bash scripts/run_perception_server.sh > sessions/ladder_plus/perception_b3se.log 2>&1 &
SRV=$!
for i in $(seq 60); do curl -sf http://127.0.0.1:8795/health > /dev/null && break; sleep 10; done
curl -sf http://127.0.0.1:8795/health > /dev/null || { echo "perception service did not start"; kill $SRV; exit 1; }
pgrep -f "[c]arla_watchdog.sh" > /dev/null || (nohup setsid bash "$HOME/workspace/carla_watchdog.sh" > /dev/null 2>&1 < /dev/null &)
echo "$(date -Is) service up; starting B3s+E lanes (cap $(cat $HOME/workspace/quota_cap))"
bash "$HOME/workspace/lanes3.sh" "$R" "$R/sessions/ladder_plus/jobs_b3se.txt" 3 6
echo "$(date -Is) B3s+E lanes done; stopping perception service"
kill $SRV; sleep 2; kill $(pgrep -f "perception_server.py --port 8795") 2>/dev/null
echo "0.0 $(date -u -d '+7 days' +%s)" > "$HOME/workspace/quota_cap"
echo "$(date -Is) cap 0.0"
