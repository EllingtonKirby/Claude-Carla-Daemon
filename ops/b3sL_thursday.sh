#!/usr/bin/env bash
# Thursday 2026-11-26 night: B3s+L (cameras + code + strong perception + driver library, new guide),
# 10 routes. Sleeps until 18:55 Paris, sets the cap, starts the perception service, runs the lanes,
# stops the service and blocks lanes again. Cancel with: pkill -f b3sL_thursday.sh
START=$(date -u -d '2026-11-26 17:55' +%s)
while [ "$(date +%s)" -lt "$START" ]; do sleep 60; done
echo "0.95 $(date -u -d '2026-11-27 12:00' +%s)" > "$HOME/workspace/quota_cap"
echo "$(date -Is) cap 0.95; starting strong perception service"
cd "$HOME/workspace/ccd_priv2" || exit 1
uv python install 3.11 3.12 > /dev/null 2>&1
BACKEND=strong PERCEPTION_PY="$HOME/workspace/venv_perception2/bin/python" \
    nohup bash scripts/run_perception_server.sh > sessions/ladder_plus/perception_b3sL.log 2>&1 &
SRV=$!
for i in $(seq 60); do curl -sf http://127.0.0.1:8795/health > /dev/null && break; sleep 10; done
curl -sf http://127.0.0.1:8795/health || { echo "perception service did not start"; kill $SRV; exit 1; }
echo
pgrep -f "[c]arla_watchdog.sh" > /dev/null || (nohup setsid bash "$HOME/workspace/carla_watchdog.sh" > /dev/null 2>&1 < /dev/null &)
bash "$HOME/workspace/lanes3.sh" "$HOME/workspace/ccd_priv2" "$HOME/workspace/ccd_priv2/sessions/ladder_plus/jobs_b3sL.txt" 3 6
echo "$(date -Is) B3s+L lanes done; stopping perception service"
kill $SRV; sleep 2; kill $(pgrep -f "perception_server.py --port 8795") 2>/dev/null
echo "0.0 $(date -u -d '2026-11-27 12:00' +%s)" > "$HOME/workspace/quota_cap"
echo "$(date -Is) cap 0.0 until the weekly reset"
