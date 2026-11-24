#!/usr/bin/env bash
# After the A0+ lanes exit: start the strong perception service (Depth Pro 2 + SAM 3) on 8795, run
# B3s+ (cameras + code tool + strong perception, plus controller), then stop the service.
# Lanes still pass quota_gate (run window, usage caps, ~/workspace/quota_cap).
while pgrep -f "[l]anes3.sh .*a0plus/jobs_a1.txt" > /dev/null; do sleep 60; done
echo "$(date -Is) A0+ lanes done; starting strong perception service"
cd "$HOME/workspace/ccd_priv2" || exit 1
uv python install 3.11 3.12 > /dev/null 2>&1   # the venvs link to uv's interpreters; ~/.local is not kept
BACKEND=strong PERCEPTION_PY="$HOME/workspace/venv_perception2/bin/python" \
    nohup bash scripts/run_perception_server.sh > sessions/ladder_plus/perception_b3s.log 2>&1 &
SRV=$!
for i in $(seq 60); do curl -sf http://127.0.0.1:8795/health > /dev/null && break; sleep 10; done
curl -sf http://127.0.0.1:8795/health || { echo "perception service did not start"; kill $SRV; exit 1; }
echo
bash "$HOME/workspace/lanes3.sh" "$HOME/workspace/ccd_priv2" "$HOME/workspace/ccd_priv2/sessions/ladder_plus/jobs_b3s.txt" 3 6
echo "$(date -Is) B3s+ lanes done; stopping perception service"
pkill -f "perception_server.py --port 8795"
