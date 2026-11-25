#!/usr/bin/env bash
# Commentary videos for the B3s+ (strong perception) and A0+ sessions. CARLA replay only: no Claude usage.
source "$HOME/workspace/hornet_envrc"
cd "$HOME/workspace/ccd_priv2" && source scripts/env.sh > /dev/null 2>&1
r() {  # session from to view out hold
    echo "$(date -Is) $1 $5"
    timeout 3000 python tools/replay_sim.py "$1" frames --from-step "$2" --to-step "$3" --view "$4" --video --captions \
        --slow 2 --hold "$6" --out "$1/replay/$5" --port 20200 > "$1/replay_$5.log" 2>&1 \
        && rm -f "$1"/replay/$5/frame_*.jpg && echo "  ok" || echo "  failed"
}
last() { tail -n 1 "$1/trajectory.jsonl" | python3 -c 'import json,sys; print(json.load(sys.stdin)["step"])'; }
B=$HOME/workspace/ccd_priv2/sessions/ladder_plus/B3s_strong_perc/a1
for s in 2091 3255 28198 25424; do S=$B/route_$s; r "$S" 0 "$(last "$S")" chase commentary 4; done
S=$HOME/workspace/ccd_priv2/sessions/a0plus/a1/route_25378_privileged; r "$S" 0 "$(last "$S")" chase commentary 4
echo "$(date -Is) done"
