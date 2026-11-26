#!/usr/bin/env bash
# Commentary videos for the A1 (Privileged+) sessions. CARLA replay only: no Claude usage.
source "$HOME/workspace/hornet_envrc"
cd "$HOME/workspace/ccd_priv2" && source scripts/env.sh > /dev/null 2>&1
A=$HOME/workspace/ccd_priv2/sessions/priv_v2/a1
r() {  # session from to view out
    echo "$(date -Is) $5"
    timeout 3000 python tools/replay_sim.py "$1" frames --from-step "$2" --to-step "$3" --view "$4" --video --captions \
        --slow 2 --hold "$6" --out "$1/replay/$5" --port 20200 > "$1/replay_$5.log" 2>&1 \
        && rm -f "$1"/replay/$5/frame_*.jpg && echo "  ok" || echo "  failed"
}
last() { tail -n 1 "$1/trajectory.jsonl" | python3 -c 'import json,sys; print(json.load(sys.stdin)["step"])'; }
for s in "$@"; do
    S=$A/route_${s}_privileged
    r "$S" 0 "$(last "$S")" chase commentary 4
done
[ -d $A/route_2091_privileged ] && r $A/route_2091_privileged 140 260 chase incident_left_turn 3
[ -d $A/route_25378_privileged ] && r $A/route_25378_privileged 40 110 rear incident_ambulance 3
echo "$(date -Is) done"
