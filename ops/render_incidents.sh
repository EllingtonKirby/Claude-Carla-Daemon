#!/usr/bin/env bash
# Short captioned clips of the four privileged-arm incidents. No Claude usage (CARLA replay only).
source "$HOME/workspace/hornet_envrc"
cd "$HOME/workspace/ccd_exp" && source scripts/env.sh > /dev/null 2>&1
B=$HOME/workspace/ccd_base/sessions/priv_opus/a1
render() {  # session from to view name
    echo "$(date -Is) $5"
    timeout 2400 python tools/replay_sim.py "$1" frames --from-step "$2" --to-step "$3" --view "$4" --video --captions \
        --slow 2 --hold 3 --out "$1/replay/incident_$5" --port 20200 > "$1/replay_incident_$5.log" 2>&1 \
        && rm -f "$1"/replay/incident_$5/frame_*.jpg && echo "  ok" || echo "  failed"
}
render $B/route_2091_privileged 300 440 top stop_sign
render $B/route_2091_privileged 580 690 chase left_turn_collision
render $B/route_25424_privileged 470 575 chase cone
render $B/route_25378_privileged 100 259 top ambulance
echo "$(date -Is) done"
