#!/usr/bin/env bash
# Launch tools/teach_perception.py: teacher on and off. Operator infrastructure.
source "$HOME/workspace/hornet_envrc"
cd "$HOME/workspace/ccd_exp" && source scripts/env.sh > /dev/null 2>&1
SC=sessions/scenes
TR=($SC/capture_route_{24211,24224,24785,24258,24240}/route_scenes)
TE=($SC/capture_route_{24367,23670}/route_scenes)
for t in "$@"; do
    setsid nohup python tools/teach_perception.py --train "${TR[@]}" --test "${TE[@]}" \
        --work "sessions/teach/teacher_$t" --rounds 4 --teacher "$t" --perception-addr 127.0.0.1:8795 \
        > "sessions/teach/teacher_$t.log" 2>&1 < /dev/null &
    sleep 20
done
