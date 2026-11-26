#!/usr/bin/env bash
# Render a captioned commentary video (tools/replay_sim.py --captions) for every VALID session that
# lacks one; repeat every 20 min until nothing is driving and everything is rendered.
# Output: <session>/replay/commentary/replay.mp4  (chase view, half speed, 4 s hold per decision)
source "$HOME/workspace/hornet_envrc"
cd "$HOME/workspace/ccd_exp" && source scripts/env.sh > /dev/null 2>&1
while true; do
    todo=0
    for s in $(python3 "$HOME/workspace/classify_sessions.py" --json | python3 -c '
import json, sys
for r in json.load(sys.stdin):
    if r["kind"] == "valid": print(r["path"])'); do
        [ -f "$s/replay/commentary/replay.mp4" ] && continue
        ls "$s"/*_rep0.log > /dev/null 2>&1 || { echo "no recording: $s"; continue; }
        todo=1
        last=$(tail -n 1 "$s/trajectory.jsonl" | python3 -c 'import json,sys; print(json.load(sys.stdin)["step"])')
        every=1; [ "$last" -gt 1200 ] && every=2
        echo "$(date -Is) render $s (steps 0-$last, every $every)"
        timeout 7200 python tools/replay_sim.py "$s" frames --from-step 0 --to-step "$last" --every "$every" --view chase \
            --video --captions --slow 2 --hold 4 --out "$s/replay/commentary" --port 20200 > "$s/replay_commentary.log" 2>&1 \
            && rm -f "$s"/replay/commentary/frame_*.jpg \
            || echo "  failed: see $s/replay_commentary.log"
    done
    if [ "$todo" = 0 ] && ! pgrep -f "scripts/run_eval.sh" > /dev/null && ! pgrep -f "teach_perception.py" > /dev/null \
        && ! pgrep -f "b4_watch.sh" > /dev/null; then
        echo "$(date -Is) all rendered"; break
    fi
    sleep 1200
done
