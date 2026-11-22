#!/usr/bin/env bash
# Run a job list from a shared queue, in N parallel lanes. Operator infrastructure, not in a repo.
#
#   bash lanes.sh <REPO> <JOBS_FILE> <N_LANES> <FIRST_SLOT>
#
# JOBS_FILE lines:  <session_dir_relative_to_REPO/sessions> <route> <track> <model> [ENV=VAL ...]
# Slot s uses CARLA_PORT 20000+10*s and PROBE_PORT 8790+s (slot 5 is skipped: 8795 is the perception
# service). A lane claims a job by mkdir of sessions/<sdir>.claim -- atomic, so lanes started later
# (another invocation, other slots) share the same queue. A job whose session already has a
# result.json and driver.json is skipped, never overwritten. Lanes start 45 s apart (JOURNAL §1.6).
REPO="$1"; JOBS="$2"; LANES="$3"; FIRST="${4:-0}"
source "$HOME/workspace/hornet_envrc"
cd "$REPO" || exit 1
port_busy() { (exec 3<>/dev/tcp/127.0.0.1/$1) 2>/dev/null; }
lane() {
    local slot=$1
    # Never share a slot: a second session on a live CARLA port loads its route INTO the running one
    # (it happened: RELAUNCHES.md, 2026-11-18). Refuse the lane if either port answers.
    if port_busy $((20000 + 10 * slot)) || port_busy $((8790 + slot)); then
        echo "$(date -Is) slot $slot REFUSED: CARLA $((20000 + 10 * slot)) or probe $((8790 + slot)) is in use"; return 1
    fi
    while IFS= read -r line; do
        [ -z "$line" ] && continue; case "$line" in \#*) continue ;; esac
        read -r sdir route track model rest <<< "$line"
        mkdir -p "$(dirname "sessions/$sdir")"
        mkdir "sessions/$sdir.claim" 2>/dev/null || continue
        if [ -e "sessions/$sdir/result.json" ] && [ -e "sessions/$sdir/driver.json" ]; then echo "skip $sdir"; continue; fi
        [ -e "sessions/$sdir" ] && mv "sessions/$sdir" "sessions/${sdir}_crashed_$(date +%s)"
        if port_busy $((20000 + 10 * slot)) || port_busy $((8790 + slot)); then
            echo "$(date -Is) slot $slot: port still busy before $sdir; waiting"; sleep 60
            bash scripts/clean_carla.sh $((20000 + 10 * slot)) > /dev/null 2>&1
        fi
        echo "$(date -Is) slot $slot start $sdir"
        env $rest GPU_RANK=0 CARLA_PORT=$((20000 + 10 * slot)) PROBE_PORT=$((8790 + slot)) \
            bash scripts/run_eval.sh "$route" "$track" "$model" "sessions/$sdir" > "sessions/$sdir.out" 2>&1 < /dev/null
        echo "$(date -Is) slot $slot done $sdir rc=$?"
        bash scripts/clean_carla.sh $((20000 + 10 * slot)) > /dev/null 2>&1
    done < "$JOBS"
}
slot=$FIRST
for i in $(seq 1 "$LANES"); do
    [ "$slot" -eq 5 ] && slot=$((slot + 1))
    lane "$slot" &
    slot=$((slot + 1))
    sleep 45
done
wait
echo "$(date -Is) lanes done"
