#!/usr/bin/env bash
# Every minute: a session whose CARLA server has crashed (segfault in carla.log, no result record) is
# hung for good -- the evaluator waits on the dead server with no timeout. End it, clean its port,
# quarantine it to <root>/_infra/<stamp>/ and append its job line again so a lane reruns it.
# Operator infrastructure (2026-11-20: route 25381 hung 40 min on a crashed server).
LOG=$HOME/workspace/carla_watchdog.log
while true; do
    pgrep -af "scripts/run_eval.sh" | while read -r pid rest; do
        sdir=$(echo "$rest" | awk '{print $NF}')
        repo=$(readlink -f /proc/$pid/cwd)
        S="$repo/$sdir"
        [ -f "$S/carla.log" ] || continue
        python3 -c "import json,sys; r=json.load(open('$S/result.json'))['_checkpoint']['records']; sys.exit(0 if r else 1)" 2>/dev/null && continue
        port=$(python3 -c "import json; print(json.load(open('$S/session.json'))['carla_port'])" 2>/dev/null)
        dead=0
        if grep -q -e "Segmentation fault" -e "Signal=11" "$S/carla.log"; then dead=1
        elif [ -n "$port" ] && [ $(( $(date +%s) - $(stat -c %Y "$S/session.json") )) -gt 360 ] \
             && ! (exec 3<>/dev/tcp/127.0.0.1/$port) 2>/dev/null; then
            # the server is gone however it died (killed, OOM, segfault without a log line): two strikes
            strike="$S/.watchdog_strike"
            if [ -f "$strike" ]; then dead=1; else touch "$strike"; fi
        else
            rm -f "$S/.watchdog_strike"
        fi
        [ "$dead" = 1 ] || continue
        echo "$(date -Is) dead server for $sdir (port $port): ending it and requeueing" >> "$LOG"
        pkill -TERM -P "$pid" 2>/dev/null; kill "$pid" 2>/dev/null; sleep 5
        pkill -f "$sdir" 2>/dev/null
        [ -n "$port" ] && (cd "$repo" && bash scripts/clean_carla.sh "$port" > /dev/null 2>&1)
        sleep 10
        root="$repo/sessions/${sdir#sessions/}"; root="${root%/a*/route_*}"
        rel="${S#$root/}"; stamp=$(date +%Y%m%dT%H%M%S)
        mkdir -p "$root/_infra/$stamp/$(dirname "$rel")"
        mv "$S" "$S.out" "$root/_infra/$stamp/$(dirname "$rel")/" 2>/dev/null; rmdir "$S.claim" 2>/dev/null
        echo "$(date -Is) ${rel} INFRA: its CARLA server died mid-route; ended by carla_watchdog.sh, quarantined to _infra/$stamp and requeued." >> "$root/RELAUNCHES.md"
        line=$(grep -h "^${sdir#sessions/} " "$root"/jobs*.txt 2>/dev/null | head -n 1)
        [ -n "$line" ] && for j in "$root"/jobs_full.txt; do [ -f "$j" ] && echo "$line" >> "$j"; done
    done
    sleep 60
done
