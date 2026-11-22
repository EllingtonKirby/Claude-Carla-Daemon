#!/usr/bin/env bash
# Block until the subscription has room for another Opus driving session. Source it, then call
#   quota_gate <five_hour_max> <seven_day_max>     (fractions, e.g. 0.75 0.95)
# It reads the rate-limit windows from a one-word Haiku call (claude -p stream-json reports them)
# and sleeps until the binding window resets. Why: on 2026-11-18 the 5-hour window ran out
# mid-corpus, five drivers were cut off mid-route and the isolation check failed closed on 133
# more. Starting a session only with headroom keeps a driver from being refused mid-route.
# The user's rule (2026-11-19): these evals use too much of the shared subscription to run between
# 09:00 and 19:00 local time. EVAL_TZ (a POSIX TZ string: this container has no zoneinfo; default = Paris time) / EVAL_BLOCK_START / EVAL_BLOCK_END override (hours, 24 h clock).
hours_gate() {
    local tz="${EVAL_TZ:-CET-1CEST,M3.5.0,M10.5.0/3}" start="${EVAL_BLOCK_START:-9}" end="${EVAL_BLOCK_END:-19}" h wait
    while true; do
        h=$(TZ="$tz" date +%-H)
        if [ "$h" -lt "$start" ] || [ "$h" -ge "$end" ]; then return 0; fi
        wait=$(( $(TZ="$tz" date -d "today $end:00" +%s) - $(date +%s) + 60 ))
        echo "$(date -Is) hours gate: $(TZ="$tz" date +%H:%M) in $tz is inside the ${start}:00-${end}:00 block; sleeping $((wait / 60)) min"
        sleep "$wait"
    done
}

quota_gate() {
    hours_gate
    local fmax="${1:-0.75}" wmax="${2:-0.95}" info f w fr wr st now cap_w cap_until
    while true; do
        # Operator cap (~/workspace/quota_cap: "SEVEN_DAY_MAX UNTIL_EPOCH"): a lower weekly ceiling
        # until a date. 2026-11-20 the user: no more than 40% of the weekly budget before Monday.
        cap_w=""; cap_until=0
        if [ -f "$HOME/workspace/quota_cap" ]; then read -r cap_w cap_until < "$HOME/workspace/quota_cap"; fi
        if [ -n "$cap_w" ] && [ "$(date +%s)" -lt "${cap_until:-0}" ] && python3 -c "import sys; sys.exit(0 if $cap_w < $wmax else 1)"; then
            wmax=$cap_w
        else
            wmax="${2:-0.95}"; cap_until=0
        fi
        info="$(cd /tmp && timeout 120 claude -p "Reply OK" --model claude-haiku-4-5 --output-format stream-json \
                --verbose --setting-sources "" 2>/dev/null | grep -m1 '"rate_limit_info"')"
        read -r f fr w wr st <<< "$(python3 -c '
import json, sys
line = sys.stdin.read()
try:
    d = json.loads(line)["rate_limit_info"]
    u = d.get("unifiedWindows", {})
    fh, sd = u.get("five_hour", {}), u.get("seven_day", {})
    print(fh.get("utilization", 1), fh.get("resetsAt", 0), sd.get("utilization", 0), sd.get("resetsAt", 0), d.get("status"))
except Exception:
    print(1, 0, 0, 0, "unknown")
' <<< "$info")"
        now=$(date +%s)
        if [ "$st" = allowed ] && python3 -c "import sys; sys.exit(0 if $f < $fmax and $w < $wmax else 1)"; then
            echo "$(date -Is) quota ok: five_hour $f, seven_day $w"
            return 0
        fi
        local until_ts=$fr
        if python3 -c "import sys; sys.exit(0 if $w >= $wmax else 1)"; then
            until_ts=$wr
            [ "$cap_until" -gt 0 ] && [ "$cap_until" -lt "${wr:-0}" ] && until_ts=$cap_until
        fi
        [ "${until_ts:-0}" -gt "$now" ] 2>/dev/null || until_ts=$((now + 600))
        echo "$(date -Is) quota gate: five_hour $f, seven_day $w, status $st; sleeping until $(date -u -d @$((until_ts + 120)) -Is)"
        sleep $((until_ts + 120 - now))
    done
}
