#!/usr/bin/env bash
# When the B3s+ chain exits: block every lane (cap 0.0) until 2026-11-26 08:00 UTC, i.e. no runs on
# Wednesday night (the user's call, 2026-11-24).
while pgrep -f "[b]3s_after_a0plus.sh" > /dev/null; do sleep 60; done
echo "0.0 $(date -u -d '2026-11-26 08:00' +%s)" > "$HOME/workspace/quota_cap"
echo "$(date -Is) B3s+ chain done; quota_cap set to 0.0 until 2026-11-26 08:00 UTC"
