#!/usr/bin/env bash
# Start the ladder_plus batch (B0+/B2+, attempt 1) on slots 6-8 once the priv_v2 A1 lanes have exited.
while pgrep -f "[l]anes3.sh .*priv_v2/jobs_full.txt" > /dev/null; do sleep 60; done
echo "$(date -Is) A1 lanes done; starting ladder_plus"
exec bash "$HOME/workspace/lanes3.sh" "$HOME/workspace/ccd_priv2" "$HOME/workspace/ccd_priv2/sessions/ladder_plus/jobs_a1.txt" 3 6
