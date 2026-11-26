#!/usr/bin/env bash
# Resume the Thursday-night B3s+L run after the quota-gate fix (allowed_warning). The strong
# perception service started at 17:55 UTC is still up on 8795. Runs the lanes, then stops the
# service and sets the cap back to 0.0 until the weekly reset.
R=$HOME/workspace/ccd_priv2
echo "$(date -Is) resuming B3s+L (gate fix); cap $(cat $HOME/workspace/quota_cap)"
pgrep -f "[c]arla_watchdog.sh" > /dev/null || (nohup setsid bash "$HOME/workspace/carla_watchdog.sh" > /dev/null 2>&1 < /dev/null &)
bash "$HOME/workspace/lanes3.sh" "$R" "$R/sessions/ladder_plus/jobs_b3sL.txt" 3 6
echo "$(date -Is) B3s+L lanes done; stopping perception service"
kill $(pgrep -f "perception_server.py --port 8795") 2>/dev/null
echo "0.0 $(date -u -d '2026-11-27 12:00' +%s)" > "$HOME/workspace/quota_cap"
echo "$(date -Is) cap 0.0 until the weekly reset"
