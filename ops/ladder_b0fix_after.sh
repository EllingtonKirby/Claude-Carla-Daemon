#!/usr/bin/env bash
# After the ladder_plus B0+/B2+ lanes exit: fix the guide's camera mount, then run B0 with the fixed guide.
while pgrep -f "[l]anes3.sh .*ladder_plus/jobs_a1.txt" > /dev/null; do sleep 60; done
echo "$(date -Is) B0+/B2+ lanes done; applying guide fix"
python3 "$HOME/workspace/apply_guide_camera_fix.py" || { echo "guide fix failed; not starting B0 rerun"; exit 1; }
exec bash "$HOME/workspace/lanes3.sh" "$HOME/workspace/ccd_priv2" "$HOME/workspace/ccd_priv2/sessions/ladder_plus/jobs_b0fix.txt" 3 6
