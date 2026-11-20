#!/usr/bin/env bash
# Ask a finished driver what it learned: resume its own `claude -p` conversation once the route is
# over, with a fixed set of questions, and save the answer as SESSION_DIR/debrief.md.
#
#   bash scripts/debrief.sh SESSION_DIR
#
# Run by run_eval.sh after the route ends (DEBRIEF=1, the default), or later to backfill a session
# whose driver workspace (driver.json:driver_ws) and saved conversation still exist. A follow-up turn,
# never part of the drive: the driving prompt and everything the driver saw are unchanged, so results
# stay comparable. No tools are allowed in the debrief, so it cannot read anything new.
set -uo pipefail
S="$(realpath "${1:?usage: debrief.sh SESSION_DIR}")"
[ -f "$S/driver.json" ] && [ -f "$S/driver_transcript.jsonl" ] || { echo "no driver in $S" >&2; exit 1; }
[ -s "$S/debrief.md" ] && { echo "debrief exists: $S/debrief.md"; exit 0; }
read -r WS MODEL <<< "$(python3 -c "import json; d=json.load(open('$S/driver.json')); print(d['driver_ws'], d['model'])")"
SID="$(grep -m1 -o '"session_id":"[^"]*"' "$S/driver_transcript.jsonl" | cut -d'"' -f4)"
[ -d "$WS" ] && [ -n "$SID" ] || { echo "driver workspace or session id missing for $S" >&2; exit 1; }
PROMPT='The route is over. Before you finish, write a debrief of this drive for the people improving the
benchmark harness and for future drivers. Use exactly these headings, be concrete (numbers, commands,
moments), and keep it under 450 words:

## Outcome
What you believe the result was, and whether anything was scored against you.
## What worked
The decisions, commands or pieces of information that most helped.
## What did not work
Mistakes, near misses, wasted time, and anything you would do differently.
## Surprises
Anything about the car, the controller, the daemon, the tools or the scenario that behaved
differently from what the guide or the replies led you to expect.
## What would have helped
Information, tools or guide changes that would have made this drive easier or safer. General
lessons only, not route-specific tactics.
## One-line lesson
The single most useful sentence for the next driver.'
cd "$WS" || exit 1
timeout 900 claude -p "$PROMPT" --resume "$SID" --model "$MODEL" --output-format json \
    --permission-mode dontAsk --tools "" --setting-sources "" > "$S/debrief.json" 2> "$S/debrief_stderr.log" < /dev/null
python3 - "$S" <<'PY'
import json, sys
s = sys.argv[1]
try:
    d = json.load(open(f"{s}/debrief.json"))
except Exception as e:
    print(f"debrief failed: {e!r}", file=sys.stderr); sys.exit(1)
if d.get("is_error") or not d.get("result"):
    print(f"debrief failed: {str(d)[:300]}", file=sys.stderr); sys.exit(1)
open(f"{s}/debrief.md", "w").write(d["result"].strip() + "\n")
print(f"wrote {s}/debrief.md (${d.get('total_cost_usd', 0):.2f})")
PY
