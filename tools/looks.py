#!/usr/bin/env python3
"""Count how often the driver actually looked, and how many camera frames it opened.

    python tools/looks.py 'sessions/dev10_r*_sensors'

Offline: reads ``commands.jsonl`` (what the driver sent) and ``driver_transcript.jsonl`` (what the
driver's own tool calls were). No simulator.

Why this exists, and why it is not a one-liner: the obvious way to count looks is to count
``cmd == "percept"``, and that undercounts by a factor of thirty. The driver mostly looks by
riding ``percept=true`` on the driving command, which returns the next look inside the reply that
reports the outcome (`.claude/skills/drive-carla/SKILL.md` §2.3). A corpus analysis that counted
only the standalone command concluded "the driver looks once per route" and drove a guidance
priority that was addressing a behaviour which was already happening; the conclusion was withdrawn
(docs/JOURNAL.md §1.3). So:

    looks = (cmd == "percept") + (driving commands carrying percept=true)

``imgs`` is the independent cross-check, and the one that cannot be gamed by accounting: how many
camera frames the driver *opened*, counted as ``Read`` tool calls on a frame path in the driver's
own transcript. A look that returns four JPG paths and is followed by zero reads is not a look.
Where ``imgs`` and ``looks`` disagree sharply, trust ``imgs``.
"""

from __future__ import annotations

import argparse
import collections
import glob
import json
import os
from typing import Counter as CounterT, Tuple

_IMG_SUFFIXES = (".jpg", ".jpeg", ".png")


def count(session_dir: str) -> Tuple[CounterT[str], int, int]:
    """-> (commands by name, driving commands with percept=true, camera frames opened)."""
    cmds: CounterT[str] = collections.Counter()
    percept_true = 0
    with open(os.path.join(session_dir, "commands.jsonl")) as fh:
        for line in fh:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if row.get("cmd"):
                cmds[row["cmd"]] += 1
            if row.get("percept") is True:
                percept_true += 1
    return cmds, percept_true, _frames_opened(session_dir)


def _frames_opened(session_dir: str) -> int:
    """``Read`` tool calls the driver made on a camera frame, from its own transcript."""
    path = os.path.join(session_dir, "driver_transcript.jsonl")
    if not os.path.exists(path):
        return 0
    n = 0
    with open(path) as fh:
        for line in fh:
            try:
                message = json.loads(line).get("message")
            except json.JSONDecodeError:
                continue
            if not isinstance(message, dict):
                continue
            content = message.get("content")
            if not isinstance(content, list):
                continue
            for block in content:
                if not isinstance(block, dict):
                    continue
                if block.get("type") != "tool_use" or block.get("name") != "Read":
                    continue
                fp = str(block.get("input", {}).get("file_path", ""))
                if "/frames/" in fp or fp.endswith(_IMG_SUFFIXES):
                    n += 1
    return n


def totals(session_dirs) -> CounterT[str]:
    tot: CounterT[str] = collections.Counter()
    for d in session_dirs:
        cmds, percept_true, imgs = count(d)
        tot["plans"] += cmds["apply_plan"]
        tot["percept_cmd"] += cmds["percept"]
        tot["percept_true"] += percept_true
        tot["looks"] += cmds["percept"] + percept_true
        tot["imgs"] += imgs
        tot["bev"] += cmds["render_bev"]
    return tot


def _report(pattern: str) -> None:
    head = "%7s %6s %8s %9s %6s %6s %5s %8s" % (
        "route", "plans", "perc_cmd", "perc=true", "looks", "imgs", "bev", "img/plan")
    print(head)
    print("-" * len(head))
    dirs = sorted(glob.glob(pattern))
    for d in dirs:
        cmds, percept_true, imgs = count(d)
        plans = cmds["apply_plan"]
        route = d.split("_r")[1].split("_")[0] if "_r" in d else os.path.basename(d)
        print("%7s %6d %8d %9d %6d %6d %5d %8.1f"
              % (route, plans, cmds["percept"], percept_true, cmds["percept"] + percept_true,
                 imgs, cmds["render_bev"], imgs / plans if plans else 0.0))
    tot = totals(dirs)
    print("-" * len(head))
    print("TOTAL  plans=%d looks=%d (percept cmd %d + percept=true %d) imgs=%d  imgs/plan=%.1f"
          % (tot["plans"], tot["looks"], tot["percept_cmd"], tot["percept_true"], tot["imgs"],
             tot["imgs"] / max(tot["plans"], 1)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pattern", help="glob of session dirs, quoted so the shell does not expand it")
    _report(ap.parse_args().pattern)


if __name__ == "__main__":
    main()
