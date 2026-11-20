#!/usr/bin/env python3
"""Collect what the drivers wrote at the end of their routes: the closing account (the final message
of driver_transcript.jsonl, always present) and the structured debrief (debrief.md, scripts/debrief.sh)
when there is one.

    python tools/extract_debriefs.py SESSION_DIR [SESSION_DIR ...] [--csv OUT.csv]

Writes SESSION_DIR/drive_account.md for each session and, with --csv, one row per session: the
debrief's one-line lesson and each section's text, for the dashboard.
"""
import argparse
import csv
import json
import re
from pathlib import Path

SECTIONS = ("Outcome", "What worked", "What did not work", "Surprises", "What would have helped", "One-line lesson")


def account(s: Path) -> str:
    tr = s / "driver_transcript.jsonl"
    if not tr.exists():
        return ""
    out = ""
    for line in tr.open():
        if '"type":"result"' in line:
            try:
                out = json.loads(line).get("result") or out
            except Exception:
                pass
    return out.strip()


def sections(md: str):
    parts = {}
    for name in SECTIONS:
        m = re.search(rf"^##\s*{re.escape(name)}\s*$(.*?)(?=^##\s|\Z)", md, re.M | re.S | re.I)
        parts[name] = m.group(1).strip() if m else ""
    return parts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--csv", type=Path)
    a = ap.parse_args()
    rows = []
    for s in a.sessions:
        acc = account(s)
        if acc:
            (s / "drive_account.md").write_text(acc + "\n")
        deb = (s / "debrief.md").read_text() if (s / "debrief.md").exists() else ""
        sec = sections(deb) if deb else {k: "" for k in SECTIONS}
        try:
            rec = json.loads((s / "result.json").read_text())["_checkpoint"]["records"][0]
            ds = rec["scores"]["score_composed"]
        except Exception:
            ds = None
        rows.append({"session": str(s), "route": re.search(r"route_(\d+)", s.name).group(1), "attempt": s.parent.name,
                     "ds": ds, "has_debrief": bool(deb), "lesson": sec["One-line lesson"],
                     "worked": sec["What worked"], "did_not_work": sec["What did not work"],
                     "surprises": sec["Surprises"], "would_help": sec["What would have helped"],
                     "account": acc})
        print(f"{s.name} ({s.parent.name}): account {len(acc)} chars, debrief {'yes' if deb else 'no'}")
    if a.csv:
        with a.csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)


if __name__ == "__main__":
    main()
