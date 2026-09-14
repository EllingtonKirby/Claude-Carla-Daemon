#!/usr/bin/env python3
"""What a run cost, and how that cost scales with turns.

    python tools/cost_report.py SESSION_DIR...            # markdown table
    python tools/cost_report.py SESSION_DIR... --json

Reads `driver_transcript.jsonl`: the stream's final `result` row, which carries Claude Code's own
totals. (Assistant messages repeat their usage as a turn streams, so summing them over-counts by
3-5x -- use the result row, not the messages.)

The dollar figures are API-equivalent: a run billed to a Claude subscription is metered against a
usage allowance, not invoiced, so compare token counts and read the money column as a proxy.

The last column is the plan's §0.3 cost model. Cost goes roughly as k * turns**2 when nothing is
ever evicted from context: context grows linearly with turns and every turn re-reads all of it.
`k` is the constant to watch -- a loop change should move `k` or the turn count, and reporting it
per configuration is what makes the next change's effect predictable before spending on a suite.
A loop with working context control should break the quadratic outright, in which case `k` falls
as routes get longer instead of holding steady.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

# USD per million tokens. Verified against a recorded run: its result row's token breakdown priced
# with this table reproduces Claude Code's own total_cost_usd to the cent. cache_write is
# TTL-dependent (5 min ~1.25x input, 1 h 2x); `claude -p` writes 1 h.
PRICES: Dict[str, Dict[str, float]] = {
    "claude-opus-5":  {"input": 5.0, "output": 25.0, "cache_read": 0.50, "cache_write_1h": 10.0, "cache_write_5m": 6.25},
    "claude-sonnet-5": {"input": 2.0, "output": 10.0, "cache_read": 0.20, "cache_write_1h": 4.0, "cache_write_5m": 2.50},
    "claude-haiku-4-5": {"input": 1.0, "output": 5.0, "cache_read": 0.10, "cache_write_1h": 2.0, "cache_write_5m": 1.25},
}


def _price(model: str, usage: Dict[str, int], ttl: str) -> Optional[float]:
    key = next((m for m in PRICES if model.startswith(m)), None)
    if key is None:
        return None
    p = PRICES[key]
    return (usage["input"] * p["input"] + usage["output"] * p["output"]
            + usage["cache_read"] * p["cache_read"]
            + usage["cache_write"] * p[f"cache_write_{ttl}"]) / 1e6


def _empty() -> Dict[str, int]:
    return {"input": 0, "output": 0, "cache_read": 0, "cache_write": 0}


def _read_run(session: Path) -> Optional[Dict[str, Any]]:
    log = session / "driver_transcript.jsonl"
    if not log.is_file():
        return None
    result, models, per_turn_input = None, set(), []
    seen = set()
    for line in log.read_text().splitlines():
        if not line.strip():
            continue
        d = json.loads(line)
        if d.get("type") == "result":
            result = d
        m = d.get("message")
        if isinstance(m, dict) and d.get("type") == "assistant" and m.get("usage"):
            models.add(m.get("model"))
            if m.get("id") not in seen:  # usage repeats while a turn streams
                seen.add(m.get("id"))
                u = m["usage"]
                per_turn_input.append(sum(u.get(k) or 0 for k in
                                          ("input_tokens", "cache_read_input_tokens", "cache_creation_input_tokens")))
    if result is None:
        return None
    u = result.get("usage") or {}
    usage = {"input": u.get("input_tokens", 0), "output": u.get("output_tokens", 0),
             "cache_read": u.get("cache_read_input_tokens", 0),
             "cache_write": u.get("cache_creation_input_tokens", 0)}
    ttl = "1h" if (u.get("cache_creation") or {}).get("ephemeral_1h_input_tokens") else "5m"
    return {"model": next(iter(models), None), "turns": result.get("num_turns"),
            "decisions": None, "usage": usage, "ttl": ttl, "avg_context": _mean(per_turn_input),
            "reported_cost": result.get("total_cost_usd")}


def _mean(xs: List[int]) -> Optional[float]:
    return sum(xs) / len(xs) if xs else None


def _decisions(session: Path) -> Optional[int]:
    """Driving decisions actually accepted by the daemon -- the denominator that matters."""
    log = session / "decisions.jsonl"
    if not log.is_file():
        return None
    return len([1 for line in log.read_text().splitlines() if line.strip()
                and json.loads(line).get("why")])


def collect(session: Path) -> Dict[str, Any]:
    row = _read_run(session) or {}
    row["session"] = session.name
    row["decisions"] = row.get("decisions") or _decisions(session)
    if row.get("model"):
        row["cost_usd"] = _price(row["model"] or "", row["usage"], row["ttl"])
        # Claude Code reports its own total; if ours disagrees the price table is stale.
        if row.get("reported_cost") and row.get("cost_usd"):
            row["price_table_error_pct"] = round(
                100.0 * (row["cost_usd"] - row["reported_cost"]) / row["reported_cost"], 2)
        turns = row.get("turns") or 0
        row["k_cost_per_turn2"] = row["cost_usd"] / turns ** 2 if turns and row.get("cost_usd") else None
        row["turns_per_decision"] = (turns / row["decisions"]) if row.get("decisions") else None
    result = session / "result.json"
    if result.is_file():
        try:
            rec = json.loads(result.read_text())["_checkpoint"]["records"][0]
            row["DS"] = rec["scores"]["score_composed"]
            row["RC"] = rec["scores"]["score_route"]
            row["status"] = rec["status"]
        except (KeyError, IndexError, json.JSONDecodeError):
            pass
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    rows = [collect(s) for s in args.sessions]
    if args.json:
        print(json.dumps(rows, indent=2))
        return 0

    head = ["session", "model", "turns", "decisions", "turns/dec", "avg ctx",
            "cache read", "output", "cost $", "k x1e-4", "DS"]
    print("| " + " | ".join(head) + " |")
    print("|" + "|".join("---" for _ in head) + "|")
    for r in rows:
        u = r.get("usage") or {}
        print("| " + " | ".join(str(v) for v in [
            r["session"], (r.get("model") or "?").replace("claude-", ""),
            r.get("turns") or "?", r.get("decisions") or "?",
            f"{r['turns_per_decision']:.1f}" if r.get("turns_per_decision") else "?",
            f"{r['avg_context']/1000:.0f}k" if r.get("avg_context") else "?",
            f"{(u.get('cache_read') or 0)/1e6:.1f}M", f"{(u.get('output') or 0)/1000:.0f}k",
            f"{r['cost_usd']:.2f}" if r.get("cost_usd") else "?",
            f"{r['k_cost_per_turn2']*1e4:.2f}" if r.get("k_cost_per_turn2") else "?",
            r.get("DS", "?"),
        ]) + " |")
    checked = [r for r in rows if r.get("price_table_error_pct") is not None]
    for r in checked:
        if abs(r["price_table_error_pct"]) > 1.0:
            print(f"\nWARNING: {r['session']}: priced {r['cost_usd']:.2f} but the run reported "
                  f"{r['reported_cost']:.2f} ({r['price_table_error_pct']:+.1f}%) -- PRICES is stale")
    if len(rows) == 2 and all(r.get("cost_usd") for r in rows):
        a, b = rows
        print(f"\n{a['session']} -> {b['session']}: cost x{a['cost_usd']/b['cost_usd']:.2f}, "
              f"turns x{(a.get('turns') or 0)/max(b.get('turns') or 1, 1):.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
