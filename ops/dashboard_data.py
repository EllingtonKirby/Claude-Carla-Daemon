#!/usr/bin/env python3
"""Build the CSVs behind the Privileged Ladder Results dashboard.

    python3 dashboard_data.py OUT_DIR

runs.csv       every VALID session of the ladder (A0 privileged, B0-B4c) and the Sep 11 reference corpus
infra.csv      sessions excluded as infrastructure failures, by cause (never driving results)
teach.csv      teacher/student rounds: per round and split, recall and error by range band
"""
import csv
import json
import re
import subprocess
import sys
from collections import Counter
from pathlib import Path

HOME = Path.home()
OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
ROUTES = HOME / "workspace/claude_carla_daemon/routes/bench2drive220"

ARMS = {
    "A1": ("A1 Privileged+", "privileged", "A0 plus actor behaviours, the scorer's rules, scenarios, predict/plan_check and a launch boost", "off", "no"),
    "A0f": ("Fable, privileged", "privileged", "as A0, driven by Claude Fable 5.2", "off", "no"),
    "A0": ("A0 Privileged", "privileged", "ground-truth object list (actors, lanes, lights, signs) + cameras", "off", "no"),
    "B0": ("B0 Sensors", "sensors", "cameras only", "off", "yes"),
    "B0+": ("B0+ Sensors, new controller", "sensors", "as B0, on the rebuilt (plus) controller", "off", "yes"),
    "B0g+": ("B0+ with the corrected camera mount", "sensors", "as B0+, after fixing the guide's CAM_FRONT position (it was 4.7 m off)", "off", "yes"),
    "B0i+": ("B0+ with info extras", "sensors", "as B0g+, plus the rules, the scenarios and predict/plan_check on its own obstacles", "off", "yes"),
    "B2i+": ("B2+ with info extras", "dense", "as B2+, plus the rules, the scenarios and predict/plan_check on its own obstacles", "on", "no"),
    "B2+": ("B2+ Perfect pixels + code, new controller", "dense", "as B2, on the rebuilt (plus) controller", "on", "no"),
    "B1": ("B1 Sensors + code", "sensors", "cameras only, plus a confined Python tool", "on", "yes"),
    "B2": ("B2 Perfect pixels + code", "dense", "cameras + ground-truth depth & instance masks, plus the Python tool", "on", "no"),
    "B3": ("B3 Sensors + code + models", "sensors", "cameras, Python tool, Depth Anything V2 + Mask2Former service", "on", "yes"),
    "B4": ("B4 + teacher's library", "sensors", "as B3, plus the perception library a GT-holding teacher coached", "on", "yes"),
    "B4c": ("B4c + self-taught library", "sensors", "as B3, plus the library the student wrote with no teacher", "on", "yes"),
}
LADDER_DIR = {"B0_sensors": "B0", "B0_sensors_guidefix": "B0g", "B0_sensors_info": "B0i", "B2_dense_code_info": "B2i", "B1_sensors_code": "B1", "B2_dense_code": "B2", "B3_sensors_code_perc": "B3",
              "B4_sensors_code_perc_teacherlib": "B4", "B4c_sensors_code_perc_selflib": "B4c"}


def town(route):
    m = re.search(r'town="([^"]+)"', (ROUTES / f"route_{route}.xml").read_text())
    return m.group(1) if m else ""


def record(d):
    try:
        recs = json.loads((d / "result.json").read_text())["_checkpoint"]["records"]
        return recs[0] if recs else None
    except Exception:
        return None


def transcript_stats(d):
    cost = turns = None
    code_runs = images = 0
    tr = d / "driver_transcript.jsonl"
    if not tr.exists():
        return cost, turns, code_runs, images
    for line in tr.open():
        if '"type":"result"' in line:
            try:
                j = json.loads(line)
                cost, turns = j.get("total_cost_usd"), j.get("num_turns")
            except Exception:
                pass
        if '"type":"tool_use"' in line:
            code_runs += line.count("carla_probe.code_runner")
            images += len(re.findall(r'"name":"Read","input":\{"file_path":"[^"]*\.(?:jpg|png)"', line))
    return cost, turns, code_runs, images


def row(arm, attempt, route, d, corpus="ladder", model="claude-opus-5", commit=None):
    r = record(d)
    if r is None:
        return None
    inf = r.get("infractions", {})
    n = {k: len(v) if isinstance(v, list) else 0 for k, v in inf.items()}
    scored = {k: v for k, v in n.items() if v and k not in ("min_speed_infractions",)}
    cost, turns, code_runs, images = transcript_stats(d)
    sess = json.loads((d / "session.json").read_text()) if (d / "session.json").exists() else {}
    dec = d / "decisions.jsonl"
    decisions = sum(1 for l in dec.open() if '"cmd"' in l) if dec.exists() else None
    video = d / "replay/commentary/replay.mp4"
    return {
        "corpus": corpus, "arm": arm, "arm_label": ARMS[arm][0] if arm in ARMS else arm, "model": model,
        "track": ARMS[arm][1] if arm in ARMS else "sensors", "attempt": attempt, "route": route, "town": town(route),
        "status": r["status"], "ds": round(r["scores"]["score_composed"], 2), "rc": round(r["scores"]["score_route"], 2),
        "is": round(r["scores"]["score_penalty"], 3),
        "infractions": "; ".join(f"{k.replace('_infractions', '').replace('_', ' ')} x{v}" for k, v in sorted(scored.items())) or "none",
        "vehicle_collisions": n.get("collisions_vehicle", 0), "layout_collisions": n.get("collisions_layout", 0),
        "pedestrian_collisions": n.get("collisions_pedestrian", 0),
        "red_lights": n.get("red_light", 0), "yield_emergency": n.get("yield_emergency_vehicle_infractions", 0),
        "decisions": decisions, "turns": turns, "cost_usd": round(cost, 2) if cost is not None else None,
        "code_runs": code_runs, "images_opened": images,
        "commit": commit or sess.get("repo_commit", ""), "video": str(video) if video.exists() else "",
        "session": str(d.relative_to(HOME)),
    }


rows = []
# valid ladder + privileged sessions, per the classifier
for c in json.loads(subprocess.run(["python3", str(HOME / "workspace/classify_sessions.py"), "--json"],
                                   capture_output=True, text=True).stdout):
    if c["kind"] != "valid":
        continue
    d = Path(c["path"])
    route = re.search(r"route_(\d+)", d.name).group(1)
    attempt = d.parent.name
    arm = ("A0" if "priv_opus" in c["session"] else "A0f" if "priv_fable" in c["session"]
           else "A1" if "priv_v2" in c["session"]
           else LADDER_DIR[d.parent.parent.name] + ("+" if "ladder_plus" in c["session"] else ""))
    model = "claude-fable-5-2" if arm == "A0f" else "claude-opus-5"
    rr = row(arm, attempt, route, d, model=model)
    if rr:
        rows.append(rr)
# Sep 11 reference corpus: three models, sensors track, one attempt, older commit
REF = HOME / "workspace/claude_carla_daemon/sessions"
for sub, model in (("20260911_164120_dev10", None), ("20260911_204737_dev10_opus", None)):
    for d in sorted((REF / sub).glob("route_*_sensors_*")):
        m = re.match(r"route_(\d+)_sensors_(\w+)", d.name)
        dj = json.loads((d / "driver.json").read_text()) if (d / "driver.json").exists() else {}
        rr = row(f"ref_{m.group(2)}", "a1", m.group(1), d, corpus="reference_sep11",
                 model=dj.get("model", m.group(2)))
        if rr:
            rr["arm_label"] = f"Sep 11 sensors, {m.group(2).capitalize()}"
            rows.append(rr)

with (OUT / "runs.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

# infrastructure exclusions
infra = Counter()
for root in (HOME / "workspace/ccd_base/sessions/priv_opus", HOME / "workspace/ccd_exp/sessions/ladder"):
    rel = (root / "RELAUNCHES.md").read_text() if (root / "RELAUNCHES.md").exists() else ""
    for line in rel.splitlines():
        m = re.match(r"\s+- (cutoff|pre_driver|incomplete): (\S+)", line)
        if m:
            infra[("usage limit: driver cut off mid-route" if m.group(1) == "cutoff" else
                   "usage limit / hang: no driver ever started" if m.group(1) == "pre_driver" else "crash after start",
                   "Fable" if "fable" in m.group(2) else "A0" if "priv" in str(root) else "ladder")] += 1
    for pat, cause in ((r"segfault", "CARLA crashed loading the map"), (r"isolation check failed closed", "safety check model declined a probe"),
                       (r"INVALID \(operator error\)", "operator error: two sessions on one simulator")):
        k = len(re.findall(pat, rel))
        if k:
            infra[(cause, "A0" if "priv" in str(root) else "ladder")] += k
with (OUT / "infra.csv").open("w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["cause", "where", "sessions"])
    for (cause, where), k in sorted(infra.items(), key=lambda x: -x[1]):
        w.writerow([cause, where, k])

# teaching rounds
trows = []
for t in ("on", "off"):
    base = HOME / f"workspace/ccd_exp/sessions/teach/teacher_{t}"
    for sc_path, split, rnd in [(p, "train", int(p.parent.name)) for p in sorted(base.glob("rounds/*/train_score.json"))] + \
                               ([(base / "test_score.json", "held-out test", 99)] if (base / "test_score.json").exists() else []):
        sc = json.loads(sc_path.read_text())
        for b in sc["bands"]:
            trows.append({"run": "with teacher" if t == "on" else "no teacher", "round": rnd if rnd != 99 else "final",
                          "split": split, "band": b["band"], "labels": b["n"],
                          "recall": round(b["recall"], 3) if b["recall"] is not None else None,
                          "mean_err_m": round(b["mean_err_m"], 2) if b["mean_err_m"] is not None else None,
                          "signed_fwd_m": round(b["signed_fwd_m"], 2) if b["signed_fwd_m"] is not None else None,
                          "signed_left_m": round(b["signed_left_m"], 2) if b["signed_left_m"] is not None else None,
                          "extras": sc["extras"], "errors": sc["errors"], "scenes": sc["scenes"],
                          "gap_abs_err_m": sc["in_path_gap"]["mean_abs_err_m"]})
with (OUT / "teach.csv").open("w", newline="") as f:
    cols = ["run", "round", "split", "band", "labels", "recall", "mean_err_m", "signed_fwd_m", "signed_left_m",
            "extras", "errors", "scenes", "gap_abs_err_m"]
    w = csv.DictWriter(f, fieldnames=cols)
    w.writeheader()
    w.writerows(trows)
print(f"runs {len(rows)}  infra {sum(infra.values())}  teach {len(trows)}")
