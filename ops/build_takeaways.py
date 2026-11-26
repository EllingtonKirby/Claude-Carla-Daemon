#!/usr/bin/env python3
"""Build takeaways.csv for the dashboard from every A1 / ladder_plus session's debrief.md or closing account.
    python3 build_takeaways.py OUT_DIR"""
import csv, json, re, subprocess, sys
from pathlib import Path
sys.path.insert(0, str(Path.home() / "workspace/ccd_priv2/tools"))
import extract_debriefs as e
out_dir = Path(sys.argv[1]); out_dir.mkdir(parents=True, exist_ok=True)
cls = json.loads(subprocess.run(["python3", str(Path.home() / "workspace/classify_sessions.py"), "--json"],
                                capture_output=True, text=True).stdout)
rows = []
for c in cls:
    if c["kind"] != "valid" or not ("priv_v2" in c["session"] or "ladder_plus" in c["session"]):
        continue
    s = Path(c["path"])
    arm = "A1" if "priv_v2" in c["session"] else {"B0_sensors": "B0+", "B2_dense_code": "B2+", "B0_sensors_guidefix": "B0g+", "B0_sensors_info": "B0i+", "B2_dense_code_info": "B2i+"}.get(s.parent.parent.name, s.parent.parent.name)
    try:
        ds = json.loads((s / "result.json").read_text())["_checkpoint"]["records"][0]["scores"]["score_composed"]
    except Exception:
        continue
    deb = (s / "debrief.md").read_text() if (s / "debrief.md").exists() else ""
    route = re.search(r"route_(\d+)", s.name).group(1)
    rows.append(dict(key=f"{arm}-{s.parent.name}-{route}", arm=arm, route=route, attempt=s.parent.name[1:], ds=f"{float(ds):.1f}",
                     source="Debrief" if deb else "Closing account", lesson=e.sections(deb)["One-line lesson"] if deb else "",
                     text=deb or e.account(s)))
rows.sort(key=lambda r: (r["source"] != "Debrief", r["arm"], r["route"], r["attempt"]))
with (out_dir / "takeaways.csv").open("w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print(len(rows), "rows,", sum(r["source"] == "Debrief" for r in rows), "debriefs")
