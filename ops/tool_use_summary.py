# Per-drive tool use from driver transcripts, per arm (images read, probe calls, scripts, perception runs, cost).
# Usage: python3 ops/tool_use_summary.py OUT.json   (paths are the hornet layout in ~/workspace)
import json, re, glob, os, sys, collections, statistics as st
from datetime import datetime
H = os.path.expanduser("~/workspace")
ARMS = [
    ("A0", f"{H}/ccd_base/sessions/priv_opus/a1/route_*_privileged"),
    ("A0+", f"{H}/ccd_priv2/sessions/a0plus/a1/route_*_privileged"),
    ("A1", f"{H}/ccd_priv2/sessions/priv_v2/a[123]/route_*_privileged"),
    ("B0+", f"{H}/ccd_priv2/sessions/ladder_plus/B0_sensors/a1/route_*"),
    ("B0g+", f"{H}/ccd_priv2/sessions/ladder_plus/B0_sensors_guidefix/a1/route_*"),
    ("B0i+", f"{H}/ccd_priv2/sessions/ladder_plus/B0_sensors_info/a1/route_*"),
    ("B2+", f"{H}/ccd_priv2/sessions/ladder_plus/B2_dense_code/a1/route_*"),
    ("B2i+", f"{H}/ccd_priv2/sessions/ladder_plus/B2_dense_code_info/a1/route_*"),
    ("B3s+", f"{H}/ccd_priv2/sessions/ladder_plus/B3s_strong_perc/a1/route_*"),
    ("B3s+L", f"{H}/ccd_priv2/sessions/ladder_plus/B3s_guide_lib/a1/route_*"),
    ("B5", f"{H}/ccd_priv2/sessions/ladder_plus/B5_tfv6/a1/route_*"),
]
CAM = re.compile(r"(CAM_[A-Z_]+?)(?:_full)?(?:_depth|_inst)?\.(?:jpg|png|npy)")
CLIENT = re.compile(r"carla_probe\.client\s+(\w+)")
PROMPTS = re.compile(r"prompts\s*=\s*\[([^\]]*)\]")


def ds_of(d):
    try:
        r = json.load(open(f"{d}/result.json"))["_checkpoint"]["records"]
        return r[0]["scores"]["score_composed"] if r else None
    except Exception:
        return None


def analyse(d):
    s = {"route": re.search(r"route_(\d+)", d).group(1), "attempt": re.search(r"/(a\d)/", d).group(1), "ds": ds_of(d)}
    tools = collections.Counter(); client = collections.Counter(); cams = collections.Counter()
    img_reads = 0; code_runs = 0; perc_runs = 0; scripts = {}; texts = 0; thinking_chars = 0
    results = {}; plan_checks = []; cost = None; out_tok = None; think_tok = None
    perc_names = set()
    code_dir = f"{d}/driver_code"
    if os.path.isdir(code_dir):
        for f in glob.glob(f"{code_dir}/**/*.py", recursive=True):
            src = open(f, errors="ignore").read()
            if "perception_client" in src or "PerceptionClient" in src:
                perc_names.add(os.path.basename(f)[:-3])
    prompts = collections.Counter()
    for f in glob.glob(f"{code_dir}/**/*.py", recursive=True) if os.path.isdir(code_dir) else []:
        for m in PROMPTS.finditer(open(f, errors="ignore").read()):
            for p in re.findall(r"[\"']([^\"']+)[\"']", m.group(1)):
                prompts[p] += 1
    tp = f"{d}/driver_transcript.jsonl"
    if not os.path.exists(tp):
        return None
    pending = {}
    for line in open(tp):
        try:
            e = json.loads(line)
        except Exception:
            continue
        if e.get("type") == "result":
            cost = e.get("total_cost_usd"); u = e.get("usage") or {}
            out_tok = u.get("output_tokens"); think_tok = (u.get("output_tokens_details") or {}).get("thinking_tokens")
        m = e.get("message")
        if not isinstance(m, dict) or not isinstance(m.get("content"), list):
            continue
        for x in m["content"]:
            if not isinstance(x, dict):
                continue
            if x.get("type") == "thinking":
                thinking_chars += len(x.get("thinking", ""))
            if x.get("type") == "text" and e.get("type") == "assistant":
                texts += len(x.get("text", ""))
            if x.get("type") == "tool_use":
                n = x["name"]; inp = x.get("input") or {}
                tools[n] += 1
                if n == "Read":
                    fp = inp.get("file_path", "")
                    if fp.endswith((".jpg", ".png")):
                        img_reads += 1
                        mm = CAM.search(os.path.basename(fp))
                        cams[mm.group(1) if mm else ("bev" if "bev" in fp else os.path.basename(fp)[:20])] += 1
                if n == "Write" and "/code/" in inp.get("file_path", ""):
                    scripts[inp["file_path"]] = inp.get("content", "").count("\n") + 1
                if n == "Bash":
                    c = inp.get("command", "")
                    for k in CLIENT.findall(c):
                        client[k] += 1
                    if "code_runner" in c:
                        code_runs += 1
                        if perc_names and re.search(r"(?:%s)\.py" % "|".join(map(re.escape, perc_names)), c):
                            perc_runs += 1
                    if "plan_check" in c:
                        pending[x["id"]] = "plan_check"
            if x.get("type") == "tool_result" and pending.get(x.get("tool_use_id")) == "plan_check":
                c = x["content"]; c = c if isinstance(c, str) else json.dumps(c)
                plan_checks.append(c)
    # plan_check outcomes: collisions/clearance problems predicted
    pc_bad = sum(1 for c in plan_checks if re.search(r'"(collides|collision|conflict|binds|pinched)"\s*:\s*(true|\[[^\]])', c) or "pinched" in c or '"min_clearance_m": -' in c)
    dec = [json.loads(l) for l in open(f"{d}/commands.jsonl") if l.strip()] if os.path.exists(f"{d}/commands.jsonl") else []
    plans = [x for x in dec if x.get("cmd") == "apply_plan"]
    try:
        dj = json.load(open(f"{d}/driver.json"))
        wall = (datetime.fromisoformat(dj["ended"]) - datetime.fromisoformat(dj["started"])).total_seconds() / 60
    except Exception:
        wall = None
    traj = f"{d}/trajectory.jsonl"
    steps = None
    if os.path.exists(traj):
        with open(traj) as f:
            last = None
            for last in f:
                pass
        try:
            steps = json.loads(last)["step"]
        except Exception:
            pass
    s.update(dict(tools=dict(tools), client=dict(client), cams=dict(cams), img_reads=img_reads, code_runs=code_runs,
                  perc_runs=perc_runs, n_scripts=len(scripts), script_lines=sum(scripts.values()), prompts=dict(prompts),
                  plan_checks=len(plan_checks), plan_checks_flagged=pc_bad, n_plans=len(plans),
                  ticks_per_plan=st.mean([p.get("ticks", 0) for p in plans]) if plans else None,
                  speeds=[p.get("target_speed") for p in plans if p.get("target_speed") is not None],
                  sim_s=steps * 0.05 if steps else None, wall_min=wall, cost=cost, out_tok=out_tok, think_tok=think_tok,
                  text_chars=texts))
    return s


out = {}
for arm, pat in ARMS:
    rows = [r for r in (analyse(d) for d in sorted(glob.glob(pat)) if os.path.isdir(d)) if r and r["ds"] is not None]
    out[arm] = rows
json.dump(out, open(sys.argv[1], "w"))
for arm, rows in out.items():
    n = len(rows)
    def m(k):
        v = [r[k] for r in rows if r.get(k) is not None]; return round(st.mean(v), 1) if v else None
    cl = collections.Counter(); tl = collections.Counter(); cm = collections.Counter(); pr = collections.Counter()
    for r in rows:
        cl.update(r["client"]); tl.update(r["tools"]); cm.update(r["cams"]); pr.update(r["prompts"])
    print(f"== {arm} n={n} DS {m('ds')} cost ${m('cost')} wall {m('wall_min')}min sim {m('sim_s')}s plans {m('n_plans')} ticks/plan {m('ticks_per_plan')} imgs {m('img_reads')} code_runs {m('code_runs')} perc_runs {m('perc_runs')} scripts {m('n_scripts')} lines {m('script_lines')} plan_checks {m('plan_checks')} flagged {m('plan_checks_flagged')} think_tok {m('think_tok')}")
    print("   client/drive", {k: round(v / n, 1) for k, v in cl.most_common(14)})
    print("   tools/drive", {k: round(v / n, 1) for k, v in tl.most_common()})
    print("   cams/drive", {k: round(v / n, 1) for k, v in cm.most_common(8)})
    if pr: print("   prompts", pr.most_common(25))
