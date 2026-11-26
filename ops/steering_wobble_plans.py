import json, glob, os, numpy as np
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "steering_wobble.py")).read().split("def analyse")[0])
def to_world(pose, wps):
    x, y, yaw = pose; c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    return np.array([[x + f * c + l * s, y + f * s - l * c] for f, l in wps])
def lat_to_poly(p, poly):
    best = None
    for a, b in zip(poly[:-1], poly[1:]):
        d = b - a; L = np.hypot(*d)
        if L < 1e-6: continue
        t = np.clip(np.dot(p - a, d) / L**2, 0, 1); q = a + t * d
        dist = np.hypot(*(p - q))
        if best is None or dist < best: best = dist
    return best
def interp(wps, f0):
    w = np.array(wps, float)
    if w[-1, 0] < f0 or w[0, 0] > f0: return None
    return [f0, float(np.interp(f0, w[:, 0], w[:, 1]))]
def analyse(d):
    T = {t["step"]: t for t in map(json.loads, open(f"{d}/trajectory.jsonl"))}
    P = [c for c in map(json.loads, open(f"{d}/commands.jsonl")) if c.get("cmd") == "apply_plan" and c.get("waypoints")]
    shifts = []; phase = []
    for a, b in zip(P[:-1], P[1:]):
        ta, tb = T.get(a["step"]), T.get(b["step"])
        if not ta or not tb or tb["speed"] < 4 or b["step"] - a["step"] > 60: continue
        pb = interp(b["waypoints"], 8.0)
        if pb is None: continue
        pa = to_world((ta["x"], ta["y"], ta["yaw"]), a["waypoints"])
        q = to_world((tb["x"], tb["y"], tb["yaw"]), [pb])[0]
        if len(pa) >= 2: shifts.append(lat_to_poly(q, pa))
    # reversal phase within plans
    steps = sorted(T); starts = np.array(sorted(c["step"] for c in P))
    last = 0
    for i in steps:
        t = T[i]
        if t["speed"] < 4: continue
        s = t.get("steer", 0); sg = 0 if abs(s) <= 0.01 else (1 if s > 0 else -1)
        if not sg: continue
        if last and sg != last:
            k = starts[starts <= i]
            if k.size: phase.append(i - k[-1])
        last = sg
    return shifts, phase
for arm, pat in ARMS:
    S = []; Ph = []
    for d in sorted(glob.glob(pat)):
        if not os.path.isdir(d) or d.endswith((".claim", ".out")): continue
        try: s, p = analyse(d)
        except Exception as e: continue
        S += [x for x in s if x is not None]; Ph += p
    S = np.array(S); Ph = np.array(Ph)
    print(f"{arm:5} plan-to-plan path shift at 8 m: median {100*np.median(S):5.1f} cm, p90 {100*np.percentile(S,90):5.1f} cm (n={len(S)})   reversals by ticks since plan start: 0-4 {np.mean(Ph<5)*100:4.0f}%  5-9 {np.mean((Ph>=5)&(Ph<10))*100:4.0f}%  10+ {np.mean(Ph>=10)*100:4.0f}%  (n={len(Ph)})")
