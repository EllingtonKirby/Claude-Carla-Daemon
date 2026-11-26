# Steering wobble per arm: reversals per 10 s of straight cruising, steer / yaw-rate spread, sway.
# Usage: ccd_priv2/.venv/bin/python ops/steering_wobble.py   (writes wobble.json in the cwd)
import json, glob, os, math, numpy as np, collections, sys
H = os.path.expanduser("~/workspace")
ARMS = [("A0", f"{H}/ccd_base/sessions/priv_opus/a1/route_*_privileged"), ("A0+", f"{H}/ccd_priv2/sessions/a0plus/a1/route_*_privileged"),
        ("A1", f"{H}/ccd_priv2/sessions/priv_v2/a[123]/route_*_privileged"), ("B0+", f"{H}/ccd_priv2/sessions/ladder_plus/B0_sensors/a1/route_*"),
        ("B0g+", f"{H}/ccd_priv2/sessions/ladder_plus/B0_sensors_guidefix/a1/route_*"), ("B2+", f"{H}/ccd_priv2/sessions/ladder_plus/B2_dense_code/a1/route_*"),
        ("B3s+", f"{H}/ccd_priv2/sessions/ladder_plus/B3s_strong_perc/a1/route_*")]
def analyse(d):
    T = [json.loads(l) for l in open(f"{d}/trajectory.jsonl")]
    if len(T) < 200: return None
    st = np.array([t["step"] for t in T]); x = np.array([t["x"] for t in T]); y = np.array([t["y"] for t in T])
    yaw = np.unwrap(np.radians([t["yaw"] for t in T])); v = np.array([t["speed"] for t in T]); s = np.array([t.get("steer", 0) for t in T])
    dt = 0.05
    yr = np.gradient(yaw) / dt                    # yaw rate rad/s
    # straight-ish cruising: speed > 4 m/s and |yaw rate| smoothed over 3 s small (no real turn)
    k = 60
    yr_s = np.convolve(yr, np.ones(k) / k, mode="same")
    cruise = (v > 4) & (np.abs(yr_s) < 0.03)
    # lateral sway: deviation of the path from its own 4 s moving average, measured across heading
    w = 80
    xs = np.convolve(x, np.ones(w) / w, mode="same"); ys = np.convolve(y, np.ones(w) / w, mode="same")
    lat = -(x - xs) * np.sin(yaw) + (y - ys) * np.cos(yaw)
    valid = cruise.copy(); valid[:w] = False; valid[-w:] = False
    # steering reversals: sign changes of steer (deadband 0.01) during cruise
    sg = np.sign(np.where(np.abs(s) > 0.01, s, 0)); rev = 0; last = 0; revsteps = []
    for i in np.nonzero(cruise)[0]:
        if sg[i] == 0: continue
        if last and sg[i] != last: rev += 1; revsteps.append(st[i])
        last = sg[i]
    secs = cruise.sum() * dt
    # plan starts
    starts = []
    if os.path.exists(f"{d}/commands.jsonl"):
        for l in open(f"{d}/commands.jsonl"):
            c = json.loads(l)
            if c.get("cmd") == "apply_plan": starts.append(c["step"])
    starts = np.array(starts)
    near = sum(1 for r in revsteps if starts.size and np.min(np.abs(starts - r)) <= 10)  # within 0.5 s of a new plan
    return dict(cruise_s=secs, rev_per_10s=10 * rev / secs if secs > 5 else None, sway_cm=100 * float(np.sqrt(np.mean(lat[valid] ** 2))) if valid.sum() > 40 else None,
                steer_std=float(np.std(s[cruise])) if cruise.sum() > 40 else None, yr_std=float(np.std(yr[cruise])) if cruise.sum() > 40 else None,
                rev=rev, rev_near_plan=near, n_plans=len(starts), plan_ticks=float(np.mean(np.diff(starts))) if len(starts) > 2 else None)
res = {}
for arm, pat in ARMS:
    rows = [r for r in (analyse(d) for d in sorted(glob.glob(pat)) if os.path.isdir(d) and not d.endswith((".claim", ".out"))) if r and r["rev_per_10s"] is not None]
    res[arm] = rows
    f = lambda k: round(float(np.median([r[k] for r in rows if r[k] is not None])), 2)
    rv = sum(r["rev"] for r in rows); nr = sum(r["rev_near_plan"] for r in rows)
    print(f"{arm:5} n={len(rows):2} cruise {f('cruise_s'):6}s  steer reversals/10s {f('rev_per_10s'):5}  sway {f('sway_cm'):5} cm  steer std {f('steer_std'):.3f}  yaw-rate std {f('yr_std'):.3f} rad/s  reversals within 0.5 s of a new plan {nr}/{rv} ({100*nr/max(1,rv):.0f}%)  ticks/plan {f('plan_ticks')}")
json.dump(res, open("wobble.json", "w"))
