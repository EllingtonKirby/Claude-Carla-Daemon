#!/usr/bin/env python3
"""Teacher / student: a model with ground truth teaches a model without it to write perception code.

    python tools/teach_perception.py --train SCENES_DIR [SCENES_DIR ...] --test SCENES_DIR [...] \\
        --work DIR [--rounds 4] [--teacher on|off] [--student-model M] [--teacher-model M] \\
        [--perception-addr host:port]

Experiment 3 of the privileged ladder. The driver is a strong coder and a weak metric estimator
(docs/JOURNAL.md 2026-09-18). Here the coding happens OFFLINE, against labelled scenes, and what
comes out is a library the sensors-track driver can call during a benchmark run (run_eval.sh
CODE_LIB=<work>/final_lib). Nothing privileged crosses into the run: the library is code, written
by a model that never saw ground truth, scored on routes neither model trained on.

  scenes    tools/capture_route_scenes.py on a PRIVILEGED session with GT_DENSE=1 and full-resolution
            frames: RGB images, plus the answer -- actors_truth / parked_truth and GT depth +
            instance arrays per camera.
  student   `claude -p` in a workspace holding ONLY the RGB images of the train scenes (staged
            copies; the GT files are never copied) and its library, lib/. It may write and run code
            through carla_probe/code_runner.py, confined exactly as in a benchmark run. It never
            sees a label.
  scorer    this script runs lib.perceive() on every scene, confined, and matches its actors to the
            truth (the tools/eval_actor_perception.py matcher): recall, error and SIGNED along/across
            error per range band, plus the error in the nearest in-path gap -- the number a
            collision is made of.
  teacher   `claude -p` with everything: the GT scenes and arrays, the student's code, its
            predictions and the score. It writes feedback.md for the next round: where the student
            is wrong and why, and what to change. It may compute anything from the GT.
  control   --teacher off runs the same number of student rounds with no feedback beyond "improve
            it", so a gain can be attributed to the teaching rather than to more turns.

The library contract (the student is told it in full):
    lib/perceive.py: perceive(frame_dir: str, cameras: dict, pc=None) -> {"actors": [
        {"fwd": m, "left": m, "length_m": m, "width_m": m, "kind": "vehicle"|"pedestrian"|...}, ...]}
    frame_dir holds <CAM>.jpg (half size) and <CAM>_full.jpg (native) for every camera; cameras is
    the parsed cameras.json; pc is a carla_probe.perception_client.PerceptionClient or None.
    lib/README.md: how a driver should use it, written for that driver.

Outputs in --work: rounds/<k>/{lib, student_transcript.jsonl, train_score.json, feedback.md},
final_lib/, and test_score.json for the held-out scenes. Model calls go through the `claude` CLI
(the subscription).
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
from eval_actor_perception import BANDS, match  # noqa: E402

PY = sys.executable
RANGE_M = 40.0
CORRIDOR_HALF_M = 2.0   # an actor "in path": |left| below this, ahead of the ego

LIB_CONTRACT = """\
`lib/perceive.py` must define

    perceive(frame_dir: str, cameras: dict, pc=None) -> dict

returning {"actors": [{"fwd": float, "left": float, "length_m": float, "width_m": float,
"kind": "vehicle" | "pedestrian" | "bicycle" | "motorcycle"}, ...]} -- every road user within 40 m,
each at the CENTRE of its body, in the ego frame: fwd = metres ahead of the ego origin (negative
behind), left = metres to the LEFT (negative = right).

- `frame_dir` holds `<CAM>.jpg` (half resolution) and `<CAM>_full.jpg` (native) for the cameras
  CAM_FRONT, CAM_FRONT_LEFT, CAM_FRONT_RIGHT, CAM_BACK.
- `cameras` is the parsed cameras.json: per camera the mount (CARLA vehicle frame: x forward,
  y RIGHT, z up, yaw + = to the right) and the pinhole intrinsics of each image size, keyed
  `jpg` / `full_jpg`. The ego origin is the vehicle-frame origin.
- `pc` is a PerceptionClient (carla_probe.perception_client) when a perception service exists,
  else None. The library must work, less well, without it.
- numpy and cv2 are available. The code runs confined (carla_probe/code_runner.py): it may read
  frame_dir and its own files, write only in its own directory, and reach no network but `pc`.
  It must finish a frame in under 60 s.

`lib/README.md` must tell a DRIVER (a model driving the car, calling this from its own scripts
during a benchmark run) how to use the library, what its outputs mean, and how far to trust them
at each range.
"""


# --------------------------------------------------------------------------- scenes
def load_scenes(dirs: List[Path]) -> List[Dict[str, Any]]:
    scenes = []
    for d in dirs:
        for f in sorted(d.glob("scene_*.json")):
            s = json.loads(f.read_text())
            # capture_route_scenes.py writes every set into a dir called route_scenes: name the set
            # after the capture it came from, or ids collide across routes.
            name = d.parent.name if d.name == "route_scenes" else d.name
            s["_id"] = f"{name}__{f.stem}"
            scenes.append(s)
    assert scenes, f"no scene_*.json in {dirs}"
    return scenes


def stage_view(scenes: List[Dict[str, Any]], root: Path) -> None:
    """Copy ONLY what a sensors driver could see into root/<scene id>/: the RGB JPGs and a camera
    model with the dense-array entries removed. The GT arrays never leave the capture session."""
    root.mkdir(parents=True, exist_ok=True)
    for s in scenes:
        dst = root / s["_id"]
        if dst.exists():
            continue
        dst.mkdir()
        for name, path in s["files"].items():
            if name.endswith(".jpg"):
                shutil.copy2(path, dst / name)
        model = json.loads(Path(s["camera_model"]).read_text())
        for cam in model["cameras"].values():
            cam["intrinsics"] = {k: v for k, v in cam["intrinsics"].items() if k in ("jpg", "full_jpg")}
        model.pop("semantic_tags", None)
        (dst / "cameras.json").write_text(json.dumps(model, indent=1))


def truth_of(s: Dict[str, Any]) -> List[Dict[str, Any]]:
    return [a for a in s.get("actors_truth", []) + s.get("parked_truth", [])
            if (a["fwd"] ** 2 + a["left"] ** 2) ** 0.5 <= RANGE_M]


# --------------------------------------------------------------------------- running the library
ENTRY = """\
import json, sys
sys.path.insert(0, {code!r})
from lib.perceive import perceive
pc = None
try:
    from carla_probe.perception_client import PerceptionClient
    import os
    if os.environ.get("CARLA_PROBE_PERCEPTION_ADDR"):
        pc = PerceptionClient()
except Exception:
    pc = None
out = {{}}
for sid in sys.argv[1:]:
    d = {view!r} + "/" + sid
    try:
        cams = json.load(open(d + "/cameras.json"))
        r = perceive(d, cams, pc)
        out[sid] = {{"actors": [{{k: (float(v) if isinstance(v, (int, float)) else v) for k, v in a.items()}}
                                for a in r.get("actors", [])]}}
    except Exception as e:
        out[sid] = {{"error": type(e).__name__ + ": " + str(e)[:300]}}
print("@@RESULT@@" + json.dumps(out))
"""


def run_library(code_dir: Path, view: Path, ids: List[str], perception_addr: str) -> Dict[str, Any]:
    """Run lib.perceive over scenes, confined exactly as in a benchmark run, a batch at a time."""
    entry = code_dir / "_score_entry.py"
    entry.write_text(ENTRY.format(code=str(code_dir), view=str(view)))
    env = {**os.environ, "PYTHONPATH": str(REPO), "CARLA_PROBE_FRAMES": str(view),
           "CARLA_PROBE_CODE_DIR": str(code_dir), "CARLA_PROBE_PERCEPTION_ADDR": perception_addr,
           "CARLA_PROBE_CODE_TIMEOUT_S": "1800", "CARLA_PROBE_CODE_MAX_OUTPUT": "100000000"}
    out: Dict[str, Any] = {}
    for i in range(0, len(ids), 20):
        batch = ids[i:i + 20]
        p = subprocess.run([PY, "-m", "carla_probe.code_runner", str(entry), *batch], capture_output=True,
                           text=True, env=env, timeout=3600)
        if "@@RESULT@@" in p.stdout:
            out.update(json.loads(p.stdout.split("@@RESULT@@", 1)[1].strip().splitlines()[0]))
        else:
            for sid in batch:
                out[sid] = {"error": (p.stdout + p.stderr)[-500:]}
    entry.unlink()
    return out


def score(scenes: List[Dict[str, Any]], preds: Dict[str, Any]) -> Dict[str, Any]:
    bands = {b: {"n": 0, "found": 0, "err": [], "fwd": [], "left": []} for b in BANDS}
    extras, errors, gap_err, per_scene = 0, 0, [], []
    for s in scenes:
        t = truth_of(s)
        T = [[a["fwd"], a["left"]] for a in t]
        p = preds.get(s["_id"], {"error": "missing"})
        if "error" in p:
            errors += 1
            per_scene.append({"scene": s["_id"], "error": p["error"]})
            continue
        P = [[a["fwd"], a["left"]] for a in p["actors"] if "fwd" in a and "left" in a]
        pairs, ex = match(P, T)
        extras += len(ex)
        hit = {i: (j, e) for i, j, e in pairs}
        for i, (f, l) in enumerate(T):
            r = (f * f + l * l) ** 0.5
            for b in BANDS:
                if b[0] <= r < b[1]:
                    d = bands[b]
                    d["n"] += 1
                    if i in hit:
                        j, e = hit[i]
                        d["found"] += 1
                        d["err"].append(e)
                        d["fwd"].append(P[j][0] - f)
                        d["left"].append(P[j][1] - l)
        # The collision number: how far ahead is the nearest body in the ego's corridor?
        def gap(pts):
            ahead = [f for f, l in pts if f > 0 and abs(l) < CORRIDOR_HALF_M]
            return min(ahead) if ahead else None
        gt, gp = gap(T), gap(P)
        if gt is not None and gt < 30:
            gap_err.append((gp if gp is not None else 30.0) - gt)
        per_scene.append({"scene": s["_id"], "truth": len(T), "reported": len(P), "matched": len(pairs),
                          "extras": len(ex), "gap_true": gt, "gap_pred": gp})
    out = {"scenes": len(scenes), "errors": errors, "extras": extras, "bands": []}
    for b in BANDS:
        d = bands[b]
        out["bands"].append({"band": f"{b[0]:.0f}-{b[1]:.0f} m", "n": d["n"],
                             "recall": d["found"] / d["n"] if d["n"] else None,
                             "mean_err_m": float(np.mean(d["err"])) if d["err"] else None,
                             "signed_fwd_m": float(np.mean(d["fwd"])) if d["fwd"] else None,
                             "signed_left_m": float(np.mean(d["left"])) if d["left"] else None})
    out["in_path_gap"] = {"n": len(gap_err),
                          "mean_abs_err_m": float(np.mean(np.abs(gap_err))) if gap_err else None,
                          "signed_err_m": float(np.mean(gap_err)) if gap_err else None,
                          "over_estimates_gt_1m": int(sum(e > 1.0 for e in gap_err))}
    out["per_scene"] = per_scene
    return out


def headline(sc: Dict[str, Any]) -> str:
    parts = []
    for b in sc["bands"]:
        if b["n"]:
            parts.append(f"{b['band']}: recall {100 * (b['recall'] or 0):.0f}% err "
                         + (f"{b['mean_err_m']:.1f}" if b["mean_err_m"] is not None else "-"))
    g = sc["in_path_gap"]
    parts.append(f"in-path gap |err| {g['mean_abs_err_m']:.1f} m (n={g['n']}, over by >1 m: {g['over_estimates_gt_1m']})"
                 if g["mean_abs_err_m"] is not None else "in-path gap -")
    return f"errors {sc['errors']}/{sc['scenes']}, extras {sc['extras']} | " + " | ".join(parts)


# --------------------------------------------------------------------------- the two models
def claude(prompt: str, cwd: Path, model: str, tools: str, allow: List[str], env: Dict[str, str],
           transcript: Path) -> int:
    settings = {"sandbox": {"enabled": False}, "env": env, "permissions": {"allow": allow}}
    (cwd / ".settings.json").write_text(json.dumps(settings))
    with open(transcript, "w") as f:
        p = subprocess.run(["claude", "-p", prompt, "--model", model, "--output-format", "stream-json", "--verbose",
                            "--permission-mode", "dontAsk", "--tools", tools, "--setting-sources", "",
                            "--settings", str(cwd / ".settings.json")],
                           cwd=str(cwd), stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, timeout=7200)
    return p.returncode


STUDENT_PROMPT = """You are writing a perception library for a car driving in the CARLA simulator. It \
will be used by a driver (another model) that sees only the car's four cameras and must know where \
the other road users are, in metres, to avoid hitting them.

{contract}

Your workspace:
- {code}/lib/            your library (it may be empty: start it). Edit only inside {code}.
- {view}/<scene>/        {n} recorded scenes: the camera JPGs of each and its cameras.json. No labels.
- {code}/feedback/       notes from earlier rounds, newest last (read them all first, if any).

Test your code by writing a script in {code} and running it, one per Bash call, as
  {py} -m carla_probe.code_runner {code}/<script>.py [args]
(the same confinement the library will run under: it can read {view}, and read/write {code}.) Look at \
the images with the Read tool to check what your code finds. {perception}

Round {round} of {rounds}. {task} When you are done, make sure lib/perceive.py and lib/README.md are \
complete and working, then reply with a short summary of what changed."""

TEACHER_PROMPT = """You are teaching. A student model is writing a perception library for a car in the \
CARLA simulator from the camera images alone; it has never seen a label. You have the ground truth.

The library contract the student works to:
{contract}

In {ws}:
- student_lib/            the student's library as it stands after round {round}
- predictions.json        what it reported on each training scene
- score.json              that scored against the truth (bands, signed errors, the in-path gap)
- scenes.json             the training scenes: image paths, actors_truth / parked_truth (the answer,
                          ego frame [fwd, left], body centres, sizes) and, under "files", GT
                          <CAM>_depth.npy (float16 metres along the optical axis) and <CAM>_inst.png
                          (cv2.imread(p)[..., ::-1]: channel 0 = semantic tag, 1*256+2 = instance id)

You can run Python with: {py} <script.py> (numpy, cv2 available). Read images with the Read tool.

Find out WHY the student is wrong, not just where: pick the worst scenes, look at the images, compare \
its intermediate quantities (e.g. what depth it assumed for a box, which pixel it took as the ground \
contact, which camera it trusted) against the GT arrays, and find the systematic errors -- the ones \
with a consistent sign or a consistent cause. Then write {ws}/feedback.md for the student: the \
diagnosed faults ranked by how much they cost, each with the evidence (numbers you measured) and a \
concrete fix, including code where that is clearer. Teach methods that generalise to unseen streets; \
do not hand over per-scene answers, which the student will never have when it matters. The student \
will be scored next on DIFFERENT routes. Reply with a one-paragraph summary when feedback.md is written."""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--train", type=Path, nargs="+", required=True)
    ap.add_argument("--test", type=Path, nargs="+", required=True)
    ap.add_argument("--work", type=Path, required=True)
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--teacher", choices=["on", "off"], default="on")
    ap.add_argument("--student-model", default="claude-opus-5")
    ap.add_argument("--teacher-model", default="claude-opus-5")
    ap.add_argument("--perception-addr", default="")
    args = ap.parse_args()

    work = args.work.resolve()
    work.mkdir(parents=True, exist_ok=True)
    train, test = load_scenes(args.train), load_scenes(args.test)
    overlap = {s["_id"].split("__")[0] for s in train} & {s["_id"].split("__")[0] for s in test}
    assert not overlap, f"train and test share scene sets: {overlap}"
    views = work / "views"
    stage_view(train, views / "train")
    stage_view(test, views / "test")
    code = work / "student" / "code"
    (code / "lib").mkdir(parents=True, exist_ok=True)
    (code / "feedback").mkdir(exist_ok=True)
    (code / "lib" / "__init__.py").touch()
    perception_note = (f"A perception service is available to your code as `pc` "
                       f"(carla_probe.perception_client.PerceptionClient(): .depth(path) metric depth, "
                       f".segment(path, prompts) instance masks; .info()); your library may use it."
                       if args.perception_addr else "There is no perception service: cv2 and numpy only.")
    log = (work / "log.md").open("a")
    log.write(f"\n# run: teacher={args.teacher} rounds={args.rounds} train={len(train)} test={len(test)} "
              f"student={args.student_model} teacher={args.teacher_model} perception={args.perception_addr or 'none'}\n")

    for k in range(1, args.rounds + 1):
        rdir = work / "rounds" / f"{k:02d}"
        if (rdir / "train_score.json").exists():
            print(f"round {k}: done already")
            continue
        rdir.mkdir(parents=True, exist_ok=True)
        task = ("Write the library." if k == 1 else
                "Improve the library, starting from the newest feedback note." if args.teacher == "on" else
                "Improve the library: find where it is wrong on these scenes and fix it.")
        prompt = STUDENT_PROMPT.format(contract=LIB_CONTRACT, code=code, view=views / "train", n=len(train),
                                       py=PY, perception=perception_note, round=k, rounds=args.rounds, task=task)
        env = {"PYTHONPATH": str(REPO), "CARLA_PROBE_FRAMES": str(views / "train"), "CARLA_PROBE_CODE_DIR": str(code),
               "CARLA_PROBE_PERCEPTION_ADDR": args.perception_addr}
        allow = [f"Read(/{views / 'train'}/**)", f"Read(/{code}/**)", f"Write(/{code}/**)", f"Edit(/{code}/**)",
                 f"Bash({PY} -m carla_probe.code_runner:*)"]
        print(f"round {k}: student", flush=True)
        claude(prompt, code, args.student_model, "Bash,Read,Write,Edit", allow, env, rdir / "student_transcript.jsonl")
        shutil.copytree(code / "lib", rdir / "lib", dirs_exist_ok=True)

        preds = run_library(code, views / "train", [s["_id"] for s in train], args.perception_addr)
        sc = score(train, preds)
        (rdir / "predictions.json").write_text(json.dumps(preds))
        (rdir / "train_score.json").write_text(json.dumps(sc, indent=1))
        line = f"round {k} train: {headline(sc)}"
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

        if args.teacher == "on" and k < args.rounds:
            tws = rdir / "teacher"
            tws.mkdir(exist_ok=True)
            shutil.copytree(code / "lib", tws / "student_lib", dirs_exist_ok=True)
            shutil.copy2(rdir / "predictions.json", tws / "predictions.json")
            shutil.copy2(rdir / "train_score.json", tws / "score.json")
            (tws / "scenes.json").write_text(json.dumps({s["_id"]: s for s in train}, indent=1))
            dirs = sorted({str(Path(p).parent) for s in train for p in s["files"].values()})
            allow = [f"Read(/{tws}/**)", f"Write(/{tws}/**)", f"Bash({PY}:*)"] + [f"Read(/{d}/**)" for d in dirs]
            print(f"round {k}: teacher", flush=True)
            claude(TEACHER_PROMPT.format(contract=LIB_CONTRACT, ws=tws, round=k, py=PY), tws, args.teacher_model,
                   "Bash,Read,Write", allow, {"PYTHONPATH": str(REPO)}, rdir / "teacher_transcript.jsonl")
            fb = tws / "feedback.md"
            if fb.exists():
                shutil.copy2(fb, code / "feedback" / f"round_{k:02d}.md")
                shutil.copy2(fb, rdir / "feedback.md")
            else:
                log.write(f"round {k}: teacher wrote no feedback.md\n")

    final = work / "final_lib"
    shutil.copytree(code / "lib", final, dirs_exist_ok=True)
    preds = run_library(code, views / "test", [s["_id"] for s in test], args.perception_addr)
    sc = score(test, preds)
    (work / "test_predictions.json").write_text(json.dumps(preds))
    (work / "test_score.json").write_text(json.dumps(sc, indent=1))
    line = f"HELD-OUT test: {headline(sc)}"
    print(line)
    log.write(line + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
