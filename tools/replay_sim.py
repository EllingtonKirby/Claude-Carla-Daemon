#!/usr/bin/env python3
"""Replay a session's CARLA recording: summary, collisions, or rendered frames for a step range.

    python tools/replay_sim.py SESSION_DIR info
    python tools/replay_sim.py SESSION_DIR collisions
    python tools/replay_sim.py SESSION_DIR frames --from-step 200 --to-step 320 [--view chase|front|top]
                                 [--every 1] [--size 960x540] [--video] [--out DIR]    # --video needs ffmpeg
                                 [--captions] [--slow 2] [--hold 4]

`--captions` puts the driver's commentary in a panel UNDER the picture instead of over it: the
command, speed vs. target, and the decision's full `why` (never truncated: the font shrinks to fit),
with scoring events in red. `--slow` stretches playback (2 = half speed) and `--hold` freezes the first
frame of each new decision for that many seconds, so every justification can actually be read --
at real time a decision is on screen for about a second.

The recording (`<route>_rep0.log`, written when run_session.sh runs with SIM_RECORD=1) holds every
actor's state on every tick, so any moment of a drive can be re-rendered from any viewpoint after the
fact, on either track. `frames` overlays each image with the agent step, the driver's decision in force
(command + `why`, from decisions.jsonl) and scoring events at that tick (score_events.jsonl), so a
reviewer can judge whether a decision fits the scene.

Step <-> recording time: the patched evaluator writes `<route>_rep0.json` with the world frame at
recorder start (F0); trajectory.jsonl maps agent steps to world frames; world frame F is replayed at
(F - F0 - 1) * fixed_delta_seconds.

Needs a CARLA 0.9.15 server. If none answers on --port, one is started on --gpu and stopped at the end
(use a port no live session uses). Run inside the venv (`source scripts/env.sh`).
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import textwrap
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import carla
import cv2
import numpy as np

VIEWS = {  # camera transform relative to the ego (CARLA vehicle frame: x forward, y right, z up)
    "chase": carla.Transform(carla.Location(x=-7.5, z=3.5), carla.Rotation(pitch=-15.0)),
    "front": carla.Transform(carla.Location(x=0.8, z=1.7)),
    "top": carla.Transform(carla.Location(z=45.0), carla.Rotation(pitch=-90.0)),
    # ahead of the ego looking back at it: what is following (an emergency vehicle, a tailgater)
    "rear": carla.Transform(carla.Location(x=9.0, z=4.0), carla.Rotation(pitch=-14.0, yaw=180.0)),
}


def _jsonl(path: Path) -> List[Dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()] if path.is_file() else []


class Session:
    def __init__(self, session_dir: Path) -> None:
        self.dir = session_dir.resolve()
        logs = sorted(self.dir.glob("*_rep*.log"))
        if not logs:
            sys.exit(f"no CARLA recording (*_rep*.log) in {self.dir}; was the session run with SIM_RECORD=1?")
        self.recording = logs[0]
        meta_path = self.recording.with_suffix(".json")
        if not meta_path.is_file():
            sys.exit(f"missing {meta_path.name} (recorder start frame); re-run setup_bench2drive.sh for the current patch")
        meta = json.loads(meta_path.read_text())
        self.start_frame = int(meta["start_world_frame"])
        self.dt = float(meta["fixed_delta_seconds"])
        traj = _jsonl(self.dir / "trajectory.jsonl")
        self.step_to_frame = {int(r["step"]): int(r["frame"]) for r in traj if "frame" in r}
        self.step_to_t = {int(r["step"]): float(r["t"]) for r in traj}
        self.step_to_speed = {int(r["step"]): float(r.get("speed", 0.0)) for r in traj}
        self.frame_to_step = {f: s for s, f in self.step_to_frame.items()}
        rows = _jsonl(self.dir / "decisions.jsonl")
        self.decisions = [r for r in rows if "cmd" in r]
        self.results = {r["i"]: r for r in rows if "result" in r}
        self.events = _jsonl(self.dir / "score_events.jsonl")
        self.route_xml = Path(json.loads((self.dir / "session.json").read_text())["routes"])

    def weather(self) -> Tuple[Optional[carla.WeatherParameters], bool]:
        """The route's start weather (the recorder does not store weather), and whether the route
        changes weather along the way (the leaderboard interpolates by route progress)."""
        if not self.route_xml.is_file():
            return None, False
        entries = ET.parse(self.route_xml).getroot().iter("weather")
        entries = sorted((dict(e.attrib) for e in entries), key=lambda a: float(a.get("route_percentage", 0)))
        if not entries:
            return None, False
        weather = carla.WeatherParameters()
        for key, value in entries[0].items():
            if key != "route_percentage" and hasattr(weather, key):
                setattr(weather, key, float(value))
        varies = any({k: v for k, v in e.items() if k != "route_percentage"} != {k: v for k, v in entries[0].items() if k != "route_percentage"} for e in entries)
        return weather, varies

    def time_of_frame(self, frame: int) -> float:
        return (frame - self.start_frame - 1) * self.dt

    def frame_of_time(self, t: float) -> int:
        return self.start_frame + 1 + int(round(t / self.dt))

    def decision_at(self, step: int) -> Optional[Dict[str, Any]]:
        """The latest decision issued at or before ``step``."""
        current = None
        for d in self.decisions:
            if d["step"] <= step:
                current = d
        return current


def connect(port: int, gpu: int) -> Tuple[carla.Client, Optional[subprocess.Popen]]:
    client = carla.Client("localhost", port)
    client.set_timeout(5.0)
    try:
        client.get_server_version()
        return client, None
    except RuntimeError:
        pass
    carla_root = os.environ.get("CARLA_ROOT", str(Path.home() / "carla"))
    print(f"starting CARLA on port {port}, GPU {gpu} ({carla_root})", flush=True)
    proc = subprocess.Popen([f"{carla_root}/CarlaUE4.sh", "-RenderOffScreen", "-nosound", f"-graphicsadapter={gpu}",
                             f"-carla-rpc-port={port}"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)
    try:
        for _ in range(180):  # a cold start on a fresh GPU can take several minutes
            time.sleep(2)
            try:
                client = carla.Client("localhost", port)  # a client created before the server is up never connects
                client.set_timeout(5.0)
                client.get_server_version()
                return client, proc
            except RuntimeError:
                if proc.poll() is not None:
                    sys.exit("CARLA exited during startup")
        sys.exit("CARLA did not answer within 360 s")
    except BaseException:  # includes SystemExit from a signal: never leave the server behind
        os.killpg(proc.pid, signal.SIGKILL)
        raise


def _image_for(images: "queue.Queue[carla.Image]", frame: int, timeout_s: float = 10.0) -> Optional[carla.Image]:
    """The camera image of server frame ``frame`` (sensor data can arrive after tick() returns)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            img = images.get(timeout=0.5)
        except queue.Empty:
            continue
        if img.frame == frame:
            return img
        if img.frame > frame:
            return None
    return None


def overlay(img: np.ndarray, sess: Session, frame: int) -> np.ndarray:
    step = sess.frame_to_step.get(frame)
    lines = [f"frame {frame}  " + (f"step {step}  t={sess.step_to_t[step]:.2f}s" if step is not None else "(before/after agent)")]
    if step is not None:
        d = sess.decision_at(step)
        if d is not None:
            args = " ".join(f"{k}={v}" for k, v in d["args"].items() if k != "waypoints")
            lines.append(f"[{d['i']}] step {d['step']}: {d['cmd']} {args}")
            lines += textwrap.wrap("why: " + d["why"], width=max(40, img.shape[1] // 11))[:4]
        for ev in sess.events:
            if abs(int(ev["frame_seen"]) - frame) <= 2:
                lines.append(f"EVENT {ev['type']}: {ev['message']}"[: img.shape[1] // 8])
    out = img.copy()
    y = 22
    for i, text in enumerate(lines):
        color = (0, 0, 255) if text.startswith("EVENT") else (255, 255, 255)
        cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        y += 22
    return out


def _wrap_px(text: str, width_px: int, scale: float) -> List[str]:
    """Greedy word wrap by MEASURED pixel width (an average-character estimate overflows on wide glyphs)."""
    lines, cur = [], ""
    for word in text.split():
        trial = f"{cur} {word}".strip()
        if cv2.getTextSize(trial, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0] <= width_px or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = word
    return lines + ([cur] if cur else [])


def _fit_lines(text: str, width_px: int, max_lines: int, scale0: float = 0.6) -> Tuple[List[str], float]:
    """Wrap `text` to width_px, shrinking the font until it fits in max_lines (truncating only if even
    the smallest font cannot hold it)."""
    scale = scale0
    while True:
        lines = _wrap_px(text, width_px, scale)
        if len(lines) <= max_lines:
            return lines, scale
        if scale <= 0.36:
            return lines[:max_lines - 1] + [lines[max_lines - 1] + " ..."], scale
        scale -= 0.04


def captioned(img: np.ndarray, sess: Session, frame: int, panel_h: int) -> Tuple[np.ndarray, Optional[int]]:
    """The picture with a commentary panel below it. Returns (image, index of the decision in force)."""
    h, w = img.shape[:2]
    out = np.zeros((h + panel_h, w, 3), np.uint8)
    out[:h] = img
    out[h:] = (24, 24, 24)
    step = sess.frame_to_step.get(frame)
    decision = sess.decision_at(step) if step is not None else None
    font = cv2.FONT_HERSHEY_SIMPLEX
    # events: red banner over the picture, where the eye already is
    # min_speed notices fire constantly and Bench2Drive does not penalise them: they would bury the rest
    ev = [e for e in sess.events if abs(int(e["frame_seen"]) - frame) <= 10 and not e["type"].startswith("min_speed")]
    for k, e in enumerate(ev[:2]):
        txt = f"{e['type']}: {e['message']}"[: w // 9]
        cv2.rectangle(out, (0, h - 30 * (k + 1)), (w, h - 30 * k), (0, 0, 160), -1)
        cv2.putText(out, txt, (8, h - 30 * k - 9), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    if step is None:
        cv2.putText(out, "(before the agent's first tick / after the route)", (10, h + 28), font, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        return out, None
    v = sess.step_to_speed.get(step, 0.0)
    head = f"t={sess.step_to_t[step]:5.1f}s  step {step}   speed {v:4.1f} m/s"
    if decision is not None:
        a = decision["args"]
        target = a.get("target_speed")
        head += (f" (target {target})" if target is not None else "") + f"   #{decision['i']} {decision['cmd']}" + \
            (f" {a['ticks']} ticks" if "ticks" in a else "")
    cv2.putText(out, head, (10, h + 26), font, 0.6, (120, 220, 255), 1, cv2.LINE_AA)
    if decision is not None:
        lines, scale = _fit_lines(decision["why"], w - 20, max_lines=max(1, (panel_h - 44) // 19))
        line_h = int(34 * scale) + 4
        y = h + 26 + line_h + 6
        for line in lines:
            cv2.putText(out, line, (10, y), font, scale, (235, 235, 235), 1, cv2.LINE_AA)
            y += line_h
    return out, (None if decision is None else int(decision["i"]))


def render_frames(client: carla.Client, sess: Session, args: argparse.Namespace) -> None:
    if args.from_step not in sess.step_to_frame or args.to_step not in sess.step_to_frame:
        sys.exit(f"steps must be within 0..{max(sess.step_to_frame)} (trajectory.jsonl)")
    first, last = sess.step_to_frame[args.from_step], sess.step_to_frame[args.to_step]
    width, height = (int(v) for v in args.size.split("x"))
    out_dir = Path(args.out or sess.dir / "replay" / f"{args.view}_step{args.from_step:05d}-{args.to_step:05d}")
    out_dir.mkdir(parents=True, exist_ok=True)

    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, sess.dt
    world.apply_settings(settings)
    warmup = 2  # ticks to spawn the replayed ego and attach the camera before the first wanted frame
    start_frame = max(sess.start_frame + 1, first - warmup)
    print(client.replay_file(str(sess.recording), sess.time_of_frame(start_frame), 0.0, 0, False).strip(), flush=True)
    world = client.get_world()
    # replay_file() turns synchronous mode OFF (settings come back synchronous_mode=False,
    # fixed_delta_seconds=None). Without this re-apply the world advances by wall-clock delta
    # (~0.023 s/tick observed) instead of sess.dt, so counting ticks drifts without bound: the
    # ego was 45 m from its labelled position by mid-route, which is the replay/overlay desync.
    world.apply_settings(settings)
    weather, varies = sess.weather()
    if weather is not None:
        world.tick()  # the replayer (re)loads the map on the first tick; set weather after it
        start_frame += 1
        world.set_weather(weather)
        if varies:
            print("note: this route changes weather along the way; the replay uses its start weather")

    camera, images, written, missing = None, queue.Queue(), [], 0
    starts: List[bool] = []   # per written frame: does a new decision begin here (for --hold)
    last_decision: Optional[int] = None
    # With synchronous mode re-applied above, one world.tick() advances exactly sess.dt, i.e. one
    # recorded frame, so counting ticks is sound. The increment belongs BEFORE the frame is used:
    # start_frame is the frame already on screen, so the tick lands on start_frame + 1. Incrementing
    # afterwards labelled every image one frame early, and the decision/why overlay with it.
    rec_frame = start_frame
    try:
        while rec_frame <= last:
            server_frame = world.tick()
            rec_frame += 1   # the tick just advanced the world to this frame
            if camera is None:
                hero = [a for a in world.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"]
                if hero:
                    bp = world.get_blueprint_library().find("sensor.camera.rgb")
                    bp.set_attribute("image_size_x", str(width))
                    bp.set_attribute("image_size_y", str(height))
                    bp.set_attribute("fov", "90" if args.view != "front" else "100")
                    camera = world.spawn_actor(bp, VIEWS[args.view], attach_to=hero[0])
                    camera.listen(images.put)
            elif rec_frame >= first and (rec_frame - first) % args.every == 0:
                img = _image_for(images, server_frame)
                if img is None:
                    missing += 1
                    continue
                arr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(img.height, img.width, 4)[:, :, :3]
                step = sess.frame_to_step.get(rec_frame)
                path = out_dir / f"frame_{rec_frame:06d}" f"{'' if step is None else f'_step_{step:05d}'}.jpg"
                if args.captions:
                    pic, di = captioned(np.ascontiguousarray(arr), sess, rec_frame, panel_h=args.panel)
                    starts.append(di is not None and di != last_decision)
                    last_decision = di if di is not None else last_decision
                else:
                    pic = overlay(np.ascontiguousarray(arr), sess, rec_frame)
                    starts.append(False)
                cv2.imwrite(str(path), pic)
                written.append(path)
    finally:
        if camera is not None:
            camera.stop()
            camera.destroy()
        client.stop_replayer(False)
        settings.synchronous_mode = False
        world.apply_settings(settings)
    print(f"wrote {len(written)} frame(s) to {out_dir}" + (f" ({missing} missing: ego not spawned yet or no image)" if missing else ""))
    if args.video and written:
        video = out_dir / "replay.mp4"
        # ffmpeg H.264 (yuv420p, faststart) plays everywhere, unlike OpenCV's mp4v. Per-frame
        # durations through the concat demuxer: real time x --slow, plus --hold on decision starts.
        frame_s = sess.dt * args.every * args.slow
        listing = out_dir / "frames.txt"
        with listing.open("w") as f:
            for path, start in zip(written, starts):
                f.write(f"file '{path.name}'\nduration {frame_s + (args.hold if start else 0.0):.4f}\n")
            f.write(f"file '{written[-1].name}'\n")   # concat needs the last file repeated to honour its duration
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(listing),
                        "-vf", "fps=20,pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                        "-crf", "20", "-movflags", "+faststart", str(video)], check=True)
        print(f"wrote {video}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("action", choices=("info", "collisions", "frames"))
    ap.add_argument("--from-step", type=int, default=0)
    ap.add_argument("--to-step", type=int, default=100)
    ap.add_argument("--every", type=int, default=1, help="keep every Nth tick")
    ap.add_argument("--view", choices=sorted(VIEWS), default="chase")
    ap.add_argument("--size", default="960x540")
    ap.add_argument("--video", action="store_true", help="also write replay.mp4")
    ap.add_argument("--captions", action="store_true", help="commentary panel under the picture (full why text)")
    ap.add_argument("--panel", type=int, default=200, help="caption panel height in px (with --captions)")
    ap.add_argument("--slow", type=float, default=1.0, help="playback slow-down factor for --video (2 = half speed)")
    ap.add_argument("--hold", type=float, default=0.0, help="seconds to freeze on each new decision (--video)")
    ap.add_argument("--out", default=None, help="default: SESSION_DIR/replay/<view>_step<from>-<to>/")
    ap.add_argument("--port", type=int, default=int(os.environ.get("REPLAY_CARLA_PORT", "20200")))
    ap.add_argument("--gpu", type=int, default=0)
    args = ap.parse_args()

    sess = Session(args.session)
    # The server we start runs in its own session: turn SIGTERM/SIGHUP (e.g. `timeout`) into a normal
    # exit so the finally below still stops it.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: sys.exit(1))
    client, proc = connect(args.port, args.gpu)
    client.set_timeout(300.0)
    try:
        if args.action == "info":
            header = [line for line in client.show_recorder_file_info(str(sess.recording), False).splitlines()
                      if line.startswith(("Map:", "Date:", "Frames:", "Duration:"))]
            print("\n".join([f"Recording: {sess.recording}"] + header))
            steps = sorted(sess.step_to_frame)
            if steps:
                print(f"Agent steps {steps[0]}..{steps[-1]} = world frames {sess.step_to_frame[steps[0]]}.."
                      f"{sess.step_to_frame[steps[-1]]} = replay time "
                      f"{sess.time_of_frame(sess.step_to_frame[steps[0]]):.2f}..{sess.time_of_frame(sess.step_to_frame[steps[-1]]):.2f} s")
            print(f"{len(sess.decisions)} decision(s), {len(sess.events)} score event(s)")
        elif args.action == "collisions":
            text = client.show_recorder_collisions(str(sess.recording), "a", "a")
            for line in text.splitlines():
                parts = line.split()
                if parts and parts[0].replace(".", "", 1).isdigit():
                    frame = sess.frame_of_time(float(parts[0]))
                    line += f"   <- ~world frame {frame}, ~step {sess.frame_to_step.get(frame)}"
                print(line)
            print("(recorder collision times are whole seconds, so steps are +-10; score_events.jsonl has exact steps)")
        else:
            render_frames(client, sess, args)
    finally:
        if proc is not None:
            os.killpg(proc.pid, signal.SIGKILL)
    return 0


if __name__ == "__main__":
    sys.exit(main())
