#!/usr/bin/env python3
"""SimLingo as a service: one front image + speed + two route points + a sentence in, a path and
a speed profile out.

    third_party/vla_venv/bin/python carla_probe/vla_server.py --port 8791 --device cuda:1

This is the VLA half of the language-supervised driver: the LLM driver writes the sentence, this
model turns it into motion at every tick. It is the stock SimLingo checkpoint (the backbone of
BLUE, third_party/BLUE) with its own agent code stripped away -- no leaderboard, no Kalman filter,
no route planner -- because the daemon already owns all of that. What is left is the exact input
pipeline of team_code/agent_simlingo.py::tick (JPEG artefacts, bottom-crop, 2-tile InternVL
preprocessing, the internlm2 chat template, <TARGET_POINT> placeholders) and the model's forward.

Runs in its OWN interpreter (Python 3.8 + torch, scripts/setup_vla.sh) because the daemon's venv
is stdlib-only and shared with the leaderboard. HTTP with JSON bodies keeps the daemon's client
side pure stdlib, as client.py must be.

Frames: the wire format is the daemon's ego frame, [forward, LEFT] metres. SimLingo's own frame is
CARLA's vehicle frame, [forward, RIGHT]; the flip happens here, in one place, on the way in and out.

Language modes, each a prompt shape the checkpoint was trained on (simlingo_training/dataloader):
  plain        "... Target waypoint: <TP><TP>. Predict the waypoints."          no language at all
  commentary   "... Target waypoint: <TP><TP>. {text} Predict the waypoints."   scene description
  command      "... Command: {text} Predict the waypoints."                     no target points
  instruction  "<INSTRUCTION_FOLLOWING> ... Target waypoint: <TP><TP>. {text}"  imperative, obeyed
  safety       "<SAFETY> ... Target waypoint: <TP><TP>. {text}"                 imperative, may refuse
  cot          "... Target waypoint: <TP><TP>. What should the ego do next?"    model writes its own

POST /act   {"image_jpeg_b64", "speed_mps", "target_points": [[fwd,left],[fwd,left]],
             "text": str|null, "mode": str}
         -> {"route": [[fwd,left]x20], "speed_wps": [[fwd,left]x10], "desired_speed_mps",
             "language": str, "control": {"steer","throttle","brake"}, "ms"}
POST /reset  clears the controller state (call at route start)
GET  /health
"""

from __future__ import annotations

import argparse
import base64
import importlib.util
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import torch

# --------------------------------------------------------------------------- prompt shapes
_TP = "Target waypoint: <TARGET_POINT><TARGET_POINT>."
_MODES = {
    "plain":       lambda v, t: f"Current speed: {v} m/s. {_TP} Predict the waypoints.",
    "commentary":  lambda v, t: f"Current speed: {v} m/s. {_TP} {t} Predict the waypoints.",
    "command":     lambda v, t: f"Current speed: {v} m/s. Command: {t} Predict the waypoints.",
    "instruction": lambda v, t: f"<INSTRUCTION_FOLLOWING> Current speed: {v} m/s. {_TP} {t}",
    "safety":      lambda v, t: f"<SAFETY> Current speed: {v} m/s. {_TP} {t}",
    "cot":         lambda v, t: f"Current speed: {v} m/s. {_TP} What should the ego do next?",
}
_NEEDS_TEXT = {"commentary", "command", "instruction", "safety"}
# The answer each mode was trained to give before the action tokens (dataset_driving.py,
# dataset_dreamer.py). Modes absent here generate theirs.
_ANSWERS = {
    "plain": "Waypoints:", "commentary": "Waypoints:", "command": "Waypoints:",
    "instruction": "Following the given instruction. Waypoints:",
}
_SPECIAL_TOKENS = ['<WAYPOINTS>', '<WAYPOINTS_DIFF>', '<ORG_WAYPOINTS_DIFF>', '<ORG_WAYPOINTS>',
                   '<WAYPOINT_LAST>', '<ROUTE>', '<ROUTE_DIFF>', '<TARGET_POINT>']


def _flip(points) -> List[List[float]]:
    """[fwd, left] <-> [fwd, right]: the same operation both ways."""
    return [[float(p[0]), -float(p[1])] for p in points]


# SimLingo's two controllers, copied from team_code/transfuser_utils.py::PIDController and
# team_code/nav_planner.py::LateralPIDController (MIT, carla_garage) because both of those modules
# import `carla`, which this interpreter does not have and must not need.
class _SpeedPID:
    def __init__(self, k_p: float, k_i: float, k_d: float, n: int):
        from collections import deque
        self.k_p, self.k_i, self.k_d = k_p, k_i, k_d
        self.window = deque([0.0 for _ in range(n)], maxlen=n)

    def step(self, error: float) -> float:
        self.window.append(error)
        integral = float(np.mean(self.window))
        derivative = self.window[-1] - self.window[-2]
        return self.k_p * error + self.k_i * integral + self.k_d * derivative


class _LateralPID:
    """Leaderboard settings of LateralPIDController(inference_mode=False): the aim point is the
    route sample clip(0.976*v_kmh + 1.92, 24, 105) x 0.1 m ahead."""

    def __init__(self, k_p=3.118357247806046, k_d=1.3782508892109167, k_i=0.6406067986034124,
                 speed_scale=0.9755321901954155, speed_offset=1.9152884533402488, n=6):
        self.k_p, self.k_d, self.k_i = k_p, k_d, k_i
        self.speed_scale, self.speed_offset, self.n = speed_scale, speed_offset, n
        self._window: List[float] = []

    def step(self, route_np: np.ndarray, current_speed: float) -> float:
        v = current_speed * 3.6
        n_lookahead = int(min(np.clip(self.speed_scale * v + self.speed_offset, 24, 105), route_np.shape[0] - 1))
        n_lookahead = min(n_lookahead, len(route_np) - 1)
        aim = route_np[n_lookahead]
        heading_error = np.arctan2(aim[1], aim[0]) % (2 * np.pi)
        heading_error = heading_error if heading_error < np.pi else heading_error - 2 * np.pi
        heading_error = heading_error * 180.0 / np.pi / 90.0
        self._window.append(float(heading_error))
        self._window = self._window[-self.n:]
        derivative = 0.0 if len(self._window) == 1 else self._window[-1] - self._window[-2]
        integral = float(np.mean(self._window))
        return float(np.clip(self.k_p * heading_error + self.k_d * derivative + self.k_i * integral, -1.0, 1.0))


class SimLingo:
    def __init__(self, blue_root: Path, ckpt: Path, device: str, dtype: str):
        sys.path.insert(0, str(blue_root))
        sys.path.insert(0, str(blue_root / "team_code"))
        import hydra
        from omegaconf import OmegaConf
        from transformers import AutoConfig, AutoProcessor
        from huggingface_hub import snapshot_download
        from team_code.config_simlingo import GlobalConfig
        from simlingo_training.utils.internvl2_utils import build_transform, dynamic_preprocess
        from simlingo_training.utils.custom_types import DrivingInput, LanguageLabel
        from team_code.simlingo_utils import get_camera_extrinsics, get_camera_intrinsics

        self.DrivingInput, self.LanguageLabel = DrivingInput, LanguageLabel
        self.dynamic_preprocess, self.transform = dynamic_preprocess, build_transform(input_size=448)
        self.device = torch.device(device)
        self.dtype = {"bf16": torch.bfloat16, "fp16": torch.float16, "fp32": torch.float32}[dtype]

        # The checkpoint's Hydra config sits three levels up, as the agent expects it.
        cfg = OmegaConf.load(ckpt.parent.parent.parent / ".hydra" / "config.yaml")
        cfg.model.vision_model.use_global_img = cfg.data_module.use_global_img
        variant = cfg.model.vision_model.variant
        processor = AutoProcessor.from_pretrained(variant, trust_remote_code=True)
        self.tokenizer = processor.tokenizer if "tokenizer" in processor.__dict__ else processor
        self.tokenizer.add_special_tokens({"additional_special_tokens": _SPECIAL_TOKENS})
        self.tokenizer.padding_side = "left"
        self.tp_token_id = self.tokenizer.convert_tokens_to_ids("<TARGET_POINT>")

        default_dtype = torch.get_default_dtype()
        torch.set_default_dtype(self.dtype)
        self.model = hydra.utils.instantiate(
            cfg.model, cfg_data_module=cfg.data_module, processor=processor,
            cache_dir=str(blue_root / "pretrained" / variant.split("/")[1]), _recursive_=False)
        torch.set_default_dtype(default_dtype)
        missing, unexpected = self.model.load_state_dict(torch.load(str(ckpt), map_location="cpu"), strict=False)
        print(f"[vla] checkpoint loaded: {len(missing)} missing, {len(unexpected)} unexpected keys", flush=True)
        self.model = self.model.to(self.device, dtype=self.dtype).eval()

        # The InternVL chat template ships with the model snapshot, as remote code.
        snap = Path(snapshot_download(repo_id=variant))
        spec = importlib.util.spec_from_file_location("get_conv_template", str(snap / "conversation.py"))
        conv_module = importlib.util.module_from_spec(spec)
        sys.modules["get_conv_template"] = conv_module
        spec.loader.exec_module(conv_module)
        self.get_conv_template = conv_module.get_conv_template
        hf_cfg = AutoConfig.from_pretrained(variant, trust_remote_code=True)
        image_size = hf_cfg.force_image_size or hf_cfg.vision_config.image_size
        self.num_image_token = int((image_size // hf_cfg.vision_config.patch_size) ** 2 * (hf_cfg.downsample_ratio ** 2))
        self.eos = self.tokenizer.eos_token_id

        # SimLingo's own controllers, so `control` in the reply is what its agent would have sent.
        self.gc = GlobalConfig()
        self.intrinsics = get_camera_intrinsics(1024, 512, 110).view(1, 3, 3).to(self.device)
        self.extrinsics = get_camera_extrinsics().view(1, 4, 4).to(self.device)
        self.reset()

    def reset(self) -> None:
        self.speed_pid = _SpeedPID(self.gc.speed_kp, self.gc.speed_ki, self.gc.speed_kd, self.gc.speed_n)
        self.turn_pid = _LateralPID()

    # ------------------------------------------------------------------ input pipeline
    def _image(self, jpeg: bytes) -> torch.Tensor:
        """agent_simlingo.tick, verbatim: JPEG round trip, drop the bottom 30 %, 2 InternVL tiles."""
        bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        assert bgr is not None, "undecodable JPEG"
        _, enc = cv2.imencode(".jpg", bgr)
        bgr = cv2.imdecode(enc, cv2.IMREAD_UNCHANGED)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = rgb[:int(rgb.shape[0] - (rgb.shape[0] * 4.8) // 16), :, :]
        from PIL import Image
        tiles = self.dynamic_preprocess(Image.fromarray(rgb), image_size=448,
                                        use_thumbnail=self.model.vision_model.use_global_img, max_num=2)
        px = torch.stack([self.transform(t) for t in tiles])           # [n, 3, 448, 448]
        assert px.shape[0] == 2, f"expected 2 tiles for a 1024x512 front image, got {px.shape[0]}"
        return px.view(1, 1, px.shape[0], 3, 448, 448)

    def _prompt(self, text: str, target_points_right: np.ndarray, answer: Optional[str]):
        """The internlm2 chat sequence. With `answer` the assistant turn is written out, which is
        the TRAINING sequence (internvl2_utils.get_custom_chat_template's `prompt_conv`); without
        it the turn is left open for the model to generate (the agent's `prompt_question`)."""
        template = self.get_conv_template("internlm2-chat")
        template.append_message(template.roles[0], "<image>\n" + text)
        template.append_message(template.roles[1], answer)
        query = template.get_prompt()
        system_prompt = template.system_template.replace("{system_message}", template.system_message) + template.sep
        query = query.replace(system_prompt, "")
        query = query.replace("<image>", "<img>" + "<IMG_CONTEXT>" * self.num_image_token * 2 + "</img>", 1)
        tok = self.tokenizer([query], padding=True, return_tensors="pt", add_special_tokens=False)
        ids = tok["input_ids"]
        valid = ids != self.tokenizer.pad_token_id
        placeholders = [{self.tp_token_id: target_points_right}] if "<TARGET_POINT>" in text else [{}]
        return self.LanguageLabel(phrase_ids=ids.to(self.device), phrase_valid=valid.to(self.device),
                                  phrase_mask=valid.to(self.device), placeholder_values=placeholders,
                                  language_string=[query], loss_masking=None)

    # ------------------------------------------------------------------ one step
    @torch.inference_mode()
    def act(self, req: Dict[str, Any]) -> Dict[str, Any]:
        t0 = time.perf_counter()
        mode = req.get("mode", "plain")
        assert mode in _MODES, f"unknown mode {mode!r}; one of {sorted(_MODES)}"
        text = (req.get("text") or "").strip()
        assert text or mode not in _NEEDS_TEXT, f"mode {mode!r} needs text"
        speed = float(req["speed_mps"])
        tps = np.asarray(_flip(req["target_points"]), dtype=np.float32)
        assert tps.shape == (2, 2), "target_points must be two [fwd, left] points"
        prompt = _MODES[mode](round(speed, 1), text)

        # Where the answer is known in advance, write it out and take the single-forward path the
        # model was trained with, instead of generating it token by token (greedy_sample has no
        # KV cache: every token is a full forward, so "Waypoints:" alone cost 4 forwards). The
        # modes whose answer is a free decision -- the model's own commentary, or a safety refusal
        # -- keep generating.
        answer = _ANSWERS.get(mode)
        ll = self._prompt(prompt, tps, answer)
        inp = self.DrivingInput(
            camera_images=self._image(base64.b64decode(req["image_jpeg_b64"])).to(self.device, self.dtype),
            image_sizes=None, camera_intrinsics=self.intrinsics, camera_extrinsics=self.extrinsics,
            vehicle_speed=torch.tensor([[speed]], dtype=torch.float32, device=self.device),
            target_point=torch.tensor(tps[:1], dtype=torch.float32, device=self.device),
            prompt=ll, prompt_inference=ll)
        if answer is None:
            speed_wps, route, language = self.model(inp)
        else:
            speed_wps, route, language = self._single_forward(inp)
        route_np = route[0].float().cpu().numpy()
        swp_np = speed_wps[0].float().cpu().numpy()

        # control_pid, verbatim (speed waypoints are 0.25 s apart: 20 fps / data_save_freq 5).
        one_second = int(self.gc.carla_fps // (self.gc.wp_dilation * self.gc.data_save_freq))
        half_second = one_second // 2
        desired = float(np.linalg.norm(swp_np[half_second - 2] - swp_np[one_second - 2]) * 2.0)
        brake = bool((desired < self.gc.brake_speed) or ((speed / max(desired, 1e-6)) > self.gc.brake_ratio))
        delta = float(np.clip(desired - speed, 0.0, self.gc.clip_delta))
        throttle = float(np.clip(self.speed_pid.step(delta), 0.0, self.gc.clip_throttle))
        steer = float(np.clip(self.turn_pid.step(self._interp(route_np), speed), -1.0, 1.0))
        return {
            "route": _flip(route_np), "speed_wps": _flip(swp_np), "desired_speed_mps": round(desired, 3),
            "language": (language[0] if language else answer) or "",
            "prompt": prompt,
            "control": {"steer": round(steer, 3), "throttle": 0.0 if brake else round(throttle, 3),
                        "brake": 1.0 if brake else 0.0},
            "ms": round((time.perf_counter() - t0) * 1000.0, 1),
        }

    def _single_forward(self, inp):
        """The training-time sequence in one pass: [image | prompt | answer | 30 action queries].
        This is BLUE's direct-action branch (driving_gate.py, `num_quiet == batch_size`) minus its
        second call to replace_placeholder_tokens, which re-ran the vision encoder for nothing.
        Upstream DrivingModel.forward's own no-language branch is broken (it indexes the tuple
        forward_model returns), so it is not used."""
        m = self.model
        d = m.adaptors(inp, inference=True)
        d = m.vision_model.image_encoder.replace_placeholder_tokens(
            adaptor_dict=d, pixel_values=inp.camera_images,
            placeholder_values=inp.prompt_inference.placeholder_values, wp_encoder=m.wp_encoder)
        out = m.language_model.model(inputs_embeds=d["inputs"].to(m.language_model.model.dtype),
                                     attention_mask=d["inputs_mask"], output_hidden_states=True, return_dict=True)
        features = out.hidden_states[-1]
        by_adaptor = m.adaptors.split_outputs_by_adaptor(d, features)
        pred = m.adaptors.driving.get_predictions(by_adaptor["driving"])
        return pred["speed_wps"], pred["route"], None

    @staticmethod
    def _interp(waypoints: np.ndarray) -> np.ndarray:
        """agent_simlingo.interpolate_waypoints: resample the path every 0.1 m from the ego."""
        from scipy.interpolate import PchipInterpolator
        wps = np.concatenate((np.zeros_like(waypoints[:1]), waypoints))
        shift = np.roll(wps, 1, axis=0)
        shift[0] = shift[1]
        dists = np.cumsum(np.linalg.norm(wps - shift, axis=1)) + np.arange(len(wps)) * 1e-4
        pts = PchipInterpolator(dists, wps, axis=0)(np.arange(0.1, dists[-1], 0.1))
        return pts if pts.shape[0] else wps[None, -1]


# --------------------------------------------------------------------------- HTTP
def serve(model: SimLingo, host: str, port: int) -> None:
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code: int, body: Dict[str, Any]) -> None:
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._send(200, {"ok": True, "device": str(model.device), "dtype": str(model.dtype)})

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
                if self.path == "/reset":
                    model.reset()
                    self._send(200, {"ok": True})
                elif self.path == "/act":
                    self._send(200, dict(ok=True, **model.act(req)))
                else:
                    self._send(404, {"ok": False, "error": f"no route {self.path}"})
            except Exception as e:  # report, never die: the daemon is blocked on us
                self._send(500, {"ok": False, "error": f"{type(e).__name__}: {e}"})

        def log_message(self, *a):  # quiet; the daemon logs each call
            pass

    httpd = HTTPServer((host, port), Handler)
    print(f"[vla] serving on {host}:{port}", flush=True)
    httpd.serve_forever()


def main() -> int:
    repo = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8791)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--dtype", default="bf16", choices=["bf16", "fp16", "fp32"])
    ap.add_argument("--blue-root", type=Path, default=repo / "third_party" / "BLUE")
    ap.add_argument("--ckpt", type=Path,
                    default=repo / "third_party" / "models" / "simlingo" / "checkpoints" / "epoch=013.ckpt" / "pytorch_model.pt")
    ap.add_argument("--warmup", action="store_true", help="run one dummy step after loading and report its time")
    args = ap.parse_args()
    os.environ.setdefault("HF_HOME", str(repo / "third_party" / "hf_cache"))
    os.chdir(args.blue_root)  # the agent code resolves `pretrained/` relative to cwd

    model = SimLingo(args.blue_root, args.ckpt, args.device, args.dtype)
    if args.warmup:
        img = np.zeros((512, 1024, 3), np.uint8)
        _, enc = cv2.imencode(".jpg", img)
        out = model.act({"image_jpeg_b64": base64.b64encode(enc.tobytes()).decode(), "speed_mps": 0.0,
                         "target_points": [[10.0, 0.0], [20.0, 0.0]], "mode": "plain"})
        print(f"[vla] warmup: {out['ms']} ms, route[0]={out['route'][0]}, language={out['language']!r}", flush=True)
        model.reset()
    serve(model, args.host, args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
