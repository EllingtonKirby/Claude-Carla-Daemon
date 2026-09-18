"""BLUE's agent with one substitution: when its gate decides the model should reason in language
before acting, the commentary comes from an LLM looking at the same image, not from SimLingo.

Everything else is BLUE's stock pipeline (third_party/BLUE/team_code/agent_simlingo.py: camera,
gate, prompt, action decoding, controllers). The hook is `greedy_sample`: BLUE's Pass 2 calls it to
generate the commentary and appends the action queries after the embeddings it returns. Here it
returns the oracle's sentence instead -- tokenized with SimLingo's tokenizer and embedded with
SimLingo's embedding matrix, i.e. re-encoded exactly as SimLingo would have encoded its own words
-- so the action head reads the LLM's words in the model's own representation. What the model
would have said is still generated, for the record, and used only if the oracle fails.

Runs inside BLUE's leaderboard (scripts/run_blue_eval.sh AGENT=oracle), in the VLA venv. The
oracle is `claude -p` on the subscription, one call per gate firing with the sim frozen, reading
the frame's JPEG. Knobs: `blue_oracle` in carla_probe/config.yaml. Every firing is logged to
SESSION_DIR/oracle_commentary.jsonl with both sentences and the call's cost.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional

_REPO = Path(__file__).resolve().parents[1]
_BLUE = _REPO / "third_party" / "BLUE"
for p in (str(_BLUE), str(_BLUE / "team_code")):
    if p not in sys.path:
        sys.path.insert(0, p)

import cv2  # noqa: E402
import torch  # noqa: E402
import torch.nn.functional as F  # noqa: E402
import yaml  # noqa: E402

from team_code import agent_simlingo  # noqa: E402

_PHRASEBOOK = """\
Answer with exactly two sentences from this phrasebook and nothing else, on one line.
Sentence 1, the manoeuvre -- one of:
  Follow the route. | Turn left. | Turn right. | Prepare to do a lane change. |
  Stay on your current lane to go around the construction site. |
  Stay on your current lane to give way to the emergency vehicle. |
  Stay on your current lane to overtake the bikes on your lane. |
  Steer clear of the parked vehicle. | Avoid the accident on your lane. |
  Avoid the vehicle that is opening its door. |
  Return to your original route after avoiding the obstacle.
Sentence 2, the speed -- one of (OBJECT is a plain noun like car, truck, bike, pedestrian; N a whole number of metres):
  Accelerate to drive with the target speed. | Maintain your current speed. | Maintain the reduced speed. |
  Decelerate to drive with the target speed. | Decelerate to stay behind the OBJECT in N meters. |
  Accelerate to follow the OBJECT in N meters. | Maintain the reduced speed to stay behind the OBJECT in N meters. |
  Remain stopped to stay behind the OBJECT. | Stop now to stay behind the OBJECT in N meters. |
  Decelerate because the traffic light is red. | Accelerate because the traffic light is green. |
  Decelerate to drive through the junction. | Accelerate to drive through the junction. |
  Wait for a gap in the traffic before changing lanes. | Decelerate due to the OBJECT in N meters."""

_PROMPT = """You are the commentary module of a self-driving car in the CARLA simulator, on the Bench2Drive \
benchmark. Look at the front camera image at {image} (open it with the Read tool). The car is doing \
{speed:.1f} m/s. The route ahead: {route}.

Say what the car should do next, as the driving model expects to hear it.
{phrasebook}"""


_MANOEUVRES = tuple(m.strip() for m in re.findall(r"^\s+(.*)$", _PHRASEBOOK.split("Sentence 2")[0], re.M)
                    for m in m.split("|") if m.strip())
_SPEED_STARTS = ("Accelerate ", "Maintain ", "Decelerate ", "Remain stopped ", "Stop now ", "Wait for a gap ")


def _phrasebook_shaped(text: str) -> bool:
    """Manoeuvre sentence from the list, speed sentence with a phrasebook verb, nothing else."""
    parts = re.split(r"(?<=\.) ", text.strip())
    return (len(parts) == 2 and parts[0] in _MANOEUVRES
            and parts[1].startswith(_SPEED_STARTS) and parts[1].endswith("."))


def get_entry_point() -> str:
    return "OracleLingoAgent"


def _route_words(tps) -> str:
    """The two target points, as words: SimLingo gets them as numbers, the oracle as a direction."""
    if tps is None or len(tps) < 2:
        return "unknown"
    far = tps[1]
    fwd, right = float(far[0]), float(far[1])   # SimLingo's frame: [forward, right]
    dist = (fwd ** 2 + right ** 2) ** 0.5
    if abs(right) < 0.25 * max(fwd, 1.0):
        return f"straight ahead for at least {dist:.0f} m"
    return f"turns {'right' if right > 0 else 'left'} within about {dist:.0f} m"


class OracleLingoAgent(agent_simlingo.LingoAgent):
    def setup(self, path_to_conf_file, route_index=None):
        super().setup(path_to_conf_file, route_index)
        cfg = yaml.safe_load((_REPO / "carla_probe" / "config.yaml").read_text())["blue_oracle"]
        self._oracle_model = str(cfg["model"])
        self._oracle_reuse = int(cfg["reuse_frames"])
        self._oracle_timeout = float(cfg["timeout_s"])
        self._oracle_log_native = bool(cfg["log_native"])
        self._oracle_dir = Path(os.environ["BLUE_OUTPUT_DIR"])
        self._oracle_log = open(self._oracle_dir / "oracle_commentary.jsonl", "a")
        self._oracle_ws = Path(tempfile.mkdtemp(prefix="blue_oracle."))
        (self._oracle_ws / "settings.json").write_text(json.dumps({
            "sandbox": {"enabled": False},
            # Claude Code's path rules take an absolute path as "//abs/path" (run_eval.sh does the same).
            "permissions": {"allow": [f"Read(/{self._oracle_dir}/oracle_frames/**)"],
                            "deny": [f"Read(/{_REPO}/**)", f"Read(/{self._oracle_dir}/*.*)"]}}))
        self._oracle_last: Optional[Dict[str, Any]] = None
        self._oracle_cost = 0.0
        lm = self.model.language_model
        native = lm.greedy_sample

        def oracle_sample(input_embeds, **kw):
            return self._oracle_sample(native, input_embeds, **kw)

        lm.greedy_sample = oracle_sample
        print(f"[oracle] commentary from {self._oracle_model}; reuse_frames={self._oracle_reuse}", flush=True)

    # ------------------------------------------------------------------ the hook
    def _oracle_sample(self, native, input_embeds, **kw):
        t0 = time.time()
        row: Dict[str, Any] = {"step": int(self.step), "speed": round(float(self.DrivingInput["vehicle_speed"][0, 0]), 2)}
        native_text = None
        if self._oracle_log_native:
            toks, _ = native(input_embeds, **kw)
            native_text = self.tokenizer.batch_decode(toks, skip_special_tokens=True)[0]
        row["native"] = native_text

        reused = self._oracle_last is not None and self.step - self._oracle_last["step"] <= self._oracle_reuse
        if reused:
            text, cost = self._oracle_last["text"], 0.0
        else:
            text, cost = self._ask_oracle()
        row.update(oracle=text, reused=reused, cost_usd=cost)
        if text is None:                       # the oracle failed: fall back to the model's own words
            row["used"] = "native"
            if native_text is None:
                toks, embeds = native(input_embeds, **kw)
            else:
                toks, embeds = self._teacher_force(input_embeds, native_text, kw)
        else:
            row["used"] = "oracle"
            self._oracle_last = {"step": int(self.step), "text": text}
            toks, embeds = self._teacher_force(input_embeds, text, kw)
        self._oracle_cost += cost
        row["ms"] = round((time.time() - t0) * 1000.0)
        self._oracle_log.write(json.dumps(row) + "\n")
        self._oracle_log.flush()
        return toks, embeds

    def _teacher_force(self, input_embeds, text: str, kw):
        """Encode `text` the way SimLingo encodes its own commentary: its tokenizer, its embedding
        matrix, the same ' Waypoints:' suffix and EOS that greedy_sample would have produced, so
        the action queries that follow see a sequence shaped exactly like a generated one."""
        text = text.strip()
        if not text.endswith("Waypoints:"):
            text = text + " Waypoints:"
        eos = kw.get("eos_token_id")
        ids = self.tokenizer(text, add_special_tokens=False)["input_ids"]
        if eos is not None:
            ids = ids + [eos]
        ids_t = torch.tensor([ids], device=input_embeds.device, dtype=torch.long)
        x = F.embedding(ids_t, kw["input_embed_matrix"]).to(input_embeds.dtype)
        return ids_t, torch.cat([input_embeds, x], dim=1)

    # ------------------------------------------------------------------ the oracle
    def _ask_oracle(self):
        frames = self._oracle_dir / "oracle_frames"
        frames.mkdir(exist_ok=True)
        img = frames / f"step_{self.step:05d}.jpg"
        cv2.imwrite(str(img), self.camera_for_viz)   # the frame SimLingo is looking at (BGR)
        prompt = _PROMPT.format(image=img, speed=float(self.DrivingInput["vehicle_speed"][0, 0]),
                                route=_route_words(self.target_points), phrasebook=_PHRASEBOOK)
        cmd = ["claude", "-p", prompt, "--model", self._oracle_model, "--output-format", "json",
               "--setting-sources", "", "--settings", str(self._oracle_ws / "settings.json"),
               "--tools", "Read", "--permission-mode", "dontAsk"]
        try:
            out = subprocess.run(cmd, cwd=str(self._oracle_ws), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 universal_newlines=True, timeout=self._oracle_timeout, stdin=subprocess.DEVNULL)
            reply = json.loads(out.stdout)
            text = str(reply.get("result") or "").strip().splitlines()[-1].strip()
            cost = float(reply.get("total_cost_usd") or 0.0)
        except Exception as e:
            print(f"[oracle] call failed at step {self.step}: {e!r}", flush=True)
            return None, 0.0
        # Two sentences, phrasebook-shaped; anything else is not what the model was trained on.
        text = re.sub(r"\s+", " ", text)
        if not _phrasebook_shaped(text):
            print(f"[oracle] unusable answer at step {self.step}: {text!r}", flush=True)
            return None, cost
        return text, cost

    def destroy(self, results=None):
        try:
            self._oracle_log.write(json.dumps({"total_cost_usd": round(self._oracle_cost, 4)}) + "\n")
            self._oracle_log.close()
        except Exception:
            pass
        super().destroy(results)
