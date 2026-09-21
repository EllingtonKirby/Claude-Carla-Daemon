"""BLUE's stock agent with the model in a different floating-point format: a diagnostic, not a
variant. The agent hard-codes bfloat16, which on Turing GPUs (no bf16 tensor cores, no
flash-attn, no TF32) runs through emulation and eager attention -- numerics the authors' Ampere
setup never saw. Setting BLUE_DTYPE=fp32|fp16 reroutes the agent's `torch.set_default_dtype(bf16)`
to that format and casts the camera tensor to match; nothing else changes.

    BLUE_DTYPE=fp32 ... leaderboard_evaluator.py --agent carla_probe/blue_dtype_agent.py ...
    (scripts/run_blue_eval.sh DTYPE=fp32)
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_BLUE = _REPO / "third_party" / "BLUE"
for p in (str(_BLUE), str(_BLUE / "team_code")):
    if p not in sys.path:
        sys.path.insert(0, p)

import torch  # noqa: E402

from team_code import agent_simlingo  # noqa: E402

_DTYPES = {"fp32": torch.float32, "fp16": torch.float16, "bf16": torch.bfloat16}


def get_entry_point() -> str:
    return "DtypeLingoAgent"


class DtypeLingoAgent(agent_simlingo.LingoAgent):
    def setup(self, path_to_conf_file, route_index=None):
        dtype = _DTYPES[os.environ["BLUE_DTYPE"]]
        orig = torch.set_default_dtype
        torch.set_default_dtype = lambda d: orig(dtype if d == torch.bfloat16 else d)
        try:
            super().setup(path_to_conf_file, route_index)
        finally:
            torch.set_default_dtype = orig
        self.model = self.model.to(dtype)
        forward = self.model.forward

        def cast_forward(inp, *a, **k):   # the agent casts camera_images to bf16 explicitly
            return forward(inp._replace(camera_images=inp.camera_images.to(dtype)), *a, **k)

        self.model.forward = cast_forward
        print(f"[dtype] model in {os.environ['BLUE_DTYPE']}", flush=True)
