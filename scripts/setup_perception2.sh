#!/usr/bin/env bash
# Build the interpreter for the strong perception backend (Depth Pro 2 + SAM 3): Python 3.12,
# separate from the daemon's venv and from the legacy venv_perception (SAM 3 needs Python >= 3.12).
#
#   bash scripts/setup_perception2.sh [VENV_DIR]        (default ~/workspace/venv_perception2)
#
# Needs uv, a CUDA 12.8-capable driver, and for SAM 3 a Hugging Face token with access to
# facebook/sam3 (gated) in ~/.cache/huggingface/token. Depth Pro 2 weights download from Apple's CDN
# on first use into the torch hub cache; they are licensed for research use only (LICENSE_MODEL).
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
V="${1:-$HOME/workspace/venv_perception2}"
TP="$REPO_ROOT/third_party"
[ -d "$TP/sam3" ] || git clone --depth 1 https://github.com/facebookresearch/sam3.git "$TP/sam3"
[ -d "$TP/ml-depth-pro-2" ] || git clone --depth 1 https://github.com/apple-aiml-research/ml-depth-pro-2.git "$TP/ml-depth-pro-2"
uv python install 3.12   # ~/.local is not kept across containers; the venv links to this interpreter
[ -x "$V/bin/python" ] || uv venv --python 3.12 "$V"
uv pip install --python "$V/bin/python" torch==2.10.0 torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install --python "$V/bin/python" -e "$TP/sam3" -e "$TP/ml-depth-pro-2" opencv-python-headless numpy einops pycocotools psutil
"$V/bin/python" -c "import torch, sam3, depth_pro_2; print('ok: torch', torch.__version__, 'cuda', torch.cuda.is_available())"
