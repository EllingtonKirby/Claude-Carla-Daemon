#!/usr/bin/env bash
# Set up the VLA service (carla_probe/vla_server.py): its own Python 3.8 + torch venv, the
# SimLingo checkpoint and the InternVL2-1B backbone. Everything lands under third_party/, which is
# gitignored, and the script is idempotent -- re-run it after a partial download.
#
#   bash scripts/setup_vla.sh
#
# Needs: uv, an NVIDIA driver for CUDA 12.1 wheels, network access to huggingface.co and PyPI,
# and third_party/BLUE (git clone git@github.com:George-Ling3/BLUE.git third_party/BLUE).
#
# flash-attn is deliberately NOT installed: it does not build for Turing GPUs (sm_75) and InternVL's
# remote code falls back to eager attention without it.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP="$REPO/third_party"
VENV="$TP/vla_venv"
MODELS="$TP/models/simlingo"
export HF_HOME="$TP/hf_cache"

[[ -d "$TP/BLUE" ]] || { echo "third_party/BLUE is missing: git clone git@github.com:George-Ling3/BLUE.git $TP/BLUE"; exit 1; }
command -v uv >/dev/null || { echo "uv is required (https://docs.astral.sh/uv/)"; exit 1; }

echo "==> venv ($VENV)"
uv python install 3.8 >/dev/null
[[ -x "$VENV/bin/python" ]] || uv venv "$VENV" --python 3.8
uv pip install --python "$VENV/bin/python" --index-url https://download.pytorch.org/whl/cu121 \
    torch==2.2.0 torchvision==0.17.0
# Pins follow third_party/BLUE/requirements.txt where they matter for the checkpoint.
uv pip install --python "$VENV/bin/python" \
    "numpy==1.23.0" "transformers==4.46.3" "peft==0.13.2" "hydra-core==1.3.2" "omegaconf==2.3.0" \
    "pytorch-lightning==2.4.0" "timm==0.9.16" "einops==0.7.0" "scipy==1.10.1" filterpy ujson pillow \
    shapely "huggingface_hub==0.27.0" "accelerate==1.0.1" sentencepiece protobuf opencv-python-headless

echo "==> SimLingo checkpoint ($MODELS)"
mkdir -p "$MODELS/checkpoints/epoch=013.ckpt" "$MODELS/.hydra"
HF=https://huggingface.co/RenzKa/simlingo/resolve/main/simlingo
[[ -s "$MODELS/.hydra/config.yaml" ]] || curl -sL -o "$MODELS/.hydra/config.yaml" "$HF/.hydra/config.yaml"
CKPT="$MODELS/checkpoints/epoch=013.ckpt/pytorch_model.pt"
# 2 569 679 322 bytes; anything smaller is a broken download.
if [[ ! -s "$CKPT" ]] || [[ "$(stat -c %s "$CKPT")" -lt 2569679322 ]]; then
    curl -L -o "$CKPT" "$HF/checkpoints/epoch%3D013.ckpt/pytorch_model.pt"
fi

echo "==> InternVL2-1B backbone ($HF_HOME)"
"$VENV/bin/python" -c "from huggingface_hub import snapshot_download; print(snapshot_download(repo_id='OpenGVLab/InternVL2-1B'))"

echo "==> smoke: load the model on GPU 0 and run one step"
cd "$REPO" && timeout 600 "$VENV/bin/python" carla_probe/vla_server.py --device cuda:0 --warmup --port 0 2>&1 | grep -E "checkpoint loaded|warmup" || true
echo "done. Start the service with: bash scripts/run_vla_server.sh"
