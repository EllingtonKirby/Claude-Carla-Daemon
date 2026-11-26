#!/usr/bin/env bash
# Start the perception service (carla_probe/perception_server.py) that CODE_TOOL=on runs may call
# via PERCEPTION_ADDR. Blocks; run it in the background. It needs its own interpreter with torch and
# a recent transformers (PERCEPTION_PY), separate from the daemon's venv.
#
#   PERCEPTION_PY=~/workspace/venv_perception/bin/python bash scripts/run_perception_server.sh
#
# Env: PERCEPTION_PY (required)  PERCEPTION_PORT (8795)  PERCEPTION_DEVICE (cuda:0)
#      SEGMENTER (auto | sam3 | panoptic): auto tries SAM 3 (gated: needs HF_TOKEN) and falls back
#      HF_HOME (third_party/hf_cache)
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${PERCEPTION_PY:?set PERCEPTION_PY to an interpreter with torch + transformers>=4.57}"
export HF_HOME="${HF_HOME:-$REPO_ROOT/third_party/hf_cache}"
exec "$PERCEPTION_PY" "$REPO_ROOT/carla_probe/perception_server.py" --port "${PERCEPTION_PORT:-8795}" \
    --device "${PERCEPTION_DEVICE:-cuda:0}" --segmenter "${SEGMENTER:-auto}"
