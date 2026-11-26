#!/usr/bin/env bash
# Start the perception service (carla_probe/perception_server.py) that CODE_TOOL=on runs may call
# via PERCEPTION_ADDR. Blocks; run it in the background. It needs its own interpreter with torch and
# a recent transformers (PERCEPTION_PY), separate from the daemon's venv.
#
#   PERCEPTION_PY=~/workspace/venv_perception/bin/python bash scripts/run_perception_server.sh
#   BACKEND=strong PERCEPTION_PY=~/workspace/venv_perception2/bin/python bash scripts/run_perception_server.sh
#
# Env: PERCEPTION_PY (required)  PERCEPTION_PORT (8795)  PERCEPTION_DEVICE (cuda:0)
#      BACKEND (legacy | strong): legacy = Depth Anything V2 + Mask2Former / transformers SAM 3;
#        strong = Depth Pro 2 + SAM 3 from their own packages (scripts/setup_perception2.sh)
#      SEGMENTER (auto | sam3 | panoptic, legacy only): auto tries SAM 3 (gated: needs HF_TOKEN) and falls back
#      DEPTH_PRESET (depthpro2-h, strong only): depthpro2-{s,b,l,h}
#      TFV6_ADDR (unset, strong only): host:port of carla_probe/tfv6_server.py, forwarded as POST /scene
#      HF_HOME (third_party/hf_cache; strong: the default hub cache, where its token lives)
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${PERCEPTION_PY:?set PERCEPTION_PY to an interpreter with torch + transformers>=4.57}"
if [ "${BACKEND:-legacy}" = strong ]; then
    exec "$PERCEPTION_PY" "$REPO_ROOT/carla_probe/perception_server.py" --port "${PERCEPTION_PORT:-8795}" \
        --device "${PERCEPTION_DEVICE:-cuda:0}" --backend strong --depth-preset "${DEPTH_PRESET:-depthpro2-h}" \
        ${TFV6_ADDR:+--tfv6-addr "$TFV6_ADDR"}
fi
export HF_HOME="${HF_HOME:-$REPO_ROOT/third_party/hf_cache}"
exec "$PERCEPTION_PY" "$REPO_ROOT/carla_probe/perception_server.py" --port "${PERCEPTION_PORT:-8795}" \
    --device "${PERCEPTION_DEVICE:-cuda:0}" --segmenter "${SEGMENTER:-auto}"
