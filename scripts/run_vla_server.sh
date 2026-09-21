#!/usr/bin/env bash
# Start the VLA service (carla_probe/vla_server.py) in its own venv, on its own GPU.
#
#   bash scripts/run_vla_server.sh [--device cuda:1] [--port 8791] [--dtype bf16]
#
# Foreground; Ctrl-C stops it. The daemon reaches it at vla.addr in carla_probe/config.yaml.
# Run scripts/setup_vla.sh first.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$REPO/third_party/vla_venv/bin/python"
[[ -x "$PY" ]] || { echo "no VLA venv: run scripts/setup_vla.sh"; exit 1; }
exec "$PY" "$REPO/carla_probe/vla_server.py" --warmup "$@"
