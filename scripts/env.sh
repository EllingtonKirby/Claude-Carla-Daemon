#!/usr/bin/env bash
# Set up and activate the simulation environment.
#
#   source scripts/env.sh
#
# SOURCE it (don't execute) so the venv activation and exports land in your shell. Idempotent: in a
# ready shell it is a fast no-op. Creates a Python 3.10 venv at .venv (CARLA 0.9.15 ships a cp310
# wheel), installs the pinned dependencies on first use, and puts the patched Bench2Drive
# leaderboard + scenario_runner on PYTHONPATH (they are not pip-installable).
#
# Env overrides: CARLA_ROOT (default $HOME/carla), VENV_DIR (default <repo>/.venv).

if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "ERROR: source this script, do not execute it:  source ${0}" >&2
    exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO_ROOT
export CARLA_ROOT="${CARLA_ROOT:-$HOME/carla}"
: "${VENV_DIR:=$REPO_ROOT/.venv}"
export B2D_ROOT="$REPO_ROOT/third_party/Bench2Drive"
export LEADERBOARD_ROOT="$B2D_ROOT/leaderboard"
export SCENARIO_RUNNER_ROOT="$B2D_ROOT/scenario_runner"

if [ ! -f "$B2D_ROOT/.probe_patched" ]; then
    echo "ERROR: patched Bench2Drive not found. Run:  bash scripts/setup_bench2drive.sh" >&2
    return 1
fi
if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
    echo "WARNING: no CARLA at CARLA_ROOT=$CARLA_ROOT (install: CARLA_ROOT=... bash scripts/setup_carla.sh)" >&2
fi

if [ ! -x "$VENV_DIR/bin/python" ]; then
    echo "==> creating Python 3.10 venv at $VENV_DIR"
    if command -v uv >/dev/null 2>&1; then
        uv venv --python 3.10 "$VENV_DIR" || return 1
    elif command -v python3.10 >/dev/null 2>&1; then
        python3.10 -m venv "$VENV_DIR" || return 1
    else
        echo "ERROR: need uv (https://docs.astral.sh/uv/) or python3.10 to create the venv" >&2
        return 1
    fi
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

export PYTHONPATH="$REPO_ROOT:$LEADERBOARD_ROOT:$SCENARIO_RUNNER_ROOT:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}"

if python - <<'PY' 2>/dev/null
import importlib.util as u, sys
need = ["carla", "leaderboard", "srunner", "cv2", "numpy", "yaml", "pkg_resources", "py_trees", "shapely", "pytest"]
sys.exit(0 if all(u.find_spec(m) for m in need) else 1)
PY
then
    echo "==> env ready (CARLA_ROOT=$CARLA_ROOT)"
    return 0
fi

echo "==> installing python dependencies"
if command -v uv >/dev/null 2>&1; then PIP=(uv pip install); else PIP=(python -m pip install); fi
# setuptools<81 still ships pkg_resources, which the leaderboard imports. numpy<2 is required by
# the carla-era opencv wheel; this exact set is the one the probe daemon was validated with.
"${PIP[@]}" 'setuptools<81' 'numpy==1.26.4' 'opencv-python-headless==4.5.4.58' 'shapely==2.0.6' pyyaml pytest || return 1
# Bench2Drive structural deps (scenario_runner needs these exact APIs).
"${PIP[@]}" py-trees==0.8.3 xmlschema==1.0.18 dictor requests pygame tabulate pexpect transforms3d \
    networkx ephem simple-watchdog-timer six psutil || return 1
# CARLA python client matching the 0.9.15 server.
"${PIP[@]}" carla==0.9.15 || return 1

python - <<'PY' || { echo "ERROR: import verification failed" >&2; return 1; }
import importlib
for m in ["carla", "leaderboard", "srunner", "cv2", "numpy", "yaml", "pkg_resources", "carla_probe.agent"]:
    importlib.import_module(m)
    print(f"  ok: {m}")
PY
echo "==> env ready (CARLA_ROOT=$CARLA_ROOT)"
