#!/usr/bin/env bash
# Set up and activate the simulation environment.
#
#   source scripts/env.sh
#
# SOURCE it (don't execute) so the venv activation and exports land in your shell. Idempotent: in a
# ready shell it is a fast no-op. Creates a Python 3.10 venv at .venv (CARLA 0.9.15 ships a cp310
# wheel), installs the pinned dependencies on first use, and puts one benchmark's leaderboard +
# scenario_runner on PYTHONPATH (they are not pip-installable).
#
# BENCHMARK selects which:
#   bench2drive (default)  the patched Bench2Drive 0.0.4 at third_party/Bench2Drive, against the
#                          stock CARLA 0.9.15 at CARLA_ROOT  (scripts/setup_bench2drive.sh)
#   fail2drive             the vendored Fail2Drive at third_party/fail2drive, against the F2D CARLA build
#                          with the 30 novel assets at F2D_CARLA_ROOT  (scripts/setup_fail2drive.sh)
#
# Both forks ship packages literally named `leaderboard` and `srunner`, and importing the wrong one
# does not raise -- it silently scores the route under the other benchmark's rules. So switching
# benchmarks REMOVES the other fork from PYTHONPATH rather than merely putting this one first, and
# re-sourcing with a different BENCHMARK in the same shell is safe.
#
# Env overrides: BENCHMARK (bench2drive), CARLA_ROOT ($HOME/carla), F2D_CARLA_ROOT ($HOME/f2d_carla),
#                VENV_DIR (<repo>/.venv).

if [ -n "${ZSH_VERSION:-}" ]; then
    _probe_env_src="${(%):-%x}"   # zsh has no BASH_SOURCE (Claude Code's shell is often zsh)
elif [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "ERROR: source this script, do not execute it:  source ${0}" >&2
    exit 1
else
    _probe_env_src="${BASH_SOURCE[0]}"
fi

REPO_ROOT="$(cd "$(dirname "$_probe_env_src")/.." && pwd)"
unset _probe_env_src
export REPO_ROOT
: "${VENV_DIR:=$REPO_ROOT/.venv}"
export BENCHMARK="${BENCHMARK:-bench2drive}"
export B2D_ROOT="$REPO_ROOT/third_party/Bench2Drive"
export F2D_ROOT="$REPO_ROOT/third_party/fail2drive"

# CARLA_ROOT means a different install per benchmark, so switching benchmarks in a live shell must
# not inherit the other one's. Without this, re-sourcing with BENCHMARK=bench2drive after a
# fail2drive shell would point the stock leaderboard at the F2D build.
if [ -n "${PROBE_BENCHMARK_ACTIVE:-}" ] && [ "$PROBE_BENCHMARK_ACTIVE" != "$BENCHMARK" ]; then
    unset CARLA_ROOT
fi
export PROBE_BENCHMARK_ACTIVE="$BENCHMARK"

case "$BENCHMARK" in
    bench2drive)
        export CARLA_ROOT="${CARLA_ROOT:-$HOME/carla}"
        export LEADERBOARD_ROOT="$B2D_ROOT/leaderboard"
        export SCENARIO_RUNNER_ROOT="$B2D_ROOT/scenario_runner"
        if [ ! -f "$B2D_ROOT/.probe_patched" ]; then
            echo "ERROR: patched Bench2Drive not found. Run:  bash scripts/setup_bench2drive.sh" >&2
            return 1
        fi
        if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
            echo "WARNING: no CARLA at CARLA_ROOT=$CARLA_ROOT (install: CARLA_ROOT=... bash scripts/setup_carla.sh)" >&2
        fi
        ;;
    fail2drive)
        # F2D needs ITS OWN CARLA: the 30 novel assets are baked into a custom Content pak, and a
        # stock 0.9.15 silently substitutes default blueprints instead of erroring.
        export F2D_CARLA_ROOT="${F2D_CARLA_ROOT:-$HOME/f2d_carla}"
        export CARLA_ROOT="$F2D_CARLA_ROOT"
        export LEADERBOARD_ROOT="$F2D_ROOT/leaderboard"
        export SCENARIO_RUNNER_ROOT="$F2D_ROOT/scenario_runner"
        if [ ! -x "$CARLA_ROOT/CarlaUE4.sh" ]; then
            echo "ERROR: no Fail2Drive CARLA at F2D_CARLA_ROOT=$CARLA_ROOT" >&2
            echo "       Run:  F2D_CARLA_ROOT=$CARLA_ROOT bash scripts/setup_fail2drive.sh" >&2
            return 1
        fi
        # srunner loads these at runtime on every route; missing => FileNotFoundError per route.
        # find, not a glob: this file is sourced from bash AND zsh, and an unmatched glob behaves
        # differently in each (zsh errors outright) -- in exactly the case being checked for.
        _probe_npy=$(find "$SCENARIO_RUNNER_ROOT/speed_limits" -maxdepth 1 \
                     -name '*_speed_limits.npy' 2>/dev/null | wc -l)
        if [ "${_probe_npy:-0}" -lt 12 ]; then
            echo "ERROR: Fail2Drive speed-limit maps missing from $SCENARIO_RUNNER_ROOT/speed_limits" >&2
            echo "       Run:  bash scripts/setup_fail2drive.sh" >&2
            unset _probe_npy
            return 1
        fi
        unset _probe_npy
        ;;
    *)
        echo "ERROR: BENCHMARK must be bench2drive or fail2drive, got '$BENCHMARK'" >&2
        return 1
        ;;
esac

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

# Drop BOTH benchmark trees from an inherited PYTHONPATH before prepending this one's: a stale
# entry for the other fork would resolve `leaderboard` / `srunner` to the wrong benchmark without
# raising, and score the route under the other one's rules.
#
# Pure parameter expansion, no pipeline: this file is SOURCED, so a `grep` that matches nothing
# would abort a caller running under `set -e -o pipefail` (scripts/run_session.sh does) -- and an
# empty inherited PYTHONPATH, the common case, is exactly that.
_probe_keep=""
_probe_rest="${PYTHONPATH:-}"
while [ -n "$_probe_rest" ]; do
    _probe_head="${_probe_rest%%:*}"
    case "$_probe_rest" in
        *:*) _probe_rest="${_probe_rest#*:}" ;;
        *)   _probe_rest="" ;;
    esac
    case "$_probe_head" in
        "" | "$REPO_ROOT" \
           | "$B2D_ROOT/leaderboard" | "$B2D_ROOT/scenario_runner" \
           | "$F2D_ROOT/leaderboard" | "$F2D_ROOT/scenario_runner") continue ;;
    esac
    _probe_keep="${_probe_keep:+$_probe_keep:}$_probe_head"
done
export PYTHONPATH="$REPO_ROOT:$LEADERBOARD_ROOT:$SCENARIO_RUNNER_ROOT:$CARLA_ROOT/PythonAPI/carla${_probe_keep:+:$_probe_keep}"
unset _probe_keep _probe_rest _probe_head

if python - <<'PY' 2>/dev/null
import importlib.util as u, sys
need = ["carla", "leaderboard", "srunner", "cv2", "numpy", "yaml", "pkg_resources", "py_trees", "shapely", "pytest"]
sys.exit(0 if all(u.find_spec(m) for m in need) else 1)
PY
then
    echo "==> env ready (BENCHMARK=$BENCHMARK, CARLA_ROOT=$CARLA_ROOT)"
    return 0
fi

echo "==> installing python dependencies"
if command -v uv >/dev/null 2>&1; then PIP=(uv pip install); else PIP=(python -m pip install); fi
# setuptools<81 still ships pkg_resources, which the leaderboard imports. numpy<2 is required by
# the carla-era opencv wheel; this exact set is the one the probe daemon was validated with.
"${PIP[@]}" 'setuptools<81' 'numpy==1.26.4' 'opencv-python-headless==4.5.4.58' 'shapely==2.0.6' pyyaml pytest || return 1
# The benchmark driver (carla_probe/drive_loop.py) talks to the Messages API. Unpinned on purpose: it
# is not part of the simulation stack, and the loop uses beta features that move forward.
"${PIP[@]}" anthropic || return 1
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
echo "==> env ready (BENCHMARK=$BENCHMARK, CARLA_ROOT=$CARLA_ROOT)"
