#!/usr/bin/env bash
# Install what the vendored Fail2Drive tree cannot carry in git: the F2D CARLA build and the
# per-town speed-limit maps.
#
#   F2D_CARLA_ROOT=$HOME/f2d_carla bash scripts/setup_fail2drive.sh
#
# Idempotent and resumable: the 12.8 GB tarball is downloaded to a file (so a retry costs only the
# remainder) and an interrupted extract is cleared rather than mistaken for an install.
# Needs internet and ~45 GB of free disk (12.8 GB tarball + ~30 GB extracted).
#
# The leaderboard and scenario_runner themselves are vendored at third_party/fail2drive/ (MIT; see
# third_party/fail2drive/VENDORED.md) -- unlike Bench2Drive there is nothing to fetch or patch.
#
# Env: F2D_CARLA_ROOT   where the F2D simulator is installed (default $HOME/f2d_carla)
#      F2D_STAGE_DIR    where the tarball is downloaded (default $F2D_CARLA_ROOT/../f2d_download)
#      F2D_CARLA_URL    override the HuggingFace URL
#      F2D_ASSET_PACKS  override the novel-asset pack names checked for
#      F2D_SKIP_ASSET_CHECK=1  accept an install whose packs this script cannot name
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
F2D_CARLA_ROOT="${F2D_CARLA_ROOT:-$HOME/f2d_carla}"
STAGE_DIR="${F2D_STAGE_DIR:-$(dirname "$F2D_CARLA_ROOT")/f2d_download}"
TARBALL="$STAGE_DIR/fail2drive_simulator.tar.gz"
F2D_CARLA_URL="${F2D_CARLA_URL:-https://huggingface.co/datasets/SimonGer/fail2drive/resolve/main/fail2drive_simulator.tar.gz}"
SPEED_LIMITS_DST="$REPO_ROOT/third_party/fail2drive/scenario_runner/speed_limits"
F2D_GH_API="https://api.github.com/repos/autonomousvision/fail2drive/contents/scenario_runner/speed_limits"
MIN_FREE_GB="${MIN_FREE_GB:-45}"

# Fail2Drive's 30 novel assets (animals, visual-noise props, adversarial obstacles) are baked into a
# custom UE4 Content pak. Run F2D routes against a stock CARLA 0.9.15 and NOTHING errors -- CARLA
# substitutes default blueprints, so a scenario that should spawn a cow spawns a car and the
# benchmark measures nothing it claims to. So "installed" means every pack present WITH FILES IN IT:
# an interrupted `curl | tar` leaves the complete directory tree with zero payload files.
#
# These six names are not published upstream; they were read off the tarball's own contents. If
# upstream renames a pack this check becomes a false negative after a 12.8 GB download, hence the
# two overrides.
F2D_ASSET_PACKS=(${F2D_ASSET_PACKS:-AnimalVarietyPack AfricanAnimalsPack FarmAnimalsPack
                 WallAssets StopOcclusions ImageAssets})

echo "=== Fail2Drive setup ==="
echo "    simulator:    $F2D_CARLA_ROOT"
echo "    staging:      $STAGE_DIR"
echo "    speed limits: $SPEED_LIMITS_DST"

# `ls glob | wc -l` is not usable here: under `set -o pipefail` a glob that matches nothing makes
# ls exit non-zero, which fails the whole pipeline and, under `set -e`, the script -- which is
# exactly the state this counts (none installed yet).
count_speed_limits() {
    local n=0 f
    for f in "$SPEED_LIMITS_DST"/*_speed_limits.npy; do [ -f "$f" ] && n=$((n + 1)); done
    printf '%s' "$n"
}

report_packs() {
    local pack n
    for pack in "${F2D_ASSET_PACKS[@]}"; do
        n=$(find "$F2D_CARLA_ROOT/CarlaUE4/Content/$pack" -type f 2>/dev/null | wc -l)
        printf '      %-22s %s files\n' "$pack" "$n"
    done
}

carla_installed() {
    [ -x "$F2D_CARLA_ROOT/CarlaUE4.sh" ] || return 1
    [ -f "$F2D_CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" ] || return 1
    [ "${F2D_SKIP_ASSET_CHECK:-0}" = "1" ] && return 0
    local pack
    for pack in "${F2D_ASSET_PACKS[@]}"; do
        [ -n "$(find "$F2D_CARLA_ROOT/CarlaUE4/Content/$pack" -type f -print -quit 2>/dev/null)" ] || return 1
    done
    return 0
}

# --- 1/2  the F2D CARLA build ------------------------------------------------------------------
if carla_installed; then
    echo "==> [1/2] F2D CARLA already installed. Skipping."
    report_packs
else
    if [ -d "$F2D_CARLA_ROOT" ] && [ -n "$(ls -A "$F2D_CARLA_ROOT" 2>/dev/null)" ]; then
        echo "==> [1/2] $F2D_CARLA_ROOT exists but is not a complete install (interrupted extract?)"
        report_packs
        echo "    clearing it and starting over"
        rm -rf "${F2D_CARLA_ROOT:?}"/*
    fi
    mkdir -p "$F2D_CARLA_ROOT" "$STAGE_DIR"
    free_gb=$(df -BG --output=avail "$STAGE_DIR" | tail -1 | tr -dc '0-9')
    if [ "${free_gb:-0}" -lt "$MIN_FREE_GB" ]; then
        echo "ERROR: only ${free_gb} GB free at $STAGE_DIR; need >= ${MIN_FREE_GB} GB" >&2
        echo "       (12.8 GB tarball + ~30 GB extracted). Point F2D_STAGE_DIR somewhere roomier." >&2
        exit 1
    fi
    echo "==> [1/2] downloading the F2D CARLA build (12.8 GB; resumable, run it under tmux)"
    # --continue resumes a partial file; a dropped download mid-transfer is the common failure.
    wget --continue --progress=dot:giga -O "$TARBALL" "$F2D_CARLA_URL" || {
        echo "ERROR: download failed. Re-run to resume from $TARBALL" >&2; exit 1; }
    echo "==> extracting into $F2D_CARLA_ROOT (~30 GB, slow)"
    tar -xzf "$TARBALL" -C "$F2D_CARLA_ROOT"
    [ -f "$F2D_CARLA_ROOT/CarlaUE4.sh" ] && chmod +x "$F2D_CARLA_ROOT/CarlaUE4.sh"
    if ! carla_installed; then
        echo "ERROR: extract finished but the install is incomplete:" >&2
        report_packs >&2
        echo "       If upstream renamed a pack, set F2D_ASSET_PACKS or F2D_SKIP_ASSET_CHECK=1." >&2
        exit 1
    fi
    report_packs
    echo "    delete the tarball once you are happy:  rm $TARBALL"
fi

# --- 2/2  the per-town speed-limit maps --------------------------------------------------------
# srunner reads $SCENARIO_RUNNER_ROOT/speed_limits/<Town>_speed_limits.npy at RUNTIME, from
# background_activity.py (which runs on every route), timer.py and atomic_behaviors.py. Missing =>
# every route dies with FileNotFoundError. 75 MB, so they are gitignored rather than vendored.
mkdir -p "$SPEED_LIMITS_DST"
have=$(count_speed_limits)
if [ "$have" -ge 12 ]; then
    echo "==> [2/2] speed-limit maps already present ($have). Skipping."
else
    echo "==> [2/2] fetching the speed-limit maps from upstream ($have/12 present)"
    urls="$(wget -qO- "$F2D_GH_API" | grep -o '"download_url": *"[^"]*\.npy"' | cut -d '"' -f 4)"
    [ -n "$urls" ] || { echo "ERROR: could not list $F2D_GH_API" >&2; exit 1; }
    echo "$urls" | while read -r u; do
        [ -n "$u" ] || continue
        wget -q --show-progress -N -P "$SPEED_LIMITS_DST" "$u" || exit 1
    done
    have=$(count_speed_limits)
    [ "$have" -ge 12 ] || {
        echo "ERROR: only $have/12 speed-limit maps present in $SPEED_LIMITS_DST" >&2
        echo "       They can also be regenerated against a running CARLA:" >&2
        echo "       python $SPEED_LIMITS_DST/compute_speed_limits.py" >&2
        exit 1; }
fi

echo "=== COMPLETE: $(count_speed_limits) speed-limit maps, CARLA at $F2D_CARLA_ROOT"
echo "    next:  BENCHMARK=fail2drive F2D_CARLA_ROOT=$F2D_CARLA_ROOT source scripts/env.sh"
echo "    Fail2Drive is MIT. Its routes, scenarios and assets are a HELD-OUT test set:"
echo "    never train or tune on them (benchmark/PROTOCOL.md)."
