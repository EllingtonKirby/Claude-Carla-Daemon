#!/usr/bin/env bash
# Download CARLA 0.9.15 (+ the additional maps Town11-15, which Bench2Drive routes need) into $CARLA_ROOT.
#
#   CARLA_ROOT=$HOME/carla bash scripts/setup_carla.sh
#
# Idempotent: skips whatever is already installed. Needs internet and ~30 GB of disk.
# Adapted from https://github.com/autonomousvision/carla_garage/blob/leaderboard_2/setup_carla.sh
set -uo pipefail

carla_root="${CARLA_ROOT:-}"
if [ -z "$carla_root" ]; then
    echo "Error: set CARLA_ROOT to the directory CARLA should be installed in." >&2
    exit 1
fi

# CARLA's own download hosts have moved (tiny.carla.org's shortlinks redirect to a BunnyCDN pull
# zone that now answers "Domain suspended or not configured" for every object). Backblaze is where
# the releases live now; override this if they move again.
CARLA_DOWNLOAD_BASE="${CARLA_DOWNLOAD_BASE:-https://carla-releases.s3.us-east-005.backblazeb2.com/Linux}"

mkdir -p "$carla_root"
cd "$carla_root" || exit 1

# --- CARLA 0.9.15 simulator -----------------------------------------------------------------
if [ -d "${carla_root}/CarlaUE4" ]; then
    echo "CARLA already installed in ${carla_root}. Skipping download."
else
    echo "Downloading CARLA 0.9.15 to ${carla_root}"
    wget "$CARLA_DOWNLOAD_BASE/CARLA_0.9.15.tar.gz" || exit 1
    echo "Unpacking CARLA (this takes a while)"
    tar -xf CARLA_0.9.15.tar.gz && rm -f CARLA_0.9.15.tar.gz
    if [ ! -d "${carla_root}/CarlaUE4" ]; then
        echo "Error: CARLA download or unpacking failed." >&2
        exit 1
    fi
fi

# --- Additional maps (Town11-15) ------------------------------------------------------------
maps_sentinel="${carla_root}/CarlaUE4/Content/Carla/Maps/Town12/Town12_Tile_5_5.uexp"
if [ -f "$maps_sentinel" ]; then
    echo "Additional maps already present. Skipping."
else
    mkdir -p "${carla_root}/Import"
    if [ ! -f "${carla_root}/Import/AdditionalMaps_0.9.15.tar.gz" ]; then
        echo "Downloading additional maps"
        ( cd "${carla_root}/Import" && wget "$CARLA_DOWNLOAD_BASE/AdditionalMaps_0.9.15.tar.gz" ) || exit 1
    fi
    echo "Importing assets"
    if [ -x "${carla_root}/ImportAssets.sh" ]; then
        ( cd "$carla_root" && ./ImportAssets.sh )
    else
        # Pruned installs lack ImportAssets.sh; for the binary release the tarball is rooted at
        # CarlaUE4/, so extracting over $CARLA_ROOT is what ImportAssets.sh does. WARNING: if the
        # base install is an older cook than the tarball, this can overwrite Engine assets the old
        # binary cannot load (instant Signal 11). Re-extract a matching CARLA_0.9.15.tar.gz to heal.
        tar -xzf "${carla_root}/Import/AdditionalMaps_0.9.15.tar.gz" -C "$carla_root"
    fi
    if [ ! -f "$maps_sentinel" ]; then
        echo "Error: additional-maps import failed." >&2
        exit 1
    fi
fi

[ -f "${carla_root}/CarlaUE4.sh" ] && chmod +x "${carla_root}/CarlaUE4.sh"
echo "CARLA installation complete in ${carla_root}"
