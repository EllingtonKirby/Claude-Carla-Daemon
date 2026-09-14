#!/usr/bin/env bash
# Fetch Bench2Drive at a pinned commit, apply this repo's leaderboard patch, and split the route set.
#
#   bash scripts/setup_bench2drive.sh
#
# Result: third_party/Bench2Drive (patched) and routes/bench2drive220/route_<id>.xml.
# Idempotent. Bench2Drive is fetched rather than vendored because its top-level license
# (CC BY-NC-ND 4.0) does not allow redistributing modified copies. The patch only touches the
# MIT-licensed leaderboard/ and scenario_runner/ trees; see docs/DESIGN.md for what it changes.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
B2D_URL="https://github.com/Thinklab-SJTU/Bench2Drive.git"
B2D_COMMIT="7ec25d1c9f7522d923ce5f3420986cef1cb2d956"  # "Release Bench2Drive 0.0.4"
DEST="$REPO_ROOT/third_party/Bench2Drive"
PATCH="$REPO_ROOT/patches/bench2drive-0.0.4-probe.patch"

if [ -f "$DEST/.probe_patched" ]; then
    echo "Bench2Drive already set up at $DEST ($(cat "$DEST/.probe_patched"))"
else
    if [ ! -d "$DEST/.git" ]; then
        mkdir -p "$DEST"
        git -C "$DEST" init -q
        git -C "$DEST" remote add origin "$B2D_URL"
    fi
    echo "==> fetching Bench2Drive @ ${B2D_COMMIT:0:7}"
    git -C "$DEST" fetch -q --depth 1 origin "$B2D_COMMIT"
    git -C "$DEST" checkout -q --force FETCH_HEAD
    echo "==> applying $(basename "$PATCH")"
    patch -p1 --forward -d "$DEST" < "$PATCH"
    echo "$B2D_COMMIT + $(basename "$PATCH")" > "$DEST/.probe_patched"
fi

python3 "$REPO_ROOT/tools/split_routes.py" \
    "$DEST/leaderboard/data/bench2drive220.xml" "$REPO_ROOT/routes/bench2drive220"
echo "done. Bench2Drive is CC BY-NC-ND 4.0 (non-commercial); its leaderboard/ and scenario_runner/ are MIT."
