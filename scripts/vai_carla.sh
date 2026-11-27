#!/usr/bin/env bash
# Run a command in this repo inside a vai cluster job built from Dockerfile_b2d (RTX 2080 Ti
# machines: cougar, impala, jaguar, lada, mustang, panda). See docs/HANDOFF.md §2.1.
#
#   sudo -u carla -H bash <repo>/scripts/vai_carla.sh <command...>
#   sudo -u carla -H bash <repo>/scripts/vai_carla.sh bash scripts/run_eval.sh 25378 sensors claude-opus-5
#
# Why `sudo -u carla`: the image's own user (eramzi) is uid 1020, but the NFS workspace belongs to
# uid 1000, which the image calls `carla`. As 1020 the checkout is unreadable (a worktree under
# .claude/ is mode 700) or read-only, so the run has to be uid 1000. carla is not a sudoer, so do
# the one-time prep as the image's user first (it touches only the container's own disk):
#
#   sudo chmod 777 /home/carla && sudo chmod -R a+rwX /home/carla/CarlaUE4/Saved
#
# What this adds on top of `source scripts/env.sh`: the image's CARLA and uv paths, headless SDL,
# and a Claude Code login for the driver. carla cannot read the image's seeded credential
# (/home/eramzi/.claude-config is eramzi's, mode 600), so it uses its own config dir and the
# long-lived `claude setup-token` token in ~eramzi/no_backup, which is uid 1000's.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

export CARLA_ROOT="${CARLA_ROOT:-/home/carla}"
export UV_PYTHON_INSTALL_DIR="${UV_PYTHON_INSTALL_DIR:-/opt/uv-python}"
export SDL_VIDEODRIVER=dummy
export PATH="/opt/npm-global/bin:/opt/node/bin:/usr/local/bin:$PATH"

export CLAUDE_CONFIG_DIR="${VAI_CLAUDE_CONFIG_DIR:-$HOME/.claude-config}"
mkdir -p "$CLAUDE_CONFIG_DIR"
_tok="${VAI_CLAUDE_TOKEN_FILE:-/home/eramzi/no_backup/.claude_code_oauth_token}"
if [ -z "${CLAUDE_CODE_OAUTH_TOKEN:-}" ] && [ -r "$_tok" ]; then
    CLAUDE_CODE_OAUTH_TOKEN="$(tr -d ' \t\n\r' < "$_tok")"
    export CLAUDE_CODE_OAUTH_TOKEN
fi
unset _tok

# shellcheck disable=SC1091
source scripts/env.sh >/dev/null
exec "$@"
