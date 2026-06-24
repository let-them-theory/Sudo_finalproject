#!/usr/bin/env bash
# Run E0509 Isaac Lab smoke test with common fixes for local setup.
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
SMOKE="${ISAACLAB}/source/isaac_e0509_pick_place/isaac_e0509_pick_place/scripts/smoke_env.py"
HEADLESS="${HEADLESS:-1}"
NUM_ENVS="${NUM_ENVS:-1}"

if [[ ! -f "${SMOKE}" ]]; then
  echo "[ERROR] Extension not linked. Run: ~/sudo_ws/src/isaac_e0509_env/scripts/install_to_isaaclab.sh"
  exit 1
fi

# Isaac Sim watches many extension dirs; low inotify limits cause errno=28 noise/crashes.
CURRENT_WATCHES="$(cat /proc/sys/fs/inotify/max_user_watches)"
if [[ "${CURRENT_WATCHES}" -lt 524288 ]]; then
  echo "[WARN] fs.inotify.max_user_watches=${CURRENT_WATCHES} (low for Isaac Sim)"
  echo "       If GUI segfaults, run once:"
  echo "       sudo sysctl -w fs.inotify.max_user_watches=524288"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/isaaclab_env.sh"

cd "${ISAACLAB}"
export TERM=xterm
export PYTHONUNBUFFERED=1

ARGS=(--num_envs "${NUM_ENVS}")
if [[ "${HEADLESS}" == "1" ]]; then
  ARGS+=(--headless)
  echo "[INFO] headless smoke (set HEADLESS=0 for GUI)"
else
  echo "[INFO] GUI smoke"
  WATCHES="$(cat /proc/sys/fs/inotify/max_user_watches)"
  if [[ "${WATCHES}" -lt 524288 ]]; then
    echo "[ERROR] inotify 한도 부족 (${WATCHES}). GUI 실행 전:"
    echo "  sudo bash ${SCRIPT_DIR}/fix_inotify.sh"
    exit 1
  fi
fi

./isaaclab.sh -p "${SMOKE}" "${ARGS[@]}"
