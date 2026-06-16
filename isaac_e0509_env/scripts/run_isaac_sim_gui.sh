#!/usr/bin/env bash
# Isaac Sim GUI launcher (use this instead of: ./isaaclab.sh -s)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GUI_PY="${HOME}/IsaacLab/source/isaac_e0509_pick_place/isaac_e0509_pick_place/scripts/open_isaac_gui.py"
MIN_WATCHES=524288

require_inotify() {
  local watches instances
  watches="$(cat /proc/sys/fs/inotify/max_user_watches)"
  instances="$(cat /proc/sys/fs/inotify/max_user_instances)"
  if [[ "${watches}" -lt "${MIN_WATCHES}" || "${instances}" -lt 512 ]]; then
    echo "[ERROR] inotify 한도 부족 (watches=${watches}, instances=${instances})"
    echo "        Isaac Sim GUI가 segfault 납니다. 먼저 한 번 실행:"
    echo "  sudo bash ${SCRIPT_DIR}/fix_inotify.sh"
    exit 1
  fi
}

if [[ ! -f "${GUI_PY}" ]]; then
  echo "[ERROR] extension 없음. 먼저:"
  echo "  bash ${SCRIPT_DIR}/install_to_isaaclab.sh"
  exit 1
fi

require_inotify

# shellcheck disable=SC1091
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate env_isaaclab

cd "${HOME}/IsaacLab"
export TERM=xterm
export PYTHONUNBUFFERED=1

echo "[INFO] Launching Isaac Lab GUI (performance rendering)..."
./isaaclab.sh -p "${GUI_PY}" --rendering_mode performance "$@"
