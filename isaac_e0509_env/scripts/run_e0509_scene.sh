#!/usr/bin/env bash
# Launch E0509 pick-place scene (ground + table + robot home + sample cube).
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
SCENE="${ISAACLAB}/source/isaac_e0509_pick_place/isaac_e0509_pick_place/scripts/setup_e0509_scene.py"

if [[ ! -f "${SCENE}" ]]; then
  echo "[ERROR] Extension not linked. Run:"
  echo "  bash ~/sudo_ws/src/isaac_e0509_env/scripts/install_to_isaaclab.sh"
  exit 1
fi

# shellcheck disable=SC1091
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate env_isaaclab

cd "${ISAACLAB}"
export TERM=xterm
export PYTHONUNBUFFERED=1

echo "[INFO] E0509 scene (GUI). Close window to exit."
./isaaclab.sh -p "${SCENE}" --num_envs 1 "$@"
