#!/usr/bin/env bash
# Symlink this extension into ~/IsaacLab/source and install with isaaclab.sh -i
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
EXT_SRC="${ENV_ROOT}/isaaclab_ext"
EXT_LINK="${HOME}/IsaacLab/source/isaac_e0509_pick_place"
ISAACLAB_SH="${HOME}/IsaacLab/isaaclab.sh"

if [[ ! -d "${HOME}/IsaacLab" ]]; then
  echo "[ERROR] ~/IsaacLab not found. Clone Isaac Lab first."
  exit 1
fi

if [[ ! -f "${ISAACLAB_SH}" ]]; then
  echo "[ERROR] ${ISAACLAB_SH} not found."
  exit 1
fi

echo "[INFO] Preparing URDF for Isaac Sim..."
python3 "${ENV_ROOT}/scripts/prepare_urdf.py"

if [[ -L "${EXT_LINK}" || -e "${EXT_LINK}" ]]; then
  echo "[INFO] Removing existing extension path: ${EXT_LINK}"
  rm -rf "${EXT_LINK}"
fi

echo "[INFO] Linking extension -> ${EXT_LINK}"
ln -s "${EXT_SRC}" "${EXT_LINK}"

echo "[INFO] Installing extension (this may take a few minutes)..."
cd "${HOME}/IsaacLab"

if [[ -f "${HOME}/miniconda3/etc/profile.d/conda.sh" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh"
  conda activate env_isaaclab
  pip install -e "${EXT_LINK}"
  echo "[OK] Installed via conda env_isaaclab"
else
  TERM=xterm ./isaaclab.sh -i none
fi

echo ""
echo "[OK] Installed. Smoke test:"
echo "  ~/sudo_ws/src/isaac_e0509_env/scripts/run_smoke_test.sh"
