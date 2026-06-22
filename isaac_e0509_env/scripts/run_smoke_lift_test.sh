#!/usr/bin/env bash
# Smoke test E0509 lift (cube on table) env.
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
SMOKE="${ISAACLAB}/source/isaac_e0509_pick_place/isaac_e0509_pick_place/scripts/smoke_lift_env.py"
HEADLESS="${HEADLESS:-1}"
NUM_ENVS="${NUM_ENVS:-1}"

if [[ ! -f "${SMOKE}" ]]; then
  echo "[ERROR] Extension not linked. Run: ~/sudo_ws/src/isaac_e0509_env/scripts/install_to_isaaclab.sh"
  exit 1
fi

# shellcheck disable=SC1091
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate env_isaaclab

cd "${ISAACLAB}"
export TERM=xterm
export PYTHONUNBUFFERED=1

ARGS=(--num_envs "${NUM_ENVS}")
if [[ "${HEADLESS}" == "1" ]]; then
  ARGS+=(--headless)
  echo "[INFO] headless lift smoke (set HEADLESS=0 for GUI)"
else
  echo "[INFO] GUI lift smoke"
fi

./isaaclab.sh -p "${SMOKE}" "${ARGS[@]}"
