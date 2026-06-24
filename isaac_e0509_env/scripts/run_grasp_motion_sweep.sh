#!/usr/bin/env bash
# Multi-angle grasp motion sweep (IK, stable — not random smoke test).
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
SWEEP="${ISAACLAB}/source/isaac_e0509_pick_place/isaac_e0509_pick_place/scripts/grasp_motion_sweep.py"

if [[ ! -f "${SWEEP}" ]]; then
  echo "[ERROR] Extension not linked. Run: ~/sudo_ws/src/isaac_e0509_env/scripts/install_to_isaaclab.sh"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/isaaclab_env.sh"

cd "${ISAACLAB}"
export TERM=xterm
export PYTHONUNBUFFERED=1

echo "[INFO] Grasp motion sweep (default preset: top_down). Examples:"
echo "  PRESET=top_down HEADLESS=0 $0"
echo "  PRESET=all HEADLESS=1 $0          # all angles, headless log only"
echo "  OBJECT_YAW_DEG=45 PRESET=yaw_45 HEADLESS=0 $0"
echo ""

PRESET="${PRESET:-top_down}"
ARGS=(--num_envs 1 --preset "${PRESET}")
if [[ -n "${OBJECT_YAW_DEG:-}" ]]; then
  ARGS+=(--object-yaw-deg "${OBJECT_YAW_DEG}")
fi
if [[ "${HEADLESS:-0}" == "1" ]]; then
  ARGS+=(--headless)
  echo "[INFO] headless sweep"
else
  echo "[INFO] GUI sweep (set HEADLESS=1 for batch)"
fi

./isaaclab.sh -p "${SWEEP}" "${ARGS[@]}" "$@"
