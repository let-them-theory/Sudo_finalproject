#!/usr/bin/env bash
# Play reach-to-cube (robot should move toward the dex cube).
set -euo pipefail
LOG_ROOT="${HOME}/IsaacLab/logs/rsl_rl/e0509_reach_cube"
CKPT="${1:-$(find "${LOG_ROOT}" -name 'model_*.pt' 2>/dev/null | sort -V | tail -1)}"
if [[ -z "${CKPT}" || ! -f "${CKPT}" ]]; then
  # Fallback: generic reach policy still moves toward table/cube region
  CKPT="${HOME}/IsaacLab/logs/rsl_rl/e0509_reach/2026-06-16_10-34-11/model_999.pt"
  TASK="Isaac-Reach-Cube-E0509-Play-v0"
  echo "[WARN] No reach_cube checkpoint; using reach policy on cube scene"
else
  TASK="Isaac-Reach-Cube-E0509-Play-v0"
  echo "[INFO] Playing: ${CKPT}"
fi
cd "${HOME}/IsaacLab"
export TERM="${TERM:-xterm}"
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task "${TASK}" --num_envs 1 --checkpoint "${CKPT}"
