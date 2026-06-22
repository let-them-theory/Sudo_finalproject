#!/usr/bin/env bash
# Play latest E0509 lift checkpoint (prefers v2_clip run).
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
LOG_ROOT="${ISAACLAB}/logs/rsl_rl/e0509_lift"
CKPT="${1:-}"

if [[ -z "${CKPT}" ]]; then
  # Prefer newest v2_clip run, else newest model_1499.pt
  CKPT="$(find "${LOG_ROOT}" -path '*v2_clip*' -name 'model_1499.pt' 2>/dev/null | sort -r | head -1 || true)"
  if [[ -z "${CKPT}" ]]; then
    CKPT="$(find "${LOG_ROOT}" -name 'model_1499.pt' 2>/dev/null | sort -r | head -1 || true)"
  fi
fi

if [[ -z "${CKPT}" || ! -f "${CKPT}" ]]; then
  echo "[ERROR] No checkpoint found under ${LOG_ROOT}" >&2
  exit 1
fi

echo "[INFO] Playing checkpoint: ${CKPT}"
cd "${ISAACLAB}"
export TERM="${TERM:-xterm}"
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Lift-Cube-E0509-Play-v0 \
  --num_envs 1 \
  --checkpoint "${CKPT}"
