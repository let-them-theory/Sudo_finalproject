#!/usr/bin/env bash
# Train E0509 lift warm-started from the reach checkpoint.
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
SCRIPT="${HOME}/sudo_ws/src/isaac_e0509_env/isaaclab_ext/isaac_e0509_pick_place/scripts/train_lift_from_reach.py"
REACH_CKPT="${HOME}/IsaacLab/logs/rsl_rl/e0509_reach/2026-06-16_10-34-11/model_999.pt"

cd "${ISAACLAB}"
export TERM="${TERM:-xterm}"
./isaaclab.sh -p "${SCRIPT}" \
  --task Isaac-Lift-Cube-E0509-v0 \
  --num_envs 64 \
  --max_iterations 1500 \
  --reach_checkpoint "${REACH_CKPT}" \
  --headless
