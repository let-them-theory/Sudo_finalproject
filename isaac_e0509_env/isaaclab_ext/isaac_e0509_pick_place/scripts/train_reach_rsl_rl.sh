#!/usr/bin/env bash
# Train E0509 reach with RSL-RL (requires isaaclab_rl installed via isaaclab.sh -i)
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
TASK="Isaac-Reach-E0509-v0"

cd "${ISAACLAB}"
./isaaclab.sh -p source/isaaclab_rl/isaaclab_rl/runners/rsl_rl_train.py \
  --task "${TASK}" \
  --num_envs 64 \
  --headless
