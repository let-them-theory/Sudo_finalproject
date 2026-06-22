#!/usr/bin/env bash
# Train E0509 cube lift (pick warmup) with RSL-RL.
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
TASK="Isaac-Lift-Cube-E0509-v0"

cd "${ISAACLAB}"
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task "${TASK}" \
  --num_envs 64 \
  --headless
