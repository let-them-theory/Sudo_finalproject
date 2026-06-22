#!/usr/bin/env bash
# Train reach-to-cube (fine-tune from working reach policy).
set -euo pipefail
cd "${HOME}/IsaacLab"
export TERM="${TERM:-xterm}"
./isaaclab.sh -p "${HOME}/sudo_ws/src/isaac_e0509_env/isaaclab_ext/isaac_e0509_pick_place/scripts/train_reach_cube.py" \
  --task Isaac-Reach-Cube-E0509-v0 --num_envs 64 --max_iterations 800 --headless
