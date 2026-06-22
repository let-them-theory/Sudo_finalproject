#!/usr/bin/env bash
# Full lift retrain: reach warm-start + clip_actions + fixed physics (v2_clip).
set -euo pipefail

ISAACLAB="${HOME}/IsaacLab"
SCRIPT="${HOME}/sudo_ws/src/isaac_e0509_env/isaaclab_ext/isaac_e0509_pick_place/scripts/train_lift_from_reach.py"
VALIDATE="${HOME}/sudo_ws/src/isaac_e0509_env/isaaclab_ext/isaac_e0509_pick_place/scripts/validate_lift_setup.py"
REACH_CKPT="${HOME}/IsaacLab/logs/rsl_rl/e0509_reach/2026-06-16_10-34-11/model_999.pt"

cd "${ISAACLAB}"
export TERM="${TERM:-xterm}"

echo "[1/3] Preflight physics check..."
./isaaclab.sh -p "${VALIDATE}" --headless

echo "[2/3] Training lift v2_clip (1500 iter, 64 envs)..."
./isaaclab.sh -p "${SCRIPT}" \
  --task Isaac-Lift-Cube-E0509-v0 \
  --num_envs 64 \
  --max_iterations 1500 \
  --reach_checkpoint "${REACH_CKPT}" \
  --headless

NEW_CKPT="$(find logs/rsl_rl/e0509_lift -path '*v2_clip*' -name 'model_1499.pt' | sort -r | head -1)"
echo "[3/3] Post-train policy check: ${NEW_CKPT}"
./isaaclab.sh -p "${VALIDATE}" --headless --checkpoint "${NEW_CKPT}"

echo ""
echo "[DONE] Play with:"
echo "  bash ${HOME}/sudo_ws/src/isaac_e0509_env/isaaclab_ext/isaac_e0509_pick_place/scripts/play_lift.sh"
