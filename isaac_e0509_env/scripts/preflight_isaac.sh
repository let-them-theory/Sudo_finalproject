#!/usr/bin/env bash
# Quick checks before running Isaac Sim / Isaac Lab
set -euo pipefail

ok=true
watches="$(cat /proc/sys/fs/inotify/max_user_watches)"
instances="$(cat /proc/sys/fs/inotify/max_user_instances)"

echo "=== Isaac preflight ==="
echo "inotify.max_user_watches  = ${watches}  (need >= 524288)"
echo "inotify.max_user_instances = ${instances} (need >= 512)"

if [[ "${watches}" -lt 524288 || "${instances}" -lt 512 ]]; then
  echo "[FAIL] inotify — run: sudo bash $(dirname "$0")/fix_inotify.sh"
  ok=false
else
  echo "[OK] inotify"
fi

if [[ ! -d "${HOME}/IsaacLab" ]]; then
  echo "[FAIL] ~/IsaacLab not found"
  ok=false
else
  echo "[OK] IsaacLab"
fi

if [[ ! -L "${HOME}/IsaacLab/source/isaac_e0509_pick_place" ]]; then
  echo "[WARN] E0509 extension not linked — run install_to_isaaclab.sh"
else
  echo "[OK] E0509 extension"
fi

if command -v conda >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source "${HOME}/miniconda3/etc/profile.d/conda.sh" 2>/dev/null || true
  if conda env list | grep -q env_isaaclab; then
    echo "[OK] conda env_isaaclab"
  else
    echo "[FAIL] conda env_isaaclab missing"
    ok=false
  fi
fi

if [[ "${ok}" == true ]]; then
  echo "=== All critical checks passed ==="
else
  echo "=== Fix failures above, then retry ==="
  exit 1
fi
