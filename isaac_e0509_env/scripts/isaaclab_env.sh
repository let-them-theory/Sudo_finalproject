#!/usr/bin/env bash
# Shared environment for Isaac Lab launch scripts.
# ROS colcon may install a stale isaac_e0509_pick_place overlay that shadows
# the editable Isaac Lab extension and breaks imports (e.g. missing tasks/lift).

# shellcheck disable=SC1091
source "${HOME}/miniconda3/etc/profile.d/conda.sh"
conda activate env_isaaclab

isaaclab_fix_pythonpath() {
  if [[ -z "${PYTHONPATH:-}" ]]; then
    return
  fi
  local cleaned=""
  local p
  IFS=':' read -ra paths <<< "${PYTHONPATH}"
  for p in "${paths[@]}"; do
    [[ -z "${p}" ]] && continue
    [[ "${p}" == *"/install/isaac_e0509_pick_place/"* ]] && continue
    if [[ -n "${cleaned}" ]]; then
      cleaned="${cleaned}:${p}"
    else
      cleaned="${p}"
    fi
  done
  if [[ -n "${cleaned}" ]]; then
    export PYTHONPATH="${cleaned}"
  else
    unset PYTHONPATH
  fi
}

isaaclab_fix_pythonpath
