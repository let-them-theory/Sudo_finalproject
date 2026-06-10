#!/usr/bin/env bash
# ROS + colcon 워크스페이스 setup.bash 를 현재 머신 경로에서 자동 탐색해 source 한다.
# 사용: source "$(dirname "${BASH_SOURCE[0]}")/_source_workspace.sh"

if [ -n "${SUDO_WS_SETUP:-}" ] && [ -f "${SUDO_WS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${SUDO_WS_SETUP}"
  return 0 2>/dev/null || exit 0
fi

# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash

_find_ws_setup() {
  local dir="$1"
  while [ -n "$dir" ] && [ "$dir" != "/" ]; do
    if [ -f "$dir/install/setup.bash" ]; then
      echo "$dir/install/setup.bash"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

_ws_setup="$(_find_ws_setup "$(cd "$(dirname "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")" && pwd)")"
if [ -z "$_ws_setup" ]; then
  for _candidate in \
    "$HOME/sudo_ws/install/setup.bash" \
    "$HOME/doosan_ws/install/setup.bash"; do
    if [ -f "$_candidate" ]; then
      _ws_setup="$_candidate"
      break
    fi
  done
fi

if [ -z "$_ws_setup" ] || [ ! -f "$_ws_setup" ]; then
  echo "[source_workspace] install/setup.bash 를 찾지 못했습니다. colcon build 후 다시 시도하세요." >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
source "$_ws_setup"
