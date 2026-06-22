#!/usr/bin/env bash
# ROS + colcon 워크스페이스를 현재 머신 경로에서 자동 탐색해 source 한다.
# setup.bash(체인) 대신 local_setup.bash 를 써서 예전 doosan_ws 경로 오염을 막는다.

if [ -n "${SUDO_WS_SETUP:-}" ] && [ -f "${SUDO_WS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${SUDO_WS_SETUP}"
  return 0 2>/dev/null || exit 0
fi

# shellcheck disable=SC1091
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash

# 동일 LAN의 AMR/Nav2 등 다른 ROS 기기 토픽이 섞이지 않게 기본 localhost만 사용.
# DDS로 다른 기기와 통신해야 하면: unset ROS_LOCALHOST_ONLY
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

_find_ws_root() {
  local dir="$1"
  while [ -n "$dir" ] && [ "$dir" != "/" ]; do
    if [ -f "$dir/install/local_setup.bash" ]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done
  return 1
}

_ws_root="$(_find_ws_root "$(cd "$(dirname "${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}")" && pwd)")"

# sudo_ws 우선 (doosan_ws 복사본보다 현재 워크스페이스를 먼저 사용)
if [ -f "$HOME/sudo_ws/install/local_setup.bash" ]; then
  _ws_root="$HOME/sudo_ws"
elif [ -z "$_ws_root" ]; then
  for _candidate in "$HOME/sudo_ws" "$HOME/doosan_ws"; do
    if [ -f "$_candidate/install/local_setup.bash" ]; then
      _ws_root="$_candidate"
      break
    fi
  done
fi

if [ -z "$_ws_root" ] || [ ! -f "$_ws_root/install/local_setup.bash" ]; then
  echo "[source_workspace] install/local_setup.bash 를 찾지 못했습니다. colcon build 후 다시 시도하세요." >&2
  return 1 2>/dev/null || exit 1
fi

# shellcheck disable=SC1090
source "$_ws_root/install/local_setup.bash"
