#!/usr/bin/env bash
# _source_workspace.sh — canonical 워크스페이스 source 헬퍼.
#   순서: ROS humble → ros2_ws(두산 underlay) → kairos_ws(프로젝트 overlay).
#   (이전엔 mini_project/_source_workspace.sh 래퍼였으나 그 파일이 git에 없어 self-contained로 복구)
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true
source "$HOME/kairos_ws/install/setup.bash" 2>/dev/null || true
