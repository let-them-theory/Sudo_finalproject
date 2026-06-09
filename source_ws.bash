#!/usr/bin/env bash
# pick_place 실행 전 환경 설정 (터미널에서: source source_ws.bash)
source /opt/ros/humble/setup.bash
if [ -f /home/user/Sudo_finalproject-main/install/setup.bash ]; then
  source /home/user/Sudo_finalproject-main/install/setup.bash
elif [ -f /home/user/Sudo_finalproject/install/setup.bash ]; then
  source /home/user/Sudo_finalproject/install/setup.bash
else
  echo "[source_ws] install/setup.bash 없음 — colcon build 먼저 실행하세요" >&2
  return 1 2>/dev/null || exit 1
fi
