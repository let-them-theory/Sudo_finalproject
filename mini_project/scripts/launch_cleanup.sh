#!/usr/bin/env bash
# ros2 launch 종료(Ctrl+C) 시 이벤트 핸들러로 기동된 노드가 남는 경우를 정리한다.
#
# pick_place.launch.py OnShutdown 에서 호출.
set -o pipefail

ROBOT_NS="${ROBOT_NS:-dsr01}"

source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash 2>/dev/null || true
for _ws in \
  "/home/user/Sudo_finalproject-main/install/setup.bash" \
  "/home/user/doosan_ws/install/setup.bash"; do
  if [ -f "$_ws" ]; then
    # shellcheck disable=SC1090
    source "$_ws" 2>/dev/null || true
    break
  fi
done

echo "[launch_cleanup] launch 종료 — 그리퍼·pick_place 잔여 프로세스 정리"

timeout 3 ros2 service call "/${ROBOT_NS}/drl/drl_stop" dsr_msgs2/srv/DrlStop "{stop_mode: 1}" \
  2>/dev/null || true

PATTERNS=(
  "wait_for_gripper_ready"
  "wait_for_robot_ready"
  "lib/dsr_gripper_tcp/gripper_service_node"
  "lib/dsr_realsense_pick_place/gripper_node"
  "lib/dsr_realsense_pick_place/pick_place_node"
)

for pat in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pat" 2>/dev/null || true
done
sleep 2
for pat in "${PATTERNS[@]}"; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    echo "[launch_cleanup] SIGKILL: $pat"
    pkill -KILL -f "$pat" 2>/dev/null || true
  fi
done

echo "[launch_cleanup] 완료"
