#!/usr/bin/env bash
# killros.sh — 이 PC의 ROS/pick_place/Doosan 관련 프로세스 + ros2 daemon 전부 종료.
#
# 사용:
#   killros          # 정상 종료(DRCF 해제) 후 잔여 강제 kill + daemon stop
#   killros -f       # 정상 종료 생략, 즉시 강제 kill + daemon stop
#
set -o pipefail

FORCE=0
for arg in "$@"; do
  case "$arg" in
    -f|--force) FORCE=1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_is_cursor_sandbox() {
  local pid="$1"
  ps -p "$pid" -o cmd= 2>/dev/null | grep -q cursorsandbox
}

_kill_pattern() {
  local pat="$1"
  local sig="${2:-TERM}"
  local killed=0
  pgrep -f "$pat" 2>/dev/null | while read -r pid; do
    _is_cursor_sandbox "$pid" && continue
    kill "-$sig" "$pid" 2>/dev/null && killed=1 && echo "[killros] $sig $pid ($pat)"
  done
}

_force_kill_patterns() {
  local patterns=(
    "pick_place.launch.py"
    "lib/dsr_realsense_pick_place/"
    "lib/dsr_gripper_tcp/"
    "ros2_control_node"
    "realsense2_camera_node"
    "rviz2"
    "ultrasonic_node"
    "static_transform_publisher"
    "robot_state_publisher"
    "spawner_dsr_controller"
    "spawner_joint_state"
    "controller_manager/spawner"
    "dsr_bringup2"
    "component_container"
    "ros2 launch dsr_realsense"
    "ros2 launch dsr_bringup"
    "ros2 launch dsr_bringup2"
    "ros2-daemon"
  )
  for pat in "${patterns[@]}"; do
    pgrep -f "$pat" 2>/dev/null | while read -r pid; do
      _is_cursor_sandbox "$pid" && continue
      kill -9 "$pid" 2>/dev/null && echo "[killros] KILL $pid ($pat)"
    done
  done
}

echo "[killros] ===== ROS 전체 종료 시작 ====="

if [ "$FORCE" -eq 0 ]; then
  echo "[killros] 정상 종료 (DRCF/DRL 해제)..."
  bash "${SCRIPT_DIR}/shutdown_nodes.sh" --kill-launch 2>/dev/null || true
  sleep 1
else
  echo "[killros] --force: 정상 종료 생략"
fi

echo "[killros] launch / 노드 SIGTERM..."
_kill_pattern "pick_place.launch.py" TERM
_kill_pattern "lib/dsr_realsense_pick_place/" TERM
_kill_pattern "lib/dsr_gripper_tcp/" TERM
_kill_pattern "ros2_control_node" TERM
_kill_pattern "realsense2_camera_node" TERM
_kill_pattern "dsr_bringup2" TERM
sleep 2

echo "[killros] 잔여 SIGKILL..."
_force_kill_patterns

echo "[killros] ros2 daemon stop..."
if [ -f "$HOME/sudo_ws/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/sudo_ws/install/local_setup.bash" 2>/dev/null
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/opt/ros/humble/setup.bash" 2>/dev/null
fi
ros2 daemon stop 2>/dev/null || true
pkill -9 -f "ros2-daemon" 2>/dev/null || true

echo ""
echo "[killros] === 남은 프로세스 ==="
REMAIN="$(pgrep -af "lib/dsr_realsense|lib/dsr_gripper|ros2_control|realsense2|pick_place.launch|dsr_bringup|spawner_.*dsr01|ros2 launch dsr" 2>/dev/null | grep -v cursorsandbox || true)"
if [ -n "$REMAIN" ]; then
  echo "$REMAIN"
else
  echo "(없음)"
fi

echo "[killros] === ros2 daemon ==="
ros2 daemon status 2>&1 || true
echo "[killros] ===== 완료 ====="
