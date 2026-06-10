#!/usr/bin/env bash
# shutdown_nodes.sh — DRCF authority / DRL 그리퍼 세션을 정상 해제한 뒤 노드를 종료한다.
#
# 종료 순서 (재연결 실패 방지):
#   0. pick_place 정지
#   1. DrlStop 서비스 (ros2_control 살아있을 때 — 플랜지 RS-485 해제)
#   2. gripper_service / gripper_node SIGTERM (bridge.close → DrlStop 재시도)
#   3. vision·보조 노드
#   4. ros2_control_node SIGTERM (Drfl.close_connection — DRCF authority 해제)
#   5. 나머지 + (--kill-launch 시 launch 부모)
#   6. 잔여만 SIGKILL
#
# 사용:
#   bash shutdown_nodes.sh
#   bash shutdown_nodes.sh --kill-launch
#   ROBOT_NS=dsr01 DRCF_GRACE=20 bash shutdown_nodes.sh

set -o pipefail

ROBOT_NS="${ROBOT_NS:-dsr01}"
DRL_STOP_SVC="/${ROBOT_NS}/drl/drl_stop"
TERM_GRACE="${TERM_GRACE:-12}"
DRCF_GRACE="${DRCF_GRACE:-15}"
KILL_LAUNCH=0

for arg in "$@"; do
  case "$arg" in
    --kill-launch) KILL_LAUNCH=1 ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../_source_workspace.sh"

wait_gone() {
  local pattern="$1"
  local timeout="$2"
  local i=0
  while pgrep -f "$pattern" >/dev/null 2>&1 && [ "$i" -lt "$timeout" ]; do
    sleep 1
    i=$((i + 1))
  done
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    return 1
  fi
  return 0
}

term_pattern() {
  pkill -TERM -f "$1" 2>/dev/null || true
}

kill_pattern() {
  pkill -KILL -f "$1" 2>/dev/null || true
}

echo "[shutdown] ===== 정상 종료 시작 (ns=${ROBOT_NS}) ====="

echo "[shutdown] [0/6] pick_place_node 정지..."
term_pattern "lib/dsr_realsense_pick_place/pick_place_node"
term_pattern "pick_place_node"
sleep 1

echo "[shutdown] [1/6] DRL 정지 (${DRL_STOP_SVC})..."
if timeout 8 ros2 service call "$DRL_STOP_SVC" dsr_msgs2/srv/DrlStop "{stop_mode: 1}" 2>/dev/null; then
  echo "[shutdown]     drl_stop 완료"
else
  echo "[shutdown]     drl_stop 미응답 (이미 정지 또는 ros2_control 미기동)"
fi
sleep 2

echo "[shutdown] [2/6] gripper 노드 SIGTERM (grace ${TERM_GRACE}s)..."
term_pattern "lib/dsr_gripper_tcp/gripper_service_node"
term_pattern "lib/dsr_realsense_pick_place/gripper_node"
if ! wait_gone "lib/dsr_gripper_tcp/gripper_service_node" "$TERM_GRACE"; then
  echo "[shutdown]     gripper_service 아직 실행 중"
fi
if ! wait_gone "lib/dsr_realsense_pick_place/gripper_node" 5; then
  echo "[shutdown]     gripper_node 아직 실행 중"
fi

echo "[shutdown] [3/6] vision·보조 노드 SIGTERM..."
term_pattern "lib/dsr_realsense_pick_place/object_detector"
term_pattern "lib/dsr_realsense_pick_place/ultrasonic_node"
term_pattern "object_detector"
term_pattern "ultrasonic_node"
term_pattern "realsense2_camera_node"
term_pattern "static_transform_publisher"
sleep 2

echo "[shutdown] [4/6] ros2_control_node SIGTERM (DRCF 해제, grace ${DRCF_GRACE}s)..."
term_pattern "ros2_control_node"
if ! wait_gone "ros2_control_node" "$DRCF_GRACE"; then
  echo "[shutdown]     ros2_control_node 아직 실행 중 (DRCF 해제 지연 가능)"
fi

echo "[shutdown] [5/6] 나머지 노드 SIGTERM..."
term_pattern "robot_state_publisher"
term_pattern "controller_manager/spawner"
term_pattern "rviz2"
term_pattern "run_emulator"
term_pattern "dsr_controller2"
sleep 2

if [ "$KILL_LAUNCH" -eq 1 ]; then
  echo "[shutdown]     pick_place.launch 부모 SIGTERM..."
  term_pattern "pick_place.launch.py"
  wait_gone "pick_place.launch.py" 8 || true
fi

echo "[shutdown] [6/6] 잔여 프로세스 SIGKILL..."
for pat in \
  gripper_service_node \
  gripper_node \
  pick_place_node \
  object_detector \
  ultrasonic_node \
  realsense2_camera_node \
  static_transform_publisher \
  robot_state_publisher \
  ros2_control_node \
  pick_place.launch.py; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    echo "[shutdown]     SIGKILL: ${pat}"
    kill_pattern "$pat"
  fi
done

echo "[shutdown] ===== 종료 완료 ====="
