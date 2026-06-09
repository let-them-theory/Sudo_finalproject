#!/usr/bin/env bash
# ros2_control / dsr_controller2 DRL 서비스가 준비될 때까지 대기한다.
# pick_place.launch — 그리퍼 기동 전 health check.
#
# 사용: wait_for_robot_ready.sh <robot_ns> [timeout_sec]
set -o pipefail

ROBOT_NS="${1:-dsr01}"
TIMEOUT="${2:-120}"

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

DRL_START="/${ROBOT_NS}/drl/drl_start"
SET_MODE="/${ROBOT_NS}/system/set_robot_mode"

echo "[wait_robot] DRL 서비스 준비 대기: ${DRL_START} (max ${TIMEOUT}s)"
if ! ros2 service wait "${DRL_START}" --timeout "${TIMEOUT}"; then
  echo "[wait_robot] FAIL: drl_start 타임아웃 — ros2_control 기동/DRCF 연결 확인"
  exit 1
fi

echo "[wait_robot] set_robot_mode 서비스 확인: ${SET_MODE}"
if ! ros2 service wait "${SET_MODE}" --timeout 30; then
  echo "[wait_robot] WARN: set_robot_mode 미응답 (그리퍼 기동은 계속)"
fi

echo "[wait_robot] OK — 로봇 컨트롤러 서비스 준비 완료"
exit 0
