#!/usr/bin/env bash
# 그리퍼 브릿지(gripper_service_node + gripper_node)만 새 프로세스로 재기동한다.
# status3(플랜지 Modbus 무응답) 복구용.
#
# 사용: restart_gripper_bridge.sh [로봇IP]   (기본 IP 110.120.1.50)
set -o pipefail

HOST="${1:-110.120.1.50}"
ROBOT_NS="${ROBOT_NS:-dsr01}"
TERM_GRACE="${TERM_GRACE:-10}"

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || true
for _ws in \
  "/home/user/Sudo_finalproject-main/install/setup.bash" \
  "/home/user/doosan_ws/install/setup.bash"; do
  if [ -f "$_ws" ]; then
    # shellcheck disable=SC1090
    source "$_ws" 2>/dev/null || true
    break
  fi
done

PKG_PREFIX="$(ros2 pkg prefix dsr_realsense_pick_place 2>/dev/null)"
PF="${PKG_PREFIX}/share/dsr_realsense_pick_place/config/pick_place_params.yaml"
if [ -z "$PKG_PREFIX" ] || [ ! -f "$PF" ]; then
  echo "[gripper-restart] ⚠️ params yaml을 못 찾음(PF=$PF). 워크스페이스 setup.bash를 먼저 source했는지 확인." >&2
fi

echo "[gripper-restart] DRL 정지 시도 (/${ROBOT_NS}/drl/drl_stop)..."
timeout 5 ros2 service call "/${ROBOT_NS}/drl/drl_stop" dsr_msgs2/srv/DrlStop "{stop_mode: 1}" \
  2>/dev/null || echo "[gripper-restart] drl_stop 미응답 (계속 진행)"

echo "[gripper-restart] 기존 그리퍼 브릿지 SIGTERM (grace ${TERM_GRACE}s)..."
pgrep -f "lib/dsr_gripper_tcp/gripper_service_node" | xargs -r kill -TERM
pgrep -f "lib/dsr_realsense_pick_place/gripper_node" | xargs -r kill -TERM

i=0
while { pgrep -f "lib/dsr_gripper_tcp/gripper_service_node" >/dev/null 2>&1 \
    || pgrep -f "lib/dsr_realsense_pick_place/gripper_node" >/dev/null 2>&1; } \
  && [ "$i" -lt "$TERM_GRACE" ]; do
  sleep 1
  i=$((i + 1))
done

if pgrep -f "lib/dsr_gripper_tcp/gripper_service_node" >/dev/null 2>&1 \
  || pgrep -f "lib/dsr_realsense_pick_place/gripper_node" >/dev/null 2>&1; then
  echo "[gripper-restart] SIGTERM 타임아웃 — SIGKILL"
  pgrep -f "lib/dsr_gripper_tcp/gripper_service_node" | xargs -r kill -9
  pgrep -f "lib/dsr_realsense_pick_place/gripper_node" | xargs -r kill -9
  sleep 2
else
  sleep 1
fi

echo "[gripper-restart] gripper_service_node 새 프로세스로 재기동 (host=$HOST)..."
setsid ros2 run dsr_gripper_tcp gripper_service_node --ros-args \
  -p controller_host:="$HOST" -p tcp_port:=20002 -p namespace:="$ROBOT_NS" \
  -p goal_current:=400 -p profile_velocity:=1500 -p profile_acceleration:=1000 \
  -p connect_timeout_sec:=60.0 -p post_drl_start_sleep_sec:=2.0 -p drl_idle_stable_sec:=2.0 \
  -p tcp_server_open_retry_sec:=0.5 -p init_attempts:=5 -p init_timeout_sec:=20.0 \
  -p init_retry_delay_sec:=1.0 > /tmp/gripper_svc_restart.log 2>&1 &

echo "[gripper-restart] gripper_node(rh_p12_rna_gripper) 새 프로세스로 재기동..."
setsid ros2 run dsr_realsense_pick_place gripper_node --ros-args \
  --params-file "$PF" -p robot_ns:="$ROBOT_NS" -r __node:=rh_p12_rna_gripper \
  > /tmp/gripper_node_restart.log 2>&1 &

echo "[gripper-restart] 재기동 명령 전송 완료. 초기화에 ~5-40초. 로그: /tmp/gripper_svc_restart.log"
