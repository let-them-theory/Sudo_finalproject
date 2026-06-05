#!/usr/bin/env bash
# 그리퍼 브릿지(gripper_service_node + gripper_node)만 새 프로세스로 재기동한다.
# status3(플랜지 Modbus 무응답) 복구용 — 로봇 전원 사이클도, 전체 시스템 리셋도 불필요.
# in-process reinitialize가 안 풀릴 때, fresh 프로세스 기동은 DrlStart가 즉시 수락돼 복구된다.
#
# 사용: restart_gripper_bridge.sh [로봇IP]   (기본 IP 110.120.1.50)
# 주의: set -u는 쓰지 않는다 — ROS setup.bash가 unbound 변수를 참조해 스크립트가 즉시 죽는다.
set -o pipefail

HOST="${1:-110.120.1.50}"
PF=/home/pyw/kairos_ws/install/dsr_realsense_pick_place/share/dsr_realsense_pick_place/config/pick_place_params.yaml

# ROS 환경 — GUI가 호출하면 보통 이미 sourced지만 단독 실행 대비 보강.
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/pyw/kairos_ws/install/setup.bash 2>/dev/null || true

echo "[gripper-restart] 기존 그리퍼 브릿지 노드 종료 (PID 기반, pkill -f 자기매칭 회피)..."
# install lib 경로로 정밀 매칭 — 스크립트 자기 명령줄(restart_gripper_bridge.sh)과 안 겹침.
pgrep -f "lib/dsr_gripper_tcp/gripper_service_node" | xargs -r kill -9
pgrep -f "lib/dsr_realsense_pick_place/gripper_node" | xargs -r kill -9
sleep 3

echo "[gripper-restart] gripper_service_node 새 프로세스로 재기동 (host=$HOST)..."
setsid ros2 run dsr_gripper_tcp gripper_service_node --ros-args \
  -p controller_host:="$HOST" -p tcp_port:=20002 -p namespace:=dsr01 \
  -p goal_current:=400 -p profile_velocity:=1500 -p profile_acceleration:=1000 \
  -p connect_timeout_sec:=60.0 -p post_drl_start_sleep_sec:=2.0 -p drl_idle_stable_sec:=2.0 \
  -p tcp_server_open_retry_sec:=0.5 -p init_attempts:=5 -p init_timeout_sec:=20.0 \
  -p init_retry_delay_sec:=1.0 > /tmp/gripper_svc_restart.log 2>&1 &

echo "[gripper-restart] gripper_node(rh_p12_rna_gripper) 새 프로세스로 재기동..."
setsid ros2 run dsr_realsense_pick_place gripper_node --ros-args \
  --params-file "$PF" -p robot_ns:=dsr01 -r __node:=rh_p12_rna_gripper \
  > /tmp/gripper_node_restart.log 2>&1 &

echo "[gripper-restart] 재기동 명령 전송 완료. 초기화에 ~5-40초. 로그: /tmp/gripper_svc_restart.log"
