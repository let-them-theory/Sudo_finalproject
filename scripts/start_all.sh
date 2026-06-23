#!/usr/bin/env bash
# start_all.sh — 전체 시스템 클린 시작.
#   점유 확인 → 정리(노드+포트) → 정리 검증 → 로봇+웹관리자 launch → 유저 키오스크 launch → 기동 검증.
#
# 사용:  bash start_all.sh [host]        (host 기본 110.120.1.50)
#
# 관리자 GUI : http://localhost:8080   (web_control)
# 유저 키오스크: http://localhost:8000   (web_kiosk)
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 패키지 루트 = scripts/ 의 부모(이 스크립트는 Sudo_finalproject/scripts/ 에 위치).
PROJ_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PKG_SCRIPTS="$PROJ_DIR/scripts"
SHUTDOWN="$PKG_SCRIPTS/shutdown_nodes.sh"
HOST="${1:-110.120.1.50}"
WEB_PORT="${WEB_PORT:-8080}"
KIOSK_PORT="${KIOSK_PORT:-8000}"

# ── workspace source (+ bashrc) ────────────────────────────────────────────
# ⚠️ .bashrc는 ros2_ws(두산)를 source 안 함(humble+kairos만). .bashrc를 먼저 부르고
#    그 뒤에 ros2_ws → kairos를 재source해 두산 underlay/overlay 순서를 보장한다.
#    (안 그러면 AMENT_PREFIX_PATH서 두산 dsr_hardware2 밀려 HW init 실패)
source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "$HOME/.bashrc" ] && source "$HOME/.bashrc" 2>/dev/null || true   # 사용자 환경(alias 등)
source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null || true          # 두산 underlay (필수)
source "$HOME/kairos_ws/install/setup.bash" 2>/dev/null || true        # 프로젝트 overlay (최상위)
export QT_QPA_PLATFORM=xcb
export DISPLAY="${DISPLAY:-:0}"

# ── [0/4] 빌드 (변경분 재빌드 — 루트 package.xml 때문에 --paths 필수) ────────
echo "[start_all] [0/4] colcon build (변경분)..."
(
  cd "$HOME/kairos_ws" || exit 1
  colcon build --paths \
    src/Sudo_finalproject/dsr_gripper_tcp_interfaces \
    src/Sudo_finalproject/dsr_gripper_tcp \
    src/Sudo_finalproject 2>&1 | tail -3
)
source "$HOME/kairos_ws/install/setup.bash" 2>/dev/null || true   # 새 install 반영

# ── helpers ────────────────────────────────────────────────────────────────
# 포트 점유 PID 목록 (ss 우선, 없으면 lsof, 없으면 fuser)
port_pids() {
    local port="$1"
    if command -v ss >/dev/null 2>&1; then
        ss -ltnp "sport = :$port" 2>/dev/null \
            | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u
    elif command -v lsof >/dev/null 2>&1; then
        lsof -tiTCP:"$port" -sTCP:LISTEN 2>/dev/null | sort -u
    else
        fuser "$port"/tcp 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u
    fi
}

# 노드/포트 점유 현황 출력. 점유 있으면 0, 없으면 1 반환.
report_busy() {
    local busy=1
    local nodes
    nodes="$(pgrep -af 'pick_place_node|web_control_node|object_detector|ros2_control_node|gripper_service_node|gripper_node|realsense2_camera_node|web_kiosk/backend/main.py|ros2 launch dsr_realsense' 2>/dev/null)"
    if [ -n "$nodes" ]; then
        echo "[start_all]   ▸ 실행 중 노드:"; echo "$nodes" | sed 's/^/[start_all]       /'
        busy=0
    fi
    local p
    for p in "$WEB_PORT" "$KIOSK_PORT"; do
        local pids; pids="$(port_pids "$p")"
        if [ -n "$pids" ]; then
            echo "[start_all]   ▸ 포트 $p 점유 PID: $(echo "$pids" | tr '\n' ' ')"
            busy=0
        fi
    done
    return $busy
}

# 포트 점유 프로세스 강제 종료
free_port() {
    local port="$1" pids
    pids="$(port_pids "$port")"
    [ -z "$pids" ] && return 0
    echo "[start_all]   포트 $port 점유 종료: $(echo "$pids" | tr '\n' ' ')"
    echo "$pids" | xargs -r kill -TERM 2>/dev/null || true
    sleep 2
    pids="$(port_pids "$port")"
    [ -n "$pids" ] && echo "$pids" | xargs -r kill -KILL 2>/dev/null || true
}

# 포트 LISTEN 대기 (성공 0 / 타임아웃 1)
wait_listen() {
    local port="$1" timeout="$2" i=0
    while [ "$i" -lt "$timeout" ]; do
        [ -n "$(port_pids "$port")" ] && return 0
        sleep 1; i=$((i + 1))
    done
    return 1
}

echo "[start_all] ===== 전체 클린 시작 (host=$HOST, web=$WEB_PORT, kiosk=$KIOSK_PORT) ====="

# ── 1. 점유 확인 ──────────────────────────────────────────────────────────
echo "[start_all] [1/4] 기존 점유 확인..."
if report_busy; then
    NEED_CLEAN=1
else
    NEED_CLEAN=0
    echo "[start_all]   점유 없음."
fi

# ── 2. 정리 (살아있으면 graceful 종료 — ros2_control은 DRCF authority 반납까지) ──
if [ "$NEED_CLEAN" -eq 1 ]; then
    echo "[start_all] [2/4] 기존 노드 정리 중 (graceful)..."
    if [ -f "$SHUTDOWN" ]; then
        bash "$SHUTDOWN" --kill-launch || true   # ros2_control SIGTERM + DRCF close 포함
    else
        echo "[start_all]   ⚠ shutdown_nodes.sh 없음 ($SHUTDOWN) — pkill 대체"
        pkill -TERM -f "ros2 launch dsr_realsense" 2>/dev/null || true
        pkill -TERM -f "ros2_control_node" 2>/dev/null || true
    fi
    pkill -TERM -f "web_kiosk/backend/main.py" 2>/dev/null || true
    pkill -TERM -f "google-chrome.*--app=http://localhost:$KIOSK_PORT" 2>/dev/null || true
    # ★ ros2_control 완전 종료(DRCF authority 반납) 대기 — 안 기다리면 새 launch가 제어권 거부.
    echo "[start_all]   ros2_control DRCF 반납 대기(최대 25s)..."
    for _i in $(seq 1 25); do
        pgrep -f "ros2_control_node" >/dev/null 2>&1 || { echo "[start_all]   DRCF 반납 완료(${_i}s)"; break; }
        sleep 1
    done
    free_port "$WEB_PORT"
    free_port "$KIOSK_PORT"
else
    echo "[start_all] [2/4] 정리 불필요 — 건너뜀."
fi

# ── 3. 정리 검증 ───────────────────────────────────────────────────────────
echo "[start_all] [3/4] 정리 검증..."
# ros2_control은 SIGINT로만 graceful 종료(소멸자 Drfl.close_connection이 DRCF authority 반납).
# SIGKILL하면 제어권 wedge → 다음 launch 거부. graceful 미완이면 SIGINT 재시도(KILL 금지).
if pgrep -f "ros2_control_node" >/dev/null 2>&1; then
    echo "[start_all]   ⚠ ros2_control 잔존 — SIGINT 재시도(제어권 반납 위해 SIGKILL 안 함)..."
    pkill -INT -f "ros2_control_node" 2>/dev/null || true
    for _i in $(seq 1 15); do pgrep -f "ros2_control_node" >/dev/null 2>&1 || break; sleep 1; done
    pgrep -f "ros2_control_node" >/dev/null 2>&1 && \
        echo "[start_all]   ❌ ros2_control graceful 실패 — 제어권 꼬임 위험. 수동 확인 후 재시도." && exit 1
fi
# ros2_control 외 잔재만 SIGKILL (ros2_control은 위에서 SIGINT 처리, 절대 KILL 안 함).
if report_busy; then
    echo "[start_all]   ⚠ 잔재 남음 — SIGKILL 강제 정리(ros2_control 제외)..."
    pkill -KILL -f "pick_place_node|web_control_node|object_detector|gripper_service|gripper_node|rh_p12_rna|realsense2|ultrasonic|web_kiosk/backend/main.py|ros2 launch dsr_realsense" 2>/dev/null || true
    free_port "$WEB_PORT"; free_port "$KIOSK_PORT"
    sleep 2
    if report_busy; then
        echo "[start_all]   ❌ 정리 실패 — 위 프로세스 수동 종료 후 재시도하라. 중단."
        exit 1
    fi
fi
echo "[start_all]   정리 완료 — 포트 비어있음."

# ── 4. 실행 ────────────────────────────────────────────────────────────────
echo "[start_all] [4/4] 로봇 + 웹 관리자 launch (web:=true, port=$WEB_PORT)..."
ros2 launch dsr_realsense_pick_place pick_place.launch.py \
    mode:=real host:="$HOST" web:=true gui:=false web_port:="$WEB_PORT" &
LAUNCH_PID=$!

sleep 4
echo "[start_all]   유저 키오스크 백엔드 launch (port=$KIOSK_PORT)..."
ros2 launch dsr_realsense_pick_place web_kiosk.launch.py kiosk_port:="$KIOSK_PORT" &
KIOSK_PID=$!

# ── 종료 시 둘 다 정리 ────────────────────────────────────────────────────
cleanup() {
    echo "[start_all] 종료 중..."
    kill "$LAUNCH_PID" "$KIOSK_PID" 2>/dev/null || true
    [ -f "$SHUTDOWN" ] && bash "$SHUTDOWN" --kill-launch 2>/dev/null || true
    pkill -TERM -f "web_kiosk/backend/main.py" 2>/dev/null || true
}
trap cleanup INT TERM

# ── 기동 검증 ──────────────────────────────────────────────────────────────
echo ""
echo "[start_all] 기동 검증 (포트 LISTEN 대기)..."
wait_listen "$WEB_PORT" 40 \
    && echo "[start_all]   ✅ 관리자  GUI : http://localhost:$WEB_PORT" \
    || echo "[start_all]   ⚠ 관리자 포트 $WEB_PORT 미응답 — launch 로그 확인 (로봇 이더넷?)"
if wait_listen "$KIOSK_PORT" 20; then
    echo "[start_all]   ✅ 유저 키오스크: http://localhost:$KIOSK_PORT"
    echo "[start_all]   관리자 패널: http://localhost:$KIOSK_PORT/admin (수동 접속)"
else
    echo "[start_all]   ⚠ 키오스크 포트 $KIOSK_PORT 미응답 — launch 로그 확인"
fi
echo "[start_all] (종료: Ctrl+C → 두 launch 모두 정리)"

wait
