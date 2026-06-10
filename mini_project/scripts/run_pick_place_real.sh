#!/usr/bin/env bash
# pick_place 실기 모드 실행 — 종료(Ctrl+C 포함) 시 DRCF/DRL을 정상 해제한다.
#
# 사용:
#   bash run_pick_place_real.sh
#   bash run_pick_place_real.sh host:=192.168.1.10 gui:=false
#
set -o pipefail

HOST="${ROBOT_HOST:-110.120.1.50}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../_source_workspace.sh"

_shutdown_ran=0
_cleanup() {
  if [ "$_shutdown_ran" -eq 1 ]; then
    return
  fi
  _shutdown_ran=1
  echo ""
  echo "[run_pick_place] 정상 종료 스크립트 실행 중..."
  ROBOT_NS="${ROBOT_NS:-dsr01}" bash "${SCRIPT_DIR}/shutdown_nodes.sh" --kill-launch || true
}

trap _cleanup EXIT INT TERM

echo "[run_pick_place] ros2 launch 시작 (host=${HOST})"
echo "[run_pick_place] 종료 시 자동으로 DRCF/DRL 해제됩니다 (Ctrl+C 포함)"

ros2 launch dsr_realsense_pick_place pick_place.launch.py \
  mode:=real \
  "host:=${HOST}" \
  "$@"
