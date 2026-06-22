#!/usr/bin/env bash
# verify_pick_place_graph.sh — Pick & Place 스택 토픽/서비스/노드 연결 점검
# 사용: source ~/sudo_ws/install/local_setup.bash && bash verify_pick_place_graph.sh

set -o pipefail

ok=0
warn=0
fail=0

pass() { echo "  OK   $1"; ok=$((ok + 1)); }
warn_msg() { echo "  WARN $1"; warn=$((warn + 1)); }
fail_msg() { echo "  FAIL $1"; fail=$((fail + 1)); }

check_node() {
  local name="$1"
  if ros2 node list 2>/dev/null | grep -qx "$name"; then
    pass "node $name"
  else
    fail_msg "node $name (미실행)"
  fi
}

check_topic_pub() {
  local topic="$1"
  local info
  info=$(ros2 topic info "$topic" -v 2>/dev/null || true)
  if [[ -z "$info" ]]; then
    fail_msg "topic $topic (없음)"
    return
  fi
  if echo "$info" | grep -q "Publisher count: [1-9]"; then
    pass "topic $topic publisher"
  else
    fail_msg "topic $topic publisher 없음"
  fi
}

check_topic_sub() {
  local topic="$1"
  local node="$2"
  local info
  info=$(ros2 topic info "$topic" -v 2>/dev/null || true)
  if [[ -z "$info" ]]; then
    fail_msg "topic $topic (없음)"
    return
  fi
  if echo "$info" | grep -q "Subscription count: [1-9]"; then
    pass "topic $topic subscriber"
  else
    fail_msg "topic $topic subscriber 없음 (기대: $node)"
  fi
}

check_service() {
  local svc="$1"
  if ros2 service list 2>/dev/null | grep -qx "$svc"; then
    pass "service $svc"
  else
    fail_msg "service $svc (미등록)"
  fi
}

echo "===== Pick & Place 그래프 점검 ====="
if ! ros2 node list &>/dev/null; then
  echo "ROS2 데몬/노드 없음. 먼저 op 또는 start_clean.sh 실행 후 다시 점검하세요."
  exit 1
fi

echo
echo "[1] 핵심 노드"
for n in /object_detector /pick_place_gui /pick_place_node /rh_p12_rna_gripper /gripper_service /camera/camera /ultrasonic_node; do
  check_node "$n"
done

echo
echo "[2] 비전 파이프라인 (object_detector → GUI)"
check_topic_pub "/detection_debug_image"
check_topic_sub "/detection_debug_image" "pick_place_gui"
check_topic_pub "/detected_objects"
check_topic_sub "/detected_objects" "pick_place_gui"

echo
echo "[3] Pick 타겟 (object_detector → pick_place_node)"
check_topic_pub "/selected_object_pose"
check_topic_sub "/selected_object_pose" "pick_place_node"
check_topic_pub "/selected_object_class"
check_topic_sub "/selected_object_class" "pick_place_node"

echo
echo "[4] GUI 선택 (GUI → object_detector)"
check_topic_pub "/selected_object_label"
check_topic_sub "/selected_object_label" "object_detector"

echo
echo "[5] 상태/서비스"
check_topic_pub "/pick_place_state"
check_topic_sub "/pick_place_state" "pick_place_gui"
check_service "/pick_place/run_once"
check_service "/object_detector/get_parameters"
check_service "/gripper/open"

echo
echo "[6] 카메라 입력 (RealSense → object_detector)"
check_topic_pub "/camera/camera/color/image_raw"
check_topic_pub "/camera/camera/aligned_depth_to_color/image_raw"

echo
echo "[7] 검출 샘플 (/detected_objects 1회)"
sample=$(timeout 3 ros2 topic echo /detected_objects --once 2>/dev/null || true)
if [[ -n "$sample" ]]; then
  nobj=$(echo "$sample" | python3 -c "import sys,json,re; d=sys.stdin.read(); m=re.search(r'data:\s*(.+)',d,re.S); print(len(json.loads(m.group(1).strip()).get('objects',[])) if m else 0)" 2>/dev/null || echo 0)
  if [[ "${nobj:-0}" -gt 0 ]]; then
    pass "detected_objects 물체 ${nobj}개"
  else
    warn_msg "detected_objects 수신했으나 objects=0 (ROI/모델/conf 확인)"
  fi
else
  warn_msg "/detected_objects 3초 내 미수신 (object_detector·카메라 확인)"
fi

echo
echo "===== 결과: OK=${ok} WARN=${warn} FAIL=${fail} ====="
if [[ "$fail" -gt 0 ]]; then
  exit 2
fi
exit 0
