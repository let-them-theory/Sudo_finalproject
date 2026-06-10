#!/usr/bin/env bash
# ============================================================
# 1회 셋업 헬퍼 — 외부 의존성 설치 + 빌드.
# 이 repo만으로는 실행 불가(외부 ROS 패키지 필요). 이 스크립트가 그걸 자동화한다.
#
# 사용: 워크스페이스 루트(예: ~/ros2_ws)에서
#         bash src/Sudo_finalproject/setup.sh
#       (이 repo가 <ws>/src/Sudo_finalproject 에 클론돼 있어야 함)
#
# 이후 실행은 IP와 모델만:
#   - config/pick_place_params.yaml 의 yolo_model 을 본인 모델 절대경로로 수정
#   - ros2 launch dsr_realsense_pick_place pick_place.launch.py mode:=real host:=<로봇IP>
# ============================================================
set -uo pipefail
ROS_DISTRO="${ROS_DISTRO:-humble}"

[ -d src ] || { echo "[setup] ❌ 워크스페이스 루트에서 실행하세요 (여기에 src/ 가 없음)"; exit 1; }
echo "[setup] 워크스페이스: $(pwd) / ROS_DISTRO=$ROS_DISTRO"

echo "[setup] 1) Doosan ROS2 스택 (dsr_msgs2 / dsr_bringup2 / dsr_controller2)..."
[ -d src/doosan-robot2 ] || git clone -b "$ROS_DISTRO" \
  https://github.com/doosan-robotics/doosan-robot2.git src/doosan-robot2 || \
  echo "[setup] ⚠️ doosan-robot2 클론 실패 — 수동 설치 필요"

echo "[setup] 2) apt 패키지 (realsense / cv_bridge / tf_transformations / message_filters)..."
sudo apt-get update
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-realsense2-camera" "ros-${ROS_DISTRO}-realsense2-description" \
  "ros-${ROS_DISTRO}-cv-bridge" "ros-${ROS_DISTRO}-tf-transformations" \
  "ros-${ROS_DISTRO}-message-filters" python3-pip || \
  echo "[setup] ⚠️ 일부 apt 패키지 실패 — 로그 확인"

echo "[setup] 3) rosdep (나머지 ROS 의존성 자동 해결)..."
sudo rosdep init 2>/dev/null || true
rosdep update || true
rosdep install --from-paths src --ignore-src -r -y || \
  echo "[setup] ⚠️ rosdep 일부 실패 — 로그 확인"

echo "[setup] 4) Python 의존성 (ultralytics / torch 등)..."
pip install -r src/Sudo_finalproject/mini_project/requirements.txt || \
  echo "[setup] ⚠️ pip 실패 — GPU torch는 별도 설치 권장(pytorch.org)"

echo "[setup] 5) colcon build..."
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install

echo ""
echo "[setup] ✅ 셋업 완료. 이제 IP와 모델만 설정하면 됩니다:"
echo "  1) source install/setup.bash"
echo "  2) src/Sudo_finalproject/mini_project/config/pick_place_params.yaml 의"
echo "     yolo_model 을 본인 모델(.pt) 절대경로로 수정"
echo "  3) export QT_QPA_PLATFORM=xcb"
echo "     ros2 launch dsr_realsense_pick_place pick_place.launch.py mode:=real host:=<로봇IP>"
