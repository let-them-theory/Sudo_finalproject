# SETUP — Doosan E0509 + RH-P12-RN Pick & Place (실행용)

> 이 repo는 **런타임 소스만** 담고 있습니다. 바로 클론해서 실행되지 않으며,
> 아래 **외부 의존성 설치 + 설정(모델 경로·로봇 IP)** 이 필요합니다.

## ⚡ 빠른 시작 (TL;DR)
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone git@github.com:let-them-theory/Sudo_finalproject.git src/Sudo_finalproject
bash src/Sudo_finalproject/setup.sh          # 외부 deps 설치 + 빌드 (1회)
source install/setup.bash
# 모델은 패키지 내장(models/proto_v3.pt) — 기본값이면 수정 불필요
ros2 launch dsr_realsense_pick_place pick_place.launch.py mode:=real host:=<로봇IP>
```
> **외부 ROS 패키지(Doosan 스택·realsense)는 repo에 없어** `setup.sh`가 자동 설치합니다.
> 그 1회 셋업 후에는 **IP(launch host:=)와 모델(yaml yolo_model) 두 가지만** 설정하면 됩니다.
> 아래는 각 단계 상세.

## 0. 환경
- Ubuntu 22.04 + **ROS 2 Humble**
- Doosan E0509 협동로봇 + RH-P12-RN(-A) 그리퍼(플랜지 RS-485) + Intel RealSense D4xx
- (검출/추론) NVIDIA GPU 권장

## 1. 워크스페이스 + repo 클론
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone git@github.com:let-them-theory/Sudo_finalproject.git
```
이 repo의 메인 패키지 `dsr_realsense_pick_place`(+ `dsr_gripper_tcp`, `dsr_gripper_tcp_interfaces`)를 빌드합니다. 루트 `package.xml` 때문에 colcon이 하위 패키지를 자동 탐색하지 못하므로 빌드 시 `--paths` 명시가 필요합니다(아래 4).

## 2. 외부 ROS 패키지 (repo에 없음 — 반드시 설치)
### 2-1. Doosan ROS2 스택 (필수: `dsr_msgs2`, `dsr_bringup2`, `dsr_controller2`, `dsr_hardware2`)
launch가 `dsr_bringup2`를 include하고, 그리퍼 브릿지가 `dsr_msgs2`(DrlStart/DrlStop/SetRobotMode 등)를 import합니다.
```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git
# (doosan-robot2의 README에 따라 emulator/의존 패키지도 설치)
```
### 2-2. RealSense
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
# librealsense2(udev 포함)는 Intel 공식 가이드대로 설치
```
### 2-3. 기타 ROS deps (rosdep)
```bash
cd ~/ros2_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
# (cv_bridge, tf_transformations, message_filters, tf2_geometry_msgs 등 자동 설치)
```

## 3. Python 의존성 (검출/추론)
```bash
pip install -r ~/ros2_ws/src/Sudo_finalproject/requirements.txt
# ultralytics(YOLO), torch/torchvision 등. GPU면 CUDA 빌드 torch 설치 권장.
```

## 4. 빌드
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash   # 두산 스택 먼저 빌드/소스
# 루트 package.xml이 하위 탐색을 막으므로 --paths 필수. --symlink-install은 쓰지 말 것
# (모델/yaml 갱신이 install에 반영 안 되는 stale 문제 발생).
colcon build --paths \
  src/Sudo_finalproject/dsr_gripper_tcp_interfaces \
  src/Sudo_finalproject/dsr_gripper_tcp \
  src/Sudo_finalproject
source install/setup.bash      # 매 터미널에서 source (또는 ~/.bashrc 에 추가)
```

## 5. 설정 (실행 전 반드시)
`config/pick_place_params.yaml` 편집:
- **`yolo_model`** — 패키지 내장 모델을 가리킴(`models/proto_v3.pt`, repo에 포함 — 재현성). 다른 모델 쓸 때만 `models/` 기준 경로로 교체.
  ```yaml
  yolo_model: "models/proto_v3.pt"
  ```
- **캘리브레이션** — `absolute_calib_*_mm`, `absolute_origin_in_camera_*` 를 현장 카메라 위치에 맞게.
  (object_detector는 이 값을 **launch 시점에만** 읽음 → 수정 후 재launch 또는 GUI "캘리브레이션 적용".)
- **그리퍼 파지 전류** — `grip_class_currents`(클래스별 mA), `grasp_detect_current`(감지 임계, 기본 100).

## 6. 실행
```bash
export QT_QPA_PLATFORM=xcb        # Wayland에서 Qt GUI 안 뜰 때
# 권장: 종료(Ctrl+C) 시 DRCF/DRL 자동 해제
bash $(ros2 pkg prefix dsr_realsense_pick_place)/share/dsr_realsense_pick_place/scripts/run_pick_place_real.sh

# 또는 직접 launch (종료 시 shutdown_nodes.sh 수동 실행 필요)
ros2 launch dsr_realsense_pick_place pick_place.launch.py \
  mode:=real host:=<로봇IP> use_realsense:=true gui:=true
```
주요 launch 인자: `mode`(virtual/real), `host`(로봇 IP, 기본 110.120.1.50), `gui`, `use_realsense`, `gripper_tcp_port`(20002).

## 7. 그리퍼 status3 복구 (통신 무응답 시)
GUI **"그리퍼 브릿지 재시작"** 버튼, 또는:
```bash
source ~/ros2_ws/install/setup.bash      # ros2 pkg prefix로 경로 동적 산출 → ws sourced 필수
bash $(ros2 pkg prefix dsr_realsense_pick_place)/share/dsr_realsense_pick_place/scripts/restart_gripper_bridge.sh <로봇IP>
```
> GUI "그리퍼 리셋"(in-process reinit)은 status3에 효과 없음 — 위 fresh 재시작 사용.
> 자세한 메커니즘은 (별도) status3 분석 보고서 참고.

## 8. 종료
```bash
bash $(ros2 pkg prefix dsr_realsense_pick_place)/share/dsr_realsense_pick_place/scripts/shutdown_nodes.sh --kill-launch
```
순서: DrlStop → gripper 정리 → ros2_control(DRCF 해제). `pkill -9` 대신 이 스크립트를 사용하면 재연결 시 로봇 전원 사이클이 대부분 불필요합니다.

---
### repo 포함 / 제외
- 포함: 모델 가중치 `models/*.pt`(proto_v3, FastSAM-s) + 키오스크 빌드물 `web_kiosk/frontend/dist` → 재현성(clone 후 바로 실행)
- 제외: 학습 데이터셋 / 학습 스크립트, build/install/log(빌드 산물), 두산 스택(별도 clone)
