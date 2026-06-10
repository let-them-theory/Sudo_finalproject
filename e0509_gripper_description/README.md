# E0509 + RH-P12-RN-A Gripper Description

Doosan E0509 로봇팔과 ROBOTIS RH-P12-RN-A 그리퍼를 결합한 ROS2 패키지

## 개요

이 패키지는 Doosan E0509 6축 로봇팔에 ROBOTIS RH-P12-RN-A 그리퍼를 장착한 통합 로봇 시스템을 위한 URDF, launch 파일, 그리퍼 컨트롤러를 제공합니다.

## 관련 레포지토리

| 레포지토리 | 설명 |
|-----------|------|
| **[sim2real](https://github.com/fhekwn549/sim2real)** | Sim2Real 실행 코드 (펜 감지, 캘리브레이션, 정책 실행) |
| **[CoWriteBotRL](https://github.com/KERNEL3-2/CoWriteBotRL)** | Isaac Lab 기반 강화학습 환경 및 학습 스크립트 |

## 특징

- E0509 + 그리퍼 결합 URDF/XACRO
- Doosan Virtual Robot (에뮬레이터) 지원
- **실제 로봇 그리퍼 제어** (Tool Flange Serial + Modbus RTU)
- **ROS2 서비스/토픽 기반 그리퍼 제어** (C++ gripper_service_node)
- **cuRobo GPU 가속 모션 플래닝** (~80ms 경로 생성)
- **RealSense depth 기반 비전 → 로봇 제어 파이프라인**
- ros2_control 기반 조인트 제어
- C++ 실시간 노드 (gripper_service_node, gripper_joint_publisher, gazebo_bridge)
- RViz 시각화
- Gazebo 시뮬레이션 지원
- Digital Twin (실제 로봇 + RViz + Isaac Sim 동기화)

## 의존성

- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- [doosan-robot2 (Fork)](https://github.com/fhekwn549/doosan-robot2) - Flange Serial 서비스 지원 포함
- [RH-P12-RN-A](https://github.com/ROBOTIS-GIT/RH-P12-RN-A)

> **중요**: 공식 doosan-robot2가 아닌 **포크한 레포**를 사용해야 합니다. 포크 버전에 그리퍼 제어를 위한 Flange Serial 서비스가 포함되어 있습니다.

---

## 설치

### 1. 워크스페이스 생성 및 패키지 클론
```bash
mkdir -p ~/doosan_ws/src
cd ~/doosan_ws/src

# 의존 패키지 클론 (포크한 doosan-robot2 사용)
git clone -b humble https://github.com/fhekwn549/doosan-robot2.git
git clone https://github.com/ROBOTIS-GIT/RH-P12-RN-A.git

# 이 패키지 클론
git clone https://github.com/fhekwn549/e0509_gripper_description.git
```

### 2. 의존성 설치 및 빌드
```bash
cd ~/doosan_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. 환경 설정

`~/.bashrc`에 다음 내용 추가:
```bash
export ROS_DISTRO='humble'
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/doosan_ws/install/rh_p12_rn_a_description/share:~/doosan_ws/install/dsr_description2/share
```

적용:
```bash
source ~/.bashrc
```

---

## Docker 설치 (Virtual Mode 필수)

Virtual mode 에뮬레이터 사용을 위해 Docker가 필요합니다.

### 1. Docker 설치
```bash
# Docker 공식 설치 (https://docs.docker.com/engine/install/ubuntu/)
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 현재 사용자를 docker 그룹에 추가 (재로그인 필요)
sudo usermod -aG docker $USER
```

### 2. 에뮬레이터 설치
```bash
cd ~/doosan_ws/src/doosan-robot2
chmod +x ./install_emulator.sh
sudo ./install_emulator.sh
```

---

## 사용법 (RViz + Virtual Robot)

### 1. RViz 시각화 (조인트 슬라이더)
```bash
ros2 launch e0509_gripper_description display.launch.py
```

### 2. Virtual Robot 실행 (에뮬레이터)
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=virtual
```

### 3. 로봇 제어
```bash
# 조인트 이동
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [30.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

# 홈 위치
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 30.0}"
```

### 4. 그리퍼 제어
```bash
# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# Stroke 값으로 제어 (0=열림, 700=완전히 닫힘)
ros2 topic pub /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 350}" --once
```

---

## 사용법 (실제 로봇 + 그리퍼)

실제 Doosan E0509 로봇에 RH-P12-RN-A 그리퍼를 연결하여 제어합니다.
그리퍼는 로봇의 **Tool Flange Serial** 포트를 통해 **Modbus RTU** 프로토콜로 통신합니다.

### 1. 실제 로봇 실행
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>
```

### 2. 로봇팔 제어 (MoveJoint 서비스)
**주의: Doosan 로봇은 도(degree) 단위를 사용합니다.**
```bash
# 조인트 이동 (joint_1~6, 단위: 도)
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

# 홈 위치로 이동
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel: 30.0, acc: 30.0}"
```

### 3. 그리퍼 제어 (ROS2 서비스/토픽)

bringup.launch.py 실행 시 그리퍼 서비스 노드가 자동으로 시작됩니다.

```bash
# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# 특정 위치로 이동 (0=열림, 700=닫힘)
ros2 topic pub /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 350}" --once
```

### 4. 그리퍼 통신 사양
| 항목 | 값 |
|------|-----|
| 프로토콜 | Modbus RTU |
| 통신 포트 | Tool Flange Serial |
| Baudrate | 57600 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Slave ID | 1 |

### 5. 주요 Modbus 레지스터
| 레지스터 | 주소 | 설명 |
|---------|------|------|
| Torque Enable | 256 (0x0100) | 1=활성화 |
| Goal Position | 282 (0x011A) | 0~700 (2 registers) |
| Goal Current | 275 (0x0113) | 기본값 400 |

---

## 사용법 (Gazebo 시뮬레이션)

### 1. Gazebo 시각화 실행
이 모드에서는 ros2_control 토픽으로만 제어 가능합니다. (Doosan 서비스 사용 불가)
```bash
ros2 launch e0509_gripper_description gazebo.launch.py
```

### 2. 로봇 제어
```bash
ros2 topic pub --once /e0509_gripper/joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  points: [{positions: [0.5, 0.3, 0.3, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}]
}"
```

### 3. 그리퍼 제어
```bash
# 그리퍼 열기
ros2 topic pub --once /e0509_gripper/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"

# 그리퍼 닫기
ros2 topic pub --once /e0509_gripper/gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.1, 1.1, 1.1, 1.1]}"
```

### 4. Gazebo + Virtual Robot 실행
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01
```

RViz 함께 실행:
```bash
ros2 launch e0509_gripper_description bringup_gazebo.launch.py mode:=virtual host:=127.0.0.1 port:=12346 name:=dsr01 gui:=true
```

---

## 그리퍼 제어 인터페이스

### RViz + Virtual Robot + Real Robot
| 인터페이스 | 타입 | 설명 |
|-----------|------|------|
| `/dsr01/gripper/open` | Service (Trigger) | 그리퍼 열기 |
| `/dsr01/gripper/close` | Service (Trigger) | 그리퍼 닫기 |
| `/dsr01/gripper/position_cmd` | Topic (Int32) | 위치 명령 (0~700) |
| `/dsr01/gripper/stroke` | Topic (Int32) | 현재 stroke 발행 (RViz용) |

### Gazebo Simulation
| 인터페이스 | 타입 | 설명 |
|-----------|------|------|
| `/dsr01/gripper_controller/commands` | Topic (Float64MultiArray) | Joint position (0.0~1.1) |

---

## Digital Twin (실제 로봇 + Gazebo 동기화)

**실제 로봇**과 **Gazebo**를 동기화하여 디지털 트윈을 구현합니다.
실제 로봇을 제어하면 Gazebo의 로봇이 동일하게 따라갑니다.

### 특징
- 실제 로봇 <-> Gazebo 실시간 동기화
- 로봇팔 + 그리퍼 모두 지원 (10축)
- 한 번의 launch로 모든 것 실행

### 실행 방법

```bash
ros2 launch e0509_gripper_description bringup_real_gazebo.launch.py mode:=real host:=<robot_ip> rviz:=false
```

> RViz도 함께 보려면 `rviz:=true` (기본값)

### 로봇 제어

```bash
# 조인트 이동 (단위: degree)
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0, time: 0.0, radius: 0.0, mode: 0, blend_type: 0, sync_type: 0}"

# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger
```

### 동작 원리
```
[실제 로봇] → [/dsr01/joint_states] → [gazebo_bridge.py] → [Gazebo Controllers]
```

### Launch 파라미터
| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `host` | 192.168.137.100 | 로봇 IP 주소 |
| `mode` | real | real 또는 virtual |
| `rviz` | true | RViz 실행 여부 |
| `gazebo_ns` | gz | Gazebo 네임스페이스 |

---

## Digital Twin (실제 로봇 + Isaac Sim + RViz 동기화)

**실제 로봇**, **RViz**, **Isaac Sim** 세 곳에서 로봇을 동시에 동기화하여 제어합니다.
로봇팔(6축)과 그리퍼(4축)가 모두 동기화됩니다.

### 특징
- 실제 로봇 <-> RViz <-> Isaac Sim 실시간 동기화
- 로봇팔 + 그리퍼 모두 지원 (10축)
- ROS2와 Isaac Sim의 Python 버전이 달라도 동작 (파일 기반 통신)

### 전체 실행 방법

**터미널 1: 실제 로봇 Bringup**
```bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>
```
> Virtual 모드의 경우: `mode:=virtual`

**터미널 2: ROS2 Bridge 실행**
```bash
cd ~/IsaacLab/pen_grasp_rl/scripts
python3 digital_twin_bridge.py
```
> Bridge는 arm + gripper 조인트를 누적 저장하여 10축 모두 동기화합니다.

**터미널 3: Isaac Sim 디지털 트윈** (CoWriteBotRL 레포 필요)
[CoWriteBotRL](https://github.com/KERNEL3-2/CoWriteBotRL)
```bash
source ~/isaacsim_env/bin/activate
cd ~/IsaacLab
python pen_grasp_rl/scripts/digital_twin.py
```

**터미널 4: 로봇 제어 테스트**
```bash
# 로봇팔 이동 (도 단위)
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}"

# 그리퍼 열기
ros2 service call /dsr01/gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /dsr01/gripper/close std_srvs/srv/Trigger

# 그리퍼 특정 위치 (0=열림, 700=닫힘)
ros2 topic pub /dsr01/gripper/position_cmd std_msgs/msg/Int32 "{data: 350}" --once
```

### 동작 원리
```
[실제 로봇/에뮬레이터] → [ROS2 joint_states 토픽]
                              ↓
                    [digital_twin_bridge.py]
                              ↓
                    [/tmp/doosan_joint_states.json]
                              ↓
                    [digital_twin.py (Isaac Sim)]
```

---

## cuRobo 비전 기반 로봇 제어

cuRobo (NVIDIA GPU 가속 모션 플래너) + Grounding DINO (제로샷 물체 인식) + RealSense depth로
물체를 인식하고 충돌 회피 경로를 생성하여 자동으로 물체를 잡습니다.

### 시스템 구성
```
[RealSense D455F] → Grounding DINO (제로샷 물체 인식)
        ↓                    ↓
[depth 기반 3D 좌표]   [3D PCA → 물체 방향]
        ↓                    ↓
[캘리브레이션 변환] → 로봇 좌표 + 그리퍼 각도
        ↓
[cuRobo Planner] → GPU 가속 충돌 회피 경로 (~70ms)
        ↓
[Doosan E0509] → 이동 + Pick (접근→열기→하강→잡기→들기)
```

### 사전 준비

1. **CUDA Toolkit 12.8 + cuRobo 설치**
```bash
sudo apt install cuda-toolkit-12-8
cd ~ && git clone https://github.com/NVlabs/curobo.git && cd curobo
pip install ninja && export CUDA_HOME=/usr/local/cuda-12.8 && pip install -e ".[dev]"
```

2. **Grounding DINO 설치**
```bash
pip install groundingdino-py
pip install transformers==4.40.2  # 호환성
```

3. **모델 다운로드**
```bash
mkdir -p ~/models && cd ~/models
wget -O groundingdino_swint_ogc.pth "https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth"
```

4. **캘리브레이션 완료** (아래 Eye-to-Hand Calibration 섹션 참조)

### 실행 방법

**터미널 1: 로봇 Bringup**
```bash
ros2 launch e0509_gripper_description bringup.launch.py \
    mode:=real host:=<로봇IP> rt_host:=<로봇IP>
```

**터미널 2: cuRobo + 물체 인식 (한 번에 실행)**
```bash
ros2 launch e0509_gripper_description curobo_vision.launch.py
```
> cuRobo planner가 먼저 로딩되고 (JIT 컴파일 ~30초), 이후 Grounding DINO가 시작됩니다.

**프롬프트 변경:**
```bash
ros2 launch e0509_gripper_description curobo_vision.launch.py \
    prompt:="cup . bottle . pen"
```

### 조작법
| 키 | 동작 |
|----|------|
| `1-9` | 감지된 물체 선택 + 잠금 (감지 멈춤, 주변 물체를 cuRobo 장애물로 등록) |
| `r` | 잠금 해제 (감지 재개) |
| `s` | 선택된 물체 위 15cm로 이동 (cuRobo 충돌 회피) |
| `p` | Pick 시퀀스: 그리퍼 열기 → 하강 (cuRobo) → 잡기 → 들기 |
| `w/x` | 그리퍼 각도 미세 조정 (±5°) |
| `q` | 종료 |

### Pick 시퀀스 상세
```
's' → cuRobo 접근 (물체 위 15cm, 주변 장애물 회피, 물체 방향에 맞게 그리퍼 회전)
'p' →
  1. 그리퍼 열기
  2. cuRobo 하강 (물체 위 12cm, 장애물 회피)
  3. 그리퍼 닫기
  4. movel 직선 위로 들기 (15cm)
```

### 물체 방향 자동 감지
- RealSense depth point cloud에서 **3D PCA**로 물체 장축 방향을 계산
- 장축에 **수직으로 그리퍼를 자동 회전**하여 잡기 자세 결정
- 카메라 각도와 무관하게 **로봇 XY 평면에서의 실제 물체 방향** 사용

### 주의사항
- 재시작 시 이전 세션 종료 후 **10초 대기** 필요 (RT 포트 해제 대기)
- 감지된 물체 중 **잠금한 물체를 제외한 나머지**가 cuRobo 장애물로 등록됨
- GPU 메모리 약 4GB 필요 (cuRobo ~2GB + Grounding DINO ~2GB)

---

## Eye-to-Hand Calibration

카메라가 외부에 고정된 Eye-to-Hand 구조에서 카메라 → 로봇 베이스 변환 행렬을 계산합니다.

> **참고**: 캘리브레이션 및 Sim2Real 실행 코드는 [sim2real 레포](https://github.com/fhekwn549/sim2real)로 이동했습니다.
> 자세한 가이드는 [SIM2REAL_GUIDE.md](https://github.com/fhekwn549/sim2real/blob/main/sim2real/SIM2REAL_GUIDE.md)를 참조하세요.

### 구조
```
[카메라 (고정)] ──(T_cam2base)──> [로봇 베이스]
                                      ↑
                              [TCP/그리퍼]
```

### 간단 실행
```bash
# 터미널 1: 로봇 연결
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<robot_ip>

# 터미널 2: 캘리브레이션 (sim2real 레포)
cd ~/sim2real/sim2real
python3 calibrate_eye_to_hand.py
```

---

## 파일 구조
```
e0509_gripper_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── gz_controllers.yaml          # Gazebo 컨트롤러 설정
├── urdf/
│   └── e0509_with_gripper.urdf.xacro
├── launch/
│   ├── display.launch.py            # RViz 시각화
│   ├── bringup.launch.py            # Virtual/Real 로봇 실행
│   ├── bringup_gazebo.launch.py     # Gazebo + RViz 시뮬레이션
│   ├── bringup_real_gazebo.launch.py # 실제로봇 + Gazebo 디지털트윈
│   └── gazebo.launch.py             # Gazebo 전용
├── include/
│   └── e0509_gripper_description/
│       └── modbus_rtu.hpp           # Modbus RTU 프로토콜 (CRC16, 프레임 생성)
├── src/
│   ├── gripper_service_node.cpp     # 그리퍼 ROS2 서비스 노드 (C++)
│   ├── gripper_joint_publisher.cpp  # 통합 조인트 상태 발행 (C++, 50Hz)
│   └── gazebo_bridge.cpp            # Gazebo 디지털트윈 브릿지 (C++, 50Hz)
├── config/
│   ├── gz_controllers.yaml          # Gazebo 컨트롤러 설정
│   └── curobo/
│       ├── e0509_gripper.urdf       # cuRobo용 클린 URDF
│       ├── e0509_gripper.yml        # cuRobo 로봇 설정
│       └── e0509_spheres.yml        # 충돌 구체 정의
├── scripts/
│   ├── curobo_planner_node.py       # cuRobo GPU 모션 플래너 노드
│   ├── gripper.py                   # 실제 그리퍼 제어 CLI (Modbus RTU)
│   ├── digital_twin_bridge.py       # Isaac Sim 연동용 ROS2 브릿지
│   └── robot_slider_control.py      # TCP 슬라이더 제어 GUI
└── rviz/
    └── display.rviz
```

> **Sim2Real 코드**: 캘리브레이션, 펜 감지, 정책 실행 코드는 [sim2real 레포](https://github.com/fhekwn549/sim2real)를 참조하세요.

## 환경

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Fortress (Ignition Gazebo 6)

## License

Apache-2.0

## Author

fhekwn549
