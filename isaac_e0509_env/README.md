# Isaac E0509 + Isaac Lab

Doosan **E0509 + RH-P12** gripper를 **Isaac Sim / Isaac Lab**에서 쓰기 위한 에셋·확장 패키지입니다.  
ROS2 `pick_place` 스택과 **분리**되어 있으며, RL은 `PRE_PICK` / `PICK` 단계만 담당하는 것을 전제로 합니다.

## 구조

```
isaac_e0509_env/
├── assets/urdf/          # prepare_urdf.py 출력 (Isaac용 절대 mesh 경로)
├── assets/usd/           # URDF→USD 변환 캐시 (선택)
├── scripts/
│   ├── prepare_urdf.py
│   └── install_to_isaaclab.sh
└── isaaclab_ext/         # Isaac Lab extension (심볼릭 링크로 설치)
    └── isaac_e0509_pick_place/
        ├── robots/e0509.py
        ├── tasks/reach/  # Isaac-Reach-E0509-v0
        └── scripts/smoke_env.py
```

## 사전 요구

| 항목 | 버전 |
|------|------|
| Ubuntu | 22.04 |
| Isaac Sim | 5.0 / 5.1 (Isaac Lab `main` 기준) |
| Isaac Lab | `~/IsaacLab` 클론 + `./isaaclab.sh -i` |
| ROS 워크스페이스 | `sudo_ws` (URDF mesh 소스) |

Isaac Lab Python은 **ROS Humble Python과 별도**입니다. `source /opt/ros/humble/setup.bash`와 섞지 마세요.

```bash
conda activate env_isaaclab   # Isaac Lab 설치 시 생성된 환경
```

## 1. URDF 준비

`package://` mesh 경로를 Isaac Sim이 읽을 수 있는 절대 경로로 변환합니다.

```bash
cd ~/sudo_ws/src/isaac_e0509_env
python3 scripts/prepare_urdf.py
# → assets/urdf/e0509_gripper_isaac.urdf
```

소스 URDF: `e0509_gripper_description/config/curobo/e0509_gripper.urdf`

## 2. Isaac Lab extension 설치

```bash
chmod +x scripts/install_to_isaaclab.sh
./scripts/install_to_isaaclab.sh
```

동작:
1. URDF 준비
2. `~/IsaacLab/source/isaac_e0509_pick_place` → `isaaclab_ext` 심볼릭 링크
3. `~/IsaacLab/./isaaclab.sh -i none` 으로 extension pip install

## Isaac Sim 실행

**주의:** `./isaaclab.sh -s` (전체 Isaac Sim)는 extension 수천 개 + inotify 한도 때문에 **segfault(139)** 날 수 있습니다. 아래 방법을 쓰세요.

### 0) 사전 점검 + inotify 수정 (GUI 필수, 한 번만)

```bash
bash ~/sudo_ws/src/isaac_e0509_env/scripts/preflight_isaac.sh
sudo bash ~/sudo_ws/src/isaac_e0509_env/scripts/fix_inotify.sh
```

`errno=28 / No space left on device` = **디스크 부족 아님**, inotify watch 한도 초과입니다.

### Isaac Sim GUI 열기

```bash
~/sudo_ws/src/isaac_e0509_env/scripts/run_isaac_sim_gui.sh
```

### E0509 스모크 테스트

```bash
~/sudo_ws/src/isaac_e0509_env/scripts/run_smoke_test.sh          # headless (안정)
HEADLESS=0 ~/sudo_ws/src/isaac_e0509_env/scripts/run_smoke_test.sh  # GUI
```

## 3. 스모크 테스트 (상세)

```bash
chmod +x ~/sudo_ws/src/isaac_e0509_env/scripts/run_smoke_test.sh
~/sudo_ws/src/isaac_e0509_env/scripts/run_smoke_test.sh
```

기본은 **headless** (안정적). GUI로 보려면:

```bash
HEADLESS=0 ~/sudo_ws/src/isaac_e0509_env/scripts/run_smoke_test.sh
```

GUI에서 **Segmentation fault (RTX)** 가 나면 inotify 한도를 올리세요 (디스크 부족이 아님):

```bash
sudo sysctl -w fs.inotify.max_user_watches=524288
sudo sysctl -w fs.inotify.max_user_instances=512
```

수동 실행:

```bash
conda activate env_isaaclab
cd ~/IsaacLab
TERM=xterm ./isaaclab.sh -p source/isaac_e0509_pick_place/isaac_e0509_pick_place/scripts/smoke_env.py --headless
```

등록된 Gym 환경:
- `Isaac-Reach-E0509-v0` — 학습용 (64 env)
- `Isaac-Reach-E0509-Play-v0` — 시각화/검증용 (16 env)

## 4. Reach RL 학습 (워밍업)

Franka Reach와 동일한 MDP(`isaaclab_tasks` reach mdp)를 재사용합니다. E0509 작업 영역에 맞게 목표 pose 범위를 조정했습니다.

```bash
cd ~/IsaacLab
./isaaclab.sh -p source/isaaclab_rl/isaaclab_rl/runners/rsl_rl_train.py \
  --task Isaac-Reach-E0509-v0 --num_envs 64 --headless
```

또는:

```bash
bash ~/sudo_ws/src/isaac_e0509_env/isaaclab_ext/isaac_e0509_pick_place/scripts/train_reach_rsl_rl.sh
```

## 실제 pick_place와의 정렬

| 항목 | ROS (`pick_place_params.yaml`) | Isaac Lab env |
|------|-------------------------------|---------------|
| Home joints | `[0,0,90,0,90,0]` deg | `joint_3/5 = π/2` |
| Workspace X | 0.15 – 1.20 m | reach command 0.25 – 0.55 m (1차) |
| Table Z | 객체 ~0.5 m | command Z 0.40 – 0.55 m |

EE body: `link_6` (URDF import 시 gripper base가 link_6에 merge됨)  
스모크 테스트 후 `commands.ee_pose.ranges.pitch` 는 실제 그리퍼 자세에 맞게 조정하세요.

## Digital Twin (선택)

ROS joint_states → Isaac 동기화는 기존 브릿지를 그대로 사용할 수 있습니다.

```bash
# 터미널 1: ROS
source /opt/ros/humble/setup.bash && source ~/sudo_ws/install/setup.bash
python3 ~/sudo_ws/src/e0509_gripper_description/scripts/digital_twin_bridge.py

# 터미널 2: Isaac (CoWriteBotRL digital_twin.py 패턴 참고)
```

참고: [CoWriteBotRL](https://github.com/KERNEL3-2/CoWriteBotRL)

## 다음 단계 (RL grasp)

1. `Isaac-Reach-E0509-v0`로 approach 정책 학습
2. 테이블 위 `RigidObject` + contact sensor 추가 → pick env
3. YOLO 검출 pose를 **residual action** 으로 주입 (detector는 ROS 유지)
4. (선택) URDF → USD export 후 `UsdFileCfg`로 로드 속도 개선

## 트러블슈팅

| 증상 | 조치 |
|------|------|
| mesh not found | `prepare_urdf.py` 재실행, `doosan-robot2` / `RH-P12-RN-A` 경로 확인 |
| extension import 실패 | `install_to_isaaclab.sh` 재실행 |
| 로봇이 이상한 자세 | `e0509_reach_env_cfg.py` 의 `pitch` / `EE_BODY` 조정 |
| ROS와 Python 충돌 | Isaac Lab 전용 터미널 사용 (`isaaclab.sh -p`) |
