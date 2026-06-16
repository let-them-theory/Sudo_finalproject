# SUDO

> 고객 공간과 로봇 작업 공간을 물리적으로 분리한 도심형 마이크로 풀필먼트 센터(MFC) 기반 무인 자동화 시스템

SUDO는 단순한 무인 판매기가 아닙니다. 고객은 상품 진열 공간에 직접 접근하지 않고, 앱·키오스크 주문 인터페이스를 통해 상품을 주문합니다. 중앙 제어 시스템(Main)이 주문을 처리하고, 로봇이 상품을 피킹하여 스마트 라커로 전달하면, 고객 또는 배달 라이더가 PIN/QR 코드로 상품을 수령합니다.

---

## 1. 핵심 설계 원칙

SUDO는 **중앙 집중식 제어** 구조입니다. 모든 판단은 Main에서만 이루어지며, 각 노드의 책임이 명확하게 분리되어 있습니다.

| 주체 | 성격 | 역할 |
|------|------|------|
| **UI** | 입출력 | 주문 요청과 상태 표시만 담당 (판단 기능 없음) |
| **Main** | 판단 | 주문 처리, 재고 판단, 작업 상태 관리, DB 접근, 예외 처리 |
| **Robot** | 실행 | Main이 내린 작업 명령만 수행 |

**불변 규칙**

- 모든 판단은 Main에서만 이루어진다.
- DB 접근은 Main만 허용한다. (UI·Robot 직접 접근 금지)
- UI ↔ Robot 직접 통신을 금지한다. 모든 정보는 Main을 거친다.
- Robot은 Main의 명령 없이 동작하지 않으며, 자의적으로 작업 순서를 바꾸지 않는다.

**흐름 방향**

- 명령 흐름: `UI → Main → Robot → Smart Locker`
- 상태 흐름: `Smart Locker → Robot → Main → UI`

---

## 2. 시스템 구성

### 2.1 전체 시스템 흐름도

```mermaid
flowchart TD
    A[고객 / 배달 라이더<br/>주문 · 결제] -->|주문 요청| B[UI Node<br/>앱 / 키오스크 / 주문 인터페이스]
    B -->|order_request| C[Main Node<br/>주문 수신 / 재고 확인 / 큐 관리 / 상태 제어]
    C <-->|조회 / 갱신| D[(SQLite DB<br/>상품 / 재고 / 좌표 / 주문 / 로그)]
    C -->|robot_task<br/>작업 명령| E[Robot Node<br/>피킹 / 이동 / 적재 수행]
    E -->|상품 전달| F[Smart Locker<br/>PIN / QR 기반 수령]
    F -->|수령| A
    E -->|작업 상태 / 에러 보고| C
    C -->|주문 상태 / 완료 정보| B
```

### 2.2 모듈 구조

```
Customer
   ↓
UI Node
   ↓
Main Node ↔ SQLite DB
   ↓
Robot Node
   ↓
Smart Locker
   ↓
Customer
```

### 2.3 핵심 노드 (초기 MVP 기준)

| 노드 | 책임 |
|------|------|
| `ui_node` | 주문 입력, 주문 진행 상태 표시, 품절/실패/완료 메시지 표시, PIN/QR 출력 |
| `main_node` | 주문 수신·검증, 주문 큐 관리, 재고 확인, DB 접근, 상태 머신, 로봇 명령 생성, 완료 처리, 로그 기록 |
| `robot_node` | 상품 위치 이동, 피킹, 위치 보정, 라커 적재, 작업 결과 보고 |

### 2.4 robot_node 내부 모듈

```
robot_node
 ├ motion_control          # 이동 제어
 ├ gripper_control         # 그리퍼 제어
 ├ vision_correction       # 카메라 / SAM3 / GraspNet / 좌표 보정
 ├ task_executor           # 작업 단계 실행
 └ hardware_status_monitor # 모터·토크·충돌 감시
```

초기 MVP에서는 비전을 `robot_node` 내부 기능으로 둡니다. 이후 비전 처리량이 커지거나 독립적인 모델 관리가 필요해지면 별도 `vision_node`로 분리할 수 있습니다.

---

## 3. 전체 서비스 흐름 (정상 흐름)

### 3.1 순차 다이어그램

```mermaid
sequenceDiagram
    autonumber
    actor C as 고객/라이더
    participant U as UI Node
    participant M as Main Node
    participant R as Robot Node
    participant L as Smart Locker

    C->>U: 주문 · 결제
    U->>M: order_request
    M->>M: 주문 검증 (형식, product_id)
    M->>M: 재고 확인 · reserved +1 (소프트 락)
    M->>M: 좌표 조회 · 라커 할당 (EMPTY→RESERVED)
    M->>R: robot_task (Action Goal)

    R-->>M: feedback: MOVING_TO_PICK
    M-->>U: order_status: PROCESSING
    Note over R: 선반으로 이동

    Note over R: Vision — 촬영 / SAM3 / GraspNet / 좌표 보정
    R-->>M: feedback: GRASPING
    M-->>U: order_status: PICKING
    Note over R: 파지 시도 (토크 감시)
    Note over R: 파지 검증 (토크 + 비전 이중 확인)

    R-->>M: feedback: MOVING_TO_LOCKER
    M-->>U: order_status: PACKING
    Note over R: 라커로 이동 (낙하 감시)
    Note over R: Vision — 라커 내부 확인 / 적재 / 재검증
    R->>L: 상품 적재

    R->>M: result: success=true
    M->>M: 재고 확정 차감 · 라커 OCCUPIED
    M->>M: PIN/QR 생성 · 주문 READY_FOR_PICKUP
    M-->>U: order_status: READY_FOR_PICKUP (PIN/QR, locker_id)
    U-->>C: PIN/QR 표시

    C->>L: PIN/QR 입력
    L->>M: PIN 검증 요청
    M->>L: 인증 성공 → 해당 라커 개방
    L-->>C: 상품 수령
    M->>M: 주문 DONE · 라커 EMPTY
```

### 3.2 단계별 상세

| STEP | 담당 | 내용 |
|------|------|------|
| 1 | 고객 → UI | 상품 선택 및 결제 완료 |
| 2 | Main | 주문 검증 — product_id 존재 여부, quantity(1 이상), request_source 유효성 |
| 3 | Main ↔ DB | 재고 확인 및 예약 — `가용 재고 = stock_count − reserved_count`, 소프트 락 `reserved_count +1` |
| 4 | Main ↔ DB | 작업 생성 — 상품 좌표 조회 + 빈 라커 할당 (EMPTY → RESERVED) |
| 5 | Main → Robot | `/robot_task` Action Goal 전송, 타임아웃 타이머 시작 (기본 5분) |
| 6 | Robot | 선반으로 이동 (MOVING_TO_PICK), 하드웨어 지속 감시 |
| 7 | Robot Vision | 촬영 → SAM3 객체 탐지 → GraspNet 파지 좌표 계산 → 좌표 보정 |
| 8 | Robot | 파지 시도, 토크 센서로 힘 실시간 감지 |
| 9 | Robot | 파지 검증 — 토크 센서 + 비전 재확인 이중 검증 (둘 다 통과해야 성공) |
| 10 | Robot | 라커로 이동 (MOVING_TO_LOCKER), 상품 낙하 감시 |
| 11 | Robot Vision | 라커 내부 확인 → 적재 → 비전 재검증 |
| 12 | Robot → Main | 작업 성공 결과 보고, 홈 포지션 복귀 |
| 13 | Main ↔ DB → UI | 재고 확정 차감 → 라커 OCCUPIED → PIN/QR 생성 → 주문 READY_FOR_PICKUP → UI 완료 통보 |
| 14 | 고객/라이더 | PIN/QR로 라커 수령 → 주문 DONE |

---

## 4. ROS2 통신 인터페이스

| 방향 | 인터페이스 | 방식 | 내용 |
|------|-----------|------|------|
| UI → Main | `/order_request` | Service | 신규 주문 요청 |
| Main → UI | `/order_status` | Topic | 주문 상태 실시간 전달 |
| Main → Robot | `/robot_task` | Action Goal | 작업 명령 (상품·위치·라커·재시도 한도) |
| Robot → Main | `/robot_task/feedback` | Action Feedback | 단계별 진행 상태 보고 |
| Robot → Main | `/robot_task/result` | Action Result | 최종 성공/실패 및 에러 코드 |
| Main ↔ DB | 내부 함수 | SQLite | 재고·좌표·주문·라커·로그 |

로봇 작업은 시간이 걸리는 동작이므로 단순 Topic보다 **Action 구조**를 사용합니다. (작업 시작 요청 / 진행 피드백 / 성공·실패 결과 반환 / 중간 취소 가능)

### 공통 메시지 구조 (초안)

```
OrderRequest      : order_id, product_id, quantity, request_source
OrderStatus       : order_id, status, message, pin_code, locker_id
RobotTask         : task_id, order_id, product_id, quantity,
                    pick_location_id, place_location_id, retry_limit
RobotTaskStatus   : task_id, order_id, status, success, error_code, message
```

---

## 5. 상태 머신

### 5.1 Main 상태 머신

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> ORDER_RECEIVED: 신규 주문 수신
    ORDER_RECEIVED --> STOCK_CHECK: 주문 검증 완료
    ORDER_RECEIVED --> ERROR: 주문 형식 오류
    STOCK_CHECK --> TASK_CREATE: 재고 있음
    STOCK_CHECK --> IDLE: 재고 없음 / 품절 통보
    TASK_CREATE --> TASK_DISPATCH: 작업 생성 완료
    TASK_CREATE --> ERROR: 좌표 누락 / 빈 라커 없음
    TASK_DISPATCH --> WAIT_ROBOT_RESULT: Robot 작업 전송
    TASK_DISPATCH --> ERROR: 전송 실패
    WAIT_ROBOT_RESULT --> COMPLETE: Robot 작업 성공
    WAIT_ROBOT_RESULT --> ERROR: 실패 / 타임아웃(E005)
    COMPLETE --> IDLE: 완료 처리
    COMPLETE --> ERROR: DB 갱신 실패
    ERROR --> IDLE: 에러 기록 및 UI 통보
```

| 상태 | 수행 내용 | 성공 전환 | 실패 전환 |
|------|----------|----------|----------|
| IDLE | 주문 큐 대기 | ORDER_RECEIVED | — |
| ORDER_RECEIVED | 주문 ID 생성, 형식 검증 | STOCK_CHECK | ERROR |
| STOCK_CHECK | DB 재고 확인, 소프트 락 | TASK_CREATE | IDLE (품절) |
| TASK_CREATE | 좌표·라커 조회, 작업 레코드 생성 | TASK_DISPATCH | ERROR |
| TASK_DISPATCH | Robot에 Action Goal 전송 | WAIT_ROBOT_RESULT | ERROR |
| WAIT_ROBOT_RESULT | 결과 대기, 타임아웃 감시 | COMPLETE | ERROR / TIMEOUT |
| COMPLETE | 재고 차감, PIN/QR 생성, UI 완료 통보 | IDLE | ERROR |
| ERROR | 실패 기록, UI 에러 통보, 예약 해제 | IDLE | — |

### 5.2 Robot 상태 머신

```mermaid
stateDiagram-v2
    [*] --> READY
    READY --> MOVE_TO_PICK: 작업 명령 수신
    MOVE_TO_PICK --> POSITION_CORRECTION: 상품 위치 도착
    MOVE_TO_PICK --> FAILED: 이동 실패 / 충돌(E001) / 모터 이상(E006)
    POSITION_CORRECTION --> GRASP: 위치 보정 완료
    POSITION_CORRECTION --> FAILED: Vision 인식 실패(E002)
    GRASP --> VERIFY_GRASP: 파지 시도 완료
    VERIFY_GRASP --> MOVE_TO_LOCKER: 파지 성공
    VERIFY_GRASP --> GRASP: 재시도 (retry_count < retry_limit)
    VERIFY_GRASP --> FAILED: 재시도 소진(E003)
    MOVE_TO_LOCKER --> PLACE: 라커 위치 도착
    MOVE_TO_LOCKER --> FAILED: 이동 실패 / 낙하(E001, E003)
    PLACE --> REPORT_RESULT: 적재 완료
    PLACE --> FAILED: 적재 실패(E004)
    REPORT_RESULT --> READY: Main에 결과 전송
    FAILED --> REPORT_RESULT: 실패 보고
```

| 상태 | 수행 내용 | 실패 조건 |
|------|----------|----------|
| READY | Main 명령 대기 | — |
| MOVE_TO_PICK | 선반 좌표로 이동, 하드웨어 감시 | 이동 실패, 충돌, 모터 이상 |
| POSITION_CORRECTION | 촬영, SAM3, GraspNet, 좌표 보정 | 신뢰도 미달, Depth 이상, 후보 없음 |
| GRASP | 그리퍼 접근 및 파지, 토크 감시 | 과압착, 접근 실패 |
| VERIFY_GRASP | 토크 + 비전 이중 파지 확인 | 토크 미달, 상품 미감지 |
| MOVE_TO_LOCKER | 라커로 이동, 낙하 감시 | 이동 실패, 낙하 감지 |
| PLACE | 라커 내부 확인, 적재, 비전 재검증 | 공간 인식 실패, 적재 후 미감지 |
| REPORT_RESULT | Main에 결과 보고, 홈 복귀 | 통신 실패 |
| FAILED | 실패 기록, 보고 대기 | — |

파지 검증 실패 시 그리퍼를 열고 재시도하며(`retry_count < retry_limit`), 재시도 한도 소진 시 FAILED로 전환합니다.

---

## 6. 예외 처리 및 에러 코드

| 코드 | 원인 | 발생 조건 |
|------|------|----------|
| E001 | 이동 실패 | 경로 이탈, 장애물 충돌, 목표 좌표 도달 불가 |
| E002 | Vision 인식 실패 | SAM3 신뢰도 미달, Depth 데이터 이상, GraspNet 후보 없음 |
| E003 | 파지 실패 | 허공 파지, 토크 임계값 미달, 비전 재확인 실패 (재시도 소진 후) |
| E004 | 라커 적재 실패 | 내부 공간 인식 불가, 적재 후 상품 미감지 |
| E005 | 통신 타임아웃 | WAIT_ROBOT_RESULT 타임아웃 경과 |
| E006 | 하드웨어 이상 | 모터 이상, 센서 오류 |

에러 발생 시 Main 공통 처리: `reserved_count -1`, 라커 할당 해제(RESERVED → EMPTY), 에러 로그 기록, UI에 ERROR 전송, IDLE 복귀. 하드웨어 이상(E006)·이동 실패(E001)는 관리자 확인 로그를 추가로 남깁니다.

별도 예외:

- **품절**: 가용 재고가 0이면 로봇 작업을 생성하지 않고 Main 단계에서 즉시 차단, UI에 OUT_OF_STOCK 전송.
- **PIN 만료/미수령**: Main이 `pin_expires_at`를 주기적으로 감시하여 만료 시 주문 EXPIRED 처리, 라커 EMPTY 전환, 관리자 확인 로그 기록. 라커 내 상품 회수는 관리자 판단에 위임.

---

## 7. 데이터베이스 (SQLite)

DB는 **Main만 접근**합니다. 본 흐름에서 참조되는 주요 테이블·컬럼은 다음과 같습니다.

**products**
`product_id`, `product_name`, `price`, `stock_count`, `reserved_count`(소프트 락), `shelf_id`, `is_available`

**locations**
`location_id`, `shelf_id`, `x, y, z`, `rx, ry, rz`, `description`

**lockers**
`locker_id`, `locker_status`(EMPTY / RESERVED / OCCUPIED), `assigned_order_id`

**orders**
`order_id`, `product_id`, `quantity`, `order_status`(RECEIVED / PROCESSING / PICKING / PACKING / READY_FOR_PICKUP / DONE / EXPIRED / ERROR), `locker_id`, `pin_code`, `pin_expires_at`, `created_at`, `completed_at`

**robot_tasks**
`task_id`, `order_id`, `product_id`, `pick_location_id`, `place_location_id`, `task_status`(CREATED / DISPATCHED / IN_PROGRESS / DONE / FAILED / ERROR), `retry_count`, `error_code`, `created_at`, `completed_at`

---

## 8. 기술 스택

- **ROS2** — 노드 간 통신 (Topic / Service / Action)
- **SQLite** — 상품·재고·좌표·주문·로그 저장
- **Vision** — Depth 카메라(RGB + Depth), SAM3(객체 탐지), GraspNet(파지 좌표 계산)

---

## 9. 개발 로드맵

- [ ] 전체 시스템 흐름도 확정
- [ ] Main FSM 확정
- [ ] Robot FSM 확정
- [ ] ROS2 Topic / Service / Action 확정
- [ ] 공통 메시지 정의 (interfaces 패키지)
- [ ] SQLite DB 스키마 정의
- [ ] 패키지 구조 설계
- [ ] 최소 동작 MVP 코드 작성

---

## 10. Pick & Place 현장 구현 (`dsr_realsense_pick_place`)

Doosan E0509 + RealSense + YOLO/FastSAM + RH-P12-RN 그리퍼 + 아두이노 초음파 센서로 동작하는 ROS 2 Humble 픽앤플레이스 스택입니다. 상세 실행 방법은 [`SETUP.md`](SETUP.md)를 참고하세요.

### 10.1 실행·종료 (권장)

```bash
# 시작 (종료 시 DRCF/DRL 자동 해제)
bash $(ros2 pkg prefix dsr_realsense_pick_place)/share/dsr_realsense_pick_place/scripts/run_pick_place_real.sh

# 직접 launch (Ctrl+C 시 launch_cleanup 자동 실행)
ros2 launch dsr_realsense_pick_place pick_place.launch.py mode:=real

# 수동 종료
bash $(ros2 pkg prefix dsr_realsense_pick_place)/share/dsr_realsense_pick_place/scripts/shutdown_nodes.sh --kill-launch

# Ctrl+C 후 그리퍼가 남았을 때
bash $(ros2 pkg prefix dsr_realsense_pick_place)/share/dsr_realsense_pick_place/scripts/launch_cleanup.sh
pgrep -af 'gripper_service|gripper_node'   # 출력 없으면 정상
```

`pkill -9`로 `ros2_control_node` / 그리퍼를 직접 죽이지 마세요. DRCF authority가 컨트롤러에 남아 **재연결 시 로봇 전원 사이클**이 필요해질 수 있습니다.

### 10.2 변경 이력 (2026-06-08)

#### A. GUI — 아두이노(초음파) 상태 표시

| 문제 | 조치 |
|------|------|
| 아두이노 연결 여부를 GUI에서 확인할 수 없음 | 좌측 상단 상태 바에 **ARD** 노드 추가 (`/ultrasonic_range` 3초 이내 수신 시 녹색) |
| 초음파 거리값이 화면에 없음 | 전류값 레이블 **위에** `초음파 거리: NN mm` 실시간 표시 |
| 런치 시 아두이노 노드 미기동 | `pick_place.launch.py`에 `ultrasonic_node` 포함 (`use_ultrasonic:=true`, `/dev/ttyACM0`, 9600 baud) |

관련 파일: `gui_node.py`, `ultrasonic_node.py`, `pick_place.launch.py`

#### B. 비전 — FastSAM 디버그 화면·ROI

| 문제 | 조치 |
|------|------|
| 카메라 HUD가 `realsense_fastsam_segment.py`와 다름 | `object_detector.py`에 `_render_scene()` 적용 (ROI 어둡게, known=녹색, unknown=컬러 마스크) |
| ROI 밖 검출·깜빡임 | ROI 360×240 필터, FastSAM `fastsam_every_n: 3`으로 프레임 스킵 (FPS 개선) |
| 좀비 `object_detector` 다중 실행 시 토픽 충돌 | 동일 토픽 퍼블리시 프로세스 중복 시 GUI 영상 불안정 — 기동 전 기존 프로세스 정리 필요 |

관련 파일: `object_detector.py`, `pick_place_params.yaml`

#### C. 픽 동작 — 초음파 기반 하강

| 문제 | 조치 |
|------|------|
| 고정 Z 하강만으로 파지 높이 부정확 | `pick_place_node` PICK 상태에서 **1 cm 단위 하강**, 초음파 ≤ 70 mm 시 그리퍼 close |
| 아두이노 출력 형식 상이 | `ultrasonic_node`가 `DIST:cm` / `Distance:NNmm` 둘 다 파싱, 기본 9600 baud |

관련 파일: `pick_place_node.py`, `ultrasonic_node.py`, `arduino/hc_sr04_sensor/`

#### D. 로봇 연결 해제 후 재기동 실패 (근본 수정)

**증상:** ROS 노드를 한 번 끊으면 DRCF authority / DRL(그리퍼) 세션이 컨트롤러에 남아, PC만 재런치해도 joint 활성화·그리퍼 초기화가 실패하고 **로봇 전원을 꺼야만** 복구되는 경우가 잦았음.

**원인 (구조적):**

1. **연결 소유권 분산** — `ros2_control`(DRCF `:12345`)와 `gripper_service`(DRL + TCP `:20002`)가 별도 프로세스인데 통합 teardown 없음
2. **잘못된 종료 순서** — 기존 `shutdown_nodes.sh`가 `ros2_control`을 먼저 kill → 그리퍼가 `DrlStop` 불가
3. **강제 kill** — `pkill -9`, `restart_gripper_bridge.sh`의 즉시 kill → `Drfl.close_connection()` / DRL 정리 미실행
4. **그리퍼 `close()`** — TCP SHUTDOWN만 하고 `DrlStop` 미호출 → 플랜지 RS-485 점유 잔류
5. **GUI 시스템 리셋** — launch 부모 프로세스는 살아 있는 채 자식만 죽이고 새 launch 중복 기동

**조치:**

| 파일 | 내용 |
|------|------|
| `scripts/shutdown_nodes.sh` | 종료 순서 전면 수정: `DrlStop` → gripper SIGTERM(12s) → vision → `ros2_control` SIGTERM(15s) → `--kill-launch` 시 launch 부모 종료 → 잔여만 SIGKILL |
| `scripts/run_pick_place_real.sh` | **신규** — Ctrl+C 포함 종료 시 위 shutdown 자동 실행 |
| `gripper_tcp_bridge.py` | `close()` 시 `stop_drl()` 항상 시도 |
| `gripper_service_node.py` | SIGTERM/SIGINT → `shutdown()` 후 즉시 종료 (`os._exit`) |
| `gripper_node.py`, `pick_place_node.py` | 종료 시 토크 OFF / `move_stop` |
| `scripts/launch_cleanup.sh` | launch `OnShutdown`(Ctrl+C) 시 고아 그리퍼·pick_place 정리 |
| `scripts/restart_gripper_bridge.sh` | `kill -9` 즉시 실행 → DrlStop → SIGTERM(10s) → 필요 시에만 kill -9 |
| `gui_node.py` | 시스템 리셋 시 `shutdown_nodes.sh --kill-launch` 사용 |

#### E. 그리퍼 기동 — 이벤트 기반 launch (2026-06-08)

고정 `TimerAction(10초)` 제거. `pick_place.launch.py`가 서비스 준비를 확인한 뒤 순서대로 기동합니다.

```
doosan_bringup ─┬─ wait_for_robot_ready.sh (/dsr01/drl/drl_start)
                └─ (병렬) realsense, detector, GUI …
                        ↓ 준비 완료
                gripper_service + gripper_node
                        ↓ OnProcessStart
                wait_for_gripper_ready.py (state.ready)
                        ↓ 준비 완료
                pick_place_node
```

런치 인자: `robot_ready_timeout_sec`(기본 120), `gripper_ready_timeout_sec`(기본 90).

초기화 파이프라인 자체(DRL 정지 ~5s + INITIALIZE)는 그대로이나, **로봇이 10초보다 빨리 붙으면 그만큼 앞당겨집니다.**

관련 파일: `pick_place.launch.py`, `scripts/wait_for_robot_ready.sh`, `scripts/wait_for_gripper_ready.py`

#### F. Ctrl+C 종료 시 그리퍼 노드 잔류

**증상:** 터미널에서 `ros2 launch`를 Ctrl+C로 끊으면 `gripper_service_node` / `gripper_node`만 살아 남음.

**원인:**

1. **이벤트 핸들러 기동** — `OnProcessExit`로 늦게 뜬 그리퍼가 launch 종료 시 SIGTERM을 못 받고 고아 프로세스가 됨
2. **`boot_bridge()` 블로킹** — DRL 초기화(수십 초) 중 `KeyboardInterrupt` 처리가 늦어 종료가 지연되거나 누락됨
3. **짧은 sigterm_timeout** — launch 기본 5초 안에 DrlStop 정리가 끝나지 않음

**조치:**

| 파일 | 내용 |
|------|------|
| `scripts/launch_cleanup.sh` | **신규** — `DrlStop` → gripper/pick_place/wait 스크립트 SIGTERM → 잔여 SIGKILL |
| `pick_place.launch.py` | `OnShutdown` → Ctrl+C 시 `launch_cleanup.sh` 자동 실행 |
| `gripper_service_node.py` | SIGINT/SIGTERM 시 `shutdown()` + **`os._exit(0)`** (`boot_bridge` 중에도 즉시 종료) |
| `gripper_node.py` | 동일 — 토크 OFF 시도 후 즉시 종료 |
| `pick_place.launch.py` | `gripper_service` sigterm_timeout **20초** (DRL 정리 여유) |

`run_pick_place_real.sh` 사용 시에는 기존처럼 `shutdown_nodes.sh --kill-launch`도 함께 실행됩니다.

#### G. 초음파 파지 거리 설정

파지 높이 임계값은 `mini_project/config/pick_place_params.yaml`의 `pick_place_node` 섹션에서 수정합니다.

```yaml
grasp_distance_m: 0.07    # m 단위. 0.07 = 70mm 이하에서 파지
ultrasonic_step_m: 0.01   # 1회 하강량 (m)
use_ultrasonic_grasp: true
```

수정 후 `pick_place_node` 재시작 또는 launch 재실행 필요.

#### H. GUI → SQLite + HTTP 웹 제어 전환 (2026-06-16)

기존 PyQt5 GUI(`gui_node.py`)가 담당하던 모든 제어 기능을 **화면 없이 SQLite 데이터베이스 + HTTP(웹 페이지/REST)** 로 수행하는 신규 노드 `web_control_node.py`로 옮겼습니다. 자세한 사용·구조는 [10.4](#104-웹-제어-sqlite--http--gui-대체)를 참고하세요.

| 문제/요구 | 조치 |
|------|------|
| GUI(Qt) 의존 없이 원격·헤드리스 제어 필요 | `web_control_node` 신규 — 표준 라이브러리 `http.server` 기반(추가 의존성 없음) |
| 설정·상태·명령을 DB로 일원화 | SQLite 4개 테이블(`settings`/`command_queue`/`state`/`detected_objects`)로 관리 |
| Qt-ROS 단일 스레드 결합 제거 | HTTP 스레드 ↔ ROS 스레드를 **SQLite 명령 큐**로 분리(스레드 안전) |
| 튜닝값 영구 저장 | DB 저장 + `pick_place_params.yaml` 해당 라인 자동 기록(인라인 주석 보존, 기존 GUI 동작 유지) |

GUI는 **일시 중단**(런치 기본 `gui:=false`)이며 제거하지 않았습니다. `gui:=true`로 언제든 기존 GUI를 다시 띄울 수 있습니다. 변경 파일: `web_control_node.py`(신규), `pick_place.launch.py`, `setup.py`. 기존 코드는 주석으로 보존했습니다.

### 10.3 주요 패키지·스크립트

| 경로 | 역할 |
|------|------|
| `mini_project/launch/pick_place.launch.py` | 전체 노드 런치 (`web:=true` 기본 → HTTP 자동 기동) |
| `mini_project/dsr_realsense_pick_place/web_control_node.py` | **웹 제어 (SQLite + HTTP)** — GUI 대체, [10.4](#104-웹-제어-sqlite--http--gui-대체) |
| `mini_project/dsr_realsense_pick_place/gui_node.py` | PyQt GUI (일시 중단, `gui:=true`로 사용 가능) |
| `mini_project/dsr_realsense_pick_place/object_detector.py` | YOLO + FastSAM 검출 |
| `mini_project/dsr_realsense_pick_place/pick_place_node.py` | 픽 FSM |
| `dsr_gripper_tcp/` | 그리퍼 TCP 브릿지 |
| `scripts/shutdown_nodes.sh` | 정상 종료 (DRCF/DRL 순서 해제) |
| `scripts/run_pick_place_real.sh` | 권장 기동 래퍼 (종료 시 shutdown 자동) |
| `scripts/launch_cleanup.sh` | Ctrl+C 후 고아 그리퍼 정리 |
| `scripts/wait_for_robot_ready.sh` | 런치 — DRL 서비스 준비 대기 |
| `scripts/wait_for_gripper_ready.py` | 런치 — gripper `ready` 대기 |
| `scripts/restart_gripper_bridge.sh` | 그리퍼만 복구 |
| `scripts/diagnose_drcf.py` | DRCF 연결 진단 |
| `mini_project/config/pick_place_params.yaml` | 초음파 파지 거리·픽 파라미터 |

### 10.4 웹 제어 (SQLite + HTTP) — GUI 대체

기존 PyQt5 GUI가 하던 **물체 선택 · 로봇/그리퍼 제어 · 파라미터 튜닝 · 상태 모니터링**을 화면 없이 **SQLite 데이터베이스 + HTTP(웹 페이지/REST API)** 로 수행합니다. 노드: `web_control_node.py`.

#### 실행 — 런치하면 HTTP가 자동으로 뜸

`pick_place.launch.py`의 인자 `web` 기본값이 `true`라서 **런치만 하면 HTTP 서버가 자동 기동**됩니다. 별도 명령이 필요 없습니다.

```bash
# 전체 스택 실행 (웹 제어 자동 기동, GUI는 비활성)
ros2 launch dsr_realsense_pick_place pick_place.launch.py mode:=real
# → 브라우저에서 http://<로봇PC IP>:8080 접속

# 기존 PyQt GUI를 다시 쓰고 싶으면
ros2 launch dsr_realsense_pick_place pick_place.launch.py gui:=true web:=false

# 웹 노드만 단독 실행 (포트/DB 경로 지정 예시)
ros2 run dsr_realsense_pick_place web_control_node \
  --ros-args -p http_port:=8080 -p db_path:=~/.config/dsr_realsense_pick_place/web_control.db
```

**런치 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `web` | `true` | 웹 제어 노드(`web_control_node`) 실행 여부 |
| `web_host` | `0.0.0.0` | HTTP 바인드 주소 (`0.0.0.0` = 외부 접속 허용) |
| `web_port` | `8080` | HTTP 포트 |
| `gui` | `false` | 기존 PyQt GUI 실행 여부 (대체되어 기본 비활성) |

#### 아키텍처 — SQLite를 "명령 버스 + 상태 저장소"로

HTTP 요청은 요청마다 별도 스레드에서 처리되고 ROS 콜백은 `rclpy.spin` 단일 스레드에서 돕니다. 이 둘을 직접 호출로 엮으면 rclpy 객체 스레드 안전 문제가 생기므로, **모든 동작을 SQLite를 거쳐** 주고받습니다.

```
HTTP 스레드 (요청마다)              ROS 스레드 (rclpy.spin)
─────────────────────              ─────────────────────────
POST /api/command  ──INSERT──▶  command_queue (pending)
                                     │  50ms 타이머가 drain
                                     ▼  ROS 서비스/파라미터 호출
                                command_queue (done/failed) ◀─ done_callback
GET /api/command/{id} ◀─SELECT──────┘

ROS 콜백(상태/검출물체) ──UPSERT──▶  state / detected_objects
GET /api/state         ◀──SELECT────┘
```

- HTTP 스레드는 **DB만** 읽고 쓴다. rclpy 객체는 ROS 스레드만 만진다.
- SQLite는 WAL 모드 + `busy_timeout`으로 교차 연결 동시 접근을 처리.

#### DB 테이블 (`~/.config/dsr_realsense_pick_place/web_control.db`)

| 테이블 | 역할 |
|--------|------|
| `settings` | 설정값(JSON)의 **단일 출처**. 부팅 시 DB값을 각 노드에 자동 적용 |
| `command_queue` | 명령 큐 겸 **감사 로그** (`pending→running→done/failed`, 결과 메시지 포함) |
| `state` | 실시간 상태 스냅샷(픽 상태/HW/속도/그리퍼 전류·위치/초음파/시스템 상태점) |
| `detected_objects` | 검출 물체(도달가능 필터 결과 포함, 매 갱신마다 교체) |

#### HTTP 엔드포인트

| 메서드·경로 | 설명 |
|------|------|
| `GET /` | 제어용 웹 페이지(브라우저 "창") |
| `GET /api/state` | 실시간 상태 + 검출 물체 + 시스템 상태점 |
| `GET /api/settings` | 저장된 설정값 전체 |
| `GET /api/commands` | 최근 명령 로그(30개) |
| `GET /api/command/{id}` | 단일 명령 결과(폴링용) |
| `GET /api/image.jpg` | 디버그 영상 1프레임(JPEG, `serve_image:=true`) |
| `POST /api/command` | `{"action": "...", "payload": {...}}` → 명령 큐에 적재, `{id}` 반환 |

```bash
# 예: 한 번 실행 / 긴급정지 / 신뢰도 변경
curl -X POST http://localhost:8080/api/command -H 'Content-Type: application/json' -d '{"action":"run_once"}'
curl -X POST http://localhost:8080/api/command -H 'Content-Type: application/json' -d '{"action":"e_stop"}'
curl -X POST http://localhost:8080/api/command -H 'Content-Type: application/json' -d '{"action":"set_confidence","payload":{"value":0.4}}'
```

#### 지원 명령(action) — 기존 GUI 버튼 전체 대응

| 분류 | action |
|------|--------|
| 긴급 제어 | `e_stop`, `cancel`, `e_stop_reset`, `clear_error` |
| 로봇 동작 | `run_once`, `go_home`, `recover_to_home`, `speed_normal`, `speed_reduced`, `servo_on`, `servo_off`, `safety_normal`, `safety_backdrive` |
| 그리퍼 | `gripper_open`, `gripper_close`, `gripper_enable`(`{enable}`), `gripper_reinit`, `restart_gripper_bridge` |
| 물체 선택 | `select_object`(`{label}`, 빈 문자열=자동) |
| 검출·카메라 | `set_confidence`(`{value}`), `set_camera_auto_exposure`(`{enable}`), `set_camera_exposure`(`{value}`) |
| 캘리브레이션 | `set_calibration`(`{x,y,z}`, IDLE 전용), `load_calibration` |
| 그리퍼 정밀/안전 | `set_gripper_params`(`{open_current,close_current,transport_current,profile_velocity,profile_acceleration}`), `set_min_safe_z`(`{value}`, IDLE 전용) |
| 물체별 파지 강도 | `set_grip_strength`(`{names,currents,default}` — `object_detector.known_classes`도 동기화) |
| 모델/시스템 | `set_model`(`{path}`), `save_yaml`, `system_reset` |

#### 설정 영속화 — DB + yaml 동시 기록

설정 변경 명령(`set_confidence` / `set_calibration` / `set_gripper_params` / `set_min_safe_z` / `set_grip_strength`)은 **DB에 즉시 저장**되고, 동시에 `config/pick_place_params.yaml`의 해당 라인을 **인라인 주석을 보존하며** 자동 갱신합니다(기존 GUI의 "yaml 저장" 동작 유지). 웹 페이지의 **`💾 전체 yaml 저장`** 버튼(또는 `save_yaml` action)으로 현재 DB 설정 전체를 수동으로 yaml에 다시 쓸 수도 있습니다.

- yaml↔DB 매핑: `confidence_threshold`, `absolute_calib_{x,y,z}_mm`, `open_current`, `close_current`, `transport_current`, `profile_velocity`, `profile_acceleration`, `min_safe_z`, `grip_current_default`, `grip_class_names`, `grip_class_currents`, `known_classes`(= `grip_class_names` 미러).
- 부팅 시 DB→노드 재적용 명령은 yaml을 건드리지 않습니다(부팅마다 파일 churn 방지).

#### 참고 / 주의

- **카메라 영상**: `serve_image:=true`(기본)일 때 `/detection_debug_image`를 JPEG로 인코딩해 `/api/image.jpg`로 제공. `cv_bridge`/`cv2` 미가용 시 자동 비활성.
- **IDLE 전용 동작**: 캘리브레이션·`min_safe_z` 적용은 픽 상태가 `IDLE`일 때만 허용(기존 GUI와 동일).
- **명령이 `running`에 머물 때**: 대상 노드가 응답해야 완료됩니다. 예) `pick_place_node`는 기동 시 로봇 모션 서비스를 기다리느라(`_wait_for_services`) **로봇 미연결 상태에서는 파라미터 요청에 응답하지 못해** 명령이 `running`으로 남을 수 있습니다. 로봇 연결 후 정상 처리됩니다.
- **도달 불가 물체**: 워크스페이스/반경 필터(`workspace_*`, `reach_radius_max`) 밖 물체는 `reachable:false`로 표시되어 웹에서 선택 비활성.

#### 검증(하드웨어 미연결 상태)

웹/DB/yaml 계층은 로봇·카메라·아두이노 없이도 동작 확인됨: 웹 페이지·REST 응답, 설정 시드, 명령 큐 drain, 상태 스냅샷, **설정→DB+yaml 기록**(인라인 주석 보존) 모두 정상. `object_detector`·`rh_p12_rna_gripper` 파라미터 적용(`set_confidence`/`set_calibration`/`set_gripper_params`)은 해당 노드만 헤드리스로 띄워도 `적용 완료` 확인됨. `pick_place_node` 파라미터는 위 "주의"대로 로봇 연결 후 적용됩니다.

---

## 11. Team

let-them-theory
