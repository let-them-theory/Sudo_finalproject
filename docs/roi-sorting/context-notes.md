# ROI 영역별 Sorting 패키징 — 설계 노트

작업 브랜치: `feat/box-roi-per-region-detection`
기반: `ㅇㅇ`(=main, c1d0d5f) + ROI per-region 검출 커밋 1개(0ca4ff6)

---

## 1. 목표

작업영역(WS)에 뿌려둔 물체를, 옆에 놓인 실제 박스 5개에 클래스별로 분류해서 옮긴다.
place 목적지를 하드코딩 좌표가 아니라 **카메라가 검출한 실제 box 위치**로 바꾼다.

---

## 2. 물리 구조

```
[카메라] 위에서 작업대 내려다봄
  ┌──────────────┐
  │  WS ROI(1)   │  물체 뿌림 = pick 소스
  └──────────────┘
  □ □ □ □ □        WS 옆 실제 박스 5개 = place 목적지
  ROI1 .. ROI5     화면 고정 영역, YOLO box(class6)로 검출
```

- **WS ROI**: 기존 unknown_roi. 물체(known 9종 + unknown) 검출
- **box ROI 1~5**: 신규(브랜치에 이미 추가됨). 각 영역에서 box 검출

---

## 3. 확정된 동작 흐름

```
1. 검출
   WS ROI  → known 9종 + unknown, 3D 좌표
   box ROI → box, 3D 좌표, ROI영역=고정번호 box_1~5

2. 선택 [확정: 자동선택 = 가장 가까운 것]
   - 이미 _choose_target에 구현됨 (selected_label 없으면 min depth_m)
   - GUI 클릭도 병행 유지
   - box는 선택 후보에서 제외 (pick 금지)

3. sorting [확정: 클래스 → box 종류별 고정 매핑]
   box_1: ramen, bsnack
   box_2: pack, ssnack, wafers
   box_3: can, water, jelly
   box_4: boxsnack
   box_5: unknown
   같은 클래스 여러 개 → 같은 box (쌓임)

4. pick & place
   PICK(WS 물체) → LIFT → MOVE_TO_PLACE(box_N 실제좌표) → PLACE → HOME
   ※ 하드코딩 place_position [0.4,-0.3,0.1] → box_N XYZ로 교체

[나중] 자동정리(전체 순회) / 재정리(box→다른곳) / 클래스 4그룹화(음료·음식·과자·봉지)
```

---

## 4. proto.pt 클래스 (10종)

```
0 ramen   1 pack    2 ssnack   3 bsnack   4 water
5 jelly   6 box     7 can      8 boxsnack 9 wafers
```

- `box`(6) = 목적지 박스. **pick 대상 아님**
- 나머지 9종 = WS 물체
- unknown = FastSAM 검출 (proto에 없는 물체)

---

## 5. 핵심 기술 과제와 현재 코드 상태

### 과제 ① box 번호를 ROI 영역에 고정

**현재 문제 (코드 확인됨):**
- `_detect_yolo_per_roi`가 `.predict()` 사용 → tracker_id 없음
- `_known_tracker`(centroid 위치기반)가 ID 부여 → `TrackedDetectionManager`가 'box' 클래스 안에서 display 번호(1,2,3..) 매김
- box 잠깐 놓침 → grace/reserve 만료 → `_next_free_number`로 **다른 번호 재할당 가능**
- box가 어느 ROI에서 나왔는지 **기록 안 함** (raw 튜플에 roi_idx 없음)

**해결:**
- box ROI는 화면 고정 영역 → ROI 인덱스 자체를 box 번호로
- `_detect_yolo_per_roi` 루프에서 각 검출에 roi_idx 부여
- box 클래스는 tracking display 번호 대신 **소속 ROI 번호 = box_N**

### 과제 ② box를 pick candidate에서 제외

**현재:** `_detect_and_publish` 루프가 모든 검출(box 포함)을 candidate에 넣음 → 자동선택이 box를 집을 수 있음

**해결:** candidate 구성에서 `label_class == 'box'` 제외. 단 box 좌표는 place 목적지용으로 따로 보관/발행

### 과제 ③ roi_id 출력

**현재:** `/detected_objects` JSON에 roi 정보 없음

**해결:** 각 candidate에 `roi_id` 추가. box는 box_N 식별

### 과제 ④ sorting 매핑 (클래스 → box_N)

config 테이블로 정의. 1줄 수정으로 변경 가능하게

### 과제 ⑤ place 목적지 = box_N 좌표 연결

**현재:** pick_place_node가 `place_position` 고정 좌표 사용 (MOVE_TO_PLACE/PLACE에서 `px,py,pz = self.place_pos`)

**해결:** 선택 물체 클래스 → box_N 결정 → box_N의 검출 좌표를 place 목적지로

---

## 6. 미결정 — 구현 설계 결정 필요

### D1. sorting 매핑 + place 좌표 계산 주체

- **옵션 A**: object_detector가 담당
  - detector가 box_N 좌표 테이블 보유 + 클래스→box 매핑 앎
  - 선택 물체의 목적지 box 좌표까지 계산해서 `/target_place_pose`로 발행
  - pick_place는 받아서 그대로 place
  - 장점: 좌표/검출 한 곳에 모임. 단점: detector가 sorting 책임까지

- **옵션 B**: pick_place_node가 담당
  - detector는 box_N 좌표 테이블만 발행(`/box_positions`)
  - pick_place가 선택 물체 클래스 → box_N → 좌표 룩업
  - 장점: place 책임이 pick_place에. 단점: box 좌표 토픽 신설 + 동기화

→ **결정 보류. 구현 직전 확정.**

### D2. box 좌표 전달 방식

- 토픽 신설 vs 기존 `/detected_objects` 확장(box도 roi_id 달아 포함, pick은 제외)
- 기존 확장이 토픽 안 늘려서 단순

### D3. box_N 좌표가 흔들릴 때

- box 검출 좌표는 매 프레임 약간 흔들림. place 직전 좌표를 latch? 평균?
- 박스 안 움직이면 거의 고정. 일단 place 시점 최신값 사용

---

## 9. 자동선택 "아예 안 움직임" 버그 분석 (코드로 확정)

증상: 현재 ROI 브랜치에서 자동선택 실행 시 로봇이 아예 안 움직임.

### 코드로 확정된 근본 원인 — box가 pick 후보에 오염

`config/pick_place_params.yaml`:
```yaml
target_classes: ["ramen","pack","ssnack","bsnack","water","jelly","box","can","boxsnack","wafers"]
known_classes:  ["ramen","pack","ssnack","bsnack","water","jelly","box","can","boxsnack","wafers"]
```
→ `box`(class6)가 known/target에 포함 → `_detect_and_publish` candidate에 box 들어감
→ `_choose_target` 자동선택 `min(candidates, key=depth_m)` 후보에 box 포함
→ 자동선택이 box를 고를 수 있음

### 안 움직이는 메커니즘 (run_once 흐름)

```
run_once → _gripper_ready 체크 → go_home → DETECTING (10초 pose 대기)
  _cb_pose: DETECTING+pick_requested && _in_workspace 통과 → PRE_PICK
  10초간 통과 pose 없음 → "타겟 좌표 수신 타임아웃" → IDLE (안 움직임)
```

box pose가 발행돼도 막히는 지점:
- box_1 x=0.152 vs workspace x_min=0.15 → 좌표 흔들리면 _in_workspace 거부
- box 선택 시 pick 불가 물체라 후속 모션/세이프티에서 멈춤

### 로봇 연결 후 판별 (2시나리오)

| 로그 | 원인 |
|------|------|
| "그리퍼 준비 미완료" | _gripper_ready=False (status3 init 실패) → run_once 거절 |
| "타겟 좌표 수신 타임아웃 (10초)" | DETECTING서 통과 pose 못 받음 → box 오염/workspace |

진단 명령:
```bash
ros2 service call /pick_place/run_once std_srvs/srv/Trigger
ros2 topic echo /pick_place_state
ros2 topic echo /selected_object_pose
ros2 topic echo /detected_objects   # candidates에 box 섞였나
```

### 해결 (과제②와 동일)

box는 **검출 유지(place 목적지 좌표 필요)** 하되 **pick candidate에서만 제외**.
- known_classes에서 box 제거 ❌ (검출 자체가 꺼져 목적지 좌표 사라짐)
- candidate 구성/`_choose_target`에서 `label_class=='box'` 만 pick 후보 제외 ✅
- box 검출 결과는 별도로 box_N 좌표 테이블에 보관

---

## 7. 안전 (EMO — 절대 우선)

메모리 [[emo-gripper-torque-off-and-param-gating]] 준수.

- EMO 시 그리퍼 토크 OFF는 SW로 직접 (별도 Modbus). 배선 3경로(`_srv_e_stop`, `_MotionInterrupt('e_stop')`, `_handle_hw_estop`) **절대 제거 금지**
- 자동정리 루프(나중 구현) 설계 시: EMO/cancel 들어오면 **루프 즉시 중단** + 진행 중 모션 안전 정지
- gripper_node `_on_set_parameters` 동작 중 무조건 거부 넣지 말 것(동적 파지강도 깨짐)

---

## 8. 작업 규칙 (AGENTS.md)

- Surgical: 요청 외 코드 안 건드림
- 새 파일 첫 줄 한국어 헤더 주석
- 한국어 문장 종결 콜론 금지
- 코드 수정 후 빌드/테스트 검증
- main 머지는 명시 승인 후에만 (메모리 [[never-commit-without-confirmation]])
