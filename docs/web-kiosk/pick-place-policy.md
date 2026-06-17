# Pick/Place 정책 — user 주문 fulfillment (제어 구현 필요)

> 의도한 흐름: **정렬된 물체를 box에서 집어 → package 영역으로 내려놓기.**
> sort_all(흩어진 것→class box)은 사전 정렬(Phase A). 이 문서는 user 주문(Phase B) 정책.
> 결정자: GUI 담당. 구현: detector/pick_place(제어) 담당.

## 현재 상태 (점검 결과)

- pick 선택(`object_detector._choose_target`) = 요청 클래스 중 **카메라 최근접(min depth_m)** 하나뿐. zone/box 구분 없음.
- place(kiosk=run_once) = **단일 `place_position`** 고정점. class→zone 정렬은 `sort_all` 별도 모드만 사용.
- 즉 "정렬 box에서 콕 집어 package로"가 알고리즘적으로 보장 안 됨 → 아래 정책으로 구현 필요.

## 결정된 정책

### ① zone 우선순위 — **box 우선**
같은 클래스가 ws와 box 양쪽에 있으면 **그 클래스의 정렬 box를 먼저** 소비. box 비었을 때만 ws.
- 구현: 각 후보에 zone 태그(`_which_box_roi`로 box_N 판정, 아니면 ws).
- 선택 순서: 요청 클래스 후보 → **(a) 그 클래스 지정 box(`sort_class_zones[class]`) 안 후보 우선** → 없으면 (b) ws 후보.
- 둘 다 없으면 "재고 없음" → FAILED.

### ② 같은 box 내 다중 — **카메라 최근접**
선택된 zone 안에서 `min(depth_m)`. (현재 metric 그대로, 단 zone으로 범위 한정.)

### ③ package 배치 — **주문(영수증)별 다른 칸**
한 주문(ticket)의 물체들은 같은 package 칸, 주문마다 다른 칸.

> ⚠ **선행: package 영역 미정의.** 현재 `place_position [0.4,-0.3,0.2]`는 임시 placeholder
> (sort_all fallback용). 진짜 package 영역(로봇 base 좌표 + 칸 N개)을 **실측해서 정의해야**
> ③ 구현 가능. 영역 위치·칸 개수·칸 좌표 먼저 확정 필요.
- 구현: package 칸 좌표 목록 정의(예: 칸 N개 좌표). 주문 → 칸 배정(라운드로빈 or ticket 기반).
- **인터페이스 신설 필요(GUI↔제어):** run_once는 payload 없는 Trigger → pick_place가 "이번 주문의 칸"을 알아야 함.
  - 방안 A: 백엔드가 run_once 직전 `/target_place_slot`(Int/Pose) 발행 → pick_place가 그 좌표로 place.
  - 방안 B: pick_place가 칸을 자체 round-robin(주문 경계 모르면 부정확).
  - → A 권장. 백엔드(kiosk)가 주문별 칸 인덱스를 발행, pick_place가 place 좌표로 사용.

## 추가 점검에서 나온 보강 필요 (정책과 별개로 실로봇 위험)

- 🔴 **package 영역 pick 후보 제외** — 놓은 물체가 재검출돼 다음 주문이 그걸 다시 집는 것 방지.
- 🟡 **stale track 처리** — `debounce_grace_sec`로 집은 후 잔존 track → 빈 자리 pick 방지(집은 직후 해당 track 무효화 or 재검출 대기).
- 🟡 **box 비었을 때** 명확한 "재고 없음" 경로(현재 DETECTING 타임아웃 FAILED).

## GUI 측 영향 (내 담당)

- ③ 구현 시: 백엔드가 주문별 package 칸 인덱스/좌표를 `/target_place_slot` 등으로 발행하는 부분 추가 필요.
- 나머지(①②, 후보 제외, stale)는 detector/pick_place 영역.
