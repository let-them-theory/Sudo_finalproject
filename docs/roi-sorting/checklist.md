# ROI Sorting 패키징 — 구현 체크리스트 (머지친화 최소변경)

설계 근거 [context-notes.md](./context-notes.md).

## 설계 원칙 (팀 머지 고려)
- 신규 토픽 0. 기존 `/detected_objects` 확장
- 상태머신/EMO/자동선택 메커니즘 안 건드림
- place_pos 변수만 동적화 (MOVE_TO_PLACE/PLACE/POST_PLACE 코드 불변)
- 모든 튜닝 yaml 일원화
- 커밋 detector/pick_place 분리

---

## Phase 1 — object_detector (3군데 surgical)

- [ ] candidate 구성에서 `box` 클래스 **pick 후보 제외** (자동선택 버그 수정)
  - box는 `/detected_objects`엔 포함하되 pick 대상 플래그 off
- [ ] box 입구높이 측정 — **신규 메서드** (`_estimate_depth_m`은 중심 small radius라 재활용 불가)
  - box bbox 영역에서 카메라 가까운 쪽(최소 depth = 최대 base-z) 대표값
  - 단순 최소 1점 ❌ → 하위 percentile median (노이즈 견고)
- [ ] box를 ROI 번호로 식별 (`_detect_yolo_per_roi` 루프에 roi_idx → box_N)
- [ ] `/detected_objects` JSON에 box_N + 좌표 + roi_id 추가
- [ ] **검증**: build + box가 항상 같은 ROI→같은 번호, pick 후보엔 box 없음

## Phase 2 — pick_place_node (기존 상태머신 불변)

- [ ] `/detected_objects` 구독 추가 → box_N 좌표 캐시 (현재 미구독)
- [ ] yaml `box_N_classes` 매핑 읽기
- [ ] **pick 시점 클래스 latch** (리스크A) — pick 확정 시 `_target_object_class`를 별도 변수에 저장
  - place 시점엔 detector가 다른 물체 선택해 `_target_object_class` 바뀔 수 있음
- [ ] place 진입 전: latch 클래스 → box_N → 캐시 좌표를 `self.place_pos`에 설정
  - 822/827/834행 코드 불변 (place_pos 값만 채움)
- [ ] **검증**: 특정 클래스 pick → 매핑 box로 place 로그

## Phase 3 — yaml 파라미터

- [ ] `box_1_classes`~`box_5_classes` (매핑)
- [ ] `box_release_margin_m: 0.03` (입구 위 release)
- [ ] 기존 값(box_roi*, absolute_calib*) 유지

---

## 치명 리스크 점검 (코드 확인 완료)

| # | 리스크 | 대응 | 심각도 |
|---|--------|------|--------|
| A | place 시 `_target_object_class`가 pick한 물체 클래스 아닐 수 있음 (pick 후 detector가 다른 물체 선택) | **pick 시점 latch** | 치명 — 엉뚱한 box |
| B | 매핑된 box_N이 화면 미검출 → 좌표 없음 | place 보류 or 명확 에러 (크래시 금지) | 치명 |
| C | box 입구 depth 노이즈 | percentile median + MAD 재활용 | 정확도 |
| D | box가 로봇 reach 밖 | `_move_to_cart`가 `_Unreachable` 차단 (기존). 물리배치 문제, 코드 OK | 물리 |

## 자동 안전 (확인됨 — 별도 작업 불필요)
- `_move_to_cart` z 하한/reach 클램프: place_pos 바뀌어도 모든 이동에 자동 적용
- EMO 3경로/cancel: 안 건드리니 그대로 동작
- place z가 box 입구라 보통 min_safe_z 위 (2단 박스도 입구 높음)

---

## 검증 (AGENTS 규칙 10)
- Phase별 `colcon build --packages-select dsr_realsense_pick_place` + `py_compile`
- 로봇 동작은 연결 후 실측
- main 머지 명시 승인 후

## 나중 (범위 외)
- 자동정리(WS 순회, EMO/cancel 루프중단) / 재정리 / 4그룹화 / place 위치 오프셋(쌓기) / box 입구→바닥 정밀화
