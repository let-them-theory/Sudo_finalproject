# Admin 통합 — context notes

## 배경
- web_kiosk(8000) = GUI(React) ↔ main.py(FastAPI+ROS노드) ↔ Robot(pick_place, 토픽/서비스). DB(task_repository)는 main만 접근, **Robot은 DB 직접 안 씀**(grep 확인). → DB 변경(통합/SQLite)은 Robot 동작과 분리됨.
- `/admin` 재고관리 패널은 `feat/kiosk-features` 브랜치에만 있었음. 현재 main(우리 QR/락커 버전)엔 없어서 `localhost:8000/admin`이 빈 화면이었음.

## 핵심 결정
- 두 task_repository는 **상호보완**(충돌 아님): 현재=락커/QR, feat-kiosk=stats/zones/set_stock. → 현재(락커 베이스) 위에 feat-kiosk admin 부분만 surgical 이식.
- `/admin`은 백엔드가 직접 HTML 반환(SPA 아님) — React 프론트 안 건드림.
- `/admin`·task_repository 모두 **JSON 기반 유지**. SQLite 전환은 다음 단계(ABC 인터페이스라 나중에 repo 한 줄 교체로 가능, admin 코드 안 깨짐).

## 의존 메서드 (feat-kiosk → 현재로 이식)
- `get_pick_stats()` (line 386), `get_zones(max)` (392), `clear_zone(id)` (419), `set_stock(class,qty)` (433) — feat/kiosk-features task_repository 원본.
- /admin이 호출하는 라우트: catalog/queue/orders/zones/stats/history.

## 결정 추가 (2026-06-22)
- **B 선택**: admin의 6-zone occupancy는 우리 시스템(box_roi 분류 + 락커 8개)에 없는 개념 → admin에서 **zone 대신 락커 현황**. 픽 통계는 우리 cycle_result 흐름에 `record_pick_result` 호출 추가해 채움.
- task_repository 이식: `record_pick_result`/`get_pick_stats`/`set_stock` 3개만. zone 메서드(occupy_next_zone/get_zones/clear_zone)는 이식 안 함.
- main.py: 주문완료(cycle_result) 처리부에 record_pick_result 호출 1줄 추가.

## ROI 표시 — 계획만 (구현 X)
- admin에 ROI별 탐지현황 + 클래스→ROI 매핑 표시. 카메라 오버레이 아님(텍스트/테이블).
- 소스: `/detected_objects`(JSON), `/selected_object_place_zone`(Int32 1~5), yaml `sort_class_names`/`sort_class_zones`.
- 상세는 checklist.md 하단 [계획만] 섹션.

## 주의
- 규칙: surgical(#3), 한국어 콜론 금지(#7), 새 파일 헤더 주석(#8), 테스트 후 완료(#10).
- origin push 금지 — 로컬만 ([[local-only-no-push-merge]]).

## Phase 2 설계 (2026-06-22, 사용자 확정)
- 페이지 역할 합의: **admin=실시간 운영(주문/큐/락커, 휘발)**, **db=영속 데이터(재고/통계/이력, SQLite)**. 중복 없음.
- 하이브리드: 영속(catalog/pick_stats/history)=SQLite, 휘발(orders/queue/lockers)=JSON. 근거 = 동시쓰기 안전·집계·누적은 SQLite, 빈번갱신·세션성은 JSON.
- box 제외: catalog 9개(box는 분류 대상이지 판매 품목 아님). 키오스크 SelectPage는 이미 box 필터 중 — 일관성 위해 catalog seed 자체서 제외.
- 이력 번호표: history에 ticket_no 저장(현재 order_id만 있어 order34로 표시됨).
- DB 시간대별 통계: 확장 여지만. history.at를 SQLite 컬럼+인덱스로 두면 나중 GROUP BY로 추가. 지금 구현 X(#2 단순성).
- HybridRepository 1개로 ABC 만족 → main/web_control/gui_node/demo_server는 repo 생성 한 줄만 교체.
