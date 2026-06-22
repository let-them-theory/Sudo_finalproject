# Admin 페이지 통합 checklist

feat/kiosk-features 의 `/admin`(재고관리 패널)을 현재 web_kiosk(우리 QR/락커 버전)에 통합 + 유저 수령코드 표시.

## 결정: B (zone→락커, 통계→cycle_result 연동)
6-zone occupancy는 우리 시스템에 없음 → admin에서 zone 대신 **락커 8개 현황**. 픽 통계는 우리 `cycle_result` 흐름에 `record_pick_result` 호출 추가해 채움.

## 1. task_repository — 통계/재고 메서드 이식 (zone 메서드 제외)
- [x] ABC에 abstract 추가: `record_pick_result`, `get_pick_stats`, `set_stock`
- [x] `JsonRepository`에 구현 이식 (feat/kiosk-features 원본)
- [x] 기존 락커 메서드 유지 확인 (assign_locker/confirm_pickup/list_lockers)
- [x] zone 메서드(occupy_next_zone/get_zones/clear_zone)는 **이식 안 함** (락커 씀)
- [x] verify: import 에러 없음

## 2. main.py — /admin + 라우트 + 통계 기록
- [x] `/api/orders`(GET 목록), `/api/history`, `/api/stats` 추가 (zone 라우트 제외, /api/lockers 기존 사용)
- [x] `_cb_cycle_result`/주문완료 처리부에 `record_pick_result(class, success)` 호출 추가
- [x] `@app.get('/admin')` HTML 라우트 추가 (재고/주문/큐/락커/통계)
- [x] verify: `curl localhost:8000/admin` HTTP 200

## 3. /admin HTML — 유저 수령코드(qr_token) + 락커 섹션
- [x] 주문/락커 항목에 qr_token(수령코드) 컬럼
- [x] zone 섹션 → 락커 8개 현황(상태/대기표/코드)
- [x] verify: 코드·락커 표시됨

## 4. 빌드 + 검증
- [x] colcon build --paths
- [x] 8000 재기동, curl /admin + /api/stats 확인
- [x] 기존 키오스크(주문/QR/락커) 정상 동작 확인

## 범위 밖 (이번 X)
- SQLite 전환 (다음 단계, ABC라 repo 한 줄 교체)
- /admin 프론트(React)화 — 백엔드 HTML 그대로
- ROI 표시 (계획만, 아래 별도)

---

## [계획만] ROI별 탐지 현황 + 클래스→ROI 매핑 (admin 패널, 카메라 창 아님)
구현 X — 다음 단계 설계.
- **목표**: admin GUI에 ① 각 box_roi(1~5)에 현재 탐지된 객체 목록 ② 어떤 클래스가 어느 ROI로 분류되는지 매핑 표.
- **데이터 소스**:
  - `/detected_objects` (JSON String) — 검출 물체 전체. label·place_zone·reachable 등. main.py 구독 → admin API.
  - `/selected_object_place_zone` (Int32) — 현재 타겟의 box_roi(1~5).
  - `sort_class_names`/`sort_class_zones` (yaml) — 클래스→zone 정적 매핑.
- **표현(예상)**: admin에 "ROI별 탐지" 테이블(ROI1~5 컬럼, 각 칸에 탐지 클래스 리스트) + "분류 규칙" 표(클래스→ROI). 실시간은 WS(`detected`) 또는 polling.
- **주의**: 카메라 영상 오버레이 아님. 텍스트/테이블만. object_detector 발행 데이터를 admin이 읽어 표시.

---

# Phase 2 — 하이브리드 SQLite + 페이지 분리 (진행)

## 설계 확정
- **저장소 하이브리드**: SQLite(영속: catalog/pick_stats/history) + JSON(휘발: orders/queue/lockers). `HybridRepository(TaskRepository)` 단일 구현체 내부서 분기. ABC라 main/web_control 코드 안 바뀜(repo 생성만 교체).
- **box 제외**: catalog seed에서 box 빼 9개(재고·오더 양쪽). box는 분류 대상이지 판매/재고 아님.
- **이력 표기**: history에 `ticket_no` 추가 저장 → order_id(order34) 대신 번호표(A-049) 표시.
- **페이지 분리**: `/admin`=운영(주문/큐/락커, 휘발), `/db`=영속(재고설정/통계/이력, SQLite). 중복 제거.
- **확장 여지(구현 X)**: DB 페이지 시간대별 통계 — history.at(timestamp) SQLite 컬럼 인덱스 → 나중 `GROUP BY 날짜/시간` 쿼리로 추가. 지금은 기본 누적 통계만.

## 단계
### 2-1. HybridRepository (task_repository)
- [x] SQLite 백엔드: catalog(class_name PK/display/grip/stock), pick_stats(class_name PK/success/fail/last_fail), history(id/ticket_no/order_id/class_name/status/at) — at 인덱스
- [x] JSON 백엔드 유지: orders/queue/lockers (기존 JsonRepository 로직 재사용)
- [x] catalog/pick_stats/history 메서드 → SQLite, 나머지 → JSON 분기
- [x] history 저장 시 ticket_no 채움
- [x] box 제외 seed
- [x] WAL + busy_timeout (web_control 동시접근)
- [x] verify: import + 스모크(주문→픽기록→통계, 재고 영속 왕복)

### 2-2. repo 생성 교체
- [x] main.py / web_control_node / gui_node / demo_server: `JsonRepository()` → `HybridRepository()`
- [x] verify: 각 노드 import 에러 없음

### 2-3. 페이지 분리
- [x] `/admin` = 운영(주문/큐/락커) — 재고/통계/이력 섹션 제거
- [x] `/db` = 영속(재고설정 9개/통계/이력 번호표) 신규
- [x] verify: 두 페이지 HTTP 200, 중복 없음

### 2-4. 빌드 + 검증
- [x] colcon build, 8000 재기동, /admin·/db curl, 기존 키오스크 회귀
