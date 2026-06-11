# User 키오스크 — 웹 전환 계획

PyQt `User_gui_node.py` → 웹앱. 같은 wifi에서 IP 접속(태블릿 키오스크/폰 모니터링).
관리자 GUI(`gui_node.py`)는 PyQt 유지(디버그용).

---

## 1. 왜 웹

- PyQt = 그 PC 화면에서만, 원격 불가, 디자인 한계(촌스러움)
- 웹 = wifi IP 접속(태블릿/폰/노트북 동시), CSS 애니메이션/그림자 자유, UI/UX 스킬 풀활용

---

## 2. 아키텍처

```
[브라우저: React+Tailwind+shadcn]   키오스크 UI (태블릿/폰)
        ↕ REST(주문 CRUD) + WebSocket(상태 푸시)
[web_backend: FastAPI + rclpy]      한 프로세스, 별도 스레드로 ROS spin
   - task_repository (큐 DB, 기존 재사용)
   - 큐 처리 루프 (tick_queue 이전)
   - ROS: /selected_object_label 발행, /pick_place/run_once 호출
           /detected_objects · /pick_place_state · /pick_place_error 구독
        ↕ 토픽/서비스
[object_detector · pick_place_node]  (기존, 무변경)
```

**핵심:** `UserGuiNode`의 로직(큐·토픽·pick_place 연동)을 FastAPI 백엔드로 이전. UI만 React로 새로.

---

## 3. 기술 스택 (결정안)

| 레이어 | 선택 | 이유 |
|--------|------|------|
| 프론트 | **Vite + React + TS + Tailwind + shadcn/ui** | 스킬 지원, 가벼운 SPA(키오스크라 SSR 불필요), 빠른 빌드 |
| 백엔드 | **FastAPI + uvicorn** | Python → rclpy 같은 프로세스 통합 쉬움, REST+WS 내장 |
| ROS 연동 | **rclpy 직접** (rosbridge 아님) | 백엔드가 ROS 노드. rosbridge 추가 인프라 불필요 |
| 큐 DB | **task_repository 재사용** | 추상 인터페이스 이미 있음, 무변경 |
| 실시간 | **WebSocket** | 검출/진행 상태 푸시 |

배포: 로봇 PC에서 FastAPI `0.0.0.0:8000`, 프론트 빌드물 정적 서빙. 기기 → `http://로봇IP:8000`.

---

## 4. API 설계

**REST (주문 CRUD)**
```
GET  /api/catalog                 상품 목록(재고)
POST /api/orders {lines:[[cls,qty]]}   주문 생성 → order_id
GET  /api/orders/{id}             주문 상태
POST /api/orders/{id}/cancel      주문 취소
GET  /api/queue                   현재 큐 (관리자/모니터링)
```

**WebSocket `/ws`** (서버→클라 푸시)
```
{type:'state', value:'PICK'}          pick_place 상태
{type:'detected', objects:[...]}      검출 물체(가용 표시)
{type:'queue', items:[...]}           큐 갱신
{type:'error', msg:'...'}             에러
```

---

## 5. 프론트 페이지 (키오스크 위저드 — 기존 흐름 유지)

```
환영 → 상품선택 → 주문확인 → 대기/완료
```
- 상품선택: Bento 카드 그리드, 담기/수량, 하단 카트(세로 스택·펼치기)
- 주문확인: 품목 목록 + 합계
- 대기: 대기번호 큰 표시 + 진행 상태(WebSocket)
- 디자인 토큰: 소프트 그림자, 200-300ms 트랜지션, 4.5:1 대비, radius 일관, 라인 아이콘(lucide-react)

---

## 6. 파일 구조

```
mini_project/
  web_kiosk/
    backend/
      main.py            FastAPI + rclpy 노드 + 큐 루프
      ros_bridge.py      토픽/서비스 래퍼
      (task_repository 재사용 — import)
    frontend/
      (Vite React 프로젝트: src/pages, components, api, ws)
      dist/              빌드물 (FastAPI가 정적 서빙)
```

---

## 7. 작업 단계 (Phase)

```
Phase 1 — 백엔드 골격
  FastAPI + rclpy 통합(별도 스레드 spin) + task_repository 연결
  REST(catalog/orders/queue) + 큐 처리 루프 이전(UserGuiNode tick_queue)
  검증: curl로 주문 생성→큐 적재→pick_place run_once 호출 확인

Phase 2 — WebSocket 상태 푸시
  /pick_place_state·/detected_objects 구독 → ws 브로드캐스트
  검증: ws 클라이언트로 상태 수신

Phase 3 — 프론트 (Vite+React+Tailwind+shadcn)
  4페이지 위저드 + 디자인 토큰 + 카트 collapsible
  REST/ws 연동(roslib 아님, fetch+WebSocket)
  검증: 브라우저서 주문→대기 흐름

Phase 4 — 정적 서빙 + 0.0.0.0 배포
  프론트 빌드 → FastAPI 정적 서빙, 0.0.0.0 바인딩
  방화벽 8000 오픈, 로봇 PC 고정 IP
  검증: 다른 기기서 http://로봇IP:8000 접속

Phase 5 — 정리
  User_gui_node.py 제거 여부 결정(관리자 gui_node는 유지)
  launch 갱신(web_backend 노드 추가)
```

---

## 8. 위험/결정 포인트

- **rclpy + uvicorn 한 프로세스** — rclpy spin은 별도 스레드, FastAPI는 메인. 검증 필요(콜백 GIL).
- **주문 큐 동시성** — 웹 다중 접속 시 큐 race. task_repository에 락 or 단일 큐 루프.
- **키오스크 모드** — 태블릿 브라우저 풀스크린/자동시작 (운영 설정, 코드 외)
- **node 빌드 환경** — frontend는 npm/node 필요. 로봇 PC에 설치.
- **EMO/안전** — 웹은 주문만. 실제 모션 안전(EMO)은 pick_place_node가 소유(무변경). 웹서 EMO 버튼 두려면 /pick_place/e_stop 호출.

---

## 9. 범위 밖 (이번 아님)
- 관리자 GUI 웹화 (PyQt 유지)
- 결제/회원
- ROI sorting 작업(별도 진행 중)
