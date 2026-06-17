# 핸드오프 — 웹 키오스크 전환 (터미널 A 작업 내역)

> 두 터미널에서 같은 작업이 진행됨. 이 문서 = 터미널 A가 완료한 것. 중복/충돌 방지용.

## 한 줄 요약
User GUI를 **PyQt → 웹**으로 전환. 관리자는 PyQt 유지 + USER 탭 추가.

---

## 생성/수정 파일

### 1. 웹 백엔드 (신규)
`mini_project/web_kiosk/backend/main.py`
- FastAPI + rclpy. `UserGuiNode`의 큐 로직(`tick_queue`/`submit_order`/`cancel_order`) 그대로 이전 — **기능 동일**
- REST: `GET /api/catalog`, `POST /api/orders`, `GET /api/orders/{id}`, `POST /api/orders/{id}/cancel`, `GET /api/queue`, `GET /api/state`
- WebSocket `/ws`: state/queue/detected/error 푸시
- 프론트 `dist` 정적 서빙, `0.0.0.0:8000`

### 2. 웹 프론트 (신규)
`mini_project/web_kiosk/frontend/` (Vite + React + TS + Tailwind v4 + lucide-react)
- `src/App.tsx` — 4페이지(환영→선택→확인→대기), 토스풍, 카트 collapsible, 대기 진행률/품목상태/큐위치
- `src/api.ts` — REST/WS 클라이언트
- `src/products.ts` — 상품 class→한글/이모지
- `src/index.css` — 토스 디자인 토큰(파랑 `#3182f6`), Pretendard 폰트
- 빌드 완료 (`dist/`)

### 3. 관리자 PyQt (수정)
`mini_project/dsr_realsense_pick_place/gui_node.py`
- **USER 탭** 추가 (4번째 탭): 주문 큐 **트리**(주문 단위 펼치기/접기 + 번호표·수량·시간·상태) + 새로고침 + 🗑 큐 비우기
- import 추가: `QTreeWidget`/`QTreeWidgetItem`/`QTableWidget`/`QHeaderView`/`QMessageBox`, `task_repository`(`JsonRepository`/`OrderStatus`/`ItemStatus`)
- 상수 추가: `PRODUCT_KR`, `_KR_STATUS`
- 메서드 추가: `_build_queue_tab`, `_refresh_queue`, `_clear_queue`
- `_update_ui`에 USER 탭 활성 시 큐 자동 갱신
- (GUI에 box좌표 표시 + 검출목록 스크롤 + 초음파 cm 튜닝도 추가됨 — ROI 작업 관련)

### 4. 그리퍼 fix 재적용
`dsr_gripper_tcp/dsr_gripper_tcp/robot_utils.py`
- `rclpy.spin_until_future_complete` → `threading.Event` (executor 충돌 → DrlStart 실패 버그 수정. 검증됨: 부팅 1.1초)

### 5. ROI sorting (별개 작업, 참고)
`object_detector.py`, `pick_place_node.py`, `config/pick_place_params.yaml`
- box per-ROI 검출(번호 ROI 고정) + pick 후보 제외 + box 입구 depth
- 클래스→box_N 매핑(yaml `box_N_classes`) + place 목적지 = box 좌표 + `box_release_margin_m`
- yaw 파지(`use_object_yaw_for_grasp`/`yaw_axis_reference: short`/`use_target_pose_yaw` → True)

---

## 의존성 설치됨
- pip: `fastapi`, `uvicorn[standard]`, `websockets`
- npm: frontend `node_modules` (vite, react, typescript, tailwindcss, @tailwindcss/vite, lucide-react)

---

## ⚠️ 충돌 주의 (두 터미널)

| 자원 | 위험 |
|------|------|
| **공유 DB** `~/.config/dsr_realsense_pick_place/user_gui_db.json` | 웹 백엔드 + 관리자 USER 탭 + 기존 `User_gui_node`가 동시 접근 |
| **`User_gui_node.py`** (PyQt) | 웹으로 대체됨(deprecated). 터미널 A가 디자인 일부 수정(색/카트). 다른 터미널이 이걸 작업했으면 **충돌** |
| **`gui_node.py`** | 두 터미널 동시 수정 시 git 충돌 |

**핵심 결정 필요:** User GUI를 **웹 버전**으로 통일할지, PyQt `User_gui_node.py` 유지할지. 터미널 A는 웹으로 만듦 → 웹 통일 권장.

---

## 실행
```bash
# 웹 백엔드 (메인 PC)
cd mini_project/web_kiosk/backend && python3 main.py    # 0.0.0.0:8000
# 폰/태블릿 → http://<메인PC_IP>:8000  (현재 172.30.1.59, DHCP라 변동)

# 관리자 GUI
ros2 run dsr_realsense_pick_place gui_node              # USER 탭 확인
```

## 미해결 / 다음
- 큐에 테스트 주문 누적(미완료 7건) — 운영 전 USER 탭 🗑큐비우기 or DB 초기화
- IP 고정 (공유기 DHCP 예약 or PC 핫스팟) — 운영 시
- background 실행이 환경(DISPLAY/nohup)에서 죽음 — foreground/`!` 또는 launch 등록 필요
- ROI sorting 로봇 실측 미완 (box 좌표 정확도, release_margin, yaw)
