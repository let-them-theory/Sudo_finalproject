# 최종본 정리 계획 (repo 경량화 — 온전한 파일만)

목표: clone하면 군더더기 없이 바로 빌드·실행되는 최종 repo. 중복·레거시·잔재 제거, 실제 동작 파일만.

## 조사 결과 (2026-06-23)

### 무거움 top
| 대상 | 용량 | git 추적 | 빌드/실행 사용 | 판정 |
|---|---|---|---|---|
| `.git` | 1.1G | — | — | gc 압축만(히스토리 유지) |
| `doosan-robot2/` (src 복제) | 479M | **1289파일** | ❌ colcon 무시, ros2_ws 두산이 진짜 | ⚠️ 삭제=repo 변경(협업 조율) |
| `web_kiosk/.../node_modules` | 167M | 미추적 | 재빌드용 | 유지(재설치 느림) |
| `mini_project/` | 69M | 56파일 | ❌ COLCON_IGNORE, 완전 복제 | 삭제(참조 정리 후) |
| `models/`(루트) | 46M | 추적 | ✅ setup.py glob | 유지 |
| build/install/log | 69M | 미추적 | 캐시 | 재생성(안전) |

### mini_project 분석
- **완전 복제** — mini 안 디렉토리가 전부 루트에도 있음(mini 전용 디렉토리 0).
- mini `scripts/` 10개 중 **루트에 없는 건 `start_all.sh` 1개뿐** (나머지는 루트 `scripts/`에 복제됨).
- `start_all.sh` 경로 로직: `PROJ_DIR=SCRIPT_DIR/../..` → 현재 mini_project 가리킴. **루트로 옮기면 자동으로 루트 `scripts/`(SIGINT shutdown_nodes) 사용** → 오히려 정상화.

### mini_project 참조처 (수정 대상)
- 코드: `dsr_realsense_pick_place/web_control_node.py`, `gui_node.py` (모델 경로 fallback 후보)
- 스크립트: `setup.sh`(requirements/config), `_source_workspace.sh`(주석), `realsense_fastsam_segment.py`
- 문서: `README.md`, `SETUP.md`, `docs/web-kiosk/HANDOFF.md`, `plan.md`

## 실행 단계 (검증하며, 한 단계씩)

### 0. 백업 (롤백 안전망)
- `git branch backup/pre-cleanup-20260623` (현 상태 박제)

### 1. start_all.sh 살리기
- `mini_project/scripts/start_all.sh` → `scripts/start_all.sh` (git mv)
- PROJ_DIR 자동으로 루트 가리킴 확인 → 루트 `scripts/shutdown_nodes.sh`(SIGINT) 쓰는지 검증
- (선택) mini scripts 중 루트와 내용 다른 게 있나 diff — 최신본 루트 유지

### 2. mini 참조 수정
- `web_control_node.py`/`gui_node.py`: mini fallback 경로 candidates 제거
- `setup.sh`: `mini_project/requirements.txt`·`config` → 루트 경로
- `_source_workspace.sh`: mini 주석 정리
- `realsense_fastsam_segment.py`: mini 참조 확인·수정
- `README.md`/`SETUP.md`/docs: start_all 경로 등 갱신

### 3. mini_project 삭제
- `git rm -r mini_project/` (COLCON_IGNORE도 같이 사라짐)

### 4. doosan-robot2 src 삭제 ⚠️ 협업 조율 필요
- 빌드는 ros2_ws 두산 쓰므로 기능 영향 0, repo 479M↓
- **단 git 1289파일 제거 = 다른 협업자 clone에 영향.** 가이드 준 사람과 합의 후
- `.gitignore`에 `doosan-robot2/` 추가 고려(재유입 방지)

### 5. 캐시 정리 (안전)
- `rm -rf build install log` + `__pycache__`

### 6. 검증 (테스트 전 완료 금지)
- 클린 빌드: `colcon build --paths dsr_gripper_tcp_interfaces dsr_gripper_tcp .`
- `start_all.sh`(새 위치) 기동 → 제어권/노드/8000/8080
- 실동작: sort_all·주문·재고·QR·admin
- clone 시뮬레이션(별 디렉토리 clone→빌드) 자립 확인

### 7. git 경량화
- `git gc --aggressive` (.git 압축, 히스토리 유지)

## 위험·조율 포인트
- **doosan src 삭제** = 최대 효과(479M)지만 협업 영향 → 합의 필수. 안 되면 보류(기능 무관하니 남겨도 됨).
- **mini 삭제** = repo 56파일 제거 + 참조 8곳 수정. 하나라도 빠지면 런타임 경로 깨짐 → 검증 필수.
- 롤백: `backup/pre-cleanup-20260623` 브랜치.

## 순서 원칙
이동(1) → 참조수정(2) → 삭제(3,4) → 캐시(5) → 검증(6) → gc(7).
**절대 삭제부터 X.** 참조 살아있는데 지우면 깨짐.
