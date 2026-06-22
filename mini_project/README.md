# mini_project (아카이브)

이 디렉터리는 **더 이상 ROS 패키지 소스가 아닙니다.**

Pick & Place 패키지(`dsr_realsense_pick_place`)는 **`src/` 루트**에 통합되었습니다.

| 항목 | 경로 |
|------|------|
| 설정 yaml | `src/config/pick_place_params.yaml` |
| Python 노드 | `src/dsr_realsense_pick_place/` |
| launch | `src/launch/` |
| 스크립트 | `src/scripts/` |
| 모델 | `src/models/` |
| 웹 키오스크 | `src/web_kiosk/` |

빌드·실행:

```bash
cd ~/sudo_ws
colcon build --packages-select dsr_realsense_pick_place
source install/setup.bash
bash src/scripts/start_clean.sh
```

`COLCON_IGNORE`가 있어 colcon은 이 폴더를 무시합니다.
