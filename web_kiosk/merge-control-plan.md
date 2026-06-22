# feat/dynamic-grasp-ultrasonic 제어/비전 머지 계획

원칙: **제어/비전/센서 = 브랜치 우선, DB/GUI = 우리 우선.** 작업로그 버림. cycle_result는 복원(키오스크 DONE 정확).

## 브랜치 통째 checkout (제어/비전/센서 — 우리 DB/GUI 무관)
- [ ] dsr_realsense_pick_place/pick_place_node.py (동적 초음파 제어)
- [ ] dsr_realsense_pick_place/ultrasonic_node.py (resolve_serial_port + 동적)
- [ ] dsr_realsense_pick_place/object_detector.py (비전/yaw)
- [ ] dsr_realsense_pick_place/gripper_node.py
- [ ] config/pick_place_params.yaml (제어/비전 파라미터 — DB/GUI 키 없음)
- [ ] arduino/hc_sr04_sensor/hc_sr04_sensor.ino
- [ ] e0509_gripper_description/scripts/{gripper_joint_publisher,gripper_service_node}.py
- [ ] scripts/check_arduino.sh

## 우리 보존 (DB/GUI — 안 건드림)
- task_repository.py(HybridRepo), web_control_node.py, web_kiosk/*(main/App/dist), demo_server.py

## 경계 파일 — 수동 병합
- [ ] launch/pick_place.launch.py: 브랜치(제어 노드) 기준 + 우리 initial_reset(캠 자동)·web:=true·web_port 보존
- [ ] cycle_result 복원: 브랜치 pick_place에 6패치(백업 633c45a 소스)
      ① __init__ _cycle_result='idle' ② pub_cycle_result publisher
      ③ POST_PLACE 'success' ④ object_lost 'dropped' ⑤ 사이클시작 'failed' ⑥ finish 발행+리셋

## 검증
- [ ] 빌드 --paths (gripper_tcp_interfaces/gripper_tcp/Sudo_finalproject)
- [ ] 인터페이스 호환: 제어 토픽 ↔ GUI(키오스크 cycle_result/state, web_control 서비스)
- [ ] 8000/admin·키오스크 정상, object_detector 모델 로드
