#!/usr/bin/env python3
"""
Robot Slider Control - RViz 스타일 TCP 위치 제어

OpenCV 트랙바를 사용하여 Doosan 로봇의 TCP 위치를 직접 제어합니다.
RViz의 조인트 슬라이더와 유사하게 슬라이더 위치 = 목표 위치 방식입니다.

사용법:
    1. 로봇 bringup 실행 (다른 터미널)
    2. python robot_slider_control.py
    3. 슬라이더로 목표 TCP 위치 설정 → 로봇이 해당 위치로 이동

슬라이더:
    - X, Y, Z: TCP 위치 (mm)
    - RX, RY, RZ: TCP 자세 (degree)
    - 슬라이더를 움직이면 로봇이 해당 위치로 이동 후 멈춤

키보드:
    - Space: 현재 로봇 위치로 슬라이더 동기화
    - h: 홈 위치로 이동
    - q: 종료
"""

import numpy as np
import cv2
import time
import threading
from typing import Tuple, List

# ROS2
try:
    import rclpy
    from rclpy.node import Node
    from dsr_msgs2.srv import GetCurrentPosx, MoveLine, MoveJoint
    HAS_ROS2 = True
except ImportError:
    print("Error: ROS2가 필요합니다")
    HAS_ROS2 = False
    exit(1)


# ==================== 설정 ====================
NAMESPACE = 'dsr01'

# TCP 위치 범위 (mm, degree)
TCP_RANGES = {
    'X':  (100, 700),    # mm
    'Y':  (-400, 400),   # mm
    'Z':  (50, 600),     # mm
    'RX': (-180, 180),   # degree
    'RY': (-180, 180),   # degree
    'RZ': (-180, 180),   # degree
}

# 이동 속도/가속도
MOVE_VEL = [100.0, 30.0]   # [mm/s, deg/s]
MOVE_ACC = [100.0, 30.0]   # [mm/s^2, deg/s^2]

# 명령 전송 간격 (초) - 너무 빠르면 로봇이 버벅임
COMMAND_INTERVAL = 0.3

# 홈 위치 (조인트 각도, degree)
HOME_POSITION = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 슬라이더 라벨
SLIDER_LABELS = ['X (mm)', 'Y (mm)', 'Z (mm)', 'RX (deg)', 'RY (deg)', 'RZ (deg)']


class RobotSliderControl:
    """슬라이더로 로봇 TCP 위치를 제어하는 클래스"""

    def __init__(self, namespace=NAMESPACE):
        self.namespace = namespace
        self.running = True
        self.current_tcp = [400.0, 0.0, 300.0, 0.0, 180.0, 0.0]  # 초기 추정값
        self.target_tcp = list(self.current_tcp)
        self.last_command_time = 0
        self.command_pending = False
        self.lock = threading.Lock()

        # ROS2 초기화
        rclpy.init()
        self.node = rclpy.create_node('robot_slider_control')

        # 서비스 클라이언트 생성
        self.cli_get_posx = self.node.create_client(
            GetCurrentPosx, f'/{namespace}/aux_control/get_current_posx')
        self.cli_move_line = self.node.create_client(
            MoveLine, f'/{namespace}/motion/move_line')
        self.cli_move_joint = self.node.create_client(
            MoveJoint, f'/{namespace}/motion/move_joint')

        print("서비스 연결 대기 중...")
        services = [
            (self.cli_get_posx, 'get_current_posx'),
            (self.cli_move_line, 'move_line'),
            (self.cli_move_joint, 'move_joint')
        ]
        for cli, name in services:
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"{name} 서비스 연결 실패")
        print("모든 서비스 연결 완료")

        # ROS2 스핀 스레드
        self.spin_thread = threading.Thread(target=self._spin_thread, daemon=True)
        self.spin_thread.start()

        # 현재 TCP 위치 읽기
        self._update_current_tcp()

    def _spin_thread(self):
        """ROS2 스핀을 별도 스레드에서 실행"""
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _update_current_tcp(self) -> bool:
        """현재 TCP 위치 업데이트"""
        req = GetCurrentPosx.Request()
        req.ref = 0  # DR_BASE

        future = self.cli_get_posx.call_async(req)

        timeout_end = time.time() + 2.0
        while not future.done() and time.time() < timeout_end:
            time.sleep(0.01)

        if future.done():
            result = future.result()
            if result and result.success and len(result.task_pos_info) > 0:
                with self.lock:
                    # 처음 6개만 사용 (X, Y, Z, RX, RY, RZ), 7번째는 solution space
                    self.current_tcp = list(result.task_pos_info[0].data)[:6]
                return True

        return False

    def get_current_tcp(self) -> List[float]:
        """현재 TCP 위치 반환"""
        with self.lock:
            return list(self.current_tcp)

    def set_target_tcp(self, index: int, value: float):
        """목표 TCP 값 설정"""
        with self.lock:
            self.target_tcp[index] = value
            self.command_pending = True

    def get_target_tcp(self) -> List[float]:
        """목표 TCP 위치 반환"""
        with self.lock:
            return list(self.target_tcp)

    def send_move_command(self) -> bool:
        """MoveLine 명령 전송"""
        current_time = time.time()

        with self.lock:
            if not self.command_pending:
                return False
            if current_time - self.last_command_time < COMMAND_INTERVAL:
                return False

            target = list(self.target_tcp)
            self.command_pending = False
            self.last_command_time = current_time

        # MoveLine 서비스 호출
        req = MoveLine.Request()
        req.pos = target
        req.vel = MOVE_VEL
        req.acc = MOVE_ACC
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0  # DR_BASE
        req.mode = 0  # DR_MV_MOD_ABS (절대 위치)
        req.blend_type = 0
        req.sync_type = 1  # ASYNC (비동기)

        future = self.cli_move_line.call_async(req)
        # 비동기로 보내고 결과는 기다리지 않음
        return True

    def move_home(self):
        """홈 위치로 이동"""
        print("\n홈 위치로 이동 중...")

        req = MoveJoint.Request()
        req.pos = HOME_POSITION
        req.vel = 30.0
        req.acc = 30.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0  # SYNC

        future = self.cli_move_joint.call_async(req)

        timeout_end = time.time() + 30.0
        while not future.done() and time.time() < timeout_end:
            time.sleep(0.1)

        if future.done() and future.result().success:
            print("홈 위치 도착!")
            time.sleep(0.5)
            self._update_current_tcp()
            return True
        else:
            print("홈 이동 실패 또는 타임아웃")
            return False

    def sync_sliders_to_robot(self):
        """슬라이더를 현재 로봇 위치로 동기화"""
        if self._update_current_tcp():
            with self.lock:
                self.target_tcp = list(self.current_tcp)
                self.command_pending = False
            return True
        return False

    def shutdown(self):
        """종료"""
        self.running = False
        time.sleep(0.1)
        self.node.destroy_node()
        rclpy.shutdown()


def value_to_slider(value: float, range_min: float, range_max: float, slider_max: int = 1000) -> int:
    """실제 값을 슬라이더 값으로 변환"""
    ratio = (value - range_min) / (range_max - range_min)
    return int(np.clip(ratio * slider_max, 0, slider_max))


def slider_to_value(slider: int, range_min: float, range_max: float, slider_max: int = 1000) -> float:
    """슬라이더 값을 실제 값으로 변환"""
    ratio = slider / slider_max
    return range_min + ratio * (range_max - range_min)


def create_slider_callback(index: int, controller: RobotSliderControl, ranges: dict):
    """슬라이더 콜백 생성"""
    keys = list(ranges.keys())
    key = keys[index]
    range_min, range_max = ranges[key]

    def callback(slider_value):
        value = slider_to_value(slider_value, range_min, range_max)
        controller.set_target_tcp(index, value)

    return callback


def main():
    print("=" * 60)
    print("  Robot Slider Control (RViz Style)")
    print("  슬라이더로 TCP 위치 직접 제어")
    print("=" * 60)

    if not HAS_ROS2:
        return

    controller = RobotSliderControl()

    # 초기 TCP 위치로 target 설정
    initial_tcp = controller.get_current_tcp()
    print(f"\n현재 TCP 위치: {initial_tcp}")

    with controller.lock:
        controller.target_tcp = list(initial_tcp)

    # OpenCV 창 생성
    window_name = "Robot Slider Control"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 600, 500)

    # 슬라이더 생성
    keys = list(TCP_RANGES.keys())
    for i, (key, (range_min, range_max)) in enumerate(TCP_RANGES.items()):
        initial_slider = value_to_slider(initial_tcp[i], range_min, range_max)
        cv2.createTrackbar(
            SLIDER_LABELS[i],
            window_name,
            initial_slider,
            1000,
            create_slider_callback(i, controller, TCP_RANGES)
        )

    print("\n" + "=" * 60)
    print("조작 방법:")
    print("  슬라이더: 원하는 TCP 위치로 설정 → 로봇이 이동")
    print("  Space: 현재 로봇 위치로 슬라이더 동기화")
    print("  H: 홈 위치로 이동")
    print("  Q: 종료")
    print("=" * 60)
    print(f"\n이동 속도: {MOVE_VEL[0]} mm/s, {MOVE_VEL[1]} deg/s")

    # 메인 루프
    try:
        last_tcp_update = time.time()
        while controller.running:
            # 표시용 이미지 생성
            display = np.zeros((500, 600, 3), dtype=np.uint8)
            display[:] = (40, 40, 40)

            # 현재 TCP 위치 주기적 업데이트
            if time.time() - last_tcp_update > 0.5:
                controller._update_current_tcp()
                last_tcp_update = time.time()

            current_tcp = controller.get_current_tcp()
            target_tcp = controller.get_target_tcp()

            # 제목
            cv2.putText(display, "Robot Slider Control (RViz Style)", (20, 35),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # 구분선
            cv2.line(display, (20, 55), (580, 55), (100, 100, 100), 1)

            # 헤더
            cv2.putText(display, "Axis", (30, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(display, "Current", (150, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(display, "Target", (280, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(display, "Range", (420, 85),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

            # 각 축 정보 표시
            y_pos = 120
            for i, (key, (range_min, range_max)) in enumerate(TCP_RANGES.items()):
                label = SLIDER_LABELS[i]
                current = current_tcp[i]
                target = target_tcp[i]

                # 라벨
                cv2.putText(display, label, (30, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

                # 현재값
                color = (0, 255, 0) if i < 3 else (0, 200, 255)
                cv2.putText(display, f"{current:.1f}", (150, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                # 목표값
                diff = abs(target - current)
                if diff > 5:
                    target_color = (0, 100, 255)  # 주황 (이동 중)
                else:
                    target_color = (0, 255, 0)  # 녹색 (도착)
                cv2.putText(display, f"{target:.1f}", (280, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, target_color, 1)

                # 범위
                cv2.putText(display, f"[{range_min:.0f} ~ {range_max:.0f}]", (420, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

                # 진행 바
                bar_y = y_pos + 15
                bar_x = 30
                bar_width = 540

                # 배경 바
                cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_width, bar_y + 8),
                             (60, 60, 60), -1)

                # 현재 위치 표시
                current_ratio = (current - range_min) / (range_max - range_min)
                current_x = bar_x + int(np.clip(current_ratio, 0, 1) * bar_width)
                cv2.circle(display, (current_x, bar_y + 4), 5, (0, 255, 0), -1)

                # 목표 위치 표시
                target_ratio = (target - range_min) / (range_max - range_min)
                target_x = bar_x + int(np.clip(target_ratio, 0, 1) * bar_width)
                cv2.drawMarker(display, (target_x, bar_y + 4), (0, 100, 255),
                              cv2.MARKER_TRIANGLE_DOWN, 8, 2)

                y_pos += 55

            # 상태 표시
            cv2.line(display, (20, 430), (580, 430), (100, 100, 100), 1)

            if controller.command_pending:
                cv2.putText(display, "Status: Moving...", (30, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 100, 255), 1)
            else:
                cv2.putText(display, "Status: Ready", (30, 460),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 1)

            # 키 안내
            cv2.putText(display, "[Space] Sync  [H] Home  [Q] Quit", (300, 460),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)

            cv2.imshow(window_name, display)

            # 명령 전송
            controller.send_move_command()

            key = cv2.waitKey(30) & 0xFF

            if key == ord(' '):
                # 슬라이더를 현재 로봇 위치로 동기화
                print("\n[동기화] 슬라이더를 현재 로봇 위치로 설정")
                if controller.sync_sliders_to_robot():
                    current_tcp = controller.get_current_tcp()
                    for i, (k, (range_min, range_max)) in enumerate(TCP_RANGES.items()):
                        slider_val = value_to_slider(current_tcp[i], range_min, range_max)
                        cv2.setTrackbarPos(SLIDER_LABELS[i], window_name, slider_val)
                    print("  동기화 완료!")

            elif key == ord('h'):
                # 홈 위치로 이동
                if controller.move_home():
                    current_tcp = controller.get_current_tcp()
                    for i, (k, (range_min, range_max)) in enumerate(TCP_RANGES.items()):
                        slider_val = value_to_slider(current_tcp[i], range_min, range_max)
                        cv2.setTrackbarPos(SLIDER_LABELS[i], window_name, slider_val)

            elif key == ord('q'):
                print("\n종료합니다...")
                break

    except KeyboardInterrupt:
        print("\n키보드 인터럽트")

    finally:
        controller.shutdown()
        cv2.destroyAllWindows()
        print("종료 완료")


if __name__ == "__main__":
    main()
