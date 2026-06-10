#!/usr/bin/env python3
"""
RH-P12-RN-A 그리퍼 ROS2 서비스 노드
Doosan E0509 로봇의 Tool Flange Serial을 통해 Modbus RTU로 제어

제공하는 인터페이스:
    서비스:
        /dsr01/gripper/open         - 그리퍼 열기 (Trigger)
        /dsr01/gripper/close        - 그리퍼 닫기 (Trigger)
    토픽:
        /dsr01/gripper/position_cmd - 위치 명령 구독 (Int32, 0~700)
        /dsr01/gripper/stroke       - 현재 stroke 발행 (Int32, RViz용)
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from dsr_msgs2.srv import FlangeSerialOpen, FlangeSerialClose, FlangeSerialWrite
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
import threading
import time


class ModbusRTU:
    """Modbus RTU CRC16 계산 및 프레임 생성"""

    SLAVE_ID = 1
    REG_TORQUE_ENABLE = 256
    REG_GOAL_POSITION = 282

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @classmethod
    def make_frame(cls, data: bytes) -> list:
        """CRC 추가하고 list[int]로 반환"""
        crc = cls.crc16(data)
        frame = data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        return list(frame)

    @classmethod
    def fc06_torque_enable(cls) -> list:
        """토크 활성화 프레임"""
        data = bytes([cls.SLAVE_ID, 0x06, 0x01, 0x00, 0x00, 0x01])
        return cls.make_frame(data)

    @classmethod
    def fc16_position(cls, position: int) -> list:
        """위치 설정 프레임 (FC16)"""
        position = max(0, min(700, position))
        data = bytes([
            cls.SLAVE_ID, 0x10,  # FC16
            0x01, 0x1A,          # Start register 282
            0x00, 0x02,          # Register count 2
            0x04,                # Byte count 4
            (position >> 8) & 0xFF, position & 0xFF,  # Position
            0x00, 0x00           # Reserved
        ])
        return cls.make_frame(data)


class GripperWorker:
    """별도 스레드에서 그리퍼 제어를 수행하는 워커"""

    def __init__(self, namespace='dsr01', real_robot_mode=False, logger=None):
        self.namespace = namespace
        self.port = 1
        self.logger = logger
        self.lock = threading.Lock()
        self.real_robot_mode = real_robot_mode

        # 별도의 ROS2 node 생성
        self.node = None
        self.cli_open = None
        self.cli_close = None
        self.cli_write = None

    def log_info(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f'[INFO] {msg}')

    def log_warn(self, msg):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f'[WARN] {msg}')

    def log_error(self, msg):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f'[ERROR] {msg}')

    def init_clients(self):
        """서비스 클라이언트 초기화 (Real 모드에서만)"""
        if not self.real_robot_mode:
            self.log_info('Virtual 모드 - 실제 그리퍼 제어 비활성화')
            return False

        self.node = rclpy.create_node('gripper_worker_node')

        prefix = f'/{self.namespace}/gripper'
        self.cli_open = self.node.create_client(FlangeSerialOpen, f'{prefix}/flange_serial_open')
        self.cli_close = self.node.create_client(FlangeSerialClose, f'{prefix}/flange_serial_close')
        self.cli_write = self.node.create_client(FlangeSerialWrite, f'{prefix}/flange_serial_write')

        # 서비스 대기
        self.log_info('Doosan flange serial 서비스 연결 대기 중...')
        services = [
            (self.cli_open, 'flange_serial_open'),
            (self.cli_close, 'flange_serial_close'),
            (self.cli_write, 'flange_serial_write')
        ]
        for cli, name in services:
            if not cli.wait_for_service(timeout_sec=10.0):
                self.log_error(f'서비스 {name} 연결 실패!')
                self.real_robot_mode = False
                return False

        self.log_info('Doosan flange serial 서비스 연결됨 - Real 모드 준비 완료')
        return True

    def call_sync(self, client, request):
        """동기 서비스 호출 (gripper.py와 동일한 방식)"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        return future.result()

    def serial_open(self, baudrate=57600, max_retries=3):
        """시리얼 포트 열기 (실패 시 강제 닫고 재시도)"""
        req = FlangeSerialOpen.Request()
        req.port = self.port
        req.baudrate = baudrate
        req.bytesize = 8
        req.parity = 0
        req.stopbits = 1

        for attempt in range(max_retries):
            result = self.call_sync(self.cli_open, req)
            if result and result.success:
                self.log_info('시리얼 포트 열림')
                return True

            # 실패 시 강제로 닫고 재시도
            if attempt < max_retries - 1:
                self.log_warn(f'시리얼 포트 열기 실패, 강제 닫기 후 재시도 ({attempt + 1}/{max_retries})')
                self.serial_close()
                time.sleep(0.3)

        self.log_error('시리얼 포트 열기 실패 (재시도 횟수 초과)')
        return False

    def serial_close(self):
        """시리얼 포트 닫기"""
        req = FlangeSerialClose.Request()
        req.port = self.port
        result = self.call_sync(self.cli_close, req)
        if result and result.success:
            self.log_info('시리얼 포트 닫힘')
        return result.success if result else False

    def serial_write(self, data: list):
        """데이터 전송"""
        req = FlangeSerialWrite.Request()
        req.port = self.port
        req.data = data

        result = self.call_sync(self.cli_write, req)
        if result and result.success:
            self.log_info(f'전송 성공: {bytes(data).hex()}')
            return True
        self.log_error('전송 실패')
        return False

    def enable_torque(self):
        """토크 활성화"""
        self.log_info('토크 활성화...')
        return self.serial_write(ModbusRTU.fc06_torque_enable())

    def execute_command(self, position: int) -> tuple:
        """그리퍼 명령 실행"""
        with self.lock:
            if not self.real_robot_mode:
                self.log_info(f'Virtual 모드: position={position}')
                return True, f'Virtual 모드: position={position}'

            self.log_info(f'Real 모드: 그리퍼 제어 시작 (position={position})')

            if not self.serial_open():
                return False, '시리얼 포트 열기 실패'

            try:
                time.sleep(0.1)
                if not self.enable_torque():
                    return False, '토크 활성화 실패'

                time.sleep(0.2)
                self.log_info(f'위치 설정: {position}')
                if not self.serial_write(ModbusRTU.fc16_position(position)):
                    return False, '위치 설정 실패'

                time.sleep(1.0)  # 그리퍼 동작 대기
                return True, f'그리퍼 위치 설정 완료: {position}'
            finally:
                self.serial_close()

    def destroy(self):
        """정리"""
        if self.node:
            self.node.destroy_node()


class GripperServiceNode(Node):
    """그리퍼 ROS2 서비스 노드"""

    def __init__(self, namespace='dsr01'):
        super().__init__('gripper_service_node')

        self.namespace = namespace
        self.current_position = 0

        # mode 파라미터 선언 및 가져오기
        self.declare_parameter('mode', 'virtual',
            ParameterDescriptor(description='Operation mode: virtual or real'))
        mode = self.get_parameter('mode').get_parameter_value().string_value
        real_robot_mode = (mode == 'real')

        self.get_logger().info(f'========================================')
        self.get_logger().info(f'모드: {mode.upper()}')
        self.get_logger().info(f'========================================')

        # 그리퍼 워커 (별도 스레드에서 실제 제어)
        self.worker = GripperWorker(
            namespace=namespace,
            real_robot_mode=real_robot_mode,
            logger=self.get_logger()
        )

        # RViz 시각화용 stroke publisher
        prefix = f'/{namespace}/gripper'
        self.stroke_pub = self.create_publisher(Int32, f'{prefix}/stroke', 10)

        # 서비스 서버 생성
        self.srv_open = self.create_service(Trigger, f'{prefix}/open', self.handle_open)
        self.srv_close = self.create_service(Trigger, f'{prefix}/close', self.handle_close)

        # 위치 명령 토픽 구독
        self.position_sub = self.create_subscription(
            Int32, f'{prefix}/position_cmd', self.handle_position_cmd, 10)

        self.get_logger().info(f'그리퍼 서비스 노드 시작 (namespace: {namespace})')
        self.get_logger().info(f'  서비스: {prefix}/open, {prefix}/close')
        self.get_logger().info(f'  토픽: {prefix}/position_cmd (구독), {prefix}/stroke (발행)')

    def init_worker(self):
        """워커 초기화"""
        self.worker.init_clients()

    def publish_stroke(self, position: int):
        """RViz 시각화 업데이트"""
        stroke_msg = Int32()
        stroke_msg.data = position
        self.stroke_pub.publish(stroke_msg)
        self.current_position = position

    def execute_in_thread(self, position: int) -> tuple:
        """별도 스레드에서 그리퍼 명령 실행"""
        # RViz 먼저 업데이트
        self.publish_stroke(position)

        # 워커에서 실제 제어
        result = [False, '']

        def worker_task():
            result[0], result[1] = self.worker.execute_command(position)

        thread = threading.Thread(target=worker_task)
        thread.start()
        thread.join(timeout=15.0)  # 최대 15초 대기

        if thread.is_alive():
            return False, '그리퍼 명령 타임아웃'

        return result[0], result[1]

    def handle_open(self, request, response):
        """그리퍼 열기 서비스 핸들러"""
        self.get_logger().info('그리퍼 열기 요청 수신')
        success, message = self.execute_in_thread(0)
        response.success = success
        response.message = message
        self.get_logger().info(f'그리퍼 열기 결과: {message}')
        return response

    def handle_close(self, request, response):
        """그리퍼 닫기 서비스 핸들러"""
        self.get_logger().info('그리퍼 닫기 요청 수신')
        success, message = self.execute_in_thread(700)
        response.success = success
        response.message = message
        self.get_logger().info(f'그리퍼 닫기 결과: {message}')
        return response

    def handle_position_cmd(self, msg):
        """위치 명령 토픽 핸들러"""
        position = msg.data
        self.get_logger().info(f'위치 명령 수신: {position}')
        success, message = self.execute_in_thread(position)
        if not success:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)

    def destroy_node(self):
        """노드 정리"""
        self.worker.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # 파라미터에서 namespace 가져오기
    import sys
    namespace = 'dsr01'
    for i, arg in enumerate(sys.argv):
        if arg == '--namespace' and i + 1 < len(sys.argv):
            namespace = sys.argv[i + 1]
        elif arg.startswith('--namespace='):
            namespace = arg.split('=')[1]

    node = GripperServiceNode(namespace=namespace)

    # 워커 초기화 (Real 모드에서만 서비스 클라이언트 연결)
    node.init_worker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
