import os
import re
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    raise ImportError("pyserial 필요: pip install pyserial")

_ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x10C4}  # Arduino, clone, CH340, CP210x


def _parse_distance_m(line: str):
    """시리얼 한 줄에서 거리(m) 추출. 지원 형식:
    - DIST:23.4        (cm, hc_sr04_sensor.ino / 115200)
    - DIstance:88mm    (mm, 현장 아두이노 스케치 / 9600, 오타 DIstance 포함)
    실패 시 None."""
    line = line.strip()
    if not line:
        return None
    if line.startswith('DIST:'):
        try:
            dist_cm = float(line[5:])
            return -1.0 if dist_cm < 0 else dist_cm / 100.0
        except ValueError:
            return None
    m = re.search(r'(?i)distance\s*:\s*([\d.]+)\s*mm', line)
    if m:
        try:
            return float(m.group(1)) / 1000.0
        except ValueError:
            return None
    return None


class UltrasonicNode(Node):
    """HC-SR04 초음파 센서 데이터를 /ultrasonic_range 토픽으로 발행."""

    def __init__(self):
        super().__init__('ultrasonic_node')

        # 'auto'(기본)면 아두이노 포트 자동탐색. 특정 포트 강제하려면 /dev/ttyACM0 등 지정.
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'ultrasonic_sensor')

        self._port_cfg = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baudrate').value)
        self._frame_id = self.get_parameter('frame_id').value

        self._pub = self.create_publisher(Range, '/ultrasonic_range', 10)

        # 죽지 않고 연결/재연결 루프 — 포트 번호(ACM0/1)가 바뀌거나 분리/재연결돼도 자동 복구.
        self._ser = None
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _find_port(self):
        """연결할 시리얼 포트 결정. port 지정 시 그대로(존재할 때), 'auto'면 아두이노 자동탐색."""
        cfg = self._port_cfg
        if cfg and cfg != 'auto':
            return cfg if os.path.exists(cfg) else None
        ports = list(list_ports.comports())
        for p in ports:                              # 1) VID로 아두이노/CDC 매칭
            if p.vid in _ARDUINO_VIDS:
                return p.device
        for p in ports:                              # 2) 폴백: ttyACM*/ttyUSB*
            if 'ACM' in p.device or 'USB' in p.device:
                return p.device
        return None

    def _ensure_serial(self) -> bool:
        """연결 보장. 연결돼 있으면 True, 새로 열면 True, 못 열면 False(호출측서 재시도)."""
        if self._ser is not None and self._ser.is_open:
            return True
        port = self._find_port()
        if port is None:
            self.get_logger().warn('아두이노 포트 못 찾음 — 2s 후 재탐색', throttle_duration_sec=10.0)
            return False
        try:
            self._ser = serial.Serial(port, self._baud, timeout=1.0)
            self.get_logger().info(f'Opened serial {port} @ {self._baud} baud')
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'{port} 열기 실패: {e} — 재시도', throttle_duration_sec=10.0)
            self._ser = None
            return False

    def _read_loop(self):
        while rclpy.ok():
            if not self._ensure_serial():
                time.sleep(2.0)
                continue
            try:
                raw = self._ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f'시리얼 읽기 에러: {e} — 재연결', throttle_duration_sec=10.0)
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(1.0)
                continue

            dist_m = _parse_distance_m(line)
            if dist_m is None:
                continue

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26   # HC-SR04 빔각 ~15°
            msg.min_range = 0.02       # 2 cm
            msg.max_range = 4.0        # 400 cm
            msg.range = dist_m

            self._pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, '_ser') and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
