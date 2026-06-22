import glob
import os
import re
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:
    raise ImportError("pyserial 필요: pip install pyserial")


def _parse_distance_mm(line: str) -> float | None:
    """아두이노 시리얼 한 줄 → 거리(mm). Distance:123.4mm / DIstance:88mm / DIST:12.3(cm)."""
    line = line.strip()
    if not line:
        return None

    m = re.search(r'(?i)distance\s*:\s*(-?[\d.]+)\s*mm', line)
    if m:
        try:
            return float(m.group(1))
        except ValueError:
            return None

    if line.startswith('DIST:'):
        try:
            return float(line[5:]) * 10.0  # cm → mm
        except ValueError:
            return None

    return None


def resolve_serial_port(requested: str) -> str:
    req = (requested or '').strip()
    if req and req.lower() not in ('auto', 'default'):
        if os.path.exists(req):
            return req
        real = os.path.realpath(req)
        if os.path.exists(real):
            return real

    for pattern in sorted(glob.glob('/dev/serial/by-id/usb-Arduino*')):
        return os.path.realpath(pattern)

    for fallback in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0'):
        if os.path.exists(fallback):
            return fallback

    return req or '/dev/ttyACM0'


class UltrasonicNode(Node):
    """HC-SR04 → /ultrasonic_range. 아두이노가 출력한 거리(mm)를 그대로 사용."""

    def __init__(self):
        super().__init__('ultrasonic_node')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'ultrasonic_sensor')
        self.declare_parameter('reconnect_sec', 2.0)

        self._port_req = str(self.get_parameter('port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._frame_id = self.get_parameter('frame_id').value
        self._reconnect_sec = float(self.get_parameter('reconnect_sec').value)

        self._pub = self.create_publisher(Range, '/ultrasonic_range', 10)
        self._ser = None
        self._ser_lock = threading.Lock()
        self._last_log_mm = None

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _open_serial(self) -> bool:
        port = resolve_serial_port(self._port_req)
        if not os.path.exists(port):
            self.get_logger().warn(
                f'아두이노 포트 없음: {port}',
                throttle_duration_sec=5.0,
            )
            return False
        try:
            with self._ser_lock:
                if self._ser is not None and self._ser.is_open:
                    self._ser.close()
                self._ser = serial.Serial(port, self._baudrate, timeout=1.0)
                time.sleep(2.0)
                self._ser.reset_input_buffer()
            self.get_logger().info(f'아두이노 연결: {port} @ {self._baudrate}')
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f'시리얼 열기 실패 ({port}): {e}', throttle_duration_sec=5.0)
            with self._ser_lock:
                self._ser = None
            return False

    def _read_loop(self):
        while rclpy.ok():
            if self._ser is None or not self._ser.is_open:
                if not self._open_serial():
                    time.sleep(self._reconnect_sec)
                    continue

            try:
                with self._ser_lock:
                    raw = self._ser.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'시리얼 읽기 오류: {e}')
                with self._ser_lock:
                    if self._ser is not None:
                        try:
                            self._ser.close()
                        except serial.SerialException:
                            pass
                    self._ser = None
                time.sleep(self._reconnect_sec)
                continue

            line = raw.decode('utf-8', errors='ignore').strip()
            dist_mm = _parse_distance_mm(line)
            if dist_mm is None:
                continue

            dist_m = dist_mm / 1000.0
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = float(dist_m)
            self._pub.publish(msg)

            log_mm = int(round(dist_mm))
            if log_mm != self._last_log_mm:
                self._last_log_mm = log_mm
                self.get_logger().info(f'초음파: {dist_mm:.1f} mm  (raw: {line!r})')

    def destroy_node(self):
        with self._ser_lock:
            if self._ser is not None and self._ser.is_open:
                self._ser.close()
            self._ser = None
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
