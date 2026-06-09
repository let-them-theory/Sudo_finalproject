import re
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:
    raise ImportError("pyserial 필요: pip install pyserial")


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

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'ultrasonic_sensor')

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        self._frame_id = self.get_parameter('frame_id').value

        self._pub = self.create_publisher(Range, 'ultrasonic_range', 10)

        try:
            self._ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port} at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open serial port {port}: {e}')
            raise

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while rclpy.ok():
            try:
                raw = self._ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break

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
