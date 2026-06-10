from __future__ import annotations

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from dsr_msgs2.srv import DrlStart, DrlStop

from std_srvs.srv import Trigger

from dsr_gripper_tcp.gripper_tcp_bridge import GripperBridge, build_drl_script
from dsr_gripper_tcp.gripper_tcp_protocol import GripperState as BridgeState
from dsr_gripper_tcp.robot_utils import build_service_root, set_robot_mode_autonomous

from dsr_gripper_tcp_interfaces.msg import GripperState
from dsr_gripper_tcp_interfaces.srv import (
    GetMotionProfile,
    SetMotionProfile,
    SetPosition,
    SetTorque,
)


class GripperServiceNode(Node):

    def __init__(self) -> None:
        super().__init__('gripper_service')

        self.declare_parameter('controller_host', '110.120.1.56')
        self.declare_parameter('tcp_port', 20002)
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('goal_current', 400)
        self.declare_parameter('profile_velocity', 1500)
        self.declare_parameter('profile_acceleration', 1000)
        self.declare_parameter('poll_rate_hz', 10.0)
        self.declare_parameter('grasp_current_threshold', 300)
        self.declare_parameter('default_move_timeout_sec', 10.0)
        self.declare_parameter('skip_set_autonomous', False)

        gp = self.get_parameter
        host = gp('controller_host').get_parameter_value().string_value
        port = gp('tcp_port').get_parameter_value().integer_value
        namespace = gp('namespace').get_parameter_value().string_value
        self._goal_current = gp('goal_current').get_parameter_value().integer_value
        self._profile_velocity = gp('profile_velocity').get_parameter_value().integer_value
        self._profile_acceleration = gp('profile_acceleration').get_parameter_value().integer_value
        self._poll_rate_hz = max(gp('poll_rate_hz').get_parameter_value().double_value, 1.0)
        self._grasp_threshold = gp('grasp_current_threshold').get_parameter_value().integer_value
        self._move_timeout = gp('default_move_timeout_sec').get_parameter_value().double_value
        self._skip_autonomous = gp('skip_set_autonomous').get_parameter_value().bool_value
        self._namespace = namespace

        self._bridge = GripperBridge(host=host, port=port)
        self._drl_script = build_drl_script(
            port=port,
            goal_current=self._goal_current,
            profile_velocity=self._profile_velocity,
            profile_acceleration=self._profile_acceleration,
        )

        svc_root = build_service_root(namespace, '')
        self._cli_drl_start = self.create_client(DrlStart, f"{svc_root}/drl/drl_start")
        self._cli_drl_stop = self.create_client(DrlStop, f"{svc_root}/drl/drl_stop")
        for cli, name in [(self._cli_drl_start, 'drl_start'), (self._cli_drl_stop, 'drl_stop')]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {name}...")

        self._lock = threading.Lock()
        self._ready = False
        self._last_state: GripperState | None = None
        self._last_goal_position = 0

        cb = ReentrantCallbackGroup()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._state_pub = self.create_publisher(GripperState, '~/state', qos)

        self.create_service(SetPosition, '~/set_position', self._handle_set_position, callback_group=cb)
        self.create_service(SetMotionProfile, '~/set_motion_profile', self._handle_set_motion_profile, callback_group=cb)
        self.create_service(GetMotionProfile, '~/get_motion_profile', self._handle_get_motion_profile, callback_group=cb)
        self.create_service(SetTorque, '~/set_torque', self._handle_set_torque, callback_group=cb)
        self.create_service(Trigger, '~/reinitialize', self._handle_reinitialize, callback_group=cb)

        self._poll_timer = None

    # ── Boot ───────────────────────────────────────────────────────────────

    def boot(self) -> None:
        if not self._skip_autonomous:
            self.get_logger().info('Setting robot to autonomous mode...')
            set_robot_mode_autonomous(self, self._namespace, '')

        self._start_drl_and_connect()

        self.get_logger().info('Initializing gripper...')
        for attempt in range(1, 4):
            try:
                with self._lock:
                    state = self._bridge.initialize(
                        goal_current=self._goal_current,
                        timeout=40.0,
                    )
                self._ready = True
                self._cache_state(state)
                self.get_logger().info(f'Gripper initialized (attempt {attempt})')
                break
            except RuntimeError as exc:
                self.get_logger().warning(f'Initialize attempt {attempt}/3 failed: {exc}')
                if attempt < 3:
                    time.sleep(1.0)
        else:
            raise RuntimeError('Gripper initialization failed after 3 attempts')

        self._poll_timer = self.create_timer(1.0 / self._poll_rate_hz, self._poll_state)
        self.get_logger().info(f'Gripper service ready at {self._poll_rate_hz:.1f} Hz')

    def shutdown(self) -> None:
        with self._lock:
            self._bridge.close()

    # ── DRL management ─────────────────────────────────────────────────────

    def _start_drl_and_connect(self) -> None:
        self.get_logger().info('Stopping existing DRL program...')
        self._call_service(self._cli_drl_stop, DrlStop.Request())
        time.sleep(1.0)

        self.get_logger().info('Starting DRL gripper server...')
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = self._drl_script
        resp = self._call_service(self._cli_drl_start, req)
        if not resp or not resp.success:
            raise RuntimeError('DrlStart failed')

        self.get_logger().info('Connecting to gripper TCP server...')
        self._bridge.connect(timeout=30.0)
        self.get_logger().info('TCP connected')

    # ── Polling ────────────────────────────────────────────────────────────

    def _poll_state(self) -> None:
        try:
            with self._lock:
                bridge_state = self._bridge.read_state()
            self._cache_state(bridge_state)
        except Exception as exc:
            self.get_logger().warning(f'State poll failed: {exc}', throttle_duration_sec=2.0)
            msg = self._empty_state(str(exc))
            self._state_pub.publish(msg)
            return
        self._state_pub.publish(self._last_state)

    # ── Service handlers ───────────────────────────────────────────────────

    def _handle_set_position(self, request, response):
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else self._move_timeout
        try:
            with self._lock:
                state = self._bridge.move_to(int(request.position), timeout_sec=timeout)
            self._last_goal_position = int(request.position)
            self._cache_state(state)
            response.success = True
            response.message = 'ok'
            response.present_position = self._last_state.present_position
            response.goal_position = self._last_state.goal_position
            response.present_current = self._last_state.present_current
            response.in_position = self._last_state.in_position
            response.grasp_detected = self._last_state.grasp_detected
            response.object_lost = False
            response.state = self._last_state
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.state = self._empty_state(str(exc))
        return response

    def _handle_set_motion_profile(self, request, response):
        try:
            with self._lock:
                state = self._bridge.set_motion_profile(
                    int(request.goal_current),
                    int(request.profile_velocity),
                    int(request.profile_acceleration),
                )
            self._goal_current = int(request.goal_current)
            self._profile_velocity = int(request.profile_velocity)
            self._profile_acceleration = int(request.profile_acceleration)
            self._cache_state(state)
            response.success = True
            response.message = 'ok'
            response.goal_current = self._goal_current
            response.profile_velocity = self._profile_velocity
            response.profile_acceleration = self._profile_acceleration
            response.state = self._last_state
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.goal_current = self._goal_current
            response.profile_velocity = self._profile_velocity
            response.profile_acceleration = self._profile_acceleration
            response.state = self._empty_state(str(exc))
        return response

    def _handle_get_motion_profile(self, request, response):  # noqa: ARG002
        response.success = True
        response.message = 'ok'
        response.goal_current = self._goal_current
        response.profile_velocity = self._profile_velocity
        response.profile_acceleration = self._profile_acceleration
        return response

    def _handle_set_torque(self, request, response):
        try:
            with self._lock:
                state = self._bridge.set_torque(bool(request.enabled))
            self._cache_state(state)
            response.success = True
            response.message = 'ok'
            response.torque_enabled = self._last_state.torque_enabled
            response.state = self._last_state
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.torque_enabled = False
            response.state = self._empty_state(str(exc))
        return response

    def _handle_reinitialize(self, request, response):  # noqa: ARG002
        try:
            with self._lock:
                self._ready = False
                self._bridge.reset()
                self._start_drl_and_connect()
                state = self._bridge.initialize(goal_current=self._goal_current, timeout=40.0)
            self._ready = True
            self._cache_state(state)
            response.success = True
            response.message = 'reinitialized'
            self.get_logger().info('Gripper reinitialized')
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Reinitialize failed: {exc}')
        return response

    # ── Helpers ────────────────────────────────────────────────────────────

    def _call_service(self, client, req):
        event = threading.Event()
        future = client.call_async(req)
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=10.0)
        return future.result()

    def _cache_state(self, bridge_state: BridgeState) -> GripperState:
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ready = self._ready and bool(bridge_state.torque_enabled)
        msg.torque_enabled = bool(bridge_state.torque_enabled)
        msg.moving = bool(bridge_state.moving)
        msg.in_position = bridge_state.in_position
        msg.status = int(bridge_state.status)
        msg.moving_status = int(bridge_state.moving_status)
        msg.present_position = int(bridge_state.present_position)
        msg.goal_position = self._last_goal_position
        msg.present_current = int(bridge_state.present_current)
        msg.current_limit = self._goal_current
        msg.present_velocity = int(bridge_state.present_velocity)
        msg.present_temperature = int(bridge_state.present_temperature)
        msg.grasp_detected = abs(int(bridge_state.present_current)) >= self._grasp_threshold
        msg.object_lost = False
        msg.status_text = 'ok'
        self._last_state = msg
        return msg

    def _empty_state(self, status_text: str) -> GripperState:
        if self._last_state is not None:
            msg = GripperState()
            msg.stamp = self.get_clock().now().to_msg()
            msg.ready = self._last_state.ready
            msg.torque_enabled = self._last_state.torque_enabled
            msg.moving = self._last_state.moving
            msg.in_position = self._last_state.in_position
            msg.status = self._last_state.status
            msg.moving_status = self._last_state.moving_status
            msg.present_position = self._last_state.present_position
            msg.goal_position = self._last_state.goal_position
            msg.present_current = self._last_state.present_current
            msg.current_limit = self._last_state.current_limit
            msg.present_velocity = self._last_state.present_velocity
            msg.present_temperature = self._last_state.present_temperature
            msg.grasp_detected = self._last_state.grasp_detected
            msg.object_lost = False
            msg.status_text = status_text
            return msg
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status_text = status_text
        return msg


def main(args=None) -> None:
    import signal
    rclpy.init(args=args)

    def _sigterm(_sig, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, _sigterm)

    node = GripperServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.boot()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
