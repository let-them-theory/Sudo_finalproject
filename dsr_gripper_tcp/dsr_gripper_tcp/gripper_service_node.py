from __future__ import annotations

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from dsr_msgs2.srv import DrlStart, DrlStop, SetRobotMode

from std_srvs.srv import Trigger

from dsr_gripper_tcp.gripper_tcp_bridge import GripperBridge, build_drl_script
from dsr_gripper_tcp.gripper_tcp_protocol import GripperState as BridgeState, raw_to_mm
from dsr_gripper_tcp.robot_utils import ROBOT_MODE_AUTONOMOUS, build_service_root

from dsr_gripper_tcp_interfaces.msg import GripperState
from dsr_gripper_tcp_interfaces.srv import SetPosition, SetTorque


class GripperServiceNode(Node):

    def __init__(self) -> None:
        super().__init__('gripper_service')

        self.declare_parameter('controller_host', '110.120.1.50')
        self.declare_parameter('tcp_port', 20002)
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('goal_current', 400)
        self.declare_parameter('profile_velocity', 1500)
        self.declare_parameter('profile_acceleration', 1000)
        self.declare_parameter('poll_rate_hz', 10.0)
        self.declare_parameter('grasp_current_threshold', 300)
        self.declare_parameter('default_move_timeout_sec', 10.0)
        self.declare_parameter('skip_set_autonomous', False)
        self.declare_parameter('drl_stop_settle_sec', 2.0)
        self.declare_parameter('drl_start_retries', 3)
        self.declare_parameter('drl_start_timeout_sec', 60.0)
        self.declare_parameter('post_drl_start_sleep_sec', 2.0)
        self.declare_parameter('init_attempts', 3)

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
        self._drl_stop_settle = gp('drl_stop_settle_sec').get_parameter_value().double_value
        self._drl_start_retries = max(1, gp('drl_start_retries').get_parameter_value().integer_value)
        self._drl_start_timeout = gp('drl_start_timeout_sec').get_parameter_value().double_value
        self._post_drl_start_sleep = gp('post_drl_start_sleep_sec').get_parameter_value().double_value
        self._init_attempts = max(1, gp('init_attempts').get_parameter_value().integer_value)
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
        self._last_goal_position_mm = 0.0

        cb = ReentrantCallbackGroup()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._state_pub = self.create_publisher(GripperState, '~/state', qos)

        self.create_service(SetPosition, '~/set_position', self._handle_set_position, callback_group=cb)
        self.create_service(SetTorque, '~/set_torque', self._handle_set_torque, callback_group=cb)
        self.create_service(Trigger, '~/reinitialize', self._handle_reinitialize, callback_group=cb)

        self._poll_timer = None

    # ── Boot: resume first, cold start only if needed ─────────────────────

    def boot(self) -> None:
        if not self._skip_autonomous:
            self.get_logger().info('Setting robot to autonomous mode...')
            self._set_robot_mode_autonomous()

        if self._try_resume_session():
            self.get_logger().info('Gripper session resumed (no DRL restart)')
        else:
            self.get_logger().info('Cold start — DRL gripper server will be started once')
            self._cold_start_drl_and_connect()
            self._initialize_gripper()

        self._poll_timer = self.create_timer(1.0 / self._poll_rate_hz, self._poll_state)
        self.get_logger().info(f'Gripper service ready at {self._poll_rate_hz:.1f} Hz')

    def _try_resume_session(self) -> bool:
        """기존 DRL/TCP 세션이 살아 있으면 재사용. 매번 drl_stop/drl_start 하지 않음."""
        try:
            self._bridge.connect(timeout=3.0)
        except RuntimeError:
            self._bridge.reset()
            return False

        try:
            with self._lock:
                state = self._bridge.read_state(timeout=3.0)
            if state.status == 0 and state.torque_enabled:
                self._ready = True
                self._cache_state(state)
                self.get_logger().info('Existing session healthy (torque on)')
                return True

            self.get_logger().info('TCP up but gripper not ready — INITIALIZE only (no DRL restart)')
            with self._lock:
                state = self._bridge.initialize(
                    goal_current=self._goal_current, timeout=40.0,
                )
            self._ready = True
            self._cache_state(state)
            return True
        except Exception as exc:
            self.get_logger().warning(f'Resume failed ({exc}) — falling back to cold start')
            self._bridge.reset()
            return False

    def _initialize_gripper(self) -> None:
        self.get_logger().info('Initializing gripper...')
        last_exc: RuntimeError | None = None
        for attempt in range(1, self._init_attempts + 1):
            try:
                with self._lock:
                    state = self._bridge.initialize(
                        goal_current=self._goal_current,
                        timeout=40.0,
                    )
                self._ready = True
                self._cache_state(state)
                self.get_logger().info(f'Gripper initialized (attempt {attempt})')
                return
            except RuntimeError as exc:
                last_exc = exc
                self.get_logger().warning(
                    f'Initialize attempt {attempt}/{self._init_attempts} failed: {exc}'
                )
                if attempt < self._init_attempts:
                    time.sleep(1.0)
        raise RuntimeError(
            f'Gripper initialization failed after {self._init_attempts} attempts: {last_exc}'
        )

    def shutdown(self) -> None:
        with self._lock:
            self._bridge.close()

    # ── DRL: cold start only ───────────────────────────────────────────────

    def _cold_start_drl_and_connect(self) -> None:
        self._stop_drl_best_effort()

        start_req = DrlStart.Request()
        start_req.robot_system = 0
        start_req.code = self._drl_script

        last_err = 'unknown'
        for attempt in range(1, self._drl_start_retries + 1):
            self.get_logger().info(
                f'Starting DRL gripper server (attempt {attempt}/{self._drl_start_retries})...'
            )
            resp = self._call_service(
                self._cli_drl_start, start_req, timeout_sec=self._drl_start_timeout,
            )
            if resp is not None and resp.success:
                break
            last_err = 'no response' if resp is None else 'success=false'
            self.get_logger().warning(f'DrlStart attempt {attempt} failed ({last_err})')
            if attempt < self._drl_start_retries:
                time.sleep(2.0)
        else:
            raise RuntimeError(
                f'DrlStart failed after {self._drl_start_retries} attempts ({last_err})'
            )

        if self._post_drl_start_sleep > 0:
            time.sleep(self._post_drl_start_sleep)

        self.get_logger().info('Connecting to gripper TCP server...')
        self._bridge.connect(timeout=30.0)
        self.get_logger().info('TCP connected')

    def _stop_drl_best_effort(self) -> None:
        """종료/복구 시에만 호출. 정상 boot 에서는 resume 실패 시 1회만."""
        with self._lock:
            self._bridge.close()
            self._bridge.reset()

        stop_req = DrlStop.Request()
        stop_req.stop_mode = 1
        stop_resp = self._call_service(self._cli_drl_stop, stop_req, timeout_sec=15.0)
        if stop_resp is not None:
            self.get_logger().info(f'drl_stop success={stop_resp.success}')
        time.sleep(max(self._drl_stop_settle, 1.0))

    def _set_robot_mode_autonomous(self) -> None:
        svc_root = build_service_root(self._namespace, '')
        service_name = f'{svc_root}/system/set_robot_mode'
        client = self.create_client(SetRobotMode, service_name)
        deadline = time.monotonic() + 30.0
        while not client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline:
                raise RuntimeError(f'Timeout waiting for {service_name}')
            self.get_logger().info(f'Waiting for {service_name}...')

        req = SetRobotMode.Request()
        req.robot_mode = ROBOT_MODE_AUTONOMOUS
        resp = self._call_service(client, req, timeout_sec=15.0)
        if resp is None or not resp.success:
            raise RuntimeError(f'Failed to set robot mode to autonomous via {service_name}')

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

    def _handle_set_position(self, request: SetPosition.Request, response: SetPosition.Response):
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else self._move_timeout
        goal_current = int(request.goal_current) if request.goal_current > 0 else self._goal_current
        profile_velocity = int(request.profile_velocity) if request.profile_velocity > 0 else self._profile_velocity
        profile_acceleration = int(request.profile_acceleration) if request.profile_acceleration > 0 else self._profile_acceleration
        try:
            with self._lock:
                state = self._bridge.move_to(
                    position_mm=float(request.position_mm),
                    goal_current=goal_current,
                    profile_velocity=profile_velocity,
                    profile_acceleration=profile_acceleration,
                    timeout_sec=timeout,
                )
            self._last_goal_position_mm = float(request.position_mm)
            self._cache_state(state)
            response.success = True
            response.message = 'ok'
            response.present_position_mm = self._last_state.present_position_mm
            response.present_current = self._last_state.present_current
            response.in_position = self._last_state.in_position
            response.grasp_detected = self._last_state.grasp_detected
            response.state = self._last_state
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.state = self._empty_state(str(exc))
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
        """명시적 복구 요청 시에만 DRL 재시작 (정상 boot 와 분리)."""
        try:
            with self._lock:
                self._ready = False
                self._stop_drl_best_effort()
                self._cold_start_drl_and_connect()
                state = self._bridge.initialize(
                    goal_current=self._goal_current, timeout=40.0,
                )
            self._ready = True
            self._cache_state(state)
            response.success = True
            response.message = 'reinitialized'
            self.get_logger().info('Gripper reinitialized (manual)')
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Reinitialize failed: {exc}')
        return response

    # ── Helpers ────────────────────────────────────────────────────────────

    def _call_service(self, client, req, timeout_sec: float = 10.0):
        future = client.call_async(req)
        deadline = time.monotonic() + max(timeout_sec, 1.0)
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().error(f'Service call timeout ({timeout_sec:.0f}s)')
                return None
            time.sleep(0.05)
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'Service call failed: {exc}')
            return None

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
        msg.present_position_mm = float(raw_to_mm(bridge_state.present_position))
        msg.goal_position = round(self._last_goal_position_mm * 1150.0 / 106.0)
        msg.goal_position_mm = float(self._last_goal_position_mm)
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
            msg.present_position_mm = self._last_state.present_position_mm
            msg.goal_position = self._last_state.goal_position
            msg.goal_position_mm = self._last_state.goal_position_mm
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
