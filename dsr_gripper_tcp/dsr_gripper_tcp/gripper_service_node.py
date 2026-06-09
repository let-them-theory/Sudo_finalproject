# 그리퍼 TCP 브릿지를 ROS 서비스와 액션으로 노출하는 단일 소유 노드
from __future__ import annotations

import os
import signal
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dsr_gripper_tcp.gripper_tcp_bridge import BridgeConfig, DoosanGripperTcpBridge
from dsr_gripper_tcp.gripper_tcp_protocol import GripperState as BridgeState
from dsr_gripper_tcp.gripper_semantics import GripperSemanticEvaluator, SemanticStateSnapshot
from dsr_gripper_tcp.robot_utils import set_robot_mode_autonomous
from dsr_gripper_tcp_interfaces.action import SafeGrasp
from dsr_gripper_tcp_interfaces.msg import GripperState
from dsr_gripper_tcp_interfaces.srv import (
    GetMotionProfile,
    GetPosition,
    GetState,
    SetMotionProfile,
    SetPosition,
    SetTorque,
)


class GripperServiceNode(Node):
    """Single owner for the gripper TCP bridge, exposed through ROS services."""

    def __init__(self) -> None:
        super().__init__('gripper_service')

        # Controller / bridge parameters.
        self.declare_parameter('controller_host', '110.120.1.56')
        self.declare_parameter('tcp_port', 20002)
        self.declare_parameter('namespace', 'dsr01')
        self.declare_parameter('service_prefix', '')
        self.declare_parameter('skip_set_autonomous', False)
        self.declare_parameter('initialize_on_start', True)
        self.declare_parameter('goal_current', 400)
        self.declare_parameter('profile_velocity', 1500)
        self.declare_parameter('profile_acceleration', 1000)
        self.declare_parameter('connect_timeout_sec', 60.0)
        self.declare_parameter('post_drl_start_sleep_sec', 2.0)
        self.declare_parameter('stop_existing_drl', True)
        self.declare_parameter('drl_stop_mode', 1)
        self.declare_parameter('drl_stop_settle_sec', 5.0)
        self.declare_parameter('drl_idle_stable_sec', 2.0)
        self.declare_parameter('drl_start_retry_count', 3)
        self.declare_parameter('drl_start_retry_delay_sec', 1.0)
        self.declare_parameter('tcp_server_open_retry_sec', 0.5)
        self.declare_parameter('init_attempts', 5)
        self.declare_parameter('init_timeout_sec', 40.0)  # DRL의 ~36초 콜드부팅 윈도우보다 커야 함
        self.declare_parameter('init_retry_delay_sec', 1.0)

        # Service node behavior.
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('joint_name', 'rh_p12_rn')
        self.declare_parameter('position_max', 1150)
        self.declare_parameter('default_move_timeout_sec', 5.0)
        self.declare_parameter('default_safe_grasp_timeout_sec', 10.0)
        self.declare_parameter('safe_grasp_feedback_rate_hz', 10.0)
        self.declare_parameter('grasp_current_threshold', 300)
        self.declare_parameter('object_lost_current_threshold', 80)
        self.declare_parameter('object_lost_position_delta', 80)
        self.declare_parameter('state_poll_timeout_sec', 2.0)
        self.declare_parameter('command_retry_count', 1)

        gp = self.get_parameter
        self.robot_namespace = gp('namespace').get_parameter_value().string_value
        self.service_prefix = gp('service_prefix').get_parameter_value().string_value
        self.skip_set_autonomous = gp('skip_set_autonomous').get_parameter_value().bool_value
        self.initialize_on_start = gp('initialize_on_start').get_parameter_value().bool_value
        self._joint_name = gp('joint_name').get_parameter_value().string_value
        self._position_max = gp('position_max').get_parameter_value().integer_value
        self._poll_rate_hz = max(gp('poll_rate_hz').get_parameter_value().double_value, 1.0)
        self._default_move_timeout = gp('default_move_timeout_sec').get_parameter_value().double_value
        self._default_safe_grasp_timeout = gp(
            'default_safe_grasp_timeout_sec'
        ).get_parameter_value().double_value
        self._safe_grasp_feedback_rate_hz = max(
            gp('safe_grasp_feedback_rate_hz').get_parameter_value().double_value,
            1.0,
        )
        self._grasp_current_threshold = gp('grasp_current_threshold').get_parameter_value().integer_value
        self._object_lost_current_threshold = gp(
            'object_lost_current_threshold'
        ).get_parameter_value().integer_value
        self._object_lost_position_delta = gp(
            'object_lost_position_delta'
        ).get_parameter_value().integer_value
        self._state_poll_timeout = max(
            gp('state_poll_timeout_sec').get_parameter_value().double_value,
            0.1,
        )

        self._goal_current = gp('goal_current').get_parameter_value().integer_value
        self._profile_velocity = gp('profile_velocity').get_parameter_value().integer_value
        self._profile_acceleration = gp('profile_acceleration').get_parameter_value().integer_value
        self._last_goal_position = 0
        self._last_state: GripperState | None = None

        cfg = BridgeConfig(
            controller_host=gp('controller_host').get_parameter_value().string_value,
            tcp_port=gp('tcp_port').get_parameter_value().integer_value,
            namespace=self.robot_namespace,
            service_prefix=self.service_prefix,
            goal_current=self._goal_current,
            profile_velocity=self._profile_velocity,
            profile_acceleration=self._profile_acceleration,
            connect_timeout_sec=gp('connect_timeout_sec').get_parameter_value().double_value,
            post_drl_start_sleep_sec=gp(
                'post_drl_start_sleep_sec'
            ).get_parameter_value().double_value,
            stop_existing_drl=gp('stop_existing_drl').get_parameter_value().bool_value,
            drl_stop_mode=gp('drl_stop_mode').get_parameter_value().integer_value,
            drl_stop_settle_sec=gp('drl_stop_settle_sec').get_parameter_value().double_value,
            drl_idle_stable_sec=gp('drl_idle_stable_sec').get_parameter_value().double_value,
            drl_start_retry_count=gp('drl_start_retry_count').get_parameter_value().integer_value,
            drl_start_retry_delay_sec=gp(
                'drl_start_retry_delay_sec'
            ).get_parameter_value().double_value,
            command_retry_count=gp('command_retry_count').get_parameter_value().integer_value,
            tcp_server_open_retry_sec=gp(
                'tcp_server_open_retry_sec'
            ).get_parameter_value().double_value,
        )

        self._bridge_lock = threading.Lock()
        # 재초기화(reinit) 진행 중 표시 — 폴링/명령이 ~90s 동안 lock에서 블록되지
        # 않도록, 이 플래그가 세워져 있으면 브릿지 접근을 건너뛴다.
        self._reinitializing = threading.Event()
        self._state_lock = threading.Lock()
        self._bridge = DoosanGripperTcpBridge(node=self, config=cfg)
        self._semantic_evaluator = GripperSemanticEvaluator(
            grasp_current_threshold=self._grasp_current_threshold,
            object_lost_current_threshold=self._object_lost_current_threshold,
            object_lost_position_delta=self._object_lost_position_delta,
        )
        self._callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._state_pub = self.create_publisher(GripperState, '~/state', qos)
        # INIT/Reinit 진행 상황 (각 retry attempt마다 발행). GUI가 "막힘 vs 진행 중" 구분용.
        self._init_progress_pub = self.create_publisher(String, '~/init_progress', 10)
        self._joint_state_pub = self.create_publisher(JointState, '~/joint_state', qos)

        self.create_service(
            GetState,
            '~/get_state',
            self._handle_get_state,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetPosition,
            '~/get_position',
            self._handle_get_position,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetPosition,
            '~/set_position',
            self._handle_set_position,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetMotionProfile,
            '~/set_motion_profile',
            self._handle_set_motion_profile,
            callback_group=self._callback_group,
        )
        self.create_service(
            GetMotionProfile,
            '~/get_motion_profile',
            self._handle_get_motion_profile,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetTorque,
            '~/set_torque',
            self._handle_set_torque,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            '~/reinitialize',
            self._handle_reinitialize,
            callback_group=self._callback_group,
        )

        self._safe_grasp_action = ActionServer(
            self,
            SafeGrasp,
            '~/safe_grasp',
            execute_callback=self._execute_safe_grasp,
            goal_callback=self._handle_safe_grasp_goal,
            cancel_callback=self._handle_safe_grasp_cancel,
            callback_group=self._callback_group,
        )

        self._poll_timer = None

    def boot_bridge(self) -> None:
        if not self.skip_set_autonomous:
            self.get_logger().info('Setting robot mode to autonomous...')
            set_robot_mode_autonomous(self, self.robot_namespace, self.service_prefix)

        self.get_logger().info('Starting DRL TCP gripper server...')
        self._bridge.start()

        if self.initialize_on_start:
            attempts = self.get_parameter('init_attempts').get_parameter_value().integer_value
            timeout_sec = self.get_parameter('init_timeout_sec').get_parameter_value().double_value
            retry_delay = self.get_parameter('init_retry_delay_sec').get_parameter_value().double_value
            with self._bridge_lock:
                state = self._bridge.initialize_with_retry(
                    attempts=attempts,
                    timeout_sec=timeout_sec,
                    retry_delay_sec=retry_delay,
                    progress_callback=self._make_init_progress_callback('INIT'),
                )
            self._update_cached_state(state, 'initialized')

        self._poll_timer = self.create_timer(1.0 / self._poll_rate_hz, self._poll_state)
        self.get_logger().info(f'Gripper service node ready at {self._poll_rate_hz:.1f} Hz')

    def shutdown(self) -> None:
        self._safe_grasp_action.destroy()
        try:
            self._bridge.close(shutdown_remote=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Bridge close failed: {exc}')

    def _make_init_progress_callback(self, tag: str):
        """initialize_with_retry용 progress 콜백 — 각 시도/실패/성공마다 ~/init_progress 발행.
        GUI는 이 String 토픽으로 "막힘 vs 진행 중" 구분 (시간이 카운트되면 진행, 멈춰있으면 막힘)."""
        start = time.monotonic()
        def cb(attempt: int, total: int, status: str) -> None:
            elapsed = time.monotonic() - start
            msg = String()
            msg.data = f'{tag} {attempt}/{total} | {elapsed:.0f}s | {status}'
            try:
                self._init_progress_pub.publish(msg)
            except Exception:
                pass
            # 터미널에도 한 줄 INFO로 (warning은 bridge가 이미 출력함)
            self.get_logger().info(f'[{tag}] {attempt}/{total} ({elapsed:.0f}s): {status}')
        return cb

    def _poll_state(self) -> None:
        if self._reinitializing.is_set():
            # 재초기화 진행 중엔 브릿지 접근을 건너뛴다(lock 블로킹/Modbus 충돌 방지).
            return
        try:
            bridge_state = self._read_bridge_state()
            state_msg = self._update_cached_state(bridge_state, 'ok')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Gripper state polling failed: {exc}', throttle_duration_sec=2.0)
            state_msg = self._last_state_or_empty(str(exc))
        self._state_pub.publish(state_msg)
        self._publish_joint_state(state_msg)

    def _handle_get_state(self, request, response):
        try:
            state_msg = self._get_state(force_read=bool(request.force_read))
            response.success = True
            response.message = 'ok'
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_get_position(self, request, response):
        try:
            state_msg = self._get_state(force_read=bool(request.force_read))
            response.success = True
            response.message = 'ok'
            response.present_position = state_msg.present_position
            response.goal_position = state_msg.goal_position
            response.present_current = state_msg.present_current
            response.present_velocity = state_msg.present_velocity
            response.moving = state_msg.moving
            response.in_position = state_msg.in_position
            response.torque_enabled = state_msg.torque_enabled
            response.grasp_detected = state_msg.grasp_detected
            response.object_lost = state_msg.object_lost
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _handle_set_position(self, request, response):
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else self._default_move_timeout
        try:
            with self._bridge_lock:
                bridge_state = self._bridge.move_to(int(request.position), timeout_sec=timeout)
            self._last_goal_position = int(request.position)
            state_msg = self._update_cached_state(bridge_state, 'position set')
            response.success = True
            response.message = 'ok'
            response.present_position = state_msg.present_position
            response.goal_position = state_msg.goal_position
            response.present_current = state_msg.present_current
            response.in_position = state_msg.in_position
            response.grasp_detected = state_msg.grasp_detected
            response.object_lost = state_msg.object_lost
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_set_motion_profile(self, request, response):
        try:
            with self._bridge_lock:
                bridge_state = self._bridge.set_motion_profile(
                    goal_current=int(request.goal_current),
                    profile_velocity=int(request.profile_velocity),
                    profile_acceleration=int(request.profile_acceleration),
                )
            self._goal_current = int(request.goal_current)
            self._profile_velocity = int(request.profile_velocity)
            self._profile_acceleration = int(request.profile_acceleration)
            state_msg = self._update_cached_state(bridge_state, 'motion profile set')
            response.success = True
            response.message = 'ok'
            response.goal_current = self._goal_current
            response.profile_velocity = self._profile_velocity
            response.profile_acceleration = self._profile_acceleration
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.goal_current = self._goal_current
            response.profile_velocity = self._profile_velocity
            response.profile_acceleration = self._profile_acceleration
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_get_motion_profile(self, request, response):  # noqa: ARG002
        response.success = True
        response.message = 'cached profile'
        response.goal_current = self._goal_current
        response.profile_velocity = self._profile_velocity
        response.profile_acceleration = self._profile_acceleration
        return response

    def _handle_set_torque(self, request, response):
        try:
            with self._bridge_lock:
                bridge_state = self._bridge.set_torque(bool(request.enabled))
            state_msg = self._update_cached_state(bridge_state, 'torque set')
            response.success = True
            response.message = 'ok'
            response.torque_enabled = state_msg.torque_enabled
            response.state = state_msg
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            response.torque_enabled = False
            response.state = self._last_state_or_empty(str(exc))
        return response

    def _handle_reinitialize(self, request, response):
        """런타임 그리퍼 리셋: DRL 서버 재시작 + 재초기화(시리얼 recycle + 토크 + 프로파일).

        그리퍼가 동작 중 에러/무응답(status 3)으로 멈췄을 때, 로봇 재부팅 없이
        flange 시리얼 포트를 재활용하고 토크를 다시 인가해 복구를 시도한다.
        boot_bridge의 초기화 시퀀스를 그대로 재사용한다.
        """
        attempts = self.get_parameter('init_attempts').get_parameter_value().integer_value
        timeout_sec = self.get_parameter('init_timeout_sec').get_parameter_value().double_value
        retry_delay = self.get_parameter('init_retry_delay_sec').get_parameter_value().double_value
        # 폴링/다른 명령이 reinit 동안 lock에서 블록되지 않도록 플래그를 먼저 세운다.
        self._reinitializing.set()
        try:
            with self._bridge_lock:
                self.get_logger().info('Re-initializing gripper (runtime reset)...')
                # 이전 세션의 (이미 죽었을 수 있는) 소켓을 먼저 정리한 뒤 DRL을 재시작한다.
                # force_stop_first=True — config 무관하게 DrlStop+wait 강제. stuck DRL이 시리얼
                # 포트를 쥔 채 남아 Modbus 충돌(STATUS_IO_ERROR)을 만드는 시나리오 차단 (audit #3).
                self._bridge.reset_connection()
                self._bridge.start(force_stop_first=True)
                state = self._bridge.initialize_with_retry(
                    attempts=attempts,
                    timeout_sec=timeout_sec,
                    retry_delay_sec=retry_delay,
                    progress_callback=self._make_init_progress_callback('REINIT'),
                )
            self._update_cached_state(state, 'reinitialized')
            response.success = True
            response.message = 'gripper reinitialized'
            self.get_logger().info('Gripper re-initialized successfully.')
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f'Gripper re-initialize failed: {exc}')
        finally:
            self._reinitializing.clear()
        return response

    def _handle_safe_grasp_goal(self, goal_request):
        if goal_request.target_position < 0 or goal_request.target_position > self._position_max:
            self.get_logger().warning(f'Rejecting safe_grasp target={goal_request.target_position}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_safe_grasp_cancel(self, goal_handle):  # noqa: ARG002
        return CancelResponse.ACCEPT

    def _execute_safe_grasp(self, goal_handle):
        goal = goal_handle.request
        result = SafeGrasp.Result()
        timeout_sec = float(goal.timeout_sec) if goal.timeout_sec > 0 else self._default_safe_grasp_timeout
        max_current = abs(int(goal.max_current)) if goal.max_current > 0 else self._goal_current
        delta_threshold = abs(int(goal.current_delta_threshold))
        feedback_interval = 1.0 / self._safe_grasp_feedback_rate_hz
        position_tolerance = 5

        try:
            with self._bridge_lock:
                self._bridge.set_motion_profile(
                    goal_current=max_current,
                    profile_velocity=self._profile_velocity,
                    profile_acceleration=self._profile_acceleration,
                )
                start_state = self._bridge.read_state(timeout_sec=self._state_poll_timeout)
            self._goal_current = max_current
            baseline_current = abs(int(start_state.present_current))
            target_position = int(goal.target_position)
            self._last_goal_position = target_position

            start_msg = self._update_cached_state(start_state, 'safe grasp starting')
            goal_handle.publish_feedback(
                self._build_safe_grasp_feedback(start_msg, current_delta=0, grasp_detected=False)
            )

            if goal_handle.is_cancel_requested:
                state_msg = self._hold_current_position()
                result.success = False
                result.message = 'safe_grasp canceled'
                result.final_position = state_msg.present_position
                result.final_current = state_msg.present_current
                result.grasp_detected = state_msg.grasp_detected
                result.object_lost = state_msg.object_lost
                result.state = state_msg
                goal_handle.canceled()
                return result

            with self._bridge_lock:
                self._bridge.move_to(target_position, timeout_sec=0.0)

            deadline = time.monotonic() + timeout_sec
            last_state_msg = start_msg
            observed_motion = False

            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    state_msg = self._hold_current_position()
                    result.success = False
                    result.message = 'safe_grasp canceled'
                    result.final_position = state_msg.present_position
                    result.final_current = state_msg.present_current
                    result.grasp_detected = state_msg.grasp_detected
                    result.object_lost = state_msg.object_lost
                    result.state = state_msg
                    goal_handle.canceled()
                    return result

                bridge_state = self._read_bridge_state()
                last_state_msg = self._update_cached_state(bridge_state, 'safe grasp polling')
                observed_motion = observed_motion or bool(last_state_msg.moving)
                target_reached = (
                    abs(last_state_msg.present_position - target_position) <= position_tolerance
                    or (observed_motion and last_state_msg.in_position)
                )
                current_abs = abs(int(bridge_state.present_current))
                current_delta = abs(current_abs - baseline_current)
                grasp_detected = current_abs >= max_current or (
                    delta_threshold > 0 and current_delta >= delta_threshold
                )

                goal_handle.publish_feedback(
                    self._build_safe_grasp_feedback(
                        last_state_msg,
                        current_delta=current_delta,
                        grasp_detected=grasp_detected,
                    )
                )

                if grasp_detected:
                    last_state_msg.grasp_detected = True
                    result.success = True
                    result.message = 'grasp detected'
                    result.final_position = last_state_msg.present_position
                    result.final_current = last_state_msg.present_current
                    result.grasp_detected = True
                    result.object_lost = last_state_msg.object_lost
                    result.state = last_state_msg
                    goal_handle.succeed()
                    return result

                if not last_state_msg.moving and target_reached:
                    result.success = False
                    result.message = 'target reached without grasp'
                    result.final_position = last_state_msg.present_position
                    result.final_current = last_state_msg.present_current
                    result.grasp_detected = False
                    result.object_lost = last_state_msg.object_lost
                    result.state = last_state_msg
                    goal_handle.abort()
                    return result

                time.sleep(feedback_interval)

            timeout_state = self._last_state_or_empty('safe_grasp timeout')
            result.success = False
            result.message = 'safe_grasp timeout'
            result.final_position = timeout_state.present_position
            result.final_current = timeout_state.present_current
            result.grasp_detected = timeout_state.grasp_detected
            result.object_lost = timeout_state.object_lost
            result.state = timeout_state
            goal_handle.abort()
            return result
        except Exception as exc:  # noqa: BLE001
            result.success = False
            result.message = str(exc)
            result.state = self._last_state_or_empty(str(exc))
            result.final_position = result.state.present_position
            result.final_current = result.state.present_current
            result.grasp_detected = result.state.grasp_detected
            result.object_lost = result.state.object_lost
            goal_handle.abort()
            return result

    def _hold_current_position(self) -> GripperState:
        with self._bridge_lock:
            bridge_state = self._bridge.read_state(timeout_sec=self._state_poll_timeout)
            bridge_state = self._bridge.move_to(int(bridge_state.present_position), timeout_sec=1.0)
        return self._update_cached_state(bridge_state, 'holding current position')

    def _get_state(self, force_read: bool = False) -> GripperState:
        if force_read:
            bridge_state = self._read_bridge_state()
            return self._update_cached_state(bridge_state, 'ok')

        with self._state_lock:
            if self._last_state is not None:
                return self._clone_state_msg(self._last_state)

        bridge_state = self._read_bridge_state()
        return self._update_cached_state(bridge_state, 'ok')

    def _read_bridge_state(self) -> BridgeState:
        with self._bridge_lock:
            return self._bridge.read_state(timeout_sec=self._state_poll_timeout)

    def _update_cached_state(self, bridge_state: BridgeState, status_text: str) -> GripperState:
        snapshot = self._semantic_evaluator.evaluate(
            bridge_state,
            goal_position=self._last_goal_position,
            current_limit=self._goal_current,
            status_text=status_text,
        )
        msg = self._state_msg_from_snapshot(snapshot)
        with self._state_lock:
            self._last_state = self._clone_state_msg(msg)
        return msg

    def _state_msg_from_snapshot(self, snapshot: SemanticStateSnapshot) -> GripperState:
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.ready = snapshot.ready
        msg.torque_enabled = snapshot.torque_enabled
        msg.moving = snapshot.moving
        msg.in_position = snapshot.in_position
        msg.status = snapshot.status
        msg.moving_status = snapshot.moving_status
        msg.present_position = snapshot.present_position
        msg.goal_position = snapshot.goal_position
        msg.present_current = snapshot.present_current
        msg.current_limit = snapshot.current_limit
        msg.present_velocity = snapshot.present_velocity
        msg.present_temperature = snapshot.present_temperature
        msg.grasp_detected = snapshot.grasp_detected
        msg.object_lost = snapshot.object_lost
        msg.status_text = snapshot.status_text
        return msg

    def _last_state_or_empty(self, status_text: str) -> GripperState:
        with self._state_lock:
            if self._last_state is not None:
                state = self._clone_state_msg(self._last_state)
                state.stamp = self.get_clock().now().to_msg()
                state.status_text = status_text
                return state
        msg = GripperState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status_text = status_text
        return msg

    def _clone_state_msg(self, state: GripperState) -> GripperState:
        msg = GripperState()
        msg.stamp = state.stamp
        msg.ready = state.ready
        msg.torque_enabled = state.torque_enabled
        msg.moving = state.moving
        msg.in_position = state.in_position
        msg.grasp_detected = state.grasp_detected
        msg.object_lost = state.object_lost
        msg.status = state.status
        msg.moving_status = state.moving_status
        msg.present_position = state.present_position
        msg.goal_position = state.goal_position
        msg.present_current = state.present_current
        msg.current_limit = state.current_limit
        msg.present_velocity = state.present_velocity
        msg.present_temperature = state.present_temperature
        msg.status_text = state.status_text
        return msg

    def _build_safe_grasp_feedback(
        self,
        state_msg: GripperState,
        current_delta: int,
        grasp_detected: bool,
    ) -> SafeGrasp.Feedback:
        feedback = SafeGrasp.Feedback()
        feedback.present_position = state_msg.present_position
        feedback.present_current = state_msg.present_current
        feedback.current_delta = int(current_delta)
        feedback.grasp_detected = bool(grasp_detected)
        feedback.object_lost = state_msg.object_lost
        feedback.state = self._clone_state_msg(state_msg)
        return feedback

    def _publish_joint_state(self, state_msg: GripperState) -> None:
        msg = JointState()
        msg.header.stamp = state_msg.stamp
        msg.name = [self._joint_name]
        msg.position = [float(state_msg.present_position) / float(max(self._position_max, 1))]
        msg.velocity = [float(state_msg.present_velocity)]
        msg.effort = [float(state_msg.present_current)]
        self._joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)

    def _request_shutdown(signum, _frame):  # noqa: ARG001
        # boot_bridge() 블로킹 중 KeyboardInterrupt가 늦게 처리되면 launch가 고아로 남긴다.
        node.get_logger().info(f'종료 신호 수신 (signum={signum}) — gripper bridge 즉시 정리')
        try:
            node.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        os._exit(0)

    signal.signal(signal.SIGTERM, _request_shutdown)
    signal.signal(signal.SIGINT, _request_shutdown)

    try:
        node.boot_bridge()
        executor.add_node(node)
        # 이 시점 이후 _call_service는 executor가 spin하므로 콜백 안전(call_async+Event)
        # 경로를 써야 한다. boot_bridge는 위에서 spin 전 컨텍스트(직접 spin)로 이미 끝났다.
        node._bridge._executor_active = True
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
