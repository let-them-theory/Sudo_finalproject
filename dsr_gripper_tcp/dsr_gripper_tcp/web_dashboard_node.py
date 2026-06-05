"""ROS 2 client version of the Doosan gripper web dashboard.

This node hosts the same Flask/SocketIO UI as :mod:`web_dashboard`, but it no
longer owns the TCP bridge. The bridge is owned only by ``gripper_service_node``;
this dashboard subscribes to its state topic and sends commands through its
services.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from flask import Flask, render_template_string
from flask_socketio import SocketIO
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Int32

from dsr_gripper_tcp.web_dashboard import (
    GOAL_CURRENT_MAX,
    GOAL_CURRENT_MIN,
    HTML_TEMPLATE,
    POSITION_MAX,
    PROFILE_ACC_MAX,
    PROFILE_ACC_MIN,
    PROFILE_VEL_MAX,
    PROFILE_VEL_MIN,
)
from dsr_gripper_tcp_interfaces.msg import GripperState
from dsr_gripper_tcp_interfaces.action import SafeGrasp
from dsr_gripper_tcp_interfaces.srv import SetMotionProfile, SetPosition, SetTorque


class GripperWebDashboardNode(Node):
    """Flask/SocketIO dashboard that acts as a ROS client of gripper_service."""

    def __init__(self) -> None:
        super().__init__('gripper_web_dashboard')

        self.declare_parameter('gripper_service_ns', '/gripper_service')
        self.declare_parameter('web_host', '0.0.0.0')
        self.declare_parameter('web_port', 5000)
        self.declare_parameter('joint_name', 'rh_p12_rn')
        self.declare_parameter('position_max', POSITION_MAX)
        self.declare_parameter('move_timeout_sec', 5.0)
        self.declare_parameter('command_timeout_sec', 5.0)
        self.declare_parameter('service_wait_timeout_sec', 2.0)

        gp = self.get_parameter
        self.web_host = gp('web_host').get_parameter_value().string_value
        self.web_port = gp('web_port').get_parameter_value().integer_value
        self._joint_name = gp('joint_name').get_parameter_value().string_value
        self._position_max = max(gp('position_max').get_parameter_value().integer_value, 1)
        self._move_timeout = gp('move_timeout_sec').get_parameter_value().double_value
        self._command_timeout = max(
            gp('command_timeout_sec').get_parameter_value().double_value,
            0.1,
        )
        self._service_wait_timeout = max(
            gp('service_wait_timeout_sec').get_parameter_value().double_value,
            0.1,
        )
        self._service_ns = self._normalize_service_ns(
            gp('gripper_service_ns').get_parameter_value().string_value
        )

        self._last_state: GripperState | None = None
        self._state_lock = threading.Lock()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            GripperState,
            f'{self._service_ns}/state',
            self._on_gripper_state,
            qos,
        )
        self.joint_state_pub = self.create_publisher(JointState, '~/joint_state', qos)
        self.raw_state_pub = self.create_publisher(Float32MultiArray, '~/raw_state', qos)

        self.create_subscription(Int32, '~/goal_position', self._on_goal_position, 10)
        self.create_subscription(Bool, '~/torque_enable', self._on_torque_enable, 10)
        self.create_subscription(Float32MultiArray, '~/motion_profile', self._on_motion_profile, 10)
        self.create_subscription(Bool, '~/emergency_stop', self._on_estop_topic, 10)

        self._set_position_client = self.create_client(
            SetPosition,
            f'{self._service_ns}/set_position',
        )
        self._set_torque_client = self.create_client(
            SetTorque,
            f'{self._service_ns}/set_torque',
        )
        self._set_motion_profile_client = self.create_client(
            SetMotionProfile,
            f'{self._service_ns}/set_motion_profile',
        )
        self._safe_grasp_client = ActionClient(
            self,
            SafeGrasp,
            f'{self._service_ns}/safe_grasp',
        )

        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, async_mode='threading', cors_allowed_origins='*')
        self._register_http_routes()
        self._register_socketio_handlers()

        self.get_logger().info(
            f'GripperWebDashboardNode using {self._service_ns} as gripper service; '
            f'web=http://{self.web_host}:{self.web_port}'
        )

    def wait_for_service_node(self) -> None:
        clients = (
            ('set_position', self._set_position_client),
            ('set_torque', self._set_torque_client),
            ('set_motion_profile', self._set_motion_profile_client),
        )
        for name, client in clients:
            service_name = f'{self._service_ns}/{name}'
            while rclpy.ok() and not client.wait_for_service(timeout_sec=self._service_wait_timeout):
                self.get_logger().info(f'Waiting for {service_name}...')
        while rclpy.ok() and not self._safe_grasp_client.wait_for_server(
            timeout_sec=self._service_wait_timeout
        ):
            self.get_logger().info(f'Waiting for {self._service_ns}/safe_grasp...')

    def shutdown(self) -> None:
        pass

    def run_web_server(self) -> None:
        self.get_logger().info(f'Web server starting on http://{self.web_host}:{self.web_port}')
        try:
            self.socketio.run(
                self.app,
                host=self.web_host,
                port=self.web_port,
                debug=False,
                use_reloader=False,
                allow_unsafe_werkzeug=True,
            )
        except TypeError:
            self.socketio.run(
                self.app,
                host=self.web_host,
                port=self.web_port,
                debug=False,
                use_reloader=False,
            )

    def _on_gripper_state(self, state: GripperState) -> None:
        with self._state_lock:
            self._last_state = state

        self.socketio.emit('state_update', {
            'status': 'ok',
            'present_position': state.present_position,
            'present_current': state.present_current,
            'present_temperature': state.present_temperature,
            'present_velocity': state.present_velocity,
            'moving': int(state.moving),
            'moving_status': state.moving_status,
            'torque_enabled': state.torque_enabled,
        })

        self._publish_legacy_topics(state)

    def _publish_legacy_topics(self, state: GripperState) -> None:
        js = JointState()
        js.header.stamp = state.stamp
        js.name = [self._joint_name]
        js.position = [float(state.present_position) / float(self._position_max)]
        js.velocity = [float(state.present_velocity)]
        js.effort = [float(state.present_current)]
        self.joint_state_pub.publish(js)

        raw = Float32MultiArray()
        raw.data = [
            float(state.present_position),
            float(state.present_current),
            float(state.present_temperature),
            float(state.present_velocity),
            1.0 if state.moving else 0.0,
            float(state.moving_status),
            1.0 if state.torque_enabled else 0.0,
        ]
        self.raw_state_pub.publish(raw)

    def _on_goal_position(self, msg: Int32) -> None:
        self._send_position_command(int(msg.data), self._move_timeout)

    def _on_torque_enable(self, msg: Bool) -> None:
        self._send_torque_command(bool(msg.data))

    def _on_motion_profile(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 3:
            self.get_logger().warning(
                'motion_profile expects [goal_current, profile_velocity, profile_acceleration]'
            )
            return
        self._send_motion_profile_command(int(msg.data[0]), int(msg.data[1]), int(msg.data[2]))

    def _on_estop_topic(self, msg: Bool) -> None:
        if msg.data:
            self._do_estop()

    def _register_http_routes(self) -> None:
        @self.app.route('/')
        def _index():
            return render_template_string(
                HTML_TEMPLATE,
                pos_max=self._position_max,
                cur_min=GOAL_CURRENT_MIN, cur_max=GOAL_CURRENT_MAX,
                vel_min=PROFILE_VEL_MIN, vel_max=PROFILE_VEL_MAX,
                acc_min=PROFILE_ACC_MIN, acc_max=PROFILE_ACC_MAX,
            )

    def _register_socketio_handlers(self) -> None:
        @self.socketio.on('move_cmd')
        def _on_move(data):
            if 'goal_position' not in data:
                return
            self._send_position_command(int(data['goal_position']), self._move_timeout)

        @self.socketio.on('torque_cmd')
        def _on_torque(data):
            if 'enabled' not in data:
                return
            self._send_torque_command(bool(data['enabled']))

        @self.socketio.on('profile_cmd')
        def _on_profile(data):
            self._send_motion_profile_command(
                int(data.get('goal_current', 400)),
                int(data.get('profile_velocity', 1500)),
                int(data.get('profile_acceleration', 1000)),
            )

        @self.socketio.on('safe_grasp_cmd')
        def _on_safe_grasp(data):
            self._send_safe_grasp_goal(
                int(data.get('target_position', 700)),
                int(data.get('max_current', 400)),
                int(data.get('current_delta_threshold', 120)),
                float(data.get('timeout_sec', 8.0)),
            )

    def _send_position_command(self, position: int, timeout_sec: float) -> None:
        request = SetPosition.Request()
        request.position = int(position)
        request.timeout_sec = float(timeout_sec)
        self._call_service_async(self._set_position_client, request, 'set_position')

    def _send_torque_command(self, enabled: bool) -> None:
        request = SetTorque.Request()
        request.enabled = bool(enabled)
        self._call_service_async(self._set_torque_client, request, 'set_torque')

    def _send_motion_profile_command(self, goal_current: int, velocity: int, acceleration: int) -> None:
        request = SetMotionProfile.Request()
        request.goal_current = int(goal_current)
        request.profile_velocity = int(velocity)
        request.profile_acceleration = int(acceleration)
        self._call_service_async(self._set_motion_profile_client, request, 'set_motion_profile')

    def _do_estop(self) -> None:
        with self._state_lock:
            state = self._last_state
        if state is None:
            self.get_logger().warning('E-stop ignored: no gripper state received yet.')
            return
        self._send_position_command(int(state.present_position), 1.0)

    def _send_safe_grasp_goal(
        self,
        target_position: int,
        max_current: int,
        current_delta_threshold: int,
        timeout_sec: float,
    ) -> None:
        def runner():
            if not self._safe_grasp_client.wait_for_server(timeout_sec=self._service_wait_timeout):
                self.get_logger().warning('safe_grasp action server is not available.')
                self.socketio.emit('state_update', {'status': 'error'})
                return

            goal = SafeGrasp.Goal()
            goal.target_position = int(target_position)
            goal.max_current = int(max_current)
            goal.current_delta_threshold = int(current_delta_threshold)
            goal.timeout_sec = float(timeout_sec)

            future = self._safe_grasp_client.send_goal_async(
                goal,
                feedback_callback=self._on_safe_grasp_feedback,
            )
            future.add_done_callback(self._on_safe_grasp_goal_response)

        threading.Thread(target=runner, daemon=True).start()

    def _on_safe_grasp_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            'safe_grasp feedback: '
            f'pos={feedback.present_position}, current={feedback.present_current}, '
            f'delta={feedback.current_delta}, grasp={feedback.grasp_detected}'
        )

    def _on_safe_grasp_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'safe_grasp goal request failed: {exc}')
            self.socketio.emit('state_update', {'status': 'error'})
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('safe_grasp goal was rejected.')
            self.socketio.emit('state_update', {'status': 'error'})
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_safe_grasp_result)

    def _on_safe_grasp_result(self, future) -> None:
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'safe_grasp result failed: {exc}')
            self.socketio.emit('state_update', {'status': 'error'})
            return
        if result.success:
            self.get_logger().info(
                f'safe_grasp succeeded: {result.message}, '
                f'pos={result.final_position}, current={result.final_current}'
            )
        else:
            self.get_logger().warning(
                f'safe_grasp failed: {result.message}, '
                f'pos={result.final_position}, current={result.final_current}'
            )

    def _call_service_async(self, client, request, label: str) -> None:
        def runner():
            if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
                self.get_logger().warning(f'{label} service is not available.')
                self.socketio.emit('state_update', {'status': 'error'})
                return

            future = client.call_async(request)
            deadline = time.monotonic() + self._command_timeout
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)

            if not future.done():
                self.get_logger().warning(f'{label} service call timed out.')
                self.socketio.emit('state_update', {'status': 'error'})
                return

            response = future.result()
            if response is None:
                self.get_logger().warning(f'{label} service returned no response.')
                self.socketio.emit('state_update', {'status': 'error'})
                return
            if hasattr(response, 'success') and not response.success:
                self.get_logger().warning(f'{label} service failed: {response.message}')
                self.socketio.emit('state_update', {'status': 'error'})

        threading.Thread(target=runner, daemon=True).start()

    def _normalize_service_ns(self, namespace: str) -> str:
        normalized = namespace.strip()
        if not normalized:
            return '/gripper_service'
        return '/' + normalized.strip('/')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperWebDashboardNode()

    try:
        node.wait_for_service_node()

        web_thread = threading.Thread(target=node.run_web_server, daemon=True)
        web_thread.start()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
