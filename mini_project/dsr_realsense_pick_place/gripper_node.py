#!/usr/bin/env python3
# Doosan-E0509-ROBOTIS-RH-P12-RN-TCP-Bridge 패키지의 서비스를 래핑하는 ROS 2 그리퍼 제어 래퍼 노드.

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult

from dsr_gripper_tcp_interfaces.srv import SetPosition, SetTorque
from dsr_gripper_tcp_interfaces.msg import GripperState


class GripperNode(Node):

    def __init__(self):
        super().__init__('rh_p12_rna_gripper')
        cb = ReentrantCallbackGroup()

        self.declare_parameter('robot_ns', 'dsr01')
        self.declare_parameter('svc_timeout', 10.0)
        self.declare_parameter('state_hz', 20.0)
        self.declare_parameter('open_current', 200)
        self.declare_parameter('close_current', 300)
        self.declare_parameter('transport_current', 150)
        self.declare_parameter('idle_current', 50)
        self.declare_parameter('profile_velocity', 1500)
        self.declare_parameter('profile_acceleration', 1000)
        self.declare_parameter('stroke_open_mm', 0.0)
        self.declare_parameter('stroke_close_mm', 92.0)
        self.declare_parameter('grasp_detect_current', 100)

        self._robot_ns = self.get_parameter('robot_ns').value
        self._timeout = self.get_parameter('svc_timeout').value

        self.open_current = self.get_parameter('open_current').value
        self.close_current = self.get_parameter('close_current').value
        self.transport_current = self.get_parameter('transport_current').value
        self.idle_current = self.get_parameter('idle_current').value
        self.profile_velocity = self.get_parameter('profile_velocity').value
        self.profile_acceleration = self.get_parameter('profile_acceleration').value
        self.stroke_open_mm = self.get_parameter('stroke_open_mm').value
        self.stroke_close_mm = self.get_parameter('stroke_close_mm').value
        self.grasp_detect_current = self.get_parameter('grasp_detect_current').value

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._cli_set_position = self.create_client(
            SetPosition, '/gripper_service/set_position',
            callback_group=cb)
        self._cli_set_torque = self.create_client(
            SetTorque, '/gripper_service/set_torque',
            callback_group=cb)

        self._sub_gripper_state = self.create_subscription(
            GripperState, '/gripper_service/state',
            self._cb_gripper_state, 10,
            callback_group=cb)

        self._pub = self.create_publisher(JointState, '/gripper/state', 10)
        self.create_timer(
            1.0 / self.get_parameter('state_hz').value,
            self._pub_state, callback_group=cb)

        self.create_service(Trigger, '/gripper/open',
                            self._srv_open, callback_group=cb)
        self.create_service(Trigger, '/gripper/close',
                            self._srv_close, callback_group=cb)
        self.create_service(Trigger, '/gripper/stop',
                            self._srv_stop, callback_group=cb)
        self.create_service(Trigger, '/gripper/hold_transport',
                            self._srv_hold_transport, callback_group=cb)
        self.create_service(Trigger, '/gripper/hold_idle',
                            self._srv_hold_idle, callback_group=cb)
        self.create_service(SetBool, '/gripper/enable',
                            self._srv_enable, callback_group=cb)

        self._last_state = None
        self._lock = threading.Lock()

        self.get_logger().info(
            '그리퍼 래퍼 노드 기동 — gripper_service(DRL/TCP) 서비스 대기'
        )

    def _on_set_parameters(self, params):
        for param in params:
            if hasattr(self, param.name):
                setattr(self, param.name, param.value)
                self.get_logger().info(f'파라미터 변경: {param.name} -> {param.value}')
        return SetParametersResult(successful=True)

    def _cb_gripper_state(self, msg: GripperState):
        with self._lock:
            self._last_state = msg

    def _pub_state(self):
        with self._lock:
            state = self._last_state

        if state is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['gripper_joint']
        msg.position = [float(state.present_position_mm)]
        msg.velocity = [float(state.present_velocity)]
        msg.effort = [float(state.present_current)]
        self._pub.publish(msg)

    def _call_service(self, client, request, label: str):
        if not client.service_is_ready():
            self.get_logger().error(f'서비스 미연결: {label}')
            return None

        event = threading.Event()
        result = [None]

        def _done(future):
            result[0] = future
            event.set()

        future = client.call_async(request)
        future.add_done_callback(_done)

        if not event.wait(timeout=self._timeout):
            self.get_logger().error(f'타임아웃 ({self._timeout}s): {label}')
            return None

        try:
            return result[0].result()
        except Exception as exc:
            self.get_logger().error(f'서비스 오류 [{label}]: {exc}')
            return None

    def _move(self, position_mm: float, goal_current: int) -> tuple:
        req = SetPosition.Request()
        req.position_mm = float(position_mm)
        req.goal_current = goal_current
        req.profile_velocity = self.profile_velocity
        req.profile_acceleration = self.profile_acceleration
        req.timeout_sec = float(self._timeout)
        res = self._call_service(self._cli_set_position, req, 'set_position')
        if not res or not res.success:
            return False, getattr(res, 'message', '이동 명령 실행 실패')
        return True, '동작 완료'

    def _srv_open(self, _, res: Trigger.Response):
        ok, msg = self._move(self.stroke_open_mm, self.open_current)
        res.success = ok
        res.message = '열기 완료' if ok else f'열기 실패: {msg}'
        return res

    def _srv_hold_transport(self, _, res: Trigger.Response):
        with self._lock:
            state = self._last_state
        pos_mm = float(state.present_position_mm) if state is not None else self.stroke_close_mm
        ok, msg = self._move(pos_mm, self.transport_current)
        res.success = ok
        res.message = (
            f'이송 전류 전환 (pos={pos_mm:.1f}mm, {self.transport_current}mA)'
            if ok else f'이송 전류 전환 실패: {msg}'
        )
        return res

    def _srv_hold_idle(self, _, res: Trigger.Response):
        with self._lock:
            state = self._last_state
        if state is None:
            res.success = False
            res.message = 'idle 위치락 실패 — 그리퍼 상태 미수신'
            return res
        pos_mm = float(state.present_position_mm)
        ok, msg = self._move(pos_mm, self.idle_current)
        res.success = ok
        res.message = (
            f'idle 위치락 (pos={pos_mm:.1f}mm, {self.idle_current}mA)'
            if ok else f'idle 위치락 실패: {msg}'
        )
        return res

    def _srv_close(self, _, res: Trigger.Response):
        ok, msg = self._move(self.stroke_close_mm, self.close_current)
        res.success = ok
        res.message = '닫기 완료' if ok else f'닫기 실패: {msg}'
        return res

    def _srv_stop(self, _, res: Trigger.Response):
        req = SetTorque.Request()
        req.enabled = False
        res_torque = self._call_service(self._cli_set_torque, req, 'set_torque')
        res.success = bool(res_torque and res_torque.success)
        res.message = '토크 비활성화 완료' if res.success else '토크 비활성화 실패'
        return res

    def _srv_enable(self, req: SetBool.Request, res: SetBool.Response):
        torque_req = SetTorque.Request()
        torque_req.enabled = req.data
        label = f"토크 {'활성화' if req.data else '비활성화'}"
        res_torque = self._call_service(self._cli_set_torque, torque_req, 'set_torque')
        res.success = bool(res_torque and res_torque.success)
        res.message = f'{label} 완료' if res.success else f'{label} 실패'
        return res

    def shutdown_safe(self, executor, timeout_sec: float = 2.0):
        try:
            if not self._cli_set_torque.service_is_ready():
                return
            req = SetTorque.Request()
            req.enabled = False
            future = self._cli_set_torque.call_async(req)
            executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        except Exception as exc:
            self.get_logger().warning(f'종료 토크 OFF 실패: {exc}')


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=4)
    node = None
    try:
        node = GripperNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None and rclpy.ok():
            node.shutdown_safe(executor)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
