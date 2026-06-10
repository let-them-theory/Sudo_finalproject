from __future__ import annotations

import sys
import types
import unittest
from types import SimpleNamespace
from unittest import mock


def _install_ros_stubs() -> None:
    if 'rclpy' not in sys.modules:
        rclpy = types.ModuleType('rclpy')
        rclpy.spin_until_future_complete = lambda *args, **kwargs: None
        sys.modules['rclpy'] = rclpy

    if 'rclpy.node' not in sys.modules:
        node_mod = types.ModuleType('rclpy.node')
        node_mod.Node = object
        sys.modules['rclpy.node'] = node_mod

    if 'dsr_msgs2' not in sys.modules:
        sys.modules['dsr_msgs2'] = types.ModuleType('dsr_msgs2')

    if 'dsr_msgs2.srv' not in sys.modules:
        srv_mod = types.ModuleType('dsr_msgs2.srv')

        class _DummyService:
            class Request:
                pass

        srv_mod.DrlStart = _DummyService
        srv_mod.DrlStop = _DummyService
        srv_mod.GetDrlState = _DummyService
        srv_mod.SetRobotMode = _DummyService
        sys.modules['dsr_msgs2.srv'] = srv_mod


_install_ros_stubs()

from dsr_gripper_tcp.gripper_tcp_bridge import (  # noqa: E402
    Command,
    ControllerStatusError,
    DoosanGripperTcpBridge,
    MoveCommandUncertainError,
    RecoverableBridgeError,
)
from dsr_gripper_tcp.gripper_tcp_protocol import STATE_STRUCT, StatusCode  # noqa: E402


class _Logger:
    def info(self, *args, **kwargs):
        return None

    def warning(self, *args, **kwargs):
        return None


def _make_bridge(command_retry_count: int = 1) -> DoosanGripperTcpBridge:
    bridge = DoosanGripperTcpBridge.__new__(DoosanGripperTcpBridge)
    bridge._config = SimpleNamespace(command_retry_count=command_retry_count)  # noqa: SLF001
    bridge._node = SimpleNamespace(get_logger=lambda: _Logger())  # noqa: SLF001
    bridge._sequence = 1  # noqa: SLF001
    bridge._socket = None  # noqa: SLF001
    return bridge


def _ok_payload(position: int = 500, current: int = 120) -> bytes:
    return STATE_STRUCT.pack(StatusCode.OK, 0, 1, 1, current, 30, 0, position)


def _status_payload(status: StatusCode, position: int = 500, current: int = 120) -> bytes:
    return STATE_STRUCT.pack(status, 0, 1, 1, current, 30, 0, position)


class DoosanGripperTcpBridgeTests(unittest.TestCase):
    def test_request_state_retries_after_recoverable_error(self):
        bridge = _make_bridge(command_retry_count=1)
        calls = {'send': 0, 'reset': 0, 'ensure': 0}

        def fake_send(command, payload, timeout_sec=None):
            calls['send'] += 1
            if calls['send'] == 1:
                raise RecoverableBridgeError('temporary disconnect')
            return _ok_payload(position=640)

        bridge._send_request = fake_send  # noqa: SLF001
        bridge.reset_connection = lambda: calls.__setitem__('reset', calls['reset'] + 1)
        bridge._ensure_socket = lambda: calls.__setitem__('ensure', calls['ensure'] + 1)  # noqa: SLF001

        state = bridge._request_state(Command.READ_STATE, b'', allow_retry=True)  # noqa: SLF001

        self.assertEqual(state.present_position, 640)
        self.assertEqual(calls, {'send': 2, 'reset': 1, 'ensure': 1})

    def test_move_failure_raises_uncertain_error_with_observed_state(self):
        bridge = _make_bridge(command_retry_count=1)

        def fake_send(command, payload, timeout_sec=None):
            if command == Command.MOVE:
                raise RecoverableBridgeError('broken pipe')
            if command == Command.READ_STATE:
                return _ok_payload(position=777, current=210)
            raise AssertionError(f'unexpected command {command}')

        bridge._send_request = fake_send  # noqa: SLF001
        bridge.reset_connection = lambda: None
        bridge._ensure_socket = lambda: None  # noqa: SLF001

        with self.assertRaises(MoveCommandUncertainError) as ctx:
            bridge.move_to(700, timeout_sec=5.0)

        self.assertIsNotNone(ctx.exception.observed_state)
        self.assertEqual(ctx.exception.observed_state.present_position, 777)

    def test_move_controller_status_error_is_not_reported_as_transport_uncertain(self):
        bridge = _make_bridge(command_retry_count=1)
        calls = {'read_state': 0}

        def fake_send(command, payload, timeout_sec=None):
            if command == Command.MOVE:
                return _status_payload(StatusCode.TIMEOUT, position=620)
            if command == Command.READ_STATE:
                calls['read_state'] += 1
                return _ok_payload(position=620)
            raise AssertionError(f'unexpected command {command}')

        bridge._send_request = fake_send  # noqa: SLF001
        bridge.reset_connection = lambda: None
        bridge._ensure_socket = lambda: None  # noqa: SLF001

        with self.assertRaises(ControllerStatusError) as ctx:
            bridge.move_to(700, timeout_sec=5.0)

        self.assertEqual(ctx.exception.state.status, StatusCode.TIMEOUT)
        self.assertEqual(calls['read_state'], 0)


if __name__ == '__main__':
    unittest.main()
