#!/usr/bin/env python3
"""gripper_node 초기화 완료(state.ready)까지 대기 — pick_place 기동 전 health check."""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from dsr_gripper_tcp_interfaces.msg import GripperState


def main(argv: list[str] | None = None) -> int:
    args = argv if argv is not None else sys.argv[1:]
    timeout_sec = float(args[0]) if args else 90.0
    poll_sec = 0.5

    rclpy.init()
    node = Node('wait_for_gripper_ready')
    ready = {'value': False}

    def _cb(msg: GripperState) -> None:
        if msg.ready:
            ready['value'] = True

    node.create_subscription(
        GripperState, '/gripper_service/state', _cb, qos_profile_sensor_data,
    )
    deadline = time.monotonic() + max(timeout_sec, 1.0)
    node.get_logger().info(
        f'gripper ready 대기 (max {timeout_sec:.0f}s, /gripper_service/state)'
    )

    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=poll_sec)
            if ready['value']:
                node.get_logger().info('gripper ready — pick_place 기동 가능')
                return 0

        node.get_logger().error(
            f'gripper ready 타임아웃 ({timeout_sec:.0f}s) — init_progress 로그 확인'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
