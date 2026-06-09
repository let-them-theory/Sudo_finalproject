#!/usr/bin/env python3
"""gripper_service 초기화 완료(state.ready)까지 대기 — pick_place 기동 전 health check."""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node

from dsr_gripper_tcp_interfaces.srv import GetState


def main(argv: list[str] | None = None) -> int:
    args = argv if argv is not None else sys.argv[1:]
    timeout_sec = float(args[0]) if args else 90.0
    poll_sec = 0.5

    rclpy.init()
    node = Node('wait_for_gripper_ready')
    client = node.create_client(GetState, '/gripper_service/get_state')
    deadline = time.monotonic() + max(timeout_sec, 1.0)

    node.get_logger().info(
        f'gripper_service ready 대기 (max {timeout_sec:.0f}s, /gripper_service/get_state)'
    )

    try:
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                time.sleep(poll_sec)
                continue

            req = GetState.Request()
            req.force_read = False
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if not future.done():
                time.sleep(poll_sec)
                continue

            res = future.result()
            if res is not None and res.success and res.state.ready:
                node.get_logger().info('gripper_service ready — pick_place 기동 가능')
                return 0

            time.sleep(poll_sec)

        node.get_logger().error(
            f'gripper_service ready 타임아웃 ({timeout_sec:.0f}s) — 로그에서 INIT 진행 확인'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
