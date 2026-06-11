from __future__ import annotations

import threading

import rclpy

from dsr_msgs2.srv import SetRobotMode


ROBOT_MODE_AUTONOMOUS = 1


def build_service_root(namespace: str, service_prefix: str = "dsr_controller2") -> str:
    normalized_namespace = namespace.strip("/")
    normalized_prefix = service_prefix.strip("/")
    if normalized_prefix:
        return f"/{normalized_namespace}/{normalized_prefix}"
    return f"/{normalized_namespace}"


def set_robot_mode_autonomous(node, namespace: str, service_prefix: str) -> None:
    service_root = build_service_root(namespace, service_prefix)
    service_name = f"{service_root}/system/set_robot_mode"
    client = node.create_client(SetRobotMode, service_name)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f"Waiting for {service_name}...")

    request = SetRobotMode.Request()
    request.robot_mode = ROBOT_MODE_AUTONOMOUS
    future = client.call_async(request)

    # spin_until_future_complete은 노드를 MultiThreadedExecutor에서 제거하고
    # 글로벌 executor에 붙였다가 반환 시 executor=None으로 만들어 콜백이 멈춘다.
    # threading.Event로 이미 실행 중인 executor가 future를 처리하게 한다.
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if not done.wait(timeout=10.0):
        raise RuntimeError(f"Timeout waiting for robot mode response via {service_name}.")

    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f"Failed to set robot mode to autonomous via {service_name}.")
