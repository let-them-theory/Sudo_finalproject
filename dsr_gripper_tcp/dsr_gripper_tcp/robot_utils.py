from __future__ import annotations

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
    rclpy.spin_until_future_complete(node, future, timeout_sec=None)

    response = future.result()
    if response is None or not response.success:
        raise RuntimeError(f"Failed to set robot mode to autonomous via {service_name}.")
