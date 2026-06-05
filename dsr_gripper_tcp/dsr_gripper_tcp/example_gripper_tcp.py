from __future__ import annotations

import argparse

import rclpy

from dsr_gripper_tcp.gripper_tcp_bridge import (
    BridgeConfig,
    DoosanGripperTcpBridge,
)
from dsr_gripper_tcp.robot_utils import set_robot_mode_autonomous


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Start a DRL TCP bridge and control the RH-P12-RN gripper with request/response semantics."
    )
    parser.add_argument("--controller-host", required=True, help="IP address of the Doosan controller.")
    parser.add_argument("--tcp-port", type=int, default=20002, help="TCP port used by the controller-side DRL server.")
    parser.add_argument("--namespace", default="dsr01", help="Robot namespace used by the DRL ROS2 services.")
    parser.add_argument(
        "--service-prefix",
        default="dsr_controller2",
        help="Service prefix under the namespace. Use an empty string if your services are directly under the namespace.",
    )
    parser.add_argument("--goal-current", type=int, default=400, help="Goal current to apply to the gripper.")
    parser.add_argument(
        "--profile-velocity",
        type=int,
        default=1500,
        help="Profile velocity register value (RH-P12-RN(A) Modbus 40281 / address 280).",
    )
    parser.add_argument(
        "--profile-acceleration",
        type=int,
        default=1000,
        help="Profile acceleration register value (RH-P12-RN(A) Modbus 40279 / address 278).",
    )
    parser.add_argument("--open-position", type=int, default=0, help="Open position pulse value.")
    parser.add_argument("--close-position", type=int, default=700, help="Close position pulse value.")
    parser.add_argument("--move-timeout", type=float, default=8.0, help="Move completion timeout in seconds.")
    parser.add_argument(
        "--skip-set-autonomous",
        action="store_true",
        help="Skip setting the robot mode to autonomous before starting the bridge.",
    )
    return parser

def main(args: list[str] | None = None) -> None:
    parser = build_argument_parser()
    parsed = parser.parse_args(args=args)

    rclpy.init(args=None)
    node = rclpy.create_node("gripper_tcp_example", namespace=parsed.namespace)

    bridge = DoosanGripperTcpBridge(
        node=node,
        config=BridgeConfig(
            controller_host=parsed.controller_host,
            tcp_port=parsed.tcp_port,
            namespace=parsed.namespace,
            service_prefix=parsed.service_prefix,
            goal_current=parsed.goal_current,
            profile_velocity=parsed.profile_velocity,
            profile_acceleration=parsed.profile_acceleration,
        ),
    )

    try:
        if not parsed.skip_set_autonomous:
            set_robot_mode_autonomous(node, parsed.namespace, parsed.service_prefix)
        bridge.start()

        init_state = bridge.read_state()
        node.get_logger().info(
            f"Bridge ready: pos={init_state.present_position}, moving={init_state.moving}"
        )

        profile_state = bridge.set_motion_profile(
            goal_current=parsed.goal_current,
            profile_velocity=parsed.profile_velocity,
            profile_acceleration=parsed.profile_acceleration,
        )
        node.get_logger().info(
            "Profile applied: "
            f"pos={profile_state.present_position}, vel={profile_state.present_velocity}, "
            f"current={profile_state.present_current}"
        )

        close_state = bridge.move_to(parsed.close_position, timeout_sec=parsed.move_timeout)
        node.get_logger().info(
            "Close move complete: "
            f"pos={close_state.present_position}, in_position={close_state.in_position}"
        )

        open_state = bridge.move_to(parsed.open_position, timeout_sec=parsed.move_timeout)
        node.get_logger().info(
            "Open move complete: "
            f"pos={open_state.present_position}, in_position={open_state.in_position}"
        )
    finally:
        bridge.close(shutdown_remote=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
