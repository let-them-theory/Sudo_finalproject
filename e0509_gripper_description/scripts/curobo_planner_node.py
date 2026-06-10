#!/usr/bin/env python3
"""
cuRobo Motion Planner Node for Doosan E0509

Subscribes to target pose, plans collision-free trajectory using cuRobo,
and executes via Doosan MoveSplineJoint service.

Usage:
    ros2 run e0509_gripper_description curobo_planner_node.py

Test:
    ros2 topic pub --once /dsr01/curobo/target_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.2, z: 0.3}, \
        orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}}"
"""

import os
import math
import time
import torch
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveSplineJoint, MoveJoint, MoveLine

from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState as CuroboJointState, RobotConfig
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
from curobo.geom.types import WorldConfig, Cuboid
from std_msgs.msg import String
import json


class CuroboPlanner(Node):
    # E0509 joint order as expected by cuRobo and Doosan services
    JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    def __init__(self):
        super().__init__("curobo_planner_node")

        # Separate callback group for service calls to avoid deadlock
        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.get_logger().info("Initializing cuRobo planner...")

        # Current joint state
        self.current_joints = None

        # Config path
        config_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "curobo"
        )
        # Fallback for installed package
        if not os.path.exists(config_dir):
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(
                get_package_share_directory("e0509_gripper_description"),
                "config", "curobo"
            )

        self.get_logger().info(f"Config dir: {config_dir}")

        # Initialize cuRobo
        tensor_args = TensorDeviceType(device=torch.device("cuda:0"))

        robot_cfg = RobotConfig.from_basic(
            urdf_path=os.path.join(config_dir, "e0509_gripper.urdf"),
            base_link="base_link",
            ee_link="gripper_rh_p12_rn_base",
            tensor_args=tensor_args,
        )

        # World: table as obstacle (adjustable later)
        world_cfg = WorldConfig(
            cuboid=[
                Cuboid(name="table", pose=[0.0, 0.0, -0.02, 1, 0, 0, 0], dims=[1.2, 1.2, 0.04]),
            ]
        )

        motion_gen_cfg = MotionGenConfig.load_from_robot_config(
            robot_cfg,
            world_cfg,
            tensor_args=tensor_args,
            num_trajopt_seeds=4,
            num_graph_seeds=4,
            collision_cache={"obb": 30, "mesh": 10},
        )
        self.motion_gen = MotionGen(motion_gen_cfg)
        self.motion_gen.warmup(warmup_js_trajopt=False)
        self.get_logger().info("cuRobo MotionGen warmed up!")

        # ROS2 interfaces
        self.joint_sub = self.create_subscription(
            JointState, "/dsr01/joint_states",
            self.joint_state_cb, 10)

        self.target_sub = self.create_subscription(
            PoseStamped, "/dsr01/curobo/target_pose",
            self.target_pose_cb, 10)

        # Pick pose subscription
        self.pick_sub = self.create_subscription(
            PoseStamped, "/dsr01/curobo/pick_pose",
            self.pick_pose_cb, 10)

        # Obstacles update subscription (JSON format)
        self.obstacles_sub = self.create_subscription(
            String, "/dsr01/curobo/obstacles",
            self.obstacles_cb, 10)

        self.tensor_args = tensor_args
        self.robot_cfg = robot_cfg

        # Doosan services (use separate callback group)
        self.cli_spline = self.create_client(
            MoveSplineJoint, "/dsr01/motion/move_spline_joint",
            callback_group=self.service_cb_group)
        self.cli_movej = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint",
            callback_group=self.service_cb_group)
        self.cli_movel = self.create_client(
            MoveLine, "/dsr01/motion/move_line",
            callback_group=self.service_cb_group)
        self.cli_gripper_open = self.create_client(
            Trigger, "/dsr01/gripper/open",
            callback_group=self.service_cb_group)
        self.cli_gripper_close = self.create_client(
            Trigger, "/dsr01/gripper/close",
            callback_group=self.service_cb_group)

        self.get_logger().info("========================================")
        self.get_logger().info("cuRobo Planner Ready!")
        self.get_logger().info("  Subscribe: /dsr01/curobo/target_pose (move)")
        self.get_logger().info("  Subscribe: /dsr01/curobo/pick_pose (pick)")
        self.get_logger().info("  Subscribe: /dsr01/curobo/obstacles (world update)")
        self.get_logger().info("========================================")

    def obstacles_cb(self, msg: String):
        """Update cuRobo world with detected obstacles.
        JSON format: [{"name": "obj1", "pos": [x,y,z], "dims": [w,h,d]}, ...]
        """
        try:
            obstacles_data = json.loads(msg.data)
            cuboids = [
                Cuboid(
                    name="table",
                    pose=[0.0, 0.0, -0.02, 1, 0, 0, 0],
                    dims=[1.2, 1.2, 0.04]
                )
            ]

            for obj in obstacles_data:
                cuboids.append(Cuboid(
                    name=obj["name"],
                    pose=[obj["pos"][0], obj["pos"][1], obj["pos"][2], 1, 0, 0, 0],
                    dims=obj.get("dims", [0.05, 0.05, 0.05])
                ))

            world_cfg = WorldConfig(cuboid=cuboids)
            self.motion_gen.update_world(world_cfg)
            self.get_logger().info(f"World updated: {len(cuboids)} obstacles (table + {len(obstacles_data)} objects)")

        except Exception as e:
            self.get_logger().error(f"Failed to update obstacles: {e}")

    def joint_state_cb(self, msg: JointState):
        """Store current joint positions in correct order."""
        joint_map = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_map[name] = msg.position[i]

        joints = []
        for name in self.JOINT_NAMES:
            if name in joint_map:
                joints.append(joint_map[name])
            else:
                return  # Missing joint data

        self.current_joints = joints

    def target_pose_cb(self, msg: PoseStamped):
        """Receive target pose, plan trajectory, and execute."""
        if self.current_joints is None:
            self.get_logger().warn("No joint state received yet")
            return

        pos = msg.pose.position
        ori = msg.pose.orientation
        self.get_logger().info(
            f"Target received: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}] "
            f"ori=[{ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f}]")

        # Plan
        traj = self.plan(
            self.current_joints,
            [pos.x, pos.y, pos.z],
            [ori.w, ori.x, ori.y, ori.z],  # cuRobo uses wxyz
        )

        if traj is not None:
            self.execute_movej(traj)
        else:
            self.get_logger().error("Planning failed!")

    def plan(self, start_joints, target_pos, target_quat_wxyz):
        """Plan trajectory using cuRobo."""
        t0 = time.time()

        start_state = CuroboJointState.from_position(
            position=torch.tensor([start_joints], device="cuda:0", dtype=torch.float32),
            joint_names=self.JOINT_NAMES,
        )

        target_pose = Pose(
            position=torch.tensor([target_pos], device="cuda:0", dtype=torch.float32),
            quaternion=torch.tensor([target_quat_wxyz], device="cuda:0", dtype=torch.float32),
        )

        result = self.motion_gen.plan_single(start_state, target_pose)
        plan_time = (time.time() - t0) * 1000

        if result.success.item():
            traj = result.get_interpolated_plan()
            positions = traj.position.cpu().numpy()
            self.get_logger().info(
                f"Planning SUCCESS: {plan_time:.1f}ms, {positions.shape[0]} points")
            return positions
        else:
            self.get_logger().error(f"Planning FAILED: {plan_time:.1f}ms")
            return None

    def execute_spline(self, traj_rad):
        """Execute trajectory via MoveSplineJoint (expects degrees)."""
        if not self.cli_spline.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveSplineJoint service not available")
            return

        # Convert rad to deg, subsample to max 100 points
        traj_deg = np.rad2deg(traj_rad)
        n_points = traj_deg.shape[0]

        # Subsample if more than 100 points
        if n_points > 100:
            indices = np.linspace(0, n_points - 1, 100, dtype=int)
            traj_deg = traj_deg[indices]
            n_points = 100

        # Build request
        req = MoveSplineJoint.Request()
        req.pos_cnt = n_points

        for i in range(n_points):
            point = Float64MultiArray()
            point.data = traj_deg[i].tolist()
            req.pos.append(point)

        req.vel = [30.0] * 6   # deg/sec
        req.acc = [60.0] * 6   # deg/sec^2
        req.time = 0.0
        req.mode = 0    # ABSOLUTE
        req.sync_type = 0  # SYNC

        self.get_logger().info(f"Executing spline trajectory ({n_points} points)...")
        self.get_logger().info(f"  Start (deg): {[f'{v:.2f}' for v in traj_deg[0]]}")
        self.get_logger().info(f"  End   (deg): {[f'{v:.2f}' for v in traj_deg[-1]]}")
        self.get_logger().info(f"  Current joints (rad): {[f'{v:.4f}' for v in self.current_joints]}")

        future = self.cli_spline.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() and future.result().success:
            self.get_logger().info("Trajectory execution complete!")
        else:
            self.get_logger().error("Trajectory execution failed!")

    def execute_movej(self, traj_rad):
        """Execute only the final pose via MoveJoint (for testing)."""
        if not self.cli_movej.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveJoint service not available")
            return

        # Take the last point of trajectory, convert to degrees
        final_joints_deg = np.rad2deg(traj_rad[-1]).tolist()

        self.get_logger().info(f"Executing MoveJoint to: {[f'{v:.2f}' for v in final_joints_deg]}")

        req = MoveJoint.Request()
        req.pos = final_joints_deg
        req.vel = 30.0
        req.acc = 30.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0      # ABSOLUTE
        req.blend_type = 0
        req.sync_type = 1  # ASYNC (don't block robot controller)

        future = self.cli_movej.call_async(req)

        # Wait for response without spin_until_future_complete (avoids deadlock)
        timeout = 30.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if future.done() and future.result() and future.result().success:
            self.get_logger().info("MoveJoint execution complete!")
        elif not future.done():
            self.get_logger().error("MoveJoint timed out!")
        else:
            self.get_logger().error("MoveJoint execution failed!")

    def pick_pose_cb(self, msg: PoseStamped):
        """Pick from current position: open → descend (cuRobo) → close → lift (cuRobo)."""
        if self.current_joints is None:
            self.get_logger().warn("No joint state received yet")
            return

        self.get_logger().info("=== PICK: open → descend → grasp → lift ===")

        pos = msg.pose.position
        ori = msg.pose.orientation
        GRASP_HEIGHT = 0.12     # 12cm above target
        LIFT_HEIGHT = 0.15      # lift 15cm after grasp
        MIN_Z = 0.02

        target_z = max(pos.z, MIN_Z)
        grasp_z = target_z + GRASP_HEIGHT

        # ===== Step 1: Open gripper =====
        self.get_logger().info("=== PICK Step 1/4: Open gripper ===")
        self.call_trigger(self.cli_gripper_open)
        time.sleep(1.5)

        # ===== Step 2: Descend to grasp height (cuRobo, avoids obstacles) =====
        self.get_logger().info(f"=== PICK Step 2/4: Descend to Z={grasp_z*1000:.1f}mm (cuRobo) ===")
        traj = self.plan(
            self.current_joints,
            [pos.x, pos.y, grasp_z],
            [ori.w, ori.x, ori.y, ori.z],
        )
        if traj is not None:
            self.execute_movej(traj)
            time.sleep(2.0)
        else:
            self.get_logger().error("Pick failed: descend planning failed")
            return

        # ===== Step 3: Close gripper =====
        self.get_logger().info("=== PICK Step 3/4: Close gripper ===")
        self.call_trigger(self.cli_gripper_close)
        time.sleep(1.5)

        # ===== Step 4: Lift up (movel, straight up) =====
        self.get_logger().info("=== PICK Step 4/4: Lift ===")
        lift_z = grasp_z + LIFT_HEIGHT
        self.move_linear(pos.x, pos.y, lift_z, vel=50.0)
        time.sleep(1.5)

        self.get_logger().info("=== PICK COMPLETE ===")

    def move_linear(self, x, y, z, vel=100.0):
        """Move TCP linearly (movel) in mm, pointing down."""
        if not self.cli_movel.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("MoveLine service not available")
            return

        req = MoveLine.Request()
        req.pos = [x * 1000, y * 1000, z * 1000, 0.0, 180.0, 0.0]  # mm, deg
        req.vel = [vel, 30.0]
        req.acc = [vel, 30.0]
        req.time = 0.0
        req.ref = 0       # base frame
        req.mode = 0       # absolute
        req.blend_type = 0
        req.sync_type = 1  # async

        future = self.cli_movel.call_async(req)
        timeout = 15.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if future.done() and future.result() and future.result().success:
            self.get_logger().info(f"MoveLine complete: Z={z*1000:.1f}mm")
        else:
            self.get_logger().error("MoveLine failed!")

    def call_trigger(self, client):
        """Call a Trigger service (gripper open/close)."""
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Trigger service not available")
            return

        req = Trigger.Request()
        future = client.call_async(req)
        timeout = 10.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)

        if future.done() and future.result():
            self.get_logger().info(f"Trigger: {future.result().message}")


def main():
    rclpy.init()
    node = CuroboPlanner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
