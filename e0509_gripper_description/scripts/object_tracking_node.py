#!/usr/bin/env python3
"""
Object Tracking Node (Grounding DINO + RealSense Depth)

텍스트 프롬프트로 물체를 제로샷 인식하고, depth로 3D 위치를 계산하여
cuRobo planner에 목표를 전송합니다.

Usage:
    ros2 run e0509_gripper_description object_tracking_node.py \
        --ros-args -p prompt:="red block"

Keys:
    s: 선택된 물체 위치로 로봇 이동
    p: 선택된 물체 pick (접근 → 열기 → 하강 → 닫기 → 들기)
    1-9: 감지된 물체 중 선택
    q: 종료
"""

import os
import numpy as np
import cv2
import pyrealsense2 as rs
import torch
import warnings
warnings.filterwarnings("ignore")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor
import json

from groundingdino.util.inference import load_model, load_image, predict


class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__("object_tracking_node")

        # Parameters
        self.declare_parameter("prompt", "red block . green block . yellow block",
            ParameterDescriptor(description="Text prompt for object detection"))
        self.declare_parameter("calibration_path",
            os.path.expanduser("~/sim2real/sim2real/config/calibration_eye_to_hand.npz"),
            ParameterDescriptor(description="Calibration file path"))
        self.declare_parameter("box_threshold", 0.3,
            ParameterDescriptor(description="Detection confidence threshold"))
        self.declare_parameter("detection_interval", 5,
            ParameterDescriptor(description="Run detection every N frames"))

        self.prompt = self.get_parameter("prompt").value
        calib_path = self.get_parameter("calibration_path").value
        self.box_threshold = self.get_parameter("box_threshold").value
        self.detection_interval = self.get_parameter("detection_interval").value

        # Load calibration
        self.get_logger().info(f"Loading calibration: {calib_path}")
        calib = np.load(calib_path)
        T = calib['T_cam_to_base']
        self.R_cal = T[:3, :3]
        self.t_cal = T[:3, 3]

        # Load Grounding DINO
        self.get_logger().info("Loading Grounding DINO...")
        self.gdino = load_model(
            os.path.expanduser("~/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"),
            os.path.expanduser("~/models/groundingdino_swint_ogc.pth"),
        )
        self.get_logger().info("Grounding DINO loaded!")

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("RealSense started")

        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, "/dsr01/curobo/target_pose", 10)
        self.pick_pub = self.create_publisher(PoseStamped, "/dsr01/curobo/pick_pose", 10)
        self.obstacles_pub = self.create_publisher(String, "/dsr01/curobo/obstacles", 10)

        # State
        self.detections = []  # list of (phrase, pos_base, logit, bbox, grasp_angle_rad)
        self.selected_idx = 0
        self.frame_count = 0
        self.locked = False
        self.locked_target = None  # (phrase, pos_base, grasp_angle_rad)

        # Camera loop timer
        self.timer = self.create_timer(1.0 / 30.0, self.camera_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info("  Object Tracking Node Ready")
        self.get_logger().info(f"  Prompt: {self.prompt}")
        self.get_logger().info("  Keys: 's'=move, 'p'=pick, 1-9=select, 'q'=quit")
        self.get_logger().info("=" * 50)

    def camera_loop(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf:
            return

        image = np.asanyarray(cf.get_data())
        display = image.copy()
        self.frame_count += 1

        # Store depth frame for angle computation
        self._last_depth_frame = df

        # Run detection periodically (skip if locked)
        if not self.locked and self.frame_count % self.detection_interval == 0:
            self.run_detection(image, df)

        # Draw detections
        h, w = image.shape[:2]
        for i, det in enumerate(self.detections):
            phrase, pos_base, logit, bbox, grasp_angle = det
            x1 = int((bbox[0] - bbox[2] / 2) * w)
            y1 = int((bbox[1] - bbox[3] / 2) * h)
            x2 = int((bbox[0] + bbox[2] / 2) * w)
            y2 = int((bbox[1] + bbox[3] / 2) * h)

            # Selected object: green, others: gray
            if i == self.selected_idx:
                color = (0, 255, 0)
                thickness = 3
            else:
                color = (150, 150, 150)
                thickness = 1

            cv2.rectangle(display, (x1, y1), (x2, y2), color, thickness)
            label = f"[{i+1}] {phrase} ({logit:.2f}) {np.degrees(det[4]):.0f}deg"
            cv2.putText(display, label, (x1, y1 - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

            if pos_base is not None:
                coord = f"({pos_base[0]*100:.1f},{pos_base[1]*100:.1f},{pos_base[2]*100:.1f})cm"
                cv2.putText(display, coord, (x1, y2 + 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

        # Status bar
        if self.locked and self.locked_target:
            phrase, pos, _angle = self.locked_target
            cv2.putText(display, f"LOCKED: {phrase} (press 'r' to unlock)",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if pos is not None:
                cv2.putText(display, f"Target: X={pos[0]*1000:.1f} Y={pos[1]*1000:.1f} Z={pos[2]*1000:.1f}",
                           (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        elif self.detections:
            sel = self.detections[self.selected_idx]
            cv2.putText(display, f"Selected: [{self.selected_idx+1}] {sel[0]}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"Prompt: {self.prompt}",
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(display, "'s'=move 'p'=pick 1-9=lock 'r'=unlock w/x=angle 'q'=quit",
                   (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

        cv2.imshow("Object Tracking", display)
        k = cv2.waitKey(1) & 0xFF

        if k == ord('s'):
            self.send_move()
        elif k == ord('p'):
            self.send_pick()
        elif ord('1') <= k <= ord('9'):
            idx = k - ord('1')
            if idx < len(self.detections):
                self.selected_idx = idx
                det = self.detections[idx]
                self.locked = True
                self.locked_target = (det[0], det[1], det[4])  # phrase, pos, angle
                self.get_logger().info(f"LOCKED: [{idx+1}] {det[0]} (grasp angle: {np.degrees(det[4]):.1f}°)")
                self.publish_obstacles()
        elif k == ord('r'):
            self.locked = False
            self.locked_target = None
            self.get_logger().info("UNLOCKED: detection resumed")
        elif k == 82 or k == ord('w'):  # Up arrow or 'w': rotate +5deg
            if self.locked and self.locked_target:
                phrase, pos, angle = self.locked_target
                angle += np.radians(5)
                self.locked_target = (phrase, pos, angle)
                self.get_logger().info(f"Angle adjusted: {np.degrees(angle):.1f}°")
        elif k == 84 or k == ord('x'):  # Down arrow or 'x': rotate -5deg
            if self.locked and self.locked_target:
                phrase, pos, angle = self.locked_target
                angle -= np.radians(5)
                self.locked_target = (phrase, pos, angle)
                self.get_logger().info(f"Angle adjusted: {np.degrees(angle):.1f}°")
        elif k == ord('q'):
            raise SystemExit

    def run_detection(self, image, depth_frame):
        """Run Grounding DINO detection and compute 3D positions."""
        cv2.imwrite("/tmp/objtrack.jpg", image)
        src, tensor = load_image("/tmp/objtrack.jpg")
        boxes, logits, phrases = predict(
            self.gdino, tensor, self.prompt,
            box_threshold=self.box_threshold, text_threshold=0.2)

        h, w = image.shape[:2]
        self.detections = []

        for box, logit, phrase in zip(boxes, logits, phrases):
            cx = int(box[0] * w)
            cy = int(box[1] * h)

            # Depth-based 3D position
            pos_base = None
            depth_m = depth_frame.get_distance(cx, cy) if depth_frame else 0
            if depth_m <= 0.1 or depth_m > 5.0:
                # Search nearby pixels
                for du in range(-5, 6):
                    for dv in range(-5, 6):
                        pu, pv = cx + du, cy + dv
                        if 0 <= pu < w and 0 <= pv < h:
                            dd = depth_frame.get_distance(pu, pv)
                            if 0.1 < dd < 5.0:
                                depth_m = dd
                                break
                    if 0.1 < depth_m < 5.0:
                        break

            if depth_m > 0.1:
                intr = depth_frame.profile.as_video_stream_profile().intrinsics
                pt3d = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth_m)
                tc = np.array(pt3d)
                pos_base = self.R_cal @ tc + self.t_cal

            # Compute grasp angle from object orientation in image
            grasp_angle_rad = self.compute_grasp_angle(image, box.cpu().numpy(), h, w)

            # Keep last valid angle if current is 0 (depth failed)
            cache_key = f"{phrase}_{int(box[0]*100)}_{int(box[1]*100)}"
            if abs(grasp_angle_rad) > 0.01:
                if not hasattr(self, '_angle_cache'):
                    self._angle_cache = {}
                self._angle_cache[cache_key] = grasp_angle_rad
            elif hasattr(self, '_angle_cache') and cache_key in self._angle_cache:
                grasp_angle_rad = self._angle_cache[cache_key]

            self.detections.append((phrase, pos_base, float(logit), box.cpu().numpy(), grasp_angle_rad))

        # Keep selection in range
        if self.selected_idx >= len(self.detections):
            self.selected_idx = 0

        # Obstacles are published once when locking a target

    def compute_grasp_angle(self, image, box, h, w):
        """Compute grasp angle using 3D PCA on depth point cloud within bbox."""
        if not hasattr(self, '_last_depth_frame') or self._last_depth_frame is None:
            return 0.0

        df = self._last_depth_frame
        intr = df.profile.as_video_stream_profile().intrinsics

        x1 = max(0, int((box[0] - box[2] / 2) * w))
        y1 = max(0, int((box[1] - box[3] / 2) * h))
        x2 = min(w, int((box[0] + box[2] / 2) * w))
        y2 = min(h, int((box[1] + box[3] / 2) * h))

        if x2 - x1 < 10 or y2 - y1 < 10:
            return 0.0

        # Collect 3D points within bbox
        # Sample every 2 pixels for speed
        points_robot = []
        center_depth = df.get_distance(int(box[0] * w), int(box[1] * h))
        if center_depth < 0.1:
            return 0.0

        for py in range(y1, y2, 2):
            for px in range(x1, x2, 2):
                d = df.get_distance(px, py)
                # Only keep points close to object depth (within 3cm of center)
                if d > 0.1 and abs(d - center_depth) < 0.03:
                    pt3d = rs.rs2_deproject_pixel_to_point(intr, [px, py], d)
                    pt_robot = self.R_cal @ np.array(pt3d) + self.t_cal
                    points_robot.append(pt_robot)

        if len(points_robot) < 20:
            return 0.0

        points = np.array(points_robot)

        # PCA on XY plane (ignore Z for table-top objects)
        xy = points[:, :2]
        center = xy.mean(axis=0)
        centered = xy - center
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # Major axis (largest eigenvalue)
        major = eigenvectors[:, -1]
        robot_angle = np.arctan2(major[1], major[0])

        # Rotate 90 degrees so gripper opens perpendicular to major axis
        return robot_angle + np.pi / 2

    @staticmethod
    def make_down_quaternion(grasp_angle_rad):
        """Create quaternion for gripper pointing down + rotated by angle around Z axis.
        grasp_angle_rad: rotation around vertical (Z) axis in robot frame.
        Returns: [x, y, z, w] quaternion
        """
        # Base quaternion: gripper pointing down
        # q_base = (w=0, x=0.7071, y=0.7071, z=0)
        # Rotation around Z: q_z = (w=cos(a/2), x=0, y=0, z=sin(a/2))
        # Combined: q_z * q_base

        ca = np.cos(grasp_angle_rad / 2)
        sa = np.sin(grasp_angle_rad / 2)

        # q_z (wxyz)
        qz_w, qz_x, qz_y, qz_z = ca, 0.0, 0.0, sa

        # q_base (wxyz)
        qb_w, qb_x, qb_y, qb_z = 0.0, 0.7071, 0.7071, 0.0

        # Quaternion multiplication q_z * q_base
        w = qz_w*qb_w - qz_x*qb_x - qz_y*qb_y - qz_z*qb_z
        x = qz_w*qb_x + qz_x*qb_w + qz_y*qb_z - qz_z*qb_y
        y = qz_w*qb_y - qz_x*qb_z + qz_y*qb_w + qz_z*qb_x
        z = qz_w*qb_z + qz_x*qb_y - qz_y*qb_x + qz_z*qb_w

        return [float(x), float(y), float(z), float(w)]

    def publish_obstacles(self):
        """Publish detected objects as obstacles to cuRobo (except target)."""
        obstacles = []
        locked_phrase = self.locked_target[0] if self.locked and self.locked_target else None

        for i, (phrase, pos_base, logit, bbox, _angle) in enumerate(self.detections):
            if pos_base is None:
                continue
            # Skip the locked target (we want to reach it, not avoid it)
            if self.locked and i == self.selected_idx:
                continue

            # Estimate object size from bbox (rough approximation)
            h_img, w_img = 480, 640
            obj_w = float(bbox[2]) * w_img * 0.001  # rough width in meters
            obj_h = float(bbox[3]) * h_img * 0.001  # rough height in meters
            obj_size = max(obj_w, obj_h, 0.03)  # minimum 3cm

            obstacles.append({
                "name": f"{phrase}_{i}",
                "pos": [float(pos_base[0]), float(pos_base[1]), float(pos_base[2])],
                "dims": [obj_size, obj_size, obj_size]
            })

        msg = String()
        msg.data = json.dumps(obstacles)
        self.obstacles_pub.publish(msg)

    def _make_pose_msg(self, pos, grasp_angle_rad=0.0):
        quat = self.make_down_quaternion(grasp_angle_rad)
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def _get_target(self):
        """Get currently selected/locked target. Returns (phrase, pos, angle)."""
        if self.locked and self.locked_target:
            phrase, pos, angle = self.locked_target
            return phrase, pos, angle
        if self.detections and self.selected_idx < len(self.detections):
            det = self.detections[self.selected_idx]
            return det[0], det[1], det[4]
        return None, None, 0.0

    def send_move(self):
        """Move to selected object (safe height above)."""
        phrase, pos, angle = self._get_target()
        if pos is None:
            self.get_logger().warn("No valid object selected")
            return
        safe_z = max(pos[2] + 0.15, 0.15)
        msg = self._make_pose_msg([pos[0], pos[1], safe_z], angle)
        self.target_pub.publish(msg)
        self.get_logger().info(f"MOVE to {phrase}: X={pos[0]*1000:.1f} Y={pos[1]*1000:.1f} Z={safe_z*1000:.1f} angle={np.degrees(angle):.1f}deg")

    def send_pick(self):
        """Pick selected object (full sequence)."""
        phrase, pos, angle = self._get_target()
        if pos is None:
            self.get_logger().warn("No valid object selected")
            return
        msg = self._make_pose_msg(pos, angle)
        self.pick_pub.publish(msg)
        self.get_logger().info(f"PICK {phrase}: X={pos[0]*1000:.1f} Y={pos[1]*1000:.1f} Z={pos[2]*1000:.1f} angle={np.degrees(angle):.1f}deg")

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
