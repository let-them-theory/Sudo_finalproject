#!/usr/bin/env python3
"""
ArUco Marker Tracking → cuRobo Pipeline Node

RealSense depth 카메라로 ArUco 마커를 인식하고,
캘리브레이션 변환을 적용하여 로봇 좌표계로 변환 후
cuRobo planner에 목표 pose를 전송합니다.

Usage:
    ros2 run e0509_gripper_description marker_tracking_node.py

    # 또는 파라미터 지정
    ros2 run e0509_gripper_description marker_tracking_node.py \
        --ros-args -p calibration_path:=/path/to/calibration.npz \
        -p marker_id:=0 -p marker_size:=0.05 -p safe_z_offset:=0.15

Keys:
    s: 현재 마커 위치로 로봇 이동
    q: 종료
"""

import numpy as np
import cv2
from cv2 import aruco
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor


class MarkerTrackingNode(Node):
    def __init__(self):
        super().__init__("marker_tracking_node")

        # Parameters
        self.declare_parameter("calibration_path",
            "config/calibration_eye_to_hand.npz",
            ParameterDescriptor(description="Path to calibration .npz file"))
        self.declare_parameter("marker_id", 0,
            ParameterDescriptor(description="ArUco marker ID to track"))
        self.declare_parameter("marker_size", 0.05,
            ParameterDescriptor(description="Marker size in meters"))
        self.declare_parameter("safe_z_offset", 0.15,
            ParameterDescriptor(description="Safety height offset above marker (meters)"))
        self.declare_parameter("target_topic", "/dsr01/curobo/target_pose",
            ParameterDescriptor(description="Target pose topic for cuRobo planner"))
        self.declare_parameter("auto_send", False,
            ParameterDescriptor(description="Automatically send target on detection"))

        calib_path = self.get_parameter("calibration_path").value
        self.marker_id = self.get_parameter("marker_id").value
        self.marker_size = self.get_parameter("marker_size").value
        self.safe_z_offset = self.get_parameter("safe_z_offset").value
        target_topic = self.get_parameter("target_topic").value
        self.auto_send = self.get_parameter("auto_send").value

        # Load calibration
        self.get_logger().info(f"Loading calibration: {calib_path}")
        try:
            calib = np.load(calib_path)
            T = calib['T_cam_to_base']
            self.R = T[:3, :3]
            self.t = T[:3, 3]
            self.get_logger().info(f"Calibration loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            raise

        # RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.get_logger().info("RealSense started (color + depth)")

        # ArUco detector
        self.detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_6X6_50),
            aruco.DetectorParameters())

        # Publishers
        self.pub = self.create_publisher(PoseStamped, target_topic, 10)
        self.pick_pub = self.create_publisher(PoseStamped, "/dsr01/curobo/pick_pose", 10)

        # EE orientation (gripper pointing down)
        self.ee_quat = [0.7071, 0.7071, 0.0, 0.0]

        # State
        self.last_pos = None

        # Timer for camera loop
        self.timer = self.create_timer(1.0 / 30.0, self.camera_loop)

        self.get_logger().info("=" * 50)
        self.get_logger().info("  Marker Tracking Node Ready")
        self.get_logger().info(f"  Marker ID: {self.marker_id}")
        self.get_logger().info(f"  Safe Z offset: {self.safe_z_offset}m")
        self.get_logger().info(f"  Target topic: {target_topic}")
        self.get_logger().info(f"  Auto send: {self.auto_send}")
        self.get_logger().info("  Keys: 's' = move, 'p' = pick, 'q' = quit")
        self.get_logger().info("=" * 50)

    def camera_loop(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        cf = aligned.get_color_frame()
        df = aligned.get_depth_frame()
        if not cf:
            return

        img = np.asanyarray(cf.get_data())
        display = img.copy()
        corners, ids, _ = self.detector.detectMarkers(img)

        if ids is not None and self.marker_id in ids.flatten():
            idx = list(ids.flatten()).index(self.marker_id)
            aruco.drawDetectedMarkers(display, corners, ids)
            center = corners[idx][0].mean(axis=0)

            # Depth-based 3D coordinate
            depth_m = df.get_distance(int(center[0]), int(center[1])) if df else 0
            if depth_m <= 0 or depth_m > 5.0:
                for du in range(-5, 6):
                    for dv in range(-5, 6):
                        pu, pv = int(center[0]) + du, int(center[1]) + dv
                        if 0 <= pu < 640 and 0 <= pv < 480:
                            dd = df.get_distance(pu, pv)
                            if 0.1 < dd < 5.0:
                                depth_m = dd
                                break
                    if 0.1 < depth_m < 5.0:
                        break

            if depth_m > 0.1:
                intr = df.profile.as_video_stream_profile().intrinsics
                pt3d = rs.rs2_deproject_pixel_to_point(intr, [center[0], center[1]], depth_m)
                tc = np.array(pt3d)
                pb = self.R @ tc + self.t
                self.last_pos = pb

                cv2.putText(display, f"X:{pb[0]*1000:.1f} Y:{pb[1]*1000:.1f} Z:{pb[2]*1000:.1f} (depth)",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display, f"dist: {depth_m*100:.1f}cm",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(display, "'s'=move  'p'=pick  'q'=quit",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                if self.auto_send:
                    self.send_target()
            else:
                cv2.putText(display, "Marker found, no depth",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(display, "No marker",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Marker Tracking", display)
        k = cv2.waitKey(1) & 0xFF

        if k == ord('s') and self.last_pos is not None:
            self.send_target()
        elif k == ord('p') and self.last_pos is not None:
            self.send_pick()
        elif k == ord('q'):
            self.get_logger().info("Shutting down...")
            raise SystemExit

    def send_target(self):
        if self.last_pos is None:
            return

        target_z = max(self.last_pos[2] + self.safe_z_offset, 0.15)

        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.last_pos[0])
        msg.pose.position.y = float(self.last_pos[1])
        msg.pose.position.z = float(target_z)
        msg.pose.orientation.x = float(self.ee_quat[0])
        msg.pose.orientation.y = float(self.ee_quat[1])
        msg.pose.orientation.z = float(self.ee_quat[2])
        msg.pose.orientation.w = float(self.ee_quat[3])
        self.pub.publish(msg)

        self.get_logger().info(
            f"Sent: X={self.last_pos[0]*1000:.1f} Y={self.last_pos[1]*1000:.1f} Z={target_z*1000:.1f}")

    def send_pick(self):
        """Send pick command (approach → open → descend → close → lift)."""
        if self.last_pos is None:
            return

        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.last_pos[0])
        msg.pose.position.y = float(self.last_pos[1])
        msg.pose.position.z = float(self.last_pos[2])  # actual object Z
        msg.pose.orientation.x = float(self.ee_quat[0])
        msg.pose.orientation.y = float(self.ee_quat[1])
        msg.pose.orientation.z = float(self.ee_quat[2])
        msg.pose.orientation.w = float(self.ee_quat[3])
        self.pick_pub.publish(msg)

        self.get_logger().info(
            f"PICK: X={self.last_pos[0]*1000:.1f} Y={self.last_pos[1]*1000:.1f} Z={self.last_pos[2]*1000:.1f}")

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerTrackingNode()
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
