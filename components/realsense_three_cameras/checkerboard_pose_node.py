#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage

try:
    from tf2_ros import TransformBroadcaster
except Exception:  # pragma: no cover - tf broadcast is optional.
    TransformBroadcaster = None


def image_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def make_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    for row in range(rows):
        for col in range(cols):
            points[row * cols + col] = (col * square_size_m, row * square_size_m, 0.0)
    return points


def decode_compressed_image(msg: CompressedImage) -> Optional[np.ndarray]:
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    return image


class CheckerboardPoseNode(Node):
    def __init__(self) -> None:
        super().__init__(f"checkerboard_pose_node_{os.getpid()}")

        self.declare_parameter("camera_name", "cam_high")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("pattern_cols", 8)
        self.declare_parameter("pattern_rows", 11)
        self.declare_parameter("square_size_m", 0.02)
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("tf_child_frame_id", "")
        self.declare_parameter("log_period_sec", 2.0)

        self.camera_name = self.get_parameter("camera_name").value
        self.pattern_cols = int(self.get_parameter("pattern_cols").value)
        self.pattern_rows = int(self.get_parameter("pattern_rows").value)
        self.square_size_m = float(self.get_parameter("square_size_m").value)
        self.log_period_sec = float(self.get_parameter("log_period_sec").value)

        image_topic = str(self.get_parameter("image_topic").value).strip()
        camera_info_topic = str(self.get_parameter("camera_info_topic").value).strip()
        pose_topic = str(self.get_parameter("pose_topic").value).strip()

        self.image_topic = image_topic or f"/{self.camera_name}/color/image_raw/compressed"
        self.camera_info_topic = camera_info_topic or f"/{self.camera_name}/color/camera_info"
        self.pose_topic = pose_topic or f"/checkerboard/{self.camera_name}/pose"
        self.tf_child_frame_id = (
            str(self.get_parameter("tf_child_frame_id").value).strip()
            or f"checkerboard_{self.camera_name}"
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.object_points = make_object_points(self.pattern_cols, self.pattern_rows, self.square_size_m)
        self.pattern_size = (self.pattern_cols, self.pattern_rows)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_frame_id = ""
        self.last_log_time = 0.0
        self.last_found_time = 0.0
        self.detected_count = 0
        self.missed_count = 0

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.tf_broadcaster = None
        if self.publish_tf and TransformBroadcaster is not None:
            self.tf_broadcaster = TransformBroadcaster(self)
        elif self.publish_tf:
            self.get_logger().warn("tf2_ros is unavailable; disabling TF broadcast")

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            image_qos(),
        )
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._on_image,
            image_qos(),
        )

        self.get_logger().info(
            "checkerboard pose node started: "
            f"camera={self.camera_name}, pattern={self.pattern_cols}x{self.pattern_rows}, "
            f"square={self.square_size_m:.4f}m, image={self.image_topic}, "
            f"camera_info={self.camera_info_topic}, pose={self.pose_topic}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=np.float64).reshape(-1, 1)
        if self.dist_coeffs.size == 0:
            self.dist_coeffs = None
        self.camera_frame_id = msg.header.frame_id

    def _on_image(self, msg: CompressedImage) -> None:
        if self.camera_matrix is None:
            self._log_waiting("waiting for camera_info")
            return

        image = decode_compressed_image(msg)
        if image is None:
            self._log_waiting("failed to decode compressed image")
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners = None
        found = False

        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray,
                self.pattern_size,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
            )

        if not found:
            found, corners = cv2.findChessboardCorners(
                gray,
                self.pattern_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        if not found or corners is None:
            self.missed_count += 1
            self._log_waiting("checkerboard not detected")
            return

        image_points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            self.missed_count += 1
            self._log_waiting("solvePnP failed")
            return

        rotation, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(rotation)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self.camera_frame_id or msg.header.frame_id
        pose_msg.pose.position.x = float(tvec[0][0])
        pose_msg.pose.position.y = float(tvec[1][0])
        pose_msg.pose.position.z = float(tvec[2][0])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)
        self._publish_tf(pose_msg)

        self.detected_count += 1
        self.last_found_time = time.monotonic()
        self._log_pose(pose_msg)

    def _publish_tf(self, pose_msg: PoseStamped) -> None:
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header = pose_msg.header
        transform.child_frame_id = self.tf_child_frame_id
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z
        transform.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _log_waiting(self, reason: str) -> None:
        now = time.monotonic()
        if now - self.last_log_time >= self.log_period_sec:
            self.last_log_time = now
            self.get_logger().warn(f"{reason}; detected={self.detected_count}, missed={self.missed_count}")

    def _log_pose(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        if now - self.last_log_time < self.log_period_sec:
            return
        self.last_log_time = now
        p = msg.pose.position
        self.get_logger().info(
            f"pose published on {self.pose_topic}: "
            f"x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, detected={self.detected_count}"
        )


def main() -> None:
    rclpy.init()
    node = CheckerboardPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
