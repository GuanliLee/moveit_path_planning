#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    for row in range(rows):
        for col in range(cols):
            points[row * cols + col] = (col * square_size_m, row * square_size_m, 0.0)
    return points


def quaternion_xyzw_to_rotation_matrix(values: list[float]) -> np.ndarray:
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("invalid zero-length quaternion")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_to_matrix(pose: Pose) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_xyzw_to_rotation_matrix(
        [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ]
    )
    matrix[:3, 3] = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]
    return matrix


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    hom = np.c_[points, np.ones((points.shape[0], 1), dtype=np.float64)]
    return (matrix @ hom.T).T[:, :3]


def decode_compressed_rgb(msg: CompressedImage) -> np.ndarray:
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("failed to decode compressed image")
    return image


def decode_depth(msg: Image) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(msg.encoding).lower()
    step = int(msg.step)
    raw = bytes(msg.data)
    if encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        item_size = 2
        scale = 0.001
    elif encoding == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        item_size = 4
        scale = 1.0
    else:
        raise RuntimeError(f"unsupported depth encoding: {msg.encoding}")
    row_values = (step or width * item_size) // item_size
    depth = np.frombuffer(raw, dtype=dtype, count=row_values * height).reshape(height, row_values)[:, :width]
    depth_m = depth.astype(np.float32) * np.float32(scale)
    return np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)


def detect_chessboard(image_bgr: np.ndarray, pattern_size: tuple[int, int]) -> tuple[bool, Optional[np.ndarray]]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    if not found or corners is None:
        return False, None
    return True, np.asarray(corners, dtype=np.float32).reshape(-1, 2)


def project_points(points_camera: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    z = np.maximum(points[:, 2], 1e-9)
    u = camera_matrix[0, 0] * points[:, 0] / z + camera_matrix[0, 2]
    v = camera_matrix[1, 1] * points[:, 1] / z + camera_matrix[1, 2]
    return np.c_[u, v]


class LiveCapture(Node):
    def __init__(
        self,
        rgb_topic: str,
        camera_info_topic: str,
        robot_pose_topic: str,
        checkerboard_pose_topic: str,
        depth_topic: str,
    ) -> None:
        super().__init__("handeye_application_visualizer_capture")
        self.rgb: Optional[CompressedImage] = None
        self.camera_info: Optional[CameraInfo] = None
        self.robot_pose: Optional[Pose] = None
        self.checkerboard_pose: Optional[PoseStamped] = None
        self.depth: Optional[Image] = None

        self.create_subscription(CompressedImage, rgb_topic, lambda msg: setattr(self, "rgb", msg), sensor_qos())
        self.create_subscription(CameraInfo, camera_info_topic, lambda msg: setattr(self, "camera_info", msg), sensor_qos())
        self.create_subscription(Pose, robot_pose_topic, lambda msg: setattr(self, "robot_pose", msg), 10)
        self.create_subscription(PoseStamped, checkerboard_pose_topic, lambda msg: setattr(self, "checkerboard_pose", msg), 10)
        if depth_topic:
            self.create_subscription(Image, depth_topic, lambda msg: setattr(self, "depth", msg), sensor_qos())

    def required_ready(self) -> bool:
        return (
            self.rgb is not None
            and self.camera_info is not None
            and self.robot_pose is not None
            and self.checkerboard_pose is not None
        )


def draw_rgb_overlay(
    image: np.ndarray,
    object_points: np.ndarray,
    t_camera_board: np.ndarray,
    camera_matrix: np.ndarray,
    detected_corners: Optional[np.ndarray],
) -> np.ndarray:
    out = image.copy()
    points_camera = transform_points(t_camera_board, object_points)
    projected = project_points(points_camera, camera_matrix)
    h, w = out.shape[:2]
    for point in projected:
        u, v = int(round(point[0])), int(round(point[1]))
        if 0 <= u < w and 0 <= v < h:
            cv2.circle(out, (u, v), 3, (0, 220, 0), -1, lineType=cv2.LINE_AA)

    if detected_corners is not None:
        for point in detected_corners:
            u, v = int(round(float(point[0]))), int(round(float(point[1])))
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(out, (u, v), 2, (0, 255, 255), 1, lineType=cv2.LINE_AA)

    axis_points = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [0.08, 0.0, 0.0],
            [0.0, 0.08, 0.0],
            [0.0, 0.0, -0.08],
        ],
        dtype=np.float64,
    )
    axis_projected = project_points(transform_points(t_camera_board, axis_points), camera_matrix)
    origin = tuple(np.round(axis_projected[0]).astype(int))
    colors = [(0, 0, 255), (0, 180, 0), (255, 0, 0)]
    labels = ["board X", "board Y", "board Z"]
    for idx, color in enumerate(colors, start=1):
        end = tuple(np.round(axis_projected[idx]).astype(int))
        cv2.arrowedLine(out, origin, end, color, 2, line_type=cv2.LINE_AA, tipLength=0.18)
        cv2.putText(out, labels[idx - 1], end, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    cv2.putText(
        out,
        "green=projected board points, yellow=detected corners",
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        out,
        "green=projected board points, yellow=detected corners",
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (20, 20, 20),
        1,
        cv2.LINE_AA,
    )
    return out


def backproject_depth(
    depth_m: np.ndarray,
    camera_matrix: np.ndarray,
    stride: int,
    max_depth_m: float,
) -> np.ndarray:
    ys = np.arange(0, depth_m.shape[0], max(1, stride), dtype=np.int32)
    xs = np.arange(0, depth_m.shape[1], max(1, stride), dtype=np.int32)
    grid_x, grid_y = np.meshgrid(xs, ys)
    z = depth_m[grid_y, grid_x].astype(np.float64)
    valid = np.isfinite(z) & (z > 0.05) & (z <= max_depth_m)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64)
    x = (grid_x[valid].astype(np.float64) - camera_matrix[0, 2]) * z[valid] / camera_matrix[0, 0]
    y = (grid_y[valid].astype(np.float64) - camera_matrix[1, 2]) * z[valid] / camera_matrix[1, 1]
    return np.c_[x, y, z[valid]]


def draw_projection(
    title: str,
    axis_a: int,
    axis_b: int,
    axis_labels: tuple[str, str],
    pointcloud_base: np.ndarray,
    board_base: np.ndarray,
    camera_origin: np.ndarray,
    gripper_origin: np.ndarray,
    canvas_size: tuple[int, int] = (960, 720),
) -> np.ndarray:
    width, height = canvas_size
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)

    groups = [board_base, camera_origin.reshape(1, 3), gripper_origin.reshape(1, 3)]
    if pointcloud_base.size:
        groups.append(pointcloud_base)
    all_points = np.vstack(groups)
    a_values = all_points[:, axis_a]
    b_values = all_points[:, axis_b]
    a_min, a_max = float(a_values.min()), float(a_values.max())
    b_min, b_max = float(b_values.min()), float(b_values.max())
    pad = max(0.05, 0.08 * max(a_max - a_min, b_max - b_min, 0.1))
    a_min -= pad
    a_max += pad
    b_min -= pad
    b_max += pad
    scale = min((width - 110) / max(a_max - a_min, 1e-6), (height - 110) / max(b_max - b_min, 1e-6))

    def pix(points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        x = 55 + (pts[:, axis_a] - a_min) * scale
        y = height - 55 - (pts[:, axis_b] - b_min) * scale
        return np.round(np.c_[x, y]).astype(np.int32)

    for value in np.arange(math.floor(a_min * 10) / 10, math.ceil(a_max * 10) / 10 + 0.001, 0.1):
        x = int(round(55 + (value - a_min) * scale))
        cv2.line(canvas, (x, 40), (x, height - 45), (232, 232, 232), 1)
    for value in np.arange(math.floor(b_min * 10) / 10, math.ceil(b_max * 10) / 10 + 0.001, 0.1):
        y = int(round(height - 55 - (value - b_min) * scale))
        cv2.line(canvas, (45, y), (width - 40, y), (232, 232, 232), 1)

    if pointcloud_base.size:
        pts = pix(pointcloud_base)
        step = max(1, len(pts) // 25000)
        for x, y in pts[::step]:
            if 0 <= x < width and 0 <= y < height:
                canvas[y, x] = (180, 180, 180)

    board_pts = pix(board_base)
    hull = cv2.convexHull(board_pts)
    cv2.polylines(canvas, [hull], isClosed=True, color=(0, 140, 0), thickness=2, lineType=cv2.LINE_AA)
    for x, y in board_pts:
        cv2.circle(canvas, (int(x), int(y)), 3, (0, 170, 0), -1, lineType=cv2.LINE_AA)

    cam_xy = tuple(pix(camera_origin.reshape(1, 3))[0])
    grip_xy = tuple(pix(gripper_origin.reshape(1, 3))[0])
    cv2.circle(canvas, cam_xy, 7, (255, 0, 0), -1, lineType=cv2.LINE_AA)
    cv2.putText(canvas, "camera", (cam_xy[0] + 8, cam_xy[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv2.circle(canvas, grip_xy, 7, (0, 0, 220), -1, lineType=cv2.LINE_AA)
    cv2.putText(canvas, "gripper", (grip_xy[0] + 8, grip_xy[1] + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 220), 1)

    cv2.putText(canvas, title, (24, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(canvas, f"{axis_labels[0]} / {axis_labels[1]} in base frame", (24, height - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 80), 1, cv2.LINE_AA)
    return canvas


def draw_report(
    rgb_overlay: np.ndarray,
    top_view: np.ndarray,
    side_view: np.ndarray,
    summary: dict[str, object],
) -> np.ndarray:
    rgb = rgb_overlay
    target_w = 960
    scale = target_w / rgb.shape[1]
    rgb_resized = cv2.resize(rgb, (target_w, int(round(rgb.shape[0] * scale))), interpolation=cv2.INTER_AREA)
    top = top_view
    side = side_view
    if top.shape[1] != side.shape[1]:
        side = cv2.resize(side, (top.shape[1], side.shape[0]), interpolation=cv2.INTER_AREA)
    right = np.vstack([top, side])
    if right.shape[0] != rgb_resized.shape[0]:
        right = cv2.resize(right, (right.shape[1], rgb_resized.shape[0]), interpolation=cv2.INTER_AREA)
    report = np.hstack([rgb_resized, right])

    footer = np.full((132, report.shape[1], 3), 250, dtype=np.uint8)
    lines = [
        f"result: {summary['result_json']}",
        f"T_base_camera translation m: {summary['t_base_camera']['position']}",
        f"T_base_board translation m: {summary['t_base_board']['position']}",
        f"depth points: {summary['pointcloud']['point_count']} ({summary['pointcloud']['status']})",
    ]
    y = 28
    for line in lines:
        cv2.putText(footer, line[:180], (18, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
        y += 30
    return np.vstack([report, footer])


def matrix_payload(matrix: np.ndarray) -> dict[str, object]:
    return {
        "position": [float(v) for v in matrix[:3, 3]],
        "matrix": [[float(v) for v in row] for row in matrix],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="cam_left")
    parser.add_argument("--result-json", default="/home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard/latest_checkerboard_handeye.json")
    parser.add_argument("--robot-pose-topic", default="/left/gripper_end_pos")
    parser.add_argument("--checkerboard-pose-topic", default="")
    parser.add_argument("--rgb-topic", default="")
    parser.add_argument("--depth-topic", default="")
    parser.add_argument("--camera-info-topic", default="")
    parser.add_argument("--pattern-cols", type=int, default=8)
    parser.add_argument("--pattern-rows", type=int, default=11)
    parser.add_argument("--square-size-m", type=float, default=0.02)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument("--point-stride", type=int, default=8)
    parser.add_argument("--max-depth-m", type=float, default=2.0)
    parser.add_argument("--out-dir", default="")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result_json = Path(args.result_json)
    result = json.loads(result_json.read_text(encoding="utf-8"))
    t_gripper_camera = np.asarray(result["result"]["matrix"], dtype=np.float64)

    camera = args.camera
    rgb_topic = args.rgb_topic or f"/{camera}/color/image_raw/compressed"
    depth_topic = args.depth_topic or f"/{camera}/aligned_depth_to_color/image_raw"
    camera_info_topic = args.camera_info_topic or f"/{camera}/color/camera_info"
    checkerboard_pose_topic = args.checkerboard_pose_topic or f"/checkerboard/{camera}/pose"
    out_dir = Path(args.out_dir) if args.out_dir else Path("/home/ligl/agilex_xpc/calibration/application_validation") / dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = LiveCapture(rgb_topic, camera_info_topic, args.robot_pose_topic, checkerboard_pose_topic, depth_topic)
    deadline = time.monotonic() + max(args.timeout_sec, 1.0)
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.required_ready():
            rclpy.spin_once(node, timeout_sec=0.05)
        if not node.required_ready():
            missing = {
                "rgb": node.rgb is None,
                "camera_info": node.camera_info is None,
                "robot_pose": node.robot_pose is None,
                "checkerboard_pose": node.checkerboard_pose is None,
            }
            raise RuntimeError(f"missing required messages: {missing}")

        if depth_topic:
            while rclpy.ok() and time.monotonic() < deadline and node.depth is None:
                rclpy.spin_once(node, timeout_sec=0.05)

        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.03)

        rgb = decode_compressed_rgb(node.rgb)
        camera_info = node.camera_info
        camera_matrix = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        t_base_gripper = pose_to_matrix(node.robot_pose)
        t_camera_board = pose_to_matrix(node.checkerboard_pose.pose)
        t_base_camera = t_base_gripper @ t_gripper_camera
        t_base_board = t_base_camera @ t_camera_board

        object_points = make_object_points(args.pattern_cols, args.pattern_rows, args.square_size_m)
        found, detected_corners = detect_chessboard(rgb, (args.pattern_cols, args.pattern_rows))
        rgb_overlay = draw_rgb_overlay(rgb, object_points, t_camera_board, camera_matrix, detected_corners)

        pointcloud_base = np.empty((0, 3), dtype=np.float64)
        pointcloud_status = "depth_not_received"
        if node.depth is not None:
            depth_m = decode_depth(node.depth)
            pointcloud_camera = backproject_depth(depth_m, camera_matrix, args.point_stride, args.max_depth_m)
            if pointcloud_camera.size:
                pointcloud_base = transform_points(t_base_camera, pointcloud_camera)
                pointcloud_status = "ok"
            else:
                pointcloud_status = "no_valid_depth_points"

        board_base = transform_points(t_base_board, object_points)
        camera_origin_base = transform_points(t_base_camera, np.zeros((1, 3), dtype=np.float64))[0]
        gripper_origin_base = transform_points(t_base_gripper, np.zeros((1, 3), dtype=np.float64))[0]

        top_view = draw_projection(
            "Base Frame Top View",
            0,
            1,
            ("X", "Y"),
            pointcloud_base,
            board_base,
            camera_origin_base,
            gripper_origin_base,
        )
        side_view = draw_projection(
            "Base Frame Side View",
            0,
            2,
            ("X", "Z"),
            pointcloud_base,
            board_base,
            camera_origin_base,
            gripper_origin_base,
        )

        summary = {
            "created_at": time.time(),
            "camera": camera,
            "result_json": str(result_json),
            "result_sample_count": result.get("sample_count"),
            "result_excluded_sample_indexes": result.get("excluded_sample_indexes", []),
            "topics": {
                "rgb": rgb_topic,
                "depth": depth_topic,
                "camera_info": camera_info_topic,
                "robot_pose": args.robot_pose_topic,
                "checkerboard_pose": checkerboard_pose_topic,
            },
            "checkerboard_detection_in_rgb": {
                "found": bool(found),
                "corner_count": int(len(detected_corners)) if detected_corners is not None else 0,
            },
            "t_gripper_camera": matrix_payload(t_gripper_camera),
            "t_base_gripper": matrix_payload(t_base_gripper),
            "t_base_camera": matrix_payload(t_base_camera),
            "t_camera_board": matrix_payload(t_camera_board),
            "t_base_board": matrix_payload(t_base_board),
            "pointcloud": {
                "status": pointcloud_status,
                "point_count": int(len(pointcloud_base)),
                "stride": int(args.point_stride),
                "max_depth_m": float(args.max_depth_m),
            },
            "outputs": {},
        }

        rgb_path = out_dir / f"{camera}_rgb_overlay.png"
        top_path = out_dir / f"{camera}_base_top_view.png"
        side_path = out_dir / f"{camera}_base_side_view.png"
        report_path = out_dir / f"{camera}_application_validation_report.png"
        summary_path = out_dir / "summary.json"

        report = draw_report(rgb_overlay, top_view, side_view, summary)
        cv2.imwrite(str(rgb_path), rgb_overlay)
        cv2.imwrite(str(top_path), top_view)
        cv2.imwrite(str(side_path), side_view)
        cv2.imwrite(str(report_path), report)
        summary["outputs"] = {
            "rgb_overlay": str(rgb_path),
            "base_top_view": str(top_path),
            "base_side_view": str(side_path),
            "report": str(report_path),
            "summary": str(summary_path),
        }
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
