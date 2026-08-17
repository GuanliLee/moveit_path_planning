#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


HAND_EYE_METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass
class TimedMessage:
    message: object
    received_at: float


def quaternion_xyzw_to_rotation_matrix(values: list[float]) -> np.ndarray:
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("invalid zero-length quaternion")
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
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
    return qx / norm, qy / norm, qz / norm, qw / norm


def rotation_matrix_to_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def pose_to_matrix(pose: Pose) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    matrix[:3, :3] = quaternion_xyzw_to_rotation_matrix([float(v) for v in q])
    matrix[:3, 3] = [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    ]
    return matrix


def matrix_payload(matrix: np.ndarray) -> dict[str, object]:
    rotation = matrix[:3, :3]
    qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(rotation)
    roll, pitch, yaw = rotation_matrix_to_rpy(rotation)
    return {
        "position": [float(v) for v in matrix[:3, 3]],
        "orientation_xyzw": [qx, qy, qz, qw],
        "rpy_rad": [roll, pitch, yaw],
        "rpy_deg": [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)],
        "matrix": [[float(v) for v in row] for row in matrix],
    }


class PairSampler(Node):
    def __init__(self, robot_pose_topic: str, target_pose_topic: str) -> None:
        super().__init__("checkerboard_handeye_pair_sampler")
        self.latest_robot_pose: Optional[TimedMessage] = None
        self.latest_target_pose: Optional[TimedMessage] = None
        self.robot_sub = self.create_subscription(Pose, robot_pose_topic, self._on_robot_pose, 10)
        self.target_sub = self.create_subscription(PoseStamped, target_pose_topic, self._on_target_pose, 10)

    def _on_robot_pose(self, msg: Pose) -> None:
        self.latest_robot_pose = TimedMessage(msg, time.monotonic())

    def _on_target_pose(self, msg: PoseStamped) -> None:
        self.latest_target_pose = TimedMessage(msg, time.monotonic())


def message_age(timed: Optional[TimedMessage]) -> float:
    if timed is None:
        return float("inf")
    return time.monotonic() - timed.received_at


def format_age(age: float) -> str:
    if not math.isfinite(age):
        return "none"
    return f"{age:.2f}s"


def run_executor(node: Node) -> tuple[SingleThreadedExecutor, threading.Thread]:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["eye_in_hand", "eye_to_hand"], required=True)
    parser.add_argument("--robot-pose-topic", required=True)
    parser.add_argument("--target-pose-topic", required=True)
    parser.add_argument("--min-samples", type=int, default=15)
    parser.add_argument("--max-age-sec", type=float, default=2.0)
    parser.add_argument("--method", choices=sorted(HAND_EYE_METHODS), default="tsai")
    parser.add_argument(
        "--result-dir",
        default="/home/ligl/agilex_xpc/calibration/results/checkerboard_handeye",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.min_samples < 5:
        raise ValueError("--min-samples must be at least 5")

    rclpy.init()
    node = PairSampler(args.robot_pose_topic, args.target_pose_topic)
    executor, thread = run_executor(node)
    samples: list[dict[str, object]] = []

    print("")
    print("Checkerboard hand-eye calibration")
    print(f"mode: {args.mode}")
    print(f"robot_pose_topic: {args.robot_pose_topic}")
    print(f"target_pose_topic: {args.target_pose_topic}")
    print(f"min_samples: {args.min_samples}")
    print("")
    print("Keys: Enter=capture, d=delete last, q=calculate, c=cancel")

    try:
        while True:
            robot_age = message_age(node.latest_robot_pose)
            target_age = message_age(node.latest_target_pose)
            prompt = (
                f"\n[{len(samples)}/{args.min_samples}] "
                f"robot_age={format_age(robot_age)} target_age={format_age(target_age)} > "
            )
            try:
                user_input = input(prompt).strip().lower()
            except EOFError:
                user_input = "q"

            if user_input == "":
                if node.latest_robot_pose is None or node.latest_target_pose is None:
                    print("missing robot pose or checkerboard pose; do not capture this sample")
                    continue
                if robot_age > args.max_age_sec or target_age > args.max_age_sec:
                    print(
                        "stale data; move slower or check publishers "
                        f"(robot_age={format_age(robot_age)}, target_age={format_age(target_age)})"
                    )
                    continue

                robot_matrix = pose_to_matrix(node.latest_robot_pose.message)
                if args.mode == "eye_to_hand":
                    robot_matrix_for_solver = np.linalg.inv(robot_matrix)
                else:
                    robot_matrix_for_solver = robot_matrix

                target_matrix = pose_to_matrix(node.latest_target_pose.message.pose)
                samples.append(
                    {
                        "captured_at": time.time(),
                        "robot_matrix": robot_matrix,
                        "robot_matrix_for_solver": robot_matrix_for_solver,
                        "target_matrix": target_matrix,
                    }
                )
                robot_t = robot_matrix[:3, 3]
                target_t = target_matrix[:3, 3]
                print(
                    "captured "
                    f"{len(samples)}: robot=[{robot_t[0]:.3f}, {robot_t[1]:.3f}, {robot_t[2]:.3f}], "
                    f"target=[{target_t[0]:.3f}, {target_t[1]:.3f}, {target_t[2]:.3f}]"
                )
            elif user_input == "d":
                if samples:
                    samples.pop()
                    print(f"deleted last sample; remaining={len(samples)}")
                else:
                    print("no samples to delete")
            elif user_input == "q":
                if len(samples) < args.min_samples:
                    print(f"need at least {args.min_samples} samples; current={len(samples)}")
                    continue
                break
            elif user_input == "c":
                print("cancelled")
                return
            else:
                print("unknown key")

        r_gripper2base = [sample["robot_matrix_for_solver"][:3, :3] for sample in samples]
        t_gripper2base = [sample["robot_matrix_for_solver"][:3, 3] for sample in samples]
        r_target2cam = [sample["target_matrix"][:3, :3] for sample in samples]
        t_target2cam = [sample["target_matrix"][:3, 3] for sample in samples]

        ret_r, ret_t = cv2.calibrateHandEye(
            r_gripper2base,
            t_gripper2base,
            r_target2cam,
            t_target2cam,
            method=HAND_EYE_METHODS[args.method],
        )
        result_matrix = np.eye(4, dtype=np.float64)
        result_matrix[:3, :3] = ret_r
        result_matrix[:3, 3] = np.asarray(ret_t, dtype=np.float64).reshape(3)

        result = {
            "mode": args.mode,
            "method": args.method,
            "robot_pose_topic": args.robot_pose_topic,
            "target_pose_topic": args.target_pose_topic,
            "sample_count": len(samples),
            "result_convention": (
                "Matches the AgileX handeye_calibration_ros OpenCV input convention. "
                "eye_in_hand returns camera-to-gripper transform; eye_to_hand uses inverted robot poses."
            ),
            "result": matrix_payload(result_matrix),
            "samples": [
                {
                    "captured_at": float(sample["captured_at"]),
                    "robot_matrix": matrix_payload(sample["robot_matrix"]),
                    "target_matrix": matrix_payload(sample["target_matrix"]),
                }
                for sample in samples
            ],
        }

        result_dir = Path(args.result_dir)
        result_dir.mkdir(parents=True, exist_ok=True)
        filename = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S_checkerboard_handeye.json")
        result_path = result_dir / filename
        latest_path = result_dir / "latest_checkerboard_handeye.json"
        payload = json.dumps(result, indent=2)
        result_path.write_text(payload + "\n", encoding="utf-8")
        latest_path.write_text(payload + "\n", encoding="utf-8")

        print("")
        print("Calibration result")
        print(json.dumps(result["result"], indent=2))
        print("")
        print(f"saved: {result_path}")
        print(f"latest: {latest_path}")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrupted")
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
