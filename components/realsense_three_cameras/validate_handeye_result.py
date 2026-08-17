#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def rotation_angle_deg(rotation: np.ndarray) -> float:
    value = (float(np.trace(rotation)) - 1.0) * 0.5
    value = max(-1.0, min(1.0, value))
    return math.degrees(math.acos(value))


def pairwise_translation_range(translations: np.ndarray) -> float:
    max_distance = 0.0
    for i in range(len(translations)):
        for j in range(i + 1, len(translations)):
            max_distance = max(max_distance, float(np.linalg.norm(translations[i] - translations[j])))
    return max_distance


def pairwise_rotation_range(rotations: list[np.ndarray]) -> float:
    max_angle = 0.0
    for i in range(len(rotations)):
        for j in range(i + 1, len(rotations)):
            max_angle = max(max_angle, rotation_angle_deg(rotations[i].T @ rotations[j]))
    return max_angle


def mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    accumulator = np.zeros((4, 4), dtype=np.float64)
    for rotation in rotations:
        quat = rotation_matrix_to_quaternion_wxyz(rotation)
        accumulator += np.outer(quat, quat)
    values, vectors = np.linalg.eigh(accumulator)
    quat = vectors[:, int(np.argmax(values))]
    if quat[0] < 0:
        quat = -quat
    return quaternion_wxyz_to_rotation_matrix(quat)


def rotation_matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
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
    quat = np.array([qw, qx, qy, qz], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def quaternion_wxyz_to_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = [float(v) for v in quat / np.linalg.norm(quat)]
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def load_matrix(payload: dict[str, object]) -> np.ndarray:
    return np.asarray(payload["matrix"], dtype=np.float64)


def verdict(position_median: float, position_max: float, rotation_median: float, rotation_max: float) -> str:
    if position_median <= 0.02 and position_max <= 0.05 and rotation_median <= 2.0 and rotation_max <= 5.0:
        return "PASS"
    if position_median <= 0.04 and position_max <= 0.10 and rotation_median <= 4.0 and rotation_max <= 10.0:
        return "BORDERLINE"
    return "FAIL"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "result_json",
        nargs="?",
        default="/home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard/latest_checkerboard_handeye.json",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result_path = Path(args.result_json)
    data = json.loads(result_path.read_text(encoding="utf-8"))
    samples = data.get("samples", [])
    if not samples:
        raise RuntimeError(f"no samples in {result_path}")

    transform = load_matrix(data["result"])
    fixed_target_poses = []
    robot_poses = []
    camera_target_poses = []

    for sample in samples:
        robot = load_matrix(sample["robot_matrix"])
        camera_target = load_matrix(sample["target_matrix"])
        fixed_target_poses.append(robot @ transform @ camera_target)
        robot_poses.append(robot)
        camera_target_poses.append(camera_target)

    fixed_target_translations = np.asarray([pose[:3, 3] for pose in fixed_target_poses])
    fixed_target_center = fixed_target_translations.mean(axis=0)
    position_errors = np.linalg.norm(fixed_target_translations - fixed_target_center, axis=1)

    fixed_target_rotations = [pose[:3, :3] for pose in fixed_target_poses]
    fixed_target_rotation_mean = mean_rotation(fixed_target_rotations)
    rotation_errors = np.asarray(
        [rotation_angle_deg(fixed_target_rotation_mean.T @ rotation) for rotation in fixed_target_rotations],
        dtype=np.float64,
    )

    robot_translations = np.asarray([pose[:3, 3] for pose in robot_poses])
    camera_target_translations = np.asarray([pose[:3, 3] for pose in camera_target_poses])

    result = {
        "result_json": str(result_path),
        "mode": data.get("mode"),
        "sample_count": int(len(samples)),
        "verdict": verdict(
            float(np.median(position_errors)),
            float(position_errors.max()),
            float(np.median(rotation_errors)),
            float(rotation_errors.max()),
        ),
        "fixed_target_position_error_m": {
            "mean": float(position_errors.mean()),
            "median": float(np.median(position_errors)),
            "max": float(position_errors.max()),
        },
        "fixed_target_rotation_error_deg": {
            "mean": float(rotation_errors.mean()),
            "median": float(np.median(rotation_errors)),
            "max": float(rotation_errors.max()),
        },
        "motion_coverage": {
            "robot_translation_range_m": pairwise_translation_range(robot_translations),
            "robot_rotation_range_deg": pairwise_rotation_range([pose[:3, :3] for pose in robot_poses]),
            "checkerboard_translation_range_in_camera_m": pairwise_translation_range(camera_target_translations),
            "checkerboard_rotation_range_in_camera_deg": pairwise_rotation_range(
                [pose[:3, :3] for pose in camera_target_poses]
            ),
        },
        "worst_samples": [
            {
                "index": int(index + 1),
                "position_error_m": float(position_errors[index]),
                "rotation_error_deg": float(rotation_errors[index]),
            }
            for index in np.argsort(-position_errors)[:5]
        ],
    }
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
