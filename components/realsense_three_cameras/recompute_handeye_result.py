#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np


METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


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


def parse_exclude(value: str) -> set[int]:
    if not value.strip():
        return set()
    return {int(item.strip()) for item in value.split(",") if item.strip()}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "result_json",
        nargs="?",
        default="/home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard/latest_checkerboard_handeye.json",
    )
    parser.add_argument("--exclude", default="", help="1-based sample indexes, comma separated")
    parser.add_argument("--method", choices=sorted(METHODS), default="")
    parser.add_argument("--write-latest", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    path = Path(args.result_json)
    data = json.loads(path.read_text(encoding="utf-8"))
    exclude = parse_exclude(args.exclude)
    samples = [sample for index, sample in enumerate(data["samples"], 1) if index not in exclude]
    if len(samples) < 5:
        raise RuntimeError("at least 5 samples are required after exclusions")

    mode = data.get("mode", "eye_in_hand")
    method = args.method or data.get("method", "tsai")
    r_gripper2base = []
    t_gripper2base = []
    r_target2cam = []
    t_target2cam = []
    for sample in samples:
        robot_matrix = np.asarray(sample["robot_matrix"]["matrix"], dtype=np.float64)
        if mode == "eye_to_hand":
            robot_matrix_for_solver = np.linalg.inv(robot_matrix)
        else:
            robot_matrix_for_solver = robot_matrix
        target_matrix = np.asarray(sample["target_matrix"]["matrix"], dtype=np.float64)
        r_gripper2base.append(robot_matrix_for_solver[:3, :3])
        t_gripper2base.append(robot_matrix_for_solver[:3, 3])
        r_target2cam.append(target_matrix[:3, :3])
        t_target2cam.append(target_matrix[:3, 3])

    ret_r, ret_t = cv2.calibrateHandEye(
        r_gripper2base,
        t_gripper2base,
        r_target2cam,
        t_target2cam,
        method=METHODS[method],
    )
    result_matrix = np.eye(4, dtype=np.float64)
    result_matrix[:3, :3] = ret_r
    result_matrix[:3, 3] = np.asarray(ret_t, dtype=np.float64).reshape(3)

    output = dict(data)
    output["method"] = method
    output["sample_count"] = len(samples)
    output["excluded_sample_indexes"] = sorted(exclude)
    output["result"] = matrix_payload(result_matrix)
    output["samples"] = samples

    suffix = f"exclude_{'-'.join(str(v) for v in sorted(exclude))}" if exclude else "recomputed"
    output_path = path.parent / f"{dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_{suffix}_checkerboard_handeye.json"
    payload = json.dumps(output, indent=2)
    output_path.write_text(payload + "\n", encoding="utf-8")
    if args.write_latest:
        (path.parent / "latest_checkerboard_handeye.json").write_text(payload + "\n", encoding="utf-8")

    print(json.dumps({"sample_count": len(samples), "excluded_sample_indexes": sorted(exclude), "result": output["result"]}, indent=2))
    print(f"saved: {output_path}")
    if args.write_latest:
        print(f"latest: {path.parent / 'latest_checkerboard_handeye.json'}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
