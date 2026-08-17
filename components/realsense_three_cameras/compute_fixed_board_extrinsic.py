#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def make_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    for row in range(rows):
        for col in range(cols):
            points[row * cols + col] = (col * square_size_m, row * square_size_m, 0.0)
    return points


def rigid_transform_3d(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if source.shape != target.shape or source.shape[0] < 3:
        raise ValueError("source and target must contain at least 3 paired points")
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    h = source_zero.T @ target_zero
    u, _s, vt = np.linalg.svd(h)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


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


def matrix_payload(matrix: np.ndarray) -> dict[str, object]:
    qx, qy, qz, qw = rotation_matrix_to_quaternion_xyzw(matrix[:3, :3])
    return {
        "position": [float(v) for v in matrix[:3, 3]],
        "orientation_xyzw": [qx, qy, qz, qw],
        "matrix": [[float(v) for v in row] for row in matrix],
    }


def reprojection_error(
    object_points: np.ndarray,
    image_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray | None,
) -> dict[str, float]:
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    error = np.linalg.norm(projected - image_points.reshape(-1, 2), axis=1)
    return {
        "mean_px": float(error.mean()),
        "median_px": float(np.median(error)),
        "max_px": float(error.max()),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sample", default="", help="snapshot name under calibration/snapshots")
    parser.add_argument("--sample-dir", default="", help="absolute snapshot directory")
    parser.add_argument("--camera", default="cam_high")
    parser.add_argument("--square-size-m", type=float, default=0.02)
    parser.add_argument(
        "--touch-file",
        default="/home/ligl/agilex_xpc/calibration/touch_points/checkerboard_touch_points.json",
    )
    parser.add_argument("--points", default="P0,P1,P2")
    parser.add_argument(
        "--result-dir",
        default="/home/ligl/agilex_xpc/calibration/results/fixed_board",
    )
    return parser.parse_args()


def resolve_sample_dir(args: argparse.Namespace) -> Path:
    if args.sample_dir:
        return Path(args.sample_dir)
    if args.sample:
        return Path("/home/ligl/agilex_xpc/calibration/snapshots") / args.sample
    raise ValueError("pass --sample or --sample-dir")


def main() -> None:
    args = parse_args()
    sample_dir = resolve_sample_dir(args)
    manifest_path = sample_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    camera = manifest["cameras"][args.camera]
    if not camera.get("chessboard_found"):
        raise RuntimeError(f"{args.camera} did not detect a checkerboard in {manifest_path}")
    corners_file = camera.get("corners_file")
    if not corners_file:
        raise RuntimeError(f"{args.camera} has no corners_file in {manifest_path}")

    pattern_cols, pattern_rows = [int(v) for v in manifest["pattern_size"]]
    object_points = make_object_points(pattern_cols, pattern_rows, args.square_size_m)
    image_points = np.load(corners_file).astype(np.float32).reshape(-1, 2)
    camera_info = camera["camera_info"]
    camera_matrix = np.asarray(camera_info["k"], dtype=np.float64).reshape(3, 3)
    dist_values = camera_info.get("d", [])
    dist_coeffs = np.asarray(dist_values, dtype=np.float64).reshape(-1, 1) if dist_values else None

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed")
    rotation_cam_board, _ = cv2.Rodrigues(rvec)
    t_cam_board = np.asarray(tvec, dtype=np.float64).reshape(3)
    t_camera_board = np.eye(4, dtype=np.float64)
    t_camera_board[:3, :3] = rotation_cam_board
    t_camera_board[:3, 3] = t_cam_board

    touch_data = json.loads(Path(args.touch_file).read_text(encoding="utf-8"))
    names = [name.strip() for name in args.points.split(",") if name.strip()]
    if len(names) < 3:
        raise ValueError("--points must contain at least 3 point names")

    board_points = []
    base_points = []
    used_points = []
    for name in names:
        point = touch_data["points"][name]
        col = int(point["col"])
        row = int(point["row"])
        board_points.append([col * args.square_size_m, row * args.square_size_m, 0.0])
        base_points.append([float(v) for v in point["position"]])
        used_points.append(point)
    board_points_np = np.asarray(board_points, dtype=np.float64)
    base_points_np = np.asarray(base_points, dtype=np.float64)
    rotation_base_board, t_base_board = rigid_transform_3d(board_points_np, base_points_np)
    t_base_board_matrix = np.eye(4, dtype=np.float64)
    t_base_board_matrix[:3, :3] = rotation_base_board
    t_base_board_matrix[:3, 3] = t_base_board

    t_base_camera = t_base_board_matrix @ np.linalg.inv(t_camera_board)

    result = {
        "created_at": time.time(),
        "camera": args.camera,
        "sample_dir": str(sample_dir),
        "touch_file": str(args.touch_file),
        "square_size_m": args.square_size_m,
        "pattern_size": [pattern_cols, pattern_rows],
        "used_touch_points": used_points,
        "reprojection_error": reprojection_error(
            object_points,
            image_points,
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        ),
        "t_camera_board": matrix_payload(t_camera_board),
        "t_base_board": matrix_payload(t_base_board_matrix),
        "t_base_camera": matrix_payload(t_base_camera),
    }

    result_dir = Path(args.result_dir)
    result_dir.mkdir(parents=True, exist_ok=True)
    result_path = result_dir / f"{args.camera}_{sample_dir.name}_fixed_board_extrinsic.json"
    latest_path = result_dir / f"{args.camera}_latest_fixed_board_extrinsic.json"
    payload = json.dumps(result, indent=2)
    result_path.write_text(payload + "\n", encoding="utf-8")
    latest_path.write_text(payload + "\n", encoding="utf-8")
    print(json.dumps({"t_base_camera": result["t_base_camera"], "reprojection_error": result["reprojection_error"]}, indent=2))
    print(f"saved: {result_path}")
    print(f"latest: {latest_path}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
