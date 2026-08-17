#!/usr/bin/env python3
"""Verify mobile ALOHA HDF5 files before LeRobot conversion."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np


STATE_KEYS = [
    ("arm/jointStatePosition/puppetLeft", 7),
    ("arm/jointStatePosition/puppetRight", 7),
    ("robotBase/state/chassis", 6),
    ("lift/motor/column", 1),
]
ACTION_KEYS = [
    ("arm/jointStatePosition/masterLeft", 7),
    ("arm/jointStatePosition/masterRight", 7),
    ("robotBase/action/chassis", 3),
    ("action/lifting/column", 1),
]
CAMERA_KEYS = [
    "camera/color/left",
    "camera/color/front",
    "camera/color/right",
    "camera/color/head",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="HDF5 file or root containing episode HDF5 files.")
    parser.add_argument("--output", required=True, help="Output directory for mobile QC report.")
    parser.add_argument("--min-frames", type=int, default=1)
    parser.add_argument("--min-effective-fps", type=float, default=20.0)
    parser.add_argument("--capture-health", help="Recorder capture_health.json for full-episode camera coverage.")
    return parser.parse_args()


def discover_hdf5_files(root: Path) -> list[Path]:
    root = root.expanduser().resolve()
    if root.is_file():
        return [root]
    if not root.exists():
        raise FileNotFoundError(root)
    return sorted(path for path in root.rglob("*.hdf5") if path.is_file())


def as_array(dataset: h5py.Dataset) -> np.ndarray:
    data = dataset[()]
    array = np.asarray(data)
    if array.ndim == 1:
        return array.reshape(-1, 1)
    return array


def check_numeric_dataset(
    h5_file: h5py.File,
    key: str,
    width: int,
    frame_count: int | None,
    issues: list[str],
) -> int | None:
    if key not in h5_file:
        issues.append(f"missing {key}")
        return frame_count
    array = as_array(h5_file[key])
    if array.ndim != 2 or array.shape[1] != width:
        issues.append(f"{key} shape {tuple(array.shape)} != (*, {width})")
    if not np.all(np.isfinite(array.astype(np.float64, copy=False))):
        issues.append(f"{key} contains non-finite values")
    if frame_count is None:
        return int(array.shape[0])
    if int(array.shape[0]) != frame_count:
        issues.append(f"{key} frame count {array.shape[0]} != {frame_count}")
    return frame_count


def hdf5_string(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8")
    if hasattr(value, "item"):
        return hdf5_string(value.item())
    return str(value)


def check_camera_dataset(
    h5_path: Path,
    h5_file: h5py.File,
    key: str,
    frame_count: int,
    issues: list[str],
) -> dict[str, Any]:
    if key not in h5_file:
        issues.append(f"missing {key}")
        return {"key": key, "frames": 0}
    dataset = h5_file[key]
    frames = int(dataset.shape[0])
    if frames != frame_count:
        issues.append(f"{key} frame count {frames} != {frame_count}")

    sample_shape = None
    if frames > 0:
        sample = dataset[0]
        if dataset.ndim == 1:
            image_path = hdf5_string(sample)
            if not os.path.isabs(image_path):
                image_path = str(h5_path.parent / image_path)
            image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
            if image is None:
                issues.append(f"{key} sample image unreadable: {image_path}")
            else:
                sample_shape = list(image.shape)
        else:
            sample_shape = list(np.asarray(sample).shape)
    return {"key": key, "frames": frames, "sample_shape": sample_shape}


def check_timestamps(
    h5_file: h5py.File,
    frame_count: int,
    min_effective_fps: float,
    issues: list[str],
) -> dict[str, Any]:
    metrics: dict[str, Any] = {
        "duration_seconds": None,
        "effective_fps": None,
        "timestamp_frames": 0,
    }
    if "timestamp" not in h5_file:
        issues.append("missing timestamp")
        return metrics

    timestamps = np.asarray(h5_file["timestamp"][()], dtype=np.float64).reshape(-1)
    metrics["timestamp_frames"] = int(timestamps.shape[0])
    if int(timestamps.shape[0]) != frame_count:
        issues.append(f"timestamp frame count {timestamps.shape[0]} != {frame_count}")
    if timestamps.shape[0] < 2:
        return metrics
    if not np.all(np.isfinite(timestamps)):
        issues.append("timestamp contains non-finite values")
        return metrics

    diffs = np.diff(timestamps)
    if np.any(diffs <= 0):
        issues.append("timestamp is not strictly increasing")

    duration = float(timestamps[-1] - timestamps[0])
    metrics["duration_seconds"] = duration
    if duration <= 0:
        issues.append(f"timestamp duration {duration:.6f} <= 0")
        return metrics

    effective_fps = float((timestamps.shape[0] - 1) / duration)
    metrics["effective_fps"] = effective_fps
    if min_effective_fps > 0 and effective_fps < min_effective_fps:
        issues.append(
            f"effective fps {effective_fps:.2f} < {min_effective_fps:.2f}; "
            "time sync likely dropped too many frames"
        )
    return metrics


def verify_file(
    path: Path,
    min_frames: int,
    min_effective_fps: float,
    capture_health: dict[str, Any] | None = None,
) -> dict[str, Any]:
    issues: list[str] = []
    frame_count: int | None = None
    with h5py.File(path, "r") as h5_file:
        for key, width in STATE_KEYS + ACTION_KEYS:
            frame_count = check_numeric_dataset(h5_file, key, width, frame_count, issues)
        if frame_count is None:
            frame_count = 0
        if frame_count < min_frames:
            issues.append(f"frame count {frame_count} < {min_frames}")
        cameras = [
            check_camera_dataset(path, h5_file, key, frame_count, issues)
            for key in CAMERA_KEYS
        ]
        timing = check_timestamps(h5_file, frame_count, min_effective_fps, issues)

    if capture_health is not None and not bool(capture_health.get("camera_ok", False)):
        failed_topics = [str(topic) for topic in capture_health.get("failed_camera_topics", [])]
        issues.append(f"capture health failed: {', '.join(failed_topics) or 'unknown camera'}")

    return {
        "path": str(path),
        "ok": not issues,
        "frames": frame_count,
        **timing,
        "state_dim": sum(width for _, width in STATE_KEYS),
        "action_dim": sum(width for _, width in ACTION_KEYS),
        "cameras": cameras,
        "capture_health": capture_health,
        "issues": issues,
    }


def write_markdown(report: dict[str, Any], output_path: Path) -> None:
    lines = [
        "# Mobile HDF5 QC",
        "",
        f"- ok: {report['ok']}",
        f"- processed: {report['processed']}",
        f"- accepted: {report['accepted']}",
        f"- rejected: {report['rejected']}",
        "",
        "| episode | ok | frames | effective_fps | issues |",
        "|---|---:|---:|---:|---|",
    ]
    for episode in report["episodes"]:
        issue_text = "; ".join(episode["issues"])
        effective_fps = episode.get("effective_fps")
        effective_fps_text = "" if effective_fps is None else f"{effective_fps:.2f}"
        lines.append(
            f"| {Path(episode['path']).parent.name} | {episode['ok']} | "
            f"{episode['frames']} | {effective_fps_text} | {issue_text} |"
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    hdf5_files = discover_hdf5_files(Path(args.input))
    if not hdf5_files:
        raise FileNotFoundError(f"No .hdf5 files found under {args.input}")

    capture_health = None
    if args.capture_health:
        capture_health_path = Path(args.capture_health).expanduser().resolve()
        if not capture_health_path.is_file():
            raise FileNotFoundError(f"Missing capture health file: {capture_health_path}")
        capture_health = json.loads(capture_health_path.read_text(encoding="utf-8"))

    episodes = [
        verify_file(path, args.min_frames, args.min_effective_fps, capture_health=capture_health)
        for path in hdf5_files
    ]
    accepted = sum(1 for episode in episodes if episode["ok"])
    report = {
        "ok": accepted == len(episodes),
        "input": str(Path(args.input).expanduser().resolve()),
        "processed": len(episodes),
        "accepted": accepted,
        "rejected": len(episodes) - accepted,
        "episodes": episodes,
    }

    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "mobile_qc.json"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    write_markdown(report, output_dir / "mobile_qc.md")

    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
