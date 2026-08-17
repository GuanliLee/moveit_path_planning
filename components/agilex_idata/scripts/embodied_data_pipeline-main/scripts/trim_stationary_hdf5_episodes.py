#!/usr/bin/env python3
"""Trim long stationary runs from converted HDF5 episode folders."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor, as_completed
import fcntl
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from quality_pipeline.episode_io import read_raw_episode
from quality_pipeline.profiles import load_profile
from quality_pipeline.qc import _quality_config, _stationary_pair_metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Trim converted HDF5 episodes by reducing each long stationary run "
            "to a fixed number of frames."
        ),
    )
    parser.add_argument("--input", required=True, help="HDF5 episode dir, HDF5 root, or aligned_joints.h5.")
    parser.add_argument("--profile", required=True, help="Path to robot_profiles/*.yaml.")
    parser.add_argument(
        "--episodeNames",
        default="",
        help="Comma-separated episode directory names to process. Default: all discovered episodes.",
    )
    parser.add_argument(
        "--keep-stationary-frames",
        type=int,
        default=15,
        help="Keep at most this many frames from each over-threshold stationary run.",
    )
    parser.add_argument(
        "--target-fps",
        type=float,
        default=30.0,
        help="Rewrite HDF5 main timestamps and output videos to this FPS.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned trims without modifying HDF5/videos/meta.",
    )
    parser.add_argument(
        "--num-workers",
        type=int,
        default=1,
        help="Number of episodes to process in parallel.",
    )
    parser.add_argument(
        "--video-workers",
        type=int,
        default=3,
        help="Number of videos to trim in parallel within each episode.",
    )
    return parser.parse_args()


def discover_episode_dirs(path: Path) -> list[Path]:
    path = path.expanduser().resolve()
    if path.is_file() and path.name in {"aligned_joints.h5", "aligned_joints.hdf5"}:
        return [path.parent.parent]
    if (path / "states" / "aligned_joints.h5").is_file():
        return [path]
    return sorted(
        {candidate.parent.parent for candidate in path.rglob("states/aligned_joints.h5")},
        key=lambda item: natural_key(str(item)),
    )


def natural_key(text: str) -> list[Any]:
    import re

    return [int(part) if part.isdigit() else part.lower() for part in re.split(r"(\d+)", text)]


def selected_names(text: str) -> set[str]:
    return {item.strip() for item in text.split(",") if item.strip()}


def stationary_runs(episode_dir: Path, profile: Any) -> tuple[list[tuple[int, int]], dict[str, Any]]:
    episode = read_raw_episode(episode_dir, profile)
    config = _quality_config(profile)
    epsilon = float(config["action_stationary_epsilon"])
    state_epsilon = float(config["stationary_state_epsilon"])
    base_velocity_epsilon = float(config["stationary_base_velocity_epsilon"])
    max_allowed = int(config["max_stationary_action_frames"])
    action_rows = stationary_action_rows(episode)
    state_rows = stationary_state_rows(episode)
    using_hdf5_stationary_vectors = bool(episode.stationary_action_vectors and episode.stationary_state_vectors)
    pair_count = min(len(action_rows), len(state_rows)) - 1

    runs: list[tuple[int, int]] = []
    current_start: int | None = None
    current_len = 0
    max_run = 0
    stationary_pairs = 0
    stationary_frames = 0

    for pair_idx in range(max(0, pair_count)):
        prev = action_rows[pair_idx]
        cur = action_rows[pair_idx + 1]
        if len(prev) != len(cur):
            is_stationary = False
            action_delta = 0.0
            state_delta = 0.0
            base_velocity_abs = 0.0
            stationary_source = "hdf5_full_motion_fields" if using_hdf5_stationary_vectors else "profile_state_action"
        else:
            metrics = _stationary_pair_metrics(
                profile,
                prev,
                cur,
                state_rows[pair_idx],
                state_rows[pair_idx + 1],
                using_hdf5_stationary_vectors,
            )
            action_delta = metrics["action_delta"]
            state_delta = metrics["state_delta"]
            base_velocity_abs = metrics["base_velocity_abs"]
            stationary_source = metrics["stationary_source"]
            is_stationary = (
                (action_delta <= epsilon or state_delta <= state_epsilon)
                and base_velocity_abs <= base_velocity_epsilon
            )

        if is_stationary:
            if current_start is None:
                current_start = pair_idx
            current_len += 1
            stationary_pairs += 1
            max_run = max(max_run, current_len)
            continue

        if current_start is not None:
            stationary_frames += current_len + 1
            if current_len + 1 > max_allowed:
                runs.append((current_start, current_start + current_len))
        current_start = None
        current_len = 0

    if current_start is not None:
        stationary_frames += current_len + 1
        if current_len + 1 > max_allowed:
            runs.append((current_start, current_start + current_len))

    return runs, {
        "frames": episode.n_frames,
        "actions": len(action_rows),
        "stationary_frames": stationary_frames,
        "stationary_pairs": stationary_pairs,
        "max_stationary_run": max_run,
        "max_stationary_run_frames": max_run + 1 if max_run else 0,
        "over_threshold_stationary_runs": len(runs),
        "max_allowed_stationary_run": max_allowed,
        "action_stationary_epsilon": epsilon,
        "state_stationary_epsilon": state_epsilon,
        "base_velocity_epsilon": base_velocity_epsilon,
        "stationary_state_dim": len(state_rows[0]) if state_rows else 0,
        "stationary_action_dim": len(action_rows[0]) if action_rows else 0,
        "stationary_source": stationary_source if pair_count > 0 else (
            "hdf5_full_motion_fields" if using_hdf5_stationary_vectors else "profile_state_action"
        ),
    }


def stationary_state_rows(episode: Any) -> list[list[float]]:
    if episode.stationary_state_vectors and len(episode.stationary_state_vectors) >= len(episode.state_frames):
        return episode.stationary_state_vectors
    return [frame.state for frame in episode.state_frames]


def stationary_action_rows(episode: Any) -> list[list[float]]:
    if episode.stationary_action_vectors and len(episode.stationary_action_vectors) >= len(episode.actions):
        return episode.stationary_action_vectors
    return episode.actions


def uniform_sample_run_positions(start_frame: int, end_frame: int, keep_frames: int) -> set[int]:
    run_frame_count = end_frame - start_frame + 1
    keep_frames = max(1, min(int(keep_frames), run_frame_count))
    if run_frame_count <= keep_frames:
        return set(range(start_frame, end_frame + 1))
    if keep_frames == 1:
        return {start_frame}
    span = run_frame_count - 1
    sampled = {
        start_frame + round(index * span / (keep_frames - 1))
        for index in range(keep_frames)
    }
    # Defensive fill: rounded linspace should be unique here, but keep exactly
    # keep_frames positions even if Python rounding ever collapses a value.
    if len(sampled) < keep_frames:
        for pos in range(start_frame, end_frame + 1):
            sampled.add(pos)
            if len(sampled) >= keep_frames:
                break
    return sampled


def keep_positions_for_runs(frame_count: int, runs: list[tuple[int, int]], keep_frames: int) -> list[int]:
    remove: set[int] = set()
    keep_frames = max(1, int(keep_frames))
    for start, end in runs:
        start_frame = max(0, start)
        end_frame = min(frame_count - 1, end)
        run_frame_count = end_frame - start_frame + 1
        if run_frame_count <= keep_frames:
            continue
        keep_in_run = uniform_sample_run_positions(start_frame, end_frame, keep_frames)
        remove.update(pos for pos in range(start_frame, end_frame + 1) if pos not in keep_in_run)
    return [idx for idx in range(frame_count) if idx not in remove]


def numeric_frame_keys(h5_path: Path) -> list[str]:
    with h5py.File(h5_path, "r") as f:
        return sorted((str(key) for key in f.keys() if str(key).isdigit()), key=lambda key: int(key))


def read_first_timestamp(h5_path: Path, first_key: str) -> int:
    with h5py.File(h5_path, "r") as f:
        value = f[f"{first_key}/main_timestamp"][()]
        return int(np.asarray(value).reshape(-1)[0])


@contextmanager
def episode_lock(episode_dir: Path) -> Any:
    lock_path = episode_dir / "states" / ".stationary_trim.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("w", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def h5_stationary_trim_attrs(h5_path: Path) -> dict[str, Any]:
    if not h5_path.exists():
        return {}
    with h5py.File(h5_path, "r") as f:
        if not bool(f.attrs.get("stationary_trimmed", False)):
            return {}
        return {
            "target_fps": float(f.attrs.get("stationary_trim_target_fps", 0.0) or 0.0),
            "original_frame_count": int(f.attrs.get("stationary_trim_original_frame_count", 0) or 0),
            "kept_frame_count": int(f.attrs.get("stationary_trim_kept_frame_count", 0) or 0),
            "removed_frame_count": int(f.attrs.get("stationary_trim_removed_frame_count", 0) or 0),
        }


def video_frame_count(path: Path) -> int:
    if not path.is_file():
        return 0
    try:
        import cv2
    except ImportError:
        return 0
    cap = cv2.VideoCapture(str(path))
    try:
        return int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    finally:
        cap.release()


def validate_no_partial_trim(episode_dir: Path, h5_path: Path) -> None:
    attrs = h5_stationary_trim_attrs(h5_path)
    if not attrs:
        return
    kept = int(attrs.get("kept_frame_count") or 0)
    meta_path = episode_dir / "meta" / "episode_meta.json"
    meta_has_trim = False
    meta_frame_count: int | None = None
    if meta_path.is_file():
        try:
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            meta_has_trim = isinstance(meta.get("stationary_trim"), dict)
            raw_frame_count = meta.get("frame_count")
            meta_frame_count = int(raw_frame_count) if raw_frame_count is not None else None
        except (OSError, ValueError, json.JSONDecodeError):
            meta_has_trim = False
    bad_videos = []
    for video_path in sorted((episode_dir / "videos").glob("*.mp4")):
        if ".stationary_trim_" in video_path.name:
            continue
        count = video_frame_count(video_path)
        if count and count != kept:
            bad_videos.append(f"{video_path.name}:{count}")
    temp_videos = sorted(path.name for path in (episode_dir / "videos").glob("*.stationary_trim_*"))
    if (not meta_has_trim) or (meta_frame_count not in (None, kept)) or bad_videos or temp_videos:
        raise RuntimeError(
            "Partial stationary trim detected; regenerate this episode from MCAP before trimming again. "
            f"episode={episode_dir.name}, hdf5_kept={kept}, meta_has_trim={meta_has_trim}, "
            f"meta_frame_count={meta_frame_count}, bad_videos={bad_videos}, temp_videos={temp_videos}"
        )


def set_timestamp_dataset(group: Any, dataset_path: str, timestamp_ns: int) -> None:
    if dataset_path not in group:
        return
    dataset = group[dataset_path]
    dtype = dataset.dtype
    shape = dataset.shape
    del group[dataset_path]
    value = np.asarray(timestamp_ns, dtype=dtype)
    if shape:
        value = np.full(shape, timestamp_ns, dtype=dtype)
    group.create_dataset(dataset_path, data=value)


def rewrite_group_timestamps(group: Any, timestamp_ns: int) -> None:
    set_timestamp_dataset(group, "main_timestamp", timestamp_ns)
    if "timestamp" not in group:
        return
    timestamp_group = group["timestamp"]
    for name in list(timestamp_group.keys()):
        item = timestamp_group[name]
        if isinstance(item, h5py.Dataset):
            set_timestamp_dataset(group, f"timestamp/{name}", timestamp_ns)
        elif isinstance(item, h5py.Group):
            for child in list(item.keys()):
                if isinstance(item[child], h5py.Dataset):
                    set_timestamp_dataset(group, f"timestamp/{name}/{child}", timestamp_ns)


def trim_hdf5(
    h5_path: Path,
    keep_positions: list[int],
    target_fps: float,
    dry_run: bool,
    attr_prefix: str = "stationary_trim",
) -> dict[str, Any]:
    frame_keys = numeric_frame_keys(h5_path)
    if not frame_keys:
        raise ValueError(f"No numeric frame groups in {h5_path}")
    if len(keep_positions) == len(frame_keys):
        first_ts = read_first_timestamp(h5_path, frame_keys[0])
        last_ts = read_first_timestamp(h5_path, frame_keys[-1])
        duration = (last_ts - first_ts) / 1_000_000_000.0 if len(frame_keys) > 1 else 0.0
        inferred_fps = (len(frame_keys) - 1) / duration if duration > 1e-9 else 0.0
        return {
            "original_frame_count": len(frame_keys),
            "kept_frame_count": len(frame_keys),
            "removed_frame_count": 0,
            "first_timestamp_ns": first_ts,
            "last_timestamp_ns": last_ts,
            "duration": duration,
            "inferred_fps": inferred_fps,
        }
    if dry_run:
        first_ts = read_first_timestamp(h5_path, frame_keys[0])
        last_ts = first_ts + round((len(keep_positions) - 1) * 1_000_000_000.0 / target_fps)
        return {
            "original_frame_count": len(frame_keys),
            "kept_frame_count": len(keep_positions),
            "removed_frame_count": len(frame_keys) - len(keep_positions),
            "first_timestamp_ns": first_ts,
            "last_timestamp_ns": last_ts,
            "duration": (len(keep_positions) - 1) / target_fps if len(keep_positions) > 1 else 0.0,
            "inferred_fps": target_fps if len(keep_positions) > 1 else 0.0,
        }

    tmp_path = h5_path.with_name(
        f"{h5_path.stem}.{attr_prefix}_tmp.{os.getpid()}.{time.time_ns()}{h5_path.suffix}"
    )
    interval_ns = int(round(1_000_000_000.0 / target_fps))
    trimmed_attr = f"{attr_prefix}med" if attr_prefix.endswith("trim") else f"{attr_prefix}_trimmed"

    try:
        with h5py.File(h5_path, "r") as src, h5py.File(tmp_path, "w") as dst:
            first_ts = int(np.asarray(src[f"{frame_keys[0]}/main_timestamp"][()]).reshape(-1)[0])
            numeric_keys = set(frame_keys)
            for key in src.keys():
                if key not in numeric_keys:
                    src.copy(key, dst, name=key)
            for attr_key, attr_value in src.attrs.items():
                dst.attrs[attr_key] = attr_value
            dst.attrs[trimmed_attr] = True
            dst.attrs[f"{attr_prefix}_target_fps"] = target_fps
            dst.attrs[f"{attr_prefix}_original_frame_count"] = len(frame_keys)
            dst.attrs[f"{attr_prefix}_removed_frame_count"] = len(frame_keys) - len(keep_positions)
            dst.attrs[f"{attr_prefix}_kept_frame_count"] = len(keep_positions)

            for out_idx, source_pos in enumerate(keep_positions):
                timestamp_ns = first_ts + out_idx * interval_ns
                src.copy(frame_keys[source_pos], dst, name=str(out_idx))
                rewrite_group_timestamps(dst[str(out_idx)], timestamp_ns)
        tmp_path.replace(h5_path)
    except Exception:
        if tmp_path.exists():
            tmp_path.unlink()
        raise

    last_ts = first_ts + (len(keep_positions) - 1) * interval_ns
    return {
        "original_frame_count": len(frame_keys),
        "kept_frame_count": len(keep_positions),
        "removed_frame_count": len(frame_keys) - len(keep_positions),
        "first_timestamp_ns": first_ts,
        "last_timestamp_ns": last_ts,
        "duration": (len(keep_positions) - 1) / target_fps if len(keep_positions) > 1 else 0.0,
        "inferred_fps": target_fps if len(keep_positions) > 1 else 0.0,
    }


def open_writer(cv2: Any, output: Path, fps: float, size: tuple[int, int]) -> Any:
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if writer.isOpened():
        return writer
    writer.release()
    raise RuntimeError(f"Could not open video writer: {output}")


def transcode_to_h264(raw_tmp: Path, final_path: Path) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raw_tmp.replace(final_path)
        return
    h264_tmp = final_path.with_name(
        f"{final_path.stem}.stationary_trim_h264_tmp.{os.getpid()}.{time.time_ns()}{final_path.suffix}"
    )
    cmd = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(raw_tmp),
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        "-tag:v",
        "avc1",
        str(h264_tmp),
    ]
    try:
        subprocess.run(cmd, check=True)
    except Exception as exc:
        print(f"[WARN] ffmpeg H264 transcode failed for {final_path}: {exc}. Keeping mp4v output.")
        if h264_tmp.exists():
            h264_tmp.unlink()
        raw_tmp.replace(final_path)
        return
    raw_tmp.unlink(missing_ok=True)
    h264_tmp.replace(final_path)


def trim_video(video_path: Path, keep_positions: list[int], target_fps: float, dry_run: bool) -> dict[str, Any]:
    if not video_path.is_file():
        return {"file": video_path.name, "status": "missing"}
    if dry_run:
        return {"file": video_path.name, "status": "planned", "frame_count": len(keep_positions)}

    import cv2

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")
    raw_tmp = video_path.with_name(
        f"{video_path.stem}.stationary_trim_opencv_tmp.{os.getpid()}.{time.time_ns()}{video_path.suffix}"
    )

    keep_set = set(keep_positions)
    max_keep = max(keep_positions) if keep_positions else -1
    written = 0
    read_count = 0
    writer = None
    try:
        while read_count <= max_keep:
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            if read_count in keep_set:
                if writer is None:
                    height, width = frame.shape[:2]
                    writer = open_writer(cv2, raw_tmp, target_fps, (width, height))
                writer.write(frame)
                written += 1
            read_count += 1
    finally:
        cap.release()
        if writer is not None:
            writer.release()

    if writer is None:
        if raw_tmp.exists():
            raw_tmp.unlink()
        return {"file": video_path.name, "status": "empty", "frame_count": 0}
    transcode_to_h264(raw_tmp, video_path)
    return {
        "file": video_path.name,
        "status": "trimmed",
        "frame_count": written,
        "expected_frame_count": len(keep_positions),
        "source_frames_read": read_count,
    }


def remap_frame_value(value: Any, source_to_new: dict[int, int]) -> Any:
    try:
        old = int(value)
    except (TypeError, ValueError):
        return value
    if old in source_to_new:
        return source_to_new[old]
    lower = [source for source in source_to_new if source <= old]
    if lower:
        return source_to_new[max(lower)]
    return 0


def remap_segments(meta: dict[str, Any], source_to_new: dict[int, int]) -> None:
    for key in ("subtask_segments", "segment_instructions"):
        segments = meta.get(key)
        if not isinstance(segments, list):
            continue
        new_segments = []
        for segment in segments:
            if not isinstance(segment, dict):
                continue
            item = dict(segment)
            if "start" in item:
                item["start"] = remap_frame_value(item["start"], source_to_new)
            if "end" in item:
                item["end"] = remap_frame_value(item["end"], source_to_new)
            if "start_frame" in item:
                item["start_frame"] = remap_frame_value(item["start_frame"], source_to_new)
            if "end_frame" in item:
                item["end_frame"] = remap_frame_value(item["end_frame"], source_to_new)
            new_segments.append(item)
        meta[key] = new_segments


def update_meta(episode_dir: Path, h5_info: dict[str, Any], trim_info: dict[str, Any], video_results: list[dict[str, Any]]) -> None:
    meta_path = episode_dir / "meta" / "episode_meta.json"
    if not meta_path.exists():
        return
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    meta["frame_count"] = h5_info["kept_frame_count"]
    meta["duration"] = h5_info["duration"]
    meta["first_timestamp_ns"] = h5_info["first_timestamp_ns"]
    meta["last_timestamp_ns"] = h5_info["last_timestamp_ns"]
    meta["inferred_state_fps"] = h5_info["inferred_fps"]
    meta["video_fps"] = h5_info["inferred_fps"]
    stored_trim_info = dict(trim_info)
    stored_trim_info.pop("source_to_new_frame", None)
    meta["stationary_trim"] = stored_trim_info
    remap_segments(meta, trim_info.get("source_to_new_frame", {}))

    video_counts = {
        item["file"]: item["frame_count"]
        for item in video_results
        if item.get("status") == "trimmed" and "frame_count" in item
    }
    for item in meta.get("available_videos", []):
        if isinstance(item, dict) and item.get("file") in video_counts:
            item["frame_count"] = video_counts[item["file"]]

    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def process_episode(
    episode_dir: Path,
    profile: Any,
    keep_stationary_frames: int,
    target_fps: float,
    dry_run: bool,
    video_workers: int = 3,
) -> dict[str, Any]:
    with episode_lock(episode_dir):
        return _process_episode_locked(
            episode_dir,
            profile,
            keep_stationary_frames,
            target_fps,
            dry_run,
            video_workers,
        )


def _process_episode_locked(
    episode_dir: Path,
    profile: Any,
    keep_stationary_frames: int,
    target_fps: float,
    dry_run: bool,
    video_workers: int = 3,
) -> dict[str, Any]:
    h5_path = episode_dir / "states" / "aligned_joints.h5"
    if not h5_path.exists():
        raise FileNotFoundError(h5_path)
    max_passes = 5
    pass_results: list[dict[str, Any]] = []
    all_video_results: list[dict[str, Any]] = []
    initial_stationary_info: dict[str, Any] | None = None
    final_stationary_info: dict[str, Any] | None = None
    h5_info: dict[str, Any] | None = None
    initial_frame_count: int | None = None

    for pass_idx in range(1, max_passes + 1):
        validate_no_partial_trim(episode_dir, h5_path)
        frame_keys = numeric_frame_keys(h5_path)
        if initial_frame_count is None:
            initial_frame_count = len(frame_keys)
        runs, stationary_info = stationary_runs(episode_dir, profile)
        if initial_stationary_info is None:
            initial_stationary_info = stationary_info
        final_stationary_info = stationary_info

        if not runs:
            if h5_info is None:
                h5_info = trim_hdf5(h5_path, list(range(len(frame_keys))), target_fps, dry_run)
            break

        keep_positions = keep_positions_for_runs(len(frame_keys), runs, keep_stationary_frames)
        source_to_new = {source: new for new, source in enumerate(keep_positions)}
        trim_info = {
            "enabled": True,
            "target_fps": target_fps,
            "keep_stationary_frames": keep_stationary_frames,
            "keep_strategy": "uniform",
            "pass": pass_idx,
            "runs": [
                {
                    "start_frame": start,
                    "end_frame": min(len(frame_keys) - 1, end),
                    "source_frame_count": min(len(frame_keys) - 1, end) - start + 1,
                    "kept_frame_count": min(keep_stationary_frames, min(len(frame_keys) - 1, end) - start + 1),
                    "keep_strategy": "uniform",
                }
                for start, end in runs
            ],
            "stationary_qc": stationary_info,
            "source_to_new_frame": source_to_new,
        }
        h5_info = trim_hdf5(h5_path, keep_positions, target_fps, dry_run)
        trim_info.update(
            {
                "original_frame_count": h5_info["original_frame_count"],
                "kept_frame_count": h5_info["kept_frame_count"],
                "removed_frame_count": h5_info["removed_frame_count"],
            }
        )

        video_results: list[dict[str, Any]] = []
        video_dir = episode_dir / "videos"
        if h5_info["removed_frame_count"] > 0 and video_dir.is_dir():
            video_paths = sorted(video_dir.glob("*.mp4"))
            if video_workers > 1 and len(video_paths) > 1:
                with ThreadPoolExecutor(max_workers=max(1, int(video_workers))) as executor:
                    future_to_path = {
                        executor.submit(trim_video, video_path, keep_positions, target_fps, dry_run): video_path
                        for video_path in video_paths
                    }
                    for future in as_completed(future_to_path):
                        video_results.append(future.result())
                video_results.sort(key=lambda item: str(item.get("file") or ""))
            else:
                for video_path in video_paths:
                    video_results.append(trim_video(video_path, keep_positions, target_fps, dry_run))

        if not dry_run:
            update_meta(episode_dir, h5_info, trim_info, video_results)

        pass_result = {
            "pass": pass_idx,
            "runs": trim_info["runs"],
            "stationary_qc": stationary_info,
            "original_frame_count": h5_info["original_frame_count"],
            "kept_frame_count": h5_info["kept_frame_count"],
            "removed_frame_count": h5_info["removed_frame_count"],
        }
        pass_results.append(pass_result)
        all_video_results.extend({**item, "pass": pass_idx} for item in video_results)

        if dry_run or h5_info["removed_frame_count"] == 0:
            break

    if h5_info is None:
        frame_keys = numeric_frame_keys(h5_path)
        h5_info = trim_hdf5(h5_path, list(range(len(frame_keys))), target_fps, dry_run)
    stationary_info = initial_stationary_info or final_stationary_info or {}
    total_removed = sum(int(item.get("removed_frame_count") or 0) for item in pass_results)
    if pass_results:
        h5_info = {
            **h5_info,
            "original_frame_count": initial_frame_count or h5_info["original_frame_count"],
            "removed_frame_count": total_removed,
        }
    if not dry_run and len(pass_results) > 1:
        meta_path = episode_dir / "meta" / "episode_meta.json"
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text(encoding="utf-8"))
            stationary_trim = meta.get("stationary_trim")
            if isinstance(stationary_trim, dict):
                stationary_trim["passes"] = pass_results
                stationary_trim["original_frame_count"] = h5_info["original_frame_count"]
                stationary_trim["kept_frame_count"] = h5_info["kept_frame_count"]
                stationary_trim["removed_frame_count"] = total_removed
                stationary_trim["final_stationary_qc"] = final_stationary_info or {}
                meta["stationary_trim"] = stationary_trim
                meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    no_trim_reason = ""
    if h5_info["removed_frame_count"] == 0:
        no_trim_reason = (
            "no stationary run exceeded threshold "
            f"{stationary_info.get('max_allowed_stationary_run')} frames; "
            f"max run was {stationary_info.get('max_stationary_run_frames')} frames"
        )

    return {
        "episode": episode_dir.name,
        "status": "trimmed" if h5_info["removed_frame_count"] > 0 else "unchanged",
        "hdf5": str(h5_path),
        "original_frame_count": h5_info["original_frame_count"],
        "kept_frame_count": h5_info["kept_frame_count"],
        "removed_frame_count": h5_info["removed_frame_count"],
        "trim_passes": len(pass_results),
        "runs": [
            {**run, "pass": item["pass"]}
            for item in pass_results
            for run in item.get("runs", [])
        ],
        "stationary_qc": stationary_info,
        "final_stationary_qc": final_stationary_info or {},
        "no_trim_reason": no_trim_reason,
        "videos": all_video_results,
    }


def process_episode_worker(args: tuple[str, str, int, float, bool, int]) -> dict[str, Any]:
    episode_dir, profile_path, keep_stationary_frames, target_fps, dry_run, video_workers = args
    profile = load_profile(profile_path)
    return process_episode(
        Path(episode_dir),
        profile,
        keep_stationary_frames,
        target_fps,
        dry_run,
        video_workers,
    )


def main() -> int:
    args = parse_args()
    profile = load_profile(args.profile)
    input_path = Path(args.input).expanduser().resolve()
    names = selected_names(args.episodeNames)
    episode_dirs = discover_episode_dirs(input_path)
    if names:
        episode_dirs = [path for path in episode_dirs if path.name in names]
    if not episode_dirs:
        raise FileNotFoundError(f"No selected HDF5 episode directories found under {input_path}")

    results = []
    num_workers = max(1, int(args.num_workers or 1))
    video_workers = max(1, int(args.video_workers or 1))
    if num_workers > 1 and len(episode_dirs) > 1:
        worker_args = [
            (
                str(episode_dir),
                str(args.profile),
                args.keep_stationary_frames,
                args.target_fps,
                args.dry_run,
                video_workers,
            )
            for episode_dir in episode_dirs
        ]
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            future_to_episode = {
                executor.submit(process_episode_worker, item): Path(item[0]).name
                for item in worker_args
            }
            for future in as_completed(future_to_episode):
                result = future.result()
                results.append(result)
                print(json.dumps(result, ensure_ascii=False), flush=True)
    else:
        for episode_dir in episode_dirs:
            result = process_episode(
                episode_dir,
                profile,
                args.keep_stationary_frames,
                args.target_fps,
                args.dry_run,
                video_workers,
            )
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)

    results.sort(key=lambda item: natural_key(str(item.get("episode") or "")))

    summary = {
        "ok": True,
        "input": str(input_path),
        "episodes": len(results),
        "trimmed": sum(1 for item in results if item["status"] == "trimmed"),
        "removed_frames": sum(int(item["removed_frame_count"]) for item in results),
        "results": results,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
