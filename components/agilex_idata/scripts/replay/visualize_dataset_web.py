#!/usr/bin/env python3
"""Web visualizer for ALOHA HDF5 and LeRobot datasets.

This script is intentionally read-only. It serves camera frames plus state/action
signals in a browser and never publishes robot commands.
"""

from __future__ import annotations

import argparse
from collections import OrderedDict
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import socket
import sys
import threading
import time
import webbrowser
from typing import Any
from urllib.parse import parse_qs, urlparse

try:
    import cv2
    import h5py
    import numpy as np
except ImportError as exc:  # pragma: no cover - runtime guard
    raise SystemExit(
        "visualize_dataset_web.py requires opencv-python, h5py and numpy."
    ) from exc

try:
    import pyarrow.parquet as pq
except ImportError:  # pragma: no cover - LeRobot-only dependency
    pq = None


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8765
DEFAULT_UI_FPS = 6.0
DEFAULT_JPEG_QUALITY = 82
DEFAULT_IMAGE_WIDTH = 640
CAMERA_ORDER = ["left", "front", "right", "head"]


@dataclass
class StageSegment:
    instruction: str
    start_frame_index: int
    end_frame_index: int


@dataclass
class EpisodeRecord:
    format: str
    dataset_name: str
    dataset_path: str
    episode_index: int
    frame_count: int
    fps: float
    task: str
    cameras: list[str]
    stage_segments: list[StageSegment]
    hdf5_path: str = ""
    parquet_path: str = ""
    video_paths: dict[str, str] | None = None

    def to_payload(self, index: int) -> dict[str, Any]:
        payload = asdict(self)
        payload["index"] = index
        payload["stage_segments"] = [asdict(item) for item in self.stage_segments]
        return payload


@dataclass
class SignalData:
    state: np.ndarray | None
    action: np.ndarray | None
    timestamps: np.ndarray
    frame_indices: np.ndarray
    state_labels: list[str]
    action_labels: list[str]
    state_column: str
    action_column: str


@dataclass
class Hdf5EpisodeData:
    camera_paths: dict[str, list[str]]
    episode_dir: Path
    signals: SignalData


@dataclass
class VideoEntry:
    path: Path
    cap: Any
    last_frame_index: int = -1
    last_frame: np.ndarray | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=None)
    parser.add_argument("--hdf5-root", type=Path, default=None)
    parser.add_argument("--lerobot-root", type=Path, default=None)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--start-format", choices=["auto", "hdf5", "lerobot"], default="auto")
    parser.add_argument("--start-episode", type=int, default=0)
    parser.add_argument("--ui-fps", type=float, default=DEFAULT_UI_FPS)
    parser.add_argument("--image-width", type=int, default=DEFAULT_IMAGE_WIDTH)
    parser.add_argument("--jpeg-quality", type=int, default=DEFAULT_JPEG_QUALITY)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--list", action="store_true", help="List discovered episodes and exit.")
    return parser.parse_args()


def natural_key(path: Path) -> list[Any]:
    out: list[Any] = []
    for part in re.split(r"(\d+)", path.name):
        if part:
            out.append(int(part) if part.isdigit() else part.lower())
    return out


def episode_index_from_name(text: str) -> int | None:
    match = re.search(r"episode[_-]?(\d+)", text)
    return int(match.group(1)) if match else None


def read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(obj, dict):
                rows.append(obj)
    return rows


def bytes_to_text(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, np.bytes_):
        return bytes(value).decode("utf-8", errors="replace")
    return str(value)


def first_dataset_text(root: h5py.File, path: str) -> str:
    if path not in root:
        return ""
    value = root[path][()]
    arr = np.asarray(value)
    if arr.size == 0:
        return ""
    return bytes_to_text(arr.reshape(-1)[0]).strip()


def infer_fps(timestamps: np.ndarray, fallback: float = 30.0) -> float:
    if timestamps.size > 2 and timestamps[-1] > timestamps[0]:
        fps = (timestamps.size - 1) / float(timestamps[-1] - timestamps[0])
        if np.isfinite(fps) and fps > 0:
            return float(fps)
    return fallback


def flatten_names(names: Any, dim: int, prefix: str) -> list[str]:
    if isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    if isinstance(names, list):
        flat = [str(item) for item in names]
        if len(flat) >= dim:
            return flat[:dim]
    return [f"{prefix}[{idx:02d}]" for idx in range(dim)]


def as_2d_array(value: Any, rows: int) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim == 0:
        arr = arr.reshape(1, 1)
    elif arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    if arr.shape[0] == rows:
        return arr
    out = np.full((rows, arr.shape[1]), np.nan, dtype=np.float64)
    n = min(rows, arr.shape[0])
    if n > 0:
        out[:n, :] = arr[:n, :]
    return out


def combine_arrays(
    root: h5py.File,
    rows: int,
    specs: list[tuple[str, list[str]]],
) -> tuple[np.ndarray | None, list[str]]:
    parts: list[np.ndarray] = []
    labels: list[str] = []
    for key, names in specs:
        if key not in root:
            continue
        arr = as_2d_array(root[key][()], rows)
        parts.append(arr)
        labels.extend(names[: arr.shape[1]])
        if len(names) < arr.shape[1]:
            labels.extend(f"{key}[{idx}]" for idx in range(len(names), arr.shape[1]))
    if not parts:
        return None, []
    return np.hstack(parts), labels


def hdf5_signal_data(path: Path) -> SignalData:
    with h5py.File(path, "r") as f:
        size = int(f["size"][()]) if "size" in f else 0
        if "timestamp" in f:
            timestamps = np.asarray(f["timestamp"][()], dtype=np.float64).reshape(-1)
            size = size or int(timestamps.size)
        else:
            timestamps = np.arange(size, dtype=np.float64) / 30.0
        if size <= 0:
            size = int(timestamps.size)
        if timestamps.size != size:
            fixed = np.arange(size, dtype=np.float64) / infer_fps(timestamps)
            fixed[: min(size, timestamps.size)] = timestamps[: min(size, timestamps.size)]
            timestamps = fixed

        state_specs = [
            ("arm/jointStatePosition/puppetLeft", [f"state.left_arm.joint{idx}" for idx in range(7)]),
            ("arm/jointStatePosition/puppetRight", [f"state.right_arm.joint{idx}" for idx in range(7)]),
            ("robotBase/state/chassis", ["base.x", "base.y", "base.yaw", "base.vx", "base.vy", "base.wz"]),
            ("lift/motor/column", ["lift.height"]),
        ]
        action_specs = [
            ("arm/jointStatePosition/masterLeft", [f"action.left_arm.joint{idx}" for idx in range(7)]),
            ("arm/jointStatePosition/masterRight", [f"action.right_arm.joint{idx}" for idx in range(7)]),
            ("robotBase/action/chassis", ["base.vx_cmd", "base.vy_cmd", "base.wz_cmd"]),
            ("action/lifting/column", ["lift.target"]),
        ]
        state, state_labels = combine_arrays(f, size, state_specs)
        action, action_labels = combine_arrays(f, size, action_specs)
    return SignalData(
        state=state,
        action=action,
        timestamps=timestamps,
        frame_indices=np.arange(size, dtype=np.int64),
        state_labels=state_labels,
        action_labels=action_labels,
        state_column="hdf5/mobile21",
        action_column="hdf5/mobile18",
    )


def hdf5_camera_names(h5_file: h5py.File) -> list[str]:
    if "camera/color" not in h5_file:
        return []
    available = list(h5_file["camera/color"].keys())
    return [name for name in CAMERA_ORDER if name in available] + [
        name for name in available if name not in CAMERA_ORDER
    ]


def hdf5_camera_paths(path: Path) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    with h5py.File(path, "r") as f:
        for name in hdf5_camera_names(f):
            key = f"camera/color/{name}"
            out[name] = [bytes_to_text(item) for item in np.asarray(f[key][()]).reshape(-1)]
    return out


def hdf5_stage_segments(path: Path, timestamps: np.ndarray) -> list[StageSegment]:
    segments: list[StageSegment] = []
    with h5py.File(path, "r") as f:
        base = "instructions/segment_instructions"
        if f"{base}/start_time" not in f or f"{base}/end_time" not in f:
            return []
        starts = [bytes_to_text(item) for item in np.asarray(f[f"{base}/start_time"][()]).reshape(-1)]
        ends = [bytes_to_text(item) for item in np.asarray(f[f"{base}/end_time"][()]).reshape(-1)]
        for idx, (start_text, end_text) in enumerate(zip(starts, ends), start=1):
            instruction = first_dataset_text(f, f"{base}/{start_text}-{end_text}/text") or f"stage {idx}"
            try:
                start = float(start_text)
                end = float(end_text)
            except ValueError:
                start_frame = 0
                end_frame = max(int(timestamps.size) - 1, 0)
            else:
                start_frame = int(np.searchsorted(timestamps, start, side="left"))
                end_frame = int(np.searchsorted(timestamps, end, side="right")) - 1
            segments.append(
                StageSegment(
                    instruction=instruction,
                    start_frame_index=max(start_frame, 0),
                    end_frame_index=max(end_frame, 0),
                )
            )
    return segments


def discover_hdf5_records(hdf5_root: Path | None) -> list[EpisodeRecord]:
    if hdf5_root is None:
        return []
    root = hdf5_root.expanduser().resolve()
    if not root.exists():
        return []
    if root.is_file() and root.suffix.lower() in {".h5", ".hdf5"}:
        h5_paths = [root]
        dataset_name = root.stem
    else:
        h5_paths = sorted(root.glob("episode*/episode*.hdf5"), key=natural_key)
        if not h5_paths:
            h5_paths = sorted(root.rglob("episode*.hdf5"), key=natural_key)
        dataset_name = root.parent.name if root.name == "aloha" else root.name

    records: list[EpisodeRecord] = []
    for h5_path in h5_paths:
        ep = episode_index_from_name(h5_path.stem)
        if ep is None:
            continue
        try:
            with h5py.File(h5_path, "r") as f:
                size = int(f["size"][()]) if "size" in f else 0
                timestamps = (
                    np.asarray(f["timestamp"][()], dtype=np.float64).reshape(-1)
                    if "timestamp" in f
                    else np.arange(size, dtype=np.float64) / 30.0
                )
                size = size or int(timestamps.size)
                fps = infer_fps(timestamps)
                task = first_dataset_text(f, "instructions/full_instructions/text")
                cameras = hdf5_camera_names(f)
            signals = hdf5_signal_data(h5_path)
            segments = hdf5_stage_segments(h5_path, signals.timestamps)
        except Exception as exc:
            print(f"[visualizer] skip invalid hdf5 {h5_path}: {exc}", file=sys.stderr)
            continue
        records.append(
            EpisodeRecord(
                format="hdf5",
                dataset_name=f"{dataset_name}/hdf5",
                dataset_path=str(root),
                episode_index=ep,
                frame_count=size,
                fps=fps,
                task=task or dataset_name,
                cameras=cameras,
                stage_segments=segments,
                hdf5_path=str(h5_path),
            )
        )
    return records


def find_lerobot_roots(root: Path) -> list[Path]:
    if not root.exists():
        return []
    if (root / "meta" / "info.json").is_file() and (root / "data").exists():
        return [root.resolve()]
    return sorted(
        (path.parent.parent.resolve() for path in root.rglob("meta/info.json") if (path.parent.parent / "data").exists()),
        key=lambda p: str(p),
    )


def lerobot_camera_dirs(dataset_root: Path, info: dict[str, Any]) -> list[str]:
    names: list[str] = []
    for key, spec in info.get("features", {}).items():
        if not isinstance(spec, dict):
            continue
        if str(spec.get("dtype", "")).lower() == "video":
            names.append(str(key))
    if not names:
        for child in sorted((dataset_root / "videos" / "chunk-000").glob("*")):
            if child.is_dir() and any(child.glob("episode_*.mp4")):
                names.append(child.name)

    def order_key(name: str) -> tuple[int, str]:
        short = name.split(".")[-1]
        if short in CAMERA_ORDER:
            return CAMERA_ORDER.index(short), short
        return len(CAMERA_ORDER), short

    return sorted(names, key=order_key)


def format_lerobot_path(pattern: str, episode_index: int, info: dict[str, Any], video_key: str = "") -> str:
    chunk_size = int(info.get("chunks_size", 1000) or 1000)
    return pattern.format(
        episode_chunk=episode_index // chunk_size,
        episode_index=episode_index,
        video_key=video_key,
    )


def discover_lerobot_records(lerobot_root: Path | None) -> list[EpisodeRecord]:
    if lerobot_root is None:
        return []
    root = lerobot_root.expanduser().resolve()
    records: list[EpisodeRecord] = []
    for dataset_root in find_lerobot_roots(root):
        info = read_json(dataset_root / "meta" / "info.json")
        tasks_rows = read_jsonl(dataset_root / "meta" / "tasks.jsonl")
        default_task = str(tasks_rows[0].get("task", dataset_root.name)) if tasks_rows else dataset_root.name
        episodes = read_jsonl(dataset_root / "meta" / "episodes.jsonl")
        if not episodes:
            for parquet in sorted((dataset_root / "data").glob("chunk-*/episode_*.parquet"), key=natural_key):
                ep = episode_index_from_name(parquet.stem)
                if ep is not None:
                    episodes.append({"episode_index": ep, "tasks": [default_task], "length": 0})
        cameras = lerobot_camera_dirs(dataset_root, info)
        fps = float(info.get("fps") or 30.0)
        data_pattern = str(info.get("data_path") or "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet")
        video_pattern = str(info.get("video_path") or "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4")
        for row in episodes:
            try:
                ep = int(row.get("episode_index"))
            except (TypeError, ValueError):
                continue
            parquet = dataset_root / format_lerobot_path(data_pattern, ep, info)
            if not parquet.is_file():
                matches = sorted(dataset_root.glob(f"data/**/episode_{ep:06d}.parquet"))
                parquet = matches[0] if matches else parquet
            if not parquet.is_file():
                continue
            video_paths: dict[str, str] = {}
            short_cameras: list[str] = []
            for key in cameras:
                video = dataset_root / format_lerobot_path(video_pattern, ep, info, key)
                if video.is_file():
                    short = key.split(".")[-1]
                    video_paths[short] = str(video)
                    short_cameras.append(short)
            length = int(row.get("length") or 0)
            if length <= 0 and pq is not None:
                try:
                    length = int(pq.read_table(parquet, columns=[]).num_rows)
                except Exception:
                    length = 0
            tasks = row.get("tasks")
            task = str(tasks[0]) if isinstance(tasks, list) and tasks else default_task
            records.append(
                EpisodeRecord(
                    format="lerobot",
                    dataset_name=f"{dataset_root.name}/lerobot",
                    dataset_path=str(dataset_root),
                    episode_index=ep,
                    frame_count=length,
                    fps=fps,
                    task=task,
                    cameras=short_cameras,
                    stage_segments=[],
                    parquet_path=str(parquet),
                    video_paths=video_paths,
                )
            )
    return records


def resolve_roots_for_path(path: Path, data_format: str) -> tuple[Path | None, Path | None]:
    """Resolve a user-entered path into HDF5 and/or LeRobot scan roots."""
    root = path.expanduser().resolve()
    if data_format == "hdf5":
        return root, None
    if data_format == "lerobot":
        return None, root

    hdf5_root: Path | None = None
    lerobot_root: Path | None = None
    if root.is_file() and root.suffix.lower() in {".h5", ".hdf5"}:
        hdf5_root = root
    elif root.is_dir():
        if (root / "meta" / "info.json").is_file() and (root / "data").exists():
            lerobot_root = root
        if any(root.glob("episode*/episode*.hdf5")) or any(root.glob("episode*.hdf5")):
            hdf5_root = root
        if (root / "aloha").is_dir():
            hdf5_root = root / "aloha"
        if (root / "lerobot" / root.name).is_dir():
            lerobot_root = root / "lerobot" / root.name
        elif (root / "lerobot").is_dir():
            lerobot_root = root / "lerobot"
    return hdf5_root, lerobot_root


def discover_records_from_roots(hdf5_root: Path | None, lerobot_root: Path | None) -> list[EpisodeRecord]:
    records = discover_hdf5_records(hdf5_root) + discover_lerobot_records(lerobot_root)
    records.sort(key=lambda item: (item.format, item.dataset_name, item.episode_index))
    return records


class Hdf5Cache:
    def __init__(self, records: list[EpisodeRecord], max_items: int = 8) -> None:
        self.records = records
        self.max_items = max_items
        self.lock = threading.Lock()
        self.items: OrderedDict[int, Hdf5EpisodeData] = OrderedDict()

    def get(self, index: int) -> Hdf5EpisodeData:
        with self.lock:
            if index in self.items:
                self.items.move_to_end(index)
                return self.items[index]
        record = self.records[index]
        data = Hdf5EpisodeData(
            camera_paths=hdf5_camera_paths(Path(record.hdf5_path)),
            episode_dir=Path(record.hdf5_path).parent,
            signals=hdf5_signal_data(Path(record.hdf5_path)),
        )
        with self.lock:
            self.items[index] = data
            self.items.move_to_end(index)
            while len(self.items) > self.max_items:
                self.items.popitem(last=False)
        return data

    def clear(self) -> None:
        with self.lock:
            self.items.clear()


class LeRobotSignalCache:
    def __init__(self, records: list[EpisodeRecord], max_items: int = 8) -> None:
        self.records = records
        self.max_items = max_items
        self.lock = threading.Lock()
        self.items: OrderedDict[int, SignalData] = OrderedDict()

    def get(self, index: int) -> SignalData:
        if pq is None:
            raise RuntimeError("pyarrow is required to read LeRobot parquet files.")
        with self.lock:
            if index in self.items:
                self.items.move_to_end(index)
                return self.items[index]
        record = self.records[index]
        table = pq.read_table(record.parquet_path)
        columns = table.column_names
        state_col = "observation.state" if "observation.state" in columns else next((c for c in columns if c.endswith(".state")), "")
        action_col = "action" if "action" in columns else next((c for c in columns if "action" in c), "")
        state = parquet_vector_column(table[state_col]) if state_col else None
        action = parquet_vector_column(table[action_col]) if action_col else None
        timestamps = (
            np.asarray(table["timestamp"].to_pylist(), dtype=np.float64)
            if "timestamp" in columns
            else np.arange(table.num_rows, dtype=np.float64) / max(record.fps, 1.0)
        )
        frame_indices = (
            np.asarray(table["frame_index"].to_pylist(), dtype=np.int64)
            if "frame_index" in columns
            else np.arange(table.num_rows, dtype=np.int64)
        )
        info = read_json(Path(record.dataset_path) / "meta" / "info.json")
        features = info.get("features", {}) if isinstance(info, dict) else {}
        state_labels = flatten_names(features.get(state_col, {}).get("names") if state_col else None, state.shape[1] if state is not None else 0, "state")
        action_labels = flatten_names(features.get(action_col, {}).get("names") if action_col else None, action.shape[1] if action is not None else 0, "action")
        data = SignalData(
            state=state,
            action=action,
            timestamps=timestamps,
            frame_indices=frame_indices,
            state_labels=state_labels,
            action_labels=action_labels,
            state_column=state_col,
            action_column=action_col,
        )
        with self.lock:
            self.items[index] = data
            self.items.move_to_end(index)
            while len(self.items) > self.max_items:
                self.items.popitem(last=False)
        return data

    def clear(self) -> None:
        with self.lock:
            self.items.clear()


class VideoCache:
    def __init__(self, max_items: int = 12) -> None:
        self.max_items = max_items
        self.lock = threading.Lock()
        self.items: OrderedDict[str, VideoEntry] = OrderedDict()

    def close(self) -> None:
        with self.lock:
            for item in self.items.values():
                item.cap.release()
            self.items.clear()

    def read(self, path: Path, frame_index: int) -> np.ndarray:
        key = str(path)
        with self.lock:
            entry = self.items.get(key)
            if entry is None:
                cap = cv2.VideoCapture(str(path))
                if not cap.isOpened():
                    raise RuntimeError(f"Cannot open video: {path}")
                entry = VideoEntry(path=path, cap=cap)
                self.items[key] = entry
            self.items.move_to_end(key)
            while len(self.items) > self.max_items:
                _, old = self.items.popitem(last=False)
                old.cap.release()

            if entry.last_frame is not None and entry.last_frame_index == frame_index:
                return entry.last_frame.copy()
            if frame_index != entry.last_frame_index + 1:
                entry.cap.set(cv2.CAP_PROP_POS_FRAMES, max(frame_index, 0))
            ok, frame = entry.cap.read()
            if not ok or frame is None:
                entry.cap.set(cv2.CAP_PROP_POS_FRAMES, max(frame_index, 0))
                ok, frame = entry.cap.read()
            if not ok or frame is None:
                raise EOFError(f"Frame {frame_index} not found in {path}")
            entry.last_frame_index = frame_index
            entry.last_frame = frame
            return frame.copy()


def parquet_vector_column(column: Any) -> np.ndarray:
    rows = column.to_pylist()
    if not rows:
        return np.zeros((0, 0), dtype=np.float64)
    width = max(len(item) if isinstance(item, list) else 1 for item in rows)
    out = np.full((len(rows), width), np.nan, dtype=np.float64)
    for idx, item in enumerate(rows):
        values = item if isinstance(item, list) else [item]
        for j, value in enumerate(values):
            if value is not None:
                out[idx, j] = float(value)
    return out


def clamp_frame(record: EpisodeRecord, frame_index: int) -> int:
    return max(0, min(frame_index, max(record.frame_count - 1, 0)))


def row_for_frame(signals: SignalData, frame_index: int) -> int:
    if signals.frame_indices.size == 0:
        return 0
    idx = int(np.searchsorted(signals.frame_indices, frame_index, side="left"))
    if idx >= signals.frame_indices.size:
        idx = signals.frame_indices.size - 1
    if idx > 0 and abs(signals.frame_indices[idx - 1] - frame_index) <= abs(signals.frame_indices[idx] - frame_index):
        idx -= 1
    return max(0, idx)


def vector_at(array: np.ndarray | None, row: int) -> list[float | None]:
    if array is None or array.size == 0:
        return []
    row = max(0, min(row, array.shape[0] - 1))
    values = []
    for item in array[row].tolist():
        try:
            val = float(item)
        except (TypeError, ValueError):
            values.append(None)
            continue
        values.append(round(val, 6) if np.isfinite(val) else None)
    return values


def vector_delta(array: np.ndarray | None, row: int) -> list[float | None]:
    if array is None or array.size == 0:
        return []
    row = max(0, min(row, array.shape[0] - 1))
    prev = max(row - 1, 0)
    return [
        round(float(item), 6) if np.isfinite(item) else None
        for item in (array[row] - array[prev]).tolist()
    ]


def sample_series(array: np.ndarray | None, max_points: int = 220) -> list[list[float | None]]:
    if array is None or array.size == 0:
        return []
    rows = array.shape[0]
    if rows <= max_points:
        indices = np.arange(rows)
    else:
        indices = np.unique(np.linspace(0, rows - 1, max_points, dtype=np.int64))
    sampled = array[indices]
    out: list[list[float | None]] = []
    for row in sampled:
        out.append([round(float(v), 6) if np.isfinite(v) else None for v in row.tolist()])
    return out


def sample_indices(length: int, max_points: int = 220) -> list[int]:
    if length <= 0:
        return []
    if length <= max_points:
        return list(range(length))
    return [int(x) for x in np.unique(np.linspace(0, length - 1, max_points, dtype=np.int64)).tolist()]


def resize_and_encode(frame: np.ndarray, width: int, quality: int) -> bytes:
    width = max(width, 64)
    if frame.shape[1] != width:
        height = max(1, int(frame.shape[0] * (width / frame.shape[1])))
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), min(max(quality, 30), 95)])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return encoded.tobytes()


def current_stage(record: EpisodeRecord, frame_index: int) -> dict[str, Any] | None:
    for idx, seg in enumerate(record.stage_segments):
        if seg.start_frame_index <= frame_index <= seg.end_frame_index:
            return {"index": idx + 1, **asdict(seg)}
    return None


def get_local_ip() -> str | None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return str(sock.getsockname()[0])
    except OSError:
        return None
    finally:
        sock.close()


def build_index_html() -> str:
    return r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ALOHA 数据可视化</title>
  <style>
    :root {
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: #f3f6f8;
      color: #17202a;
    }
    * { box-sizing: border-box; }
    body { margin: 0; background: #f3f6f8; }
    header {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 12px;
      align-items: center;
      padding: 12px 16px;
      border-bottom: 1px solid #d7dee8;
      background: #fff;
      position: sticky;
      top: 0;
      z-index: 2;
    }
    h1 { margin: 0; font-size: 18px; line-height: 1.3; letter-spacing: 0; }
    .status { font-size: 12px; color: #64748b; font-weight: 700; }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1.25fr) minmax(360px, 0.75fr);
      gap: 10px;
      padding: 10px;
      min-height: calc(100vh - 58px);
    }
    .panel {
      border: 1px solid #d7dee8;
      border-radius: 8px;
      background: #fff;
      overflow: hidden;
      min-width: 0;
    }
    .panel-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      min-height: 38px;
      padding: 0 12px;
      border-bottom: 1px solid #e1e7ef;
      background: #f8fafc;
      font-size: 13px;
      font-weight: 800;
      color: #334155;
    }
    .panel-body { padding: 12px; }
    .controls {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      margin-bottom: 10px;
    }
    .path-loader {
      display: grid;
      grid-template-columns: 140px minmax(0, 1fr) auto;
      gap: 8px;
      align-items: end;
      margin-bottom: 10px;
      padding: 10px;
      border: 1px solid #e1e7ef;
      border-radius: 8px;
      background: #f8fafc;
    }
    label { display: grid; gap: 4px; font-size: 12px; font-weight: 750; color: #475569; }
    select, input, button {
      min-height: 34px;
      border: 1px solid #c7d0dc;
      border-radius: 6px;
      padding: 0 9px;
      background: #fff;
      color: #1f2937;
      font-size: 13px;
      letter-spacing: 0;
    }
    button { cursor: pointer; font-weight: 800; }
    button.primary { background: #14532d; border-color: #14532d; color: #fff; }
    .toolbar {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
      margin-bottom: 10px;
    }
    .progress {
      display: grid;
      grid-template-columns: 60px 1fr 72px;
      gap: 8px;
      align-items: center;
      margin-bottom: 10px;
      font-size: 12px;
      color: #64748b;
      font-weight: 750;
    }
    input[type="range"] { width: 100%; accent-color: #14532d; }
    .camera-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }
    .camera-grid.single { grid-template-columns: minmax(0, 1fr); }
    .camera-card {
      border: 1px solid #d7dee8;
      border-radius: 8px;
      overflow: hidden;
      background: #0a0f16;
      min-width: 0;
    }
    .camera-title {
      min-height: 28px;
      padding: 6px 9px;
      background: #111827;
      color: #e5e7eb;
      font-size: 12px;
      font-weight: 800;
    }
    .camera-card img,
    .camera-card video {
      display: block;
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #000;
    }
    .meta-grid {
      display: grid;
      grid-template-columns: 110px 1fr;
      gap: 7px 10px;
      font-size: 13px;
      word-break: break-word;
    }
    .meta-grid .key { color: #64748b; font-weight: 800; }
    .task {
      margin-top: 10px;
      padding: 10px;
      border-radius: 8px;
      background: #f8fafc;
      border: 1px solid #e1e7ef;
      font-size: 13px;
      line-height: 1.5;
    }
    .signal-grid {
      display: grid;
      gap: 10px;
    }
    canvas {
      width: 100%;
      height: 130px;
      display: block;
      background: #0b1220;
      border-radius: 6px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 11px;
      font-variant-numeric: tabular-nums;
    }
    th, td {
      padding: 4px 6px;
      border-bottom: 1px solid #e5eaf1;
      text-align: right;
      white-space: nowrap;
    }
    th:first-child, td:first-child { text-align: left; color: #64748b; }
    .table-wrap { max-height: 260px; overflow: auto; border: 1px solid #e1e7ef; border-radius: 6px; }
    .stage {
      margin-top: 10px;
      padding: 8px 10px;
      border-radius: 6px;
      background: #ecfdf5;
      color: #14532d;
      font-size: 12px;
      font-weight: 800;
      line-height: 1.4;
    }
    @media (max-width: 980px) {
      main { grid-template-columns: 1fr; }
      .controls { grid-template-columns: 1fr 1fr; }
      .path-loader { grid-template-columns: 1fr; }
      .camera-grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <header>
    <h1>ALOHA 数据可视化</h1>
    <div id="status" class="status">加载中</div>
  </header>
  <main>
    <section class="panel">
      <div class="panel-head"><span>图像回放</span><span id="recordCount">0 / 0</span></div>
      <div class="panel-body">
        <div class="path-loader">
          <label>加载格式
            <select id="loadFormatSelect">
              <option value="auto">自动识别</option>
              <option value="hdf5">HDF5</option>
              <option value="lerobot">LeRobot</option>
            </select>
          </label>
          <label>数据路径
            <input id="pathInput" type="text" placeholder="/home/agilex/data/<dataset> 或 .../aloha 或 .../lerobot/<dataset>">
          </label>
          <button id="loadPathBtn" class="primary">加载路径</button>
        </div>
        <div class="controls">
          <label>格式<select id="formatSelect"></select></label>
          <label>数据集<select id="datasetSelect"></select></label>
          <label>Episode<select id="episodeSelect"></select></label>
          <label>视角<select id="viewSelect"><option value="all">全部视角</option><option value="first">仅第一视角</option></select></label>
        </div>
        <div class="toolbar">
          <button id="prevBtn">上一条</button>
          <button id="playBtn" class="primary">播放</button>
          <button id="nextBtn">下一条</button>
          <button id="reloadBtn">刷新列表</button>
          <label>刷新 FPS<input id="fpsInput" type="number" min="1" max="30" step="1" value="6"></label>
          <label>步长<input id="stepInput" type="number" min="1" max="120" step="1" value="4"></label>
        </div>
        <div class="progress">
          <span id="frameLeft">0</span>
          <input id="frameSlider" type="range" min="0" max="0" value="0">
          <span id="frameRight">0</span>
        </div>
        <div id="cameraGrid" class="camera-grid"></div>
      </div>
    </section>
    <aside class="panel">
      <div class="panel-head"><span>状态与动作</span><span id="signalSummary">-</span></div>
      <div class="panel-body signal-grid">
        <div class="meta-grid">
          <div class="key">格式</div><div id="metaFormat">-</div>
          <div class="key">数据集</div><div id="metaDataset">-</div>
          <div class="key">Episode</div><div id="metaEpisode">-</div>
          <div class="key">帧率</div><div id="metaFps">-</div>
          <div class="key">路径</div><div id="metaPath">-</div>
        </div>
        <div class="task" id="taskText">-</div>
        <div class="stage" id="stageText" style="display:none"></div>
        <div>
          <div class="panel-head" style="border:1px solid #e1e7ef;border-radius:6px 6px 0 0">state 曲线</div>
          <canvas id="stateChart"></canvas>
        </div>
        <div>
          <div class="panel-head" style="border:1px solid #e1e7ef;border-radius:6px 6px 0 0">action 曲线</div>
          <canvas id="actionChart"></canvas>
        </div>
        <div class="table-wrap">
          <table><thead><tr><th>state</th><th>value</th><th>delta</th></tr></thead><tbody id="stateRows"></tbody></table>
        </div>
        <div class="table-wrap">
          <table><thead><tr><th>action</th><th>value</th><th>delta</th></tr></thead><tbody id="actionRows"></tbody></table>
        </div>
      </div>
    </aside>
  </main>
<script>
const state = {
  records: [],
  index: 0,
  frame: 0,
  playing: false,
  timer: null,
  fps: 6,
  step: 4,
  series: null,
  frameToken: 0,
  renderToken: 0,
  signalTimer: null,
  lastSignalFrame: -1,
  lastSignalMs: 0,
  pendingSignal: false,
};
const $ = (id) => document.getElementById(id);
const fmtSel = $("formatSelect");
const dsSel = $("datasetSelect");
const epSel = $("episodeSelect");
const viewSel = $("viewSelect");
const cameraGrid = $("cameraGrid");
const slider = $("frameSlider");
const pathInput = $("pathInput");
const loadFormatSelect = $("loadFormatSelect");
const colors = ["#38bdf8", "#22c55e", "#f97316", "#e879f9", "#f43f5e", "#eab308", "#a78bfa", "#14b8a6", "#fb7185", "#84cc16", "#60a5fa", "#f59e0b"];

function current() { return state.records[state.index]; }
function escapeHtml(text) {
  return String(text ?? "").replace(/[&<>"']/g, (ch) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));
}
function fmtNum(value) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "-";
  const n = Number(value);
  if (Math.abs(n) >= 1000 || (Math.abs(n) > 0 && Math.abs(n) < 0.001)) return n.toExponential(3);
  return n.toFixed(5);
}
function filteredRecords(format, dataset) {
  return state.records.filter((r) => (!format || r.format === format) && (!dataset || r.dataset_name === dataset));
}
function unique(values) { return [...new Set(values)]; }
function clearRecordUi(message) {
  state.records = [];
  state.index = 0;
  state.frame = 0;
  state.series = null;
  fmtSel.innerHTML = "";
  dsSel.innerHTML = "";
  epSel.innerHTML = "";
  cameraGrid.innerHTML = "";
  cameraGrid.classList.add("single");
  slider.max = "0";
  slider.value = "0";
  $("frameLeft").textContent = "0";
  $("frameRight").textContent = "0";
  $("recordCount").textContent = "0 / 0";
  $("signalSummary").textContent = "-";
  $("metaFormat").textContent = "-";
  $("metaDataset").textContent = "-";
  $("metaEpisode").textContent = "-";
  $("metaFps").textContent = "-";
  $("metaPath").textContent = "-";
  $("taskText").textContent = "-";
  $("stageText").style.display = "none";
  renderRows("stateRows", [], [], []);
  renderRows("actionRows", [], [], []);
  drawCharts();
  $("status").textContent = message;
}
function rebuildSelectors(keepRecord=true, wantedFormat=null, wantedDataset=null) {
  const old = keepRecord && current() ? current() : null;
  const formats = unique(state.records.map((r) => r.format));
  fmtSel.innerHTML = formats.map((f) => `<option value="${f}">${f.toUpperCase()}</option>`).join("");
  const formatValue = wantedFormat || (old ? old.format : fmtSel.value);
  if (formats.includes(formatValue)) fmtSel.value = formatValue;
  const datasets = unique(state.records.filter((r) => !fmtSel.value || r.format === fmtSel.value).map((r) => r.dataset_name));
  dsSel.innerHTML = datasets.map((d) => `<option value="${escapeHtml(d)}">${escapeHtml(d)}</option>`).join("");
  const datasetValue = wantedDataset || (old ? old.dataset_name : dsSel.value);
  if (datasets.includes(datasetValue)) dsSel.value = datasetValue;
  const episodes = filteredRecords(fmtSel.value, dsSel.value);
  epSel.innerHTML = episodes.map((r) => `<option value="${r.index}">episode${r.episode_index}</option>`).join("");
  if (old) {
    const same = episodes.find((r) => r.format === old.format && r.dataset_name === old.dataset_name && r.episode_index === old.episode_index);
    if (same) state.index = same.index;
    else if (episodes[0]) state.index = episodes[0].index;
  } else if (episodes[0]) {
    state.index = episodes[0].index;
  }
  epSel.value = String(state.index);
}
function syncSelectorsFromCurrent() {
  const r = current();
  if (!r) return;
  fmtSel.value = r.format;
  const datasets = unique(state.records.filter((item) => item.format === r.format).map((item) => item.dataset_name));
  dsSel.innerHTML = datasets.map((d) => `<option value="${escapeHtml(d)}">${escapeHtml(d)}</option>`).join("");
  dsSel.value = r.dataset_name;
  const episodes = filteredRecords(r.format, r.dataset_name);
  epSel.innerHTML = episodes.map((item) => `<option value="${item.index}">episode${item.episode_index}</option>`).join("");
  epSel.value = String(state.index);
}
async function applyRecordsPayload(data, statusPrefix="已加载") {
  state.records = data.records || [];
  if (data.lerobot_root && loadFormatSelect.value === "lerobot") pathInput.value = data.lerobot_root;
  if (data.hdf5_root && loadFormatSelect.value === "hdf5") pathInput.value = data.hdf5_root;
  if (!state.records.length) {
    clearRecordUi("没有找到 HDF5 或 LeRobot episode");
    return;
  }
  rebuildSelectors(false);
  if (data.initial_index >= 0 && data.initial_index < state.records.length) state.index = data.initial_index;
  syncSelectorsFromCurrent();
  await loadEpisode(true);
  $("status").textContent = `${statusPrefix} ${state.records.length} 条 episode`;
}
async function loadRecords() {
  setPlaying(false);
  $("status").textContent = "加载数据列表";
  try {
    const res = await fetch("api/episodes", {cache:"no-store"});
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      throw new Error(data.error || `HTTP ${res.status}`);
    }
    await applyRecordsPayload(data, "已加载");
  } catch (err) {
    clearRecordUi("加载失败: " + (err && err.message ? err.message : err));
  }
}
async function loadPathFromInput() {
  const path = pathInput.value.trim();
  if (!path) {
    $("status").textContent = "请输入数据路径";
    return;
  }
  setPlaying(false);
  $("status").textContent = "正在扫描路径...";
  try {
    const res = await fetch("api/load", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({path, format: loadFormatSelect.value || "auto"}),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || !data.ok) {
      throw new Error(data.error || `加载失败 HTTP ${res.status}`);
    }
    await applyRecordsPayload(data, "已切换到");
  } catch (err) {
    clearRecordUi("加载失败: " + (err && err.message ? err.message : err));
  }
}
function renderCameras() {
  const r = current();
  if (!r) {
    cameraGrid.innerHTML = "";
    cameraGrid.classList.add("single");
    return;
  }
  const cameras = (viewSel.value === "first") ? (r.cameras || []).slice(0, 1) : (r.cameras || []);
  cameraGrid.classList.toggle("single", cameras.length <= 1);
  cameraGrid.innerHTML = cameras.map((cam) => `
    <div class="camera-card">
      <div class="camera-title">${escapeHtml(cam)}</div>
      ${r.format === "lerobot"
        ? `<video id="cam-${escapeHtml(cam)}" muted playsinline controls preload="metadata"></video>`
        : `<img id="cam-${escapeHtml(cam)}" alt="${escapeHtml(cam)}">`}
    </div>
  `).join("");
}
function currentCameraElements() {
  const r = current();
  if (!r) return [];
  const cameras = (viewSel.value === "first") ? (r.cameras || []).slice(0, 1) : (r.cameras || []);
  return cameras.map((cam) => [cam, document.getElementById(`cam-${cam}`)]).filter((item) => item[1]);
}
function syncVideoSources() {
  const r = current();
  if (!r || r.format !== "lerobot") return;
  for (const [cam, el] of currentCameraElements()) {
    const src = `api/video?index=${state.index}&camera=${encodeURIComponent(cam)}`;
    if (el.dataset.src !== src) {
      el.dataset.src = src;
      el.src = src;
      el.load();
    }
  }
}
function seekVideosToFrame() {
  const r = current();
  if (!r || r.format !== "lerobot") return;
  const t = state.frame / Math.max(Number(r.fps || 30), 1);
  for (const [, el] of currentCameraElements()) {
    try {
      if (!Number.isFinite(el.duration) || Math.abs((el.currentTime || 0) - t) > 0.08) {
        el.currentTime = t;
      }
    } catch (_) {}
  }
}
function masterVideo() {
  const items = currentCameraElements();
  return items.length ? items[0][1] : null;
}
async function playVideos() {
  syncVideoSources();
  seekVideosToFrame();
  const plays = currentCameraElements().map(([, el]) => el.play().catch(() => null));
  await Promise.allSettled(plays);
}
function pauseVideos() {
  for (const [, el] of currentCameraElements()) {
    if (el && typeof el.pause === "function") el.pause();
  }
}
async function loadSeries() {
  state.series = null;
  const res = await fetch(`api/series?index=${state.index}`, {cache:"no-store"});
  const data = await res.json();
  if (data.ok) state.series = data;
  drawCharts();
}
async function loadEpisode(resetFrame) {
  const r = current();
  if (!r) return;
  if (resetFrame) state.frame = 0;
  state.frame = Math.max(0, Math.min(state.frame, Math.max((r.frame_count || 1) - 1, 0)));
  syncSelectorsFromCurrent();
  renderCameras();
  $("metaFormat").textContent = r.format;
  $("metaDataset").textContent = r.dataset_name;
  $("metaEpisode").textContent = `episode${r.episode_index}`;
  $("metaFps").textContent = `${r.fps.toFixed ? r.fps.toFixed(2) : r.fps} Hz`;
  $("metaPath").textContent = r.hdf5_path || r.dataset_path;
  $("taskText").textContent = r.task || "-";
  $("recordCount").textContent = `${state.index + 1} / ${state.records.length}`;
  await loadSeries();
  await renderFrame();
}
async function renderFrame() {
  const r = current();
  if (!r) return;
  const renderToken = ++state.renderToken;
  state.frame = Math.max(0, Math.min(state.frame, Math.max((r.frame_count || 1) - 1, 0)));
  $("frameLeft").textContent = String(state.frame);
  $("frameRight").textContent = String(Math.max((r.frame_count || 1) - 1, 0));
  slider.max = String(Math.max((r.frame_count || 1) - 1, 0));
  slider.value = String(state.frame);
  const cameras = (viewSel.value === "first") ? (r.cameras || []).slice(0, 1) : (r.cameras || []);
  if (r.format === "lerobot") {
    syncVideoSources();
    if (!state.playing) seekVideosToFrame();
  } else {
    for (const cam of cameras) {
      const img = document.getElementById(`cam-${cam}`);
      if (img) img.src = `api/image?index=${state.index}&camera=${encodeURIComponent(cam)}&frame=${state.frame}&t=${Date.now()}`;
    }
  }
  if (renderToken !== state.renderToken) return;
  if (state.playing) {
    refreshFrameSignal(false);
  } else {
    await refreshFrameSignal(true);
  }
}
async function refreshFrameSignal(force=false) {
  if (!current()) return;
  const now = performance.now();
  if (!force && state.playing && now - state.lastSignalMs < 500) return;
  if (!force && state.pendingSignal) return;
  state.pendingSignal = true;
  state.lastSignalMs = now;
  const token = ++state.frameToken;
  try {
    const res = await fetch(`api/frame_signal?index=${state.index}&frame=${state.frame}`, {cache:"no-store"});
    const data = await res.json().catch(() => ({}));
    if (token !== state.frameToken) return;
    if (!res.ok || !data.ok) return;
    $("signalSummary").textContent = `state ${data.state_values.length}D / action ${data.action_values.length}D`;
    renderRows("stateRows", data.state_labels, data.state_values, data.state_delta);
    renderRows("actionRows", data.action_labels, data.action_values, data.action_delta);
    if (data.stage) {
      $("stageText").style.display = "block";
      $("stageText").textContent = `阶段 ${data.stage.index}: ${data.stage.instruction}`;
    } else {
      $("stageText").style.display = "none";
    }
    if (force || !state.playing) drawCharts();
  } finally {
    if (token === state.frameToken) {
      state.pendingSignal = false;
      state.lastSignalFrame = state.frame;
    }
  }
}
function renderRows(id, labels, values, deltas) {
  const rows = [];
  for (let i = 0; i < Math.max(values.length, labels.length); i++) {
    rows.push(`<tr><td>${escapeHtml(labels[i] || i)}</td><td>${fmtNum(values[i])}</td><td>${fmtNum(deltas[i])}</td></tr>`);
  }
  $(id).innerHTML = rows.length ? rows.join("") : `<tr><td colspan="3">无数据</td></tr>`;
}
function drawOne(canvas, series, dims) {
  const ctx = canvas.getContext("2d");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const w = canvas.width, h = canvas.height;
  ctx.fillStyle = "#0b1220";
  ctx.fillRect(0, 0, w, h);
  if (!series || !series.length || !dims) return;
  const pad = 18 * dpr;
  const plotW = w - pad * 2;
  const plotH = h - pad * 2;
  ctx.strokeStyle = "#263244";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad + plotH * i / 4;
    ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(w - pad, y); ctx.stroke();
  }
  const maxDims = Math.min(dims, 24);
  for (let d = 0; d < maxDims; d++) {
    let vals = series.map((row) => row[d]).filter((v) => v !== null && Number.isFinite(Number(v))).map(Number);
    if (!vals.length) continue;
    let mn = Math.min(...vals), mx = Math.max(...vals);
    if (Math.abs(mx - mn) < 1e-9) mx = mn + 1;
    ctx.strokeStyle = colors[d % colors.length];
    ctx.globalAlpha = maxDims > 16 ? 0.65 : 0.85;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < series.length; i++) {
      const raw = series[i][d];
      if (raw === null || !Number.isFinite(Number(raw))) continue;
      const x = pad + plotW * i / Math.max(series.length - 1, 1);
      const y = pad + plotH * (1 - (Number(raw) - mn) / (mx - mn));
      if (!started) { ctx.moveTo(x, y); started = true; }
      else ctx.lineTo(x, y);
    }
    if (started) ctx.stroke();
  }
  ctx.globalAlpha = 1;
  const r = current();
  const cursorX = pad + plotW * Math.min(Math.max(state.frame / Math.max(((r && r.frame_count) || 1) - 1, 1), 0), 1);
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cursorX, pad); ctx.lineTo(cursorX, h - pad); ctx.stroke();
}
function drawCharts() {
  drawOne($("stateChart"), state.series && state.series.state_series, state.series ? state.series.state_dim : 0);
  drawOne($("actionChart"), state.series && state.series.action_series, state.series ? state.series.action_dim : 0);
}
function selectByIndex(index) {
  if (index < 0 || index >= state.records.length) return;
  setPlaying(false);
  state.index = index;
  loadEpisode(true);
}
function nextRecord(delta) {
  if (!state.records.length) return;
  const next = Math.max(0, Math.min(state.index + delta, state.records.length - 1));
  selectByIndex(next);
}
function setPlaying(value) {
  if (value && !current()) value = false;
  state.playing = value;
  $("playBtn").textContent = value ? "暂停" : "播放";
  if (state.timer) clearInterval(state.timer);
  state.timer = null;
  pauseVideos();
  if (value) {
    const r = current();
    if (r && r.format === "lerobot") {
      playVideos();
      state.timer = setInterval(() => {
        const record = current();
        const master = masterVideo();
        if (!record || record.format !== "lerobot" || !master) return;
        state.frame = Math.max(0, Math.min(
          Math.round((master.currentTime || 0) * Math.max(Number(record.fps || 30), 1)),
          Math.max((record.frame_count || 1) - 1, 0)
        ));
        $("frameLeft").textContent = String(state.frame);
        $("frameRight").textContent = String(Math.max((record.frame_count || 1) - 1, 0));
        slider.max = String(Math.max((record.frame_count || 1) - 1, 0));
        slider.value = String(state.frame);
        refreshFrameSignal(false);
        if (master.ended || state.frame >= Math.max((record.frame_count || 1) - 2, 0)) {
          if (state.index < state.records.length - 1) {
            state.index += 1;
            loadEpisode(true).then(() => {
              if (state.playing) playVideos();
            });
          } else {
            setPlaying(false);
          }
        }
      }, Math.max(80, 1000 / Math.max(state.fps, 1)));
      return;
    }
    state.timer = setInterval(() => {
      const r = current();
      state.frame += state.step;
      if (state.frame >= (r.frame_count || 1)) {
        if (state.index < state.records.length - 1) {
          state.index += 1;
          loadEpisode(true);
          return;
        }
        setPlaying(false);
        return;
      }
      renderFrame();
    }, Math.max(30, 1000 / Math.max(state.fps, 1)));
  }
}
$("playBtn").onclick = () => setPlaying(!state.playing);
$("prevBtn").onclick = () => nextRecord(-1);
$("nextBtn").onclick = () => nextRecord(1);
$("reloadBtn").onclick = () => loadRecords();
fmtSel.onchange = () => { setPlaying(false); rebuildSelectors(false, fmtSel.value, null); loadEpisode(true); };
dsSel.onchange = () => { setPlaying(false); rebuildSelectors(false, fmtSel.value, dsSel.value); loadEpisode(true); };
epSel.onchange = () => selectByIndex(Number(epSel.value));
viewSel.onchange = () => { setPlaying(false); renderCameras(); renderFrame(); };
slider.oninput = () => { setPlaying(false); state.frame = Number(slider.value); renderFrame(); };
$("loadPathBtn").onclick = () => loadPathFromInput();
pathInput.addEventListener("keydown", (event) => {
  if (event.key === "Enter") {
    event.preventDefault();
    loadPathFromInput();
  }
});
$("fpsInput").onchange = () => { state.fps = Math.max(1, Number($("fpsInput").value || 10)); if (state.playing) setPlaying(true); };
$("stepInput").onchange = () => { state.step = Math.max(1, Number($("stepInput").value || 4)); };
window.addEventListener("resize", drawCharts);
document.addEventListener("keydown", (event) => {
  if (event.target && ["INPUT", "SELECT"].includes(event.target.tagName)) return;
  if (event.code === "Space") { event.preventDefault(); setPlaying(!state.playing); }
  if (event.code === "ArrowRight") { state.frame += state.step; renderFrame(); }
  if (event.code === "ArrowLeft") { state.frame -= state.step; renderFrame(); }
});
loadRecords().catch((err) => {
  $("status").textContent = "加载失败: " + err;
});
</script>
</body>
</html>"""


class VisualizerServer(ThreadingHTTPServer):
    allow_reuse_address = True


def make_handler(
    records: list[EpisodeRecord],
    hdf5_cache: Hdf5Cache,
    lerobot_cache: LeRobotSignalCache,
    video_cache: VideoCache,
    initial_index: int,
    image_width: int,
    jpeg_quality: int,
    hdf5_root: Path | None,
    lerobot_root: Path | None,
) -> type[BaseHTTPRequestHandler]:
    html = build_index_html().encode("utf-8")
    current_initial_index = {"value": initial_index}
    current_roots = {
        "hdf5_root": str(hdf5_root) if hdf5_root else "",
        "lerobot_root": str(lerobot_root) if lerobot_root else "",
    }
    records_lock = threading.Lock()

    def record_payloads() -> list[dict[str, Any]]:
        return [record.to_payload(i) for i, record in enumerate(records)]

    def replace_records(new_records: list[EpisodeRecord], new_hdf5_root: Path | None, new_lerobot_root: Path | None) -> None:
        with records_lock:
            records[:] = new_records
            current_initial_index["value"] = 0
            current_roots["hdf5_root"] = str(new_hdf5_root) if new_hdf5_root else ""
            current_roots["lerobot_root"] = str(new_lerobot_root) if new_lerobot_root else ""
            hdf5_cache.clear()
            lerobot_cache.clear()
            video_cache.close()

    class Handler(BaseHTTPRequestHandler):
        server_version = "AlohaDatasetVisualizer/1.0"

        def do_HEAD(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/api/video":
                self.serve_video(parsed.query, head_only=True)
                return
            self.send_response(404)
            self.end_headers()

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path == "/api/load":
                self.serve_load()
                return
            self.write_json({"ok": False, "error": "not found"}, 404)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            if parsed.path in {"/", "/index.html"}:
                self.write_bytes(200, html, "text/html; charset=utf-8")
                return
            if parsed.path == "/api/episodes":
                self.write_json(
                    {
                        "records": record_payloads(),
                        "initial_index": current_initial_index["value"],
                        "default_image_width": image_width,
                        "default_jpeg_quality": jpeg_quality,
                        **current_roots,
                    }
                )
                return
            if parsed.path == "/api/image":
                self.serve_hdf5_image(parsed.query)
                return
            if parsed.path == "/api/video":
                self.serve_video(parsed.query)
                return
            if parsed.path == "/api/frame":
                self.serve_frame(parsed.query)
                return
            if parsed.path == "/api/series":
                self.serve_series(parsed.query)
                return
            if parsed.path == "/api/frame_signal":
                self.serve_frame_signal(parsed.query)
                return
            if parsed.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return
            self.write_json({"ok": False, "error": "not found"}, 404)

        def parse_index(self, query: str) -> tuple[int, dict[str, list[str]]] | None:
            params = parse_qs(query)
            try:
                index = int(params.get("index", ["0"])[0])
            except ValueError:
                self.write_json({"ok": False, "error": "invalid index"}, 400)
                return None
            if index < 0 or index >= len(records):
                self.write_json({"ok": False, "error": "index out of range"}, 404)
                return None
            return index, params

        def signal_data(self, index: int) -> SignalData:
            record = records[index]
            if record.format == "hdf5":
                return hdf5_cache.get(index).signals
            return lerobot_cache.get(index)

        def read_json_body(self) -> dict[str, Any]:
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = 0
            if length <= 0:
                return {}
            raw = self.rfile.read(min(length, 1024 * 1024))
            return json.loads(raw.decode("utf-8")) if raw else {}

        def serve_load(self) -> None:
            try:
                payload = self.read_json_body()
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.write_json({"ok": False, "error": f"invalid json: {exc}"}, 400)
                return
            raw_path = str(payload.get("path") or "").strip()
            data_format = str(payload.get("format") or "auto").strip().lower()
            if data_format not in {"auto", "hdf5", "lerobot"}:
                self.write_json({"ok": False, "error": f"unknown format: {data_format}"}, 400)
                return
            if not raw_path:
                self.write_json({"ok": False, "error": "path is required"}, 400)
                return
            path = Path(raw_path).expanduser()
            hdf5_root, lerobot_root = resolve_roots_for_path(path, data_format)
            new_records = discover_records_from_roots(hdf5_root, lerobot_root)
            if not new_records:
                self.write_json(
                    {
                        "ok": False,
                        "error": f"没有在 {path} 下找到可视化的 {data_format} 数据",
                        "hdf5_root": str(hdf5_root) if hdf5_root else "",
                        "lerobot_root": str(lerobot_root) if lerobot_root else "",
                    },
                    404,
                )
                return
            replace_records(new_records, hdf5_root, lerobot_root)
            self.write_json(
                {
                    "ok": True,
                    "records": record_payloads(),
                    "initial_index": 0,
                    "hdf5_root": str(hdf5_root) if hdf5_root else "",
                    "lerobot_root": str(lerobot_root) if lerobot_root else "",
                }
            )

        def resolve_camera_image_path(self, index: int, camera: str, frame_index: int) -> Path:
            record = records[index]
            if record.format != "hdf5":
                raise FileNotFoundError("record is not HDF5")
            data = hdf5_cache.get(index)
            paths = data.camera_paths.get(camera)
            if not paths:
                raise FileNotFoundError(f"unknown camera: {camera}")
            rel = paths[min(frame_index, len(paths) - 1)]
            img_path = Path(rel)
            if not img_path.is_absolute():
                img_path = data.episode_dir / rel
            if not img_path.is_file():
                raise FileNotFoundError(str(img_path))
            return img_path

        def serve_hdf5_image(self, query: str) -> None:
            parsed = self.parse_index(query)
            if parsed is None:
                return
            index, params = parsed
            record = records[index]
            camera = params.get("camera", [""])[0]
            try:
                frame_index = int(params.get("frame", ["0"])[0])
            except ValueError:
                self.write_json({"ok": False, "error": "invalid frame"}, 400)
                return
            frame_index = clamp_frame(record, frame_index)
            try:
                path = self.resolve_camera_image_path(index, camera, frame_index)
                body = path.read_bytes()
            except Exception as exc:
                blank = np.full((480, 640, 3), 48, dtype=np.uint8)
                cv2.putText(blank, str(exc)[:80], (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                body = resize_and_encode(blank, width=image_width, quality=jpeg_quality)
                self.write_bytes(200, body, "image/jpeg", cache="no-store")
                return
            suffix = path.suffix.lower()
            content_type = "image/png" if suffix == ".png" else "image/jpeg"
            self.write_bytes(200, body, content_type, cache="public, max-age=600")

        def serve_video(self, query: str, head_only: bool = False) -> None:
            parsed = self.parse_index(query)
            if parsed is None:
                return
            index, params = parsed
            record = records[index]
            camera = params.get("camera", [""])[0]
            if record.format != "lerobot":
                self.write_json({"ok": False, "error": "record is not LeRobot"}, 400)
                return
            video_paths = record.video_paths or {}
            path_text = video_paths.get(camera)
            if not path_text:
                self.write_json({"ok": False, "error": f"unknown camera: {camera}"}, 404)
                return
            path = Path(path_text)
            if not path.is_file():
                self.write_json({"ok": False, "error": f"missing video: {path}"}, 404)
                return
            size = path.stat().st_size
            start = 0
            end = size - 1
            status = 200
            range_header = self.headers.get("Range", "")
            if range_header.startswith("bytes="):
                spec = range_header.split("=", 1)[1].split(",", 1)[0].strip()
                try:
                    if spec.startswith("-"):
                        suffix_len = int(spec[1:])
                        start = max(size - suffix_len, 0)
                    else:
                        raw_start, _, raw_end = spec.partition("-")
                        start = int(raw_start)
                        if raw_end:
                            end = min(int(raw_end), size - 1)
                    if start < 0 or start >= size or end < start:
                        self.send_response(416)
                        self.send_header("Content-Range", f"bytes */{size}")
                        self.end_headers()
                        return
                    status = 206
                except ValueError:
                    self.write_json({"ok": False, "error": "invalid range"}, 400)
                    return
            length = end - start + 1
            self.send_response(status)
            self.send_header("Content-Type", "video/mp4")
            self.send_header("Accept-Ranges", "bytes")
            self.send_header("Content-Length", str(length))
            self.send_header("Cache-Control", "public, max-age=600")
            if status == 206:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if head_only or self.command == "HEAD":
                return
            try:
                with path.open("rb") as f:
                    f.seek(start)
                    remaining = length
                    while remaining > 0:
                        chunk = f.read(min(1024 * 1024, remaining))
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        remaining -= len(chunk)
            except (BrokenPipeError, ConnectionResetError):
                return

        def serve_frame(self, query: str) -> None:
            parsed = self.parse_index(query)
            if parsed is None:
                return
            index, params = parsed
            record = records[index]
            camera = params.get("camera", [""])[0]
            try:
                frame_index = int(params.get("frame", ["0"])[0])
                width = int(params.get("width", [str(image_width)])[0])
                quality = int(params.get("quality", [str(jpeg_quality)])[0])
            except ValueError:
                self.write_json({"ok": False, "error": "invalid frame/width/quality"}, 400)
                return
            frame_index = clamp_frame(record, frame_index)
            try:
                if record.format == "hdf5":
                    data = hdf5_cache.get(index)
                    paths = data.camera_paths.get(camera)
                    if not paths:
                        raise FileNotFoundError(f"unknown camera: {camera}")
                    rel = paths[min(frame_index, len(paths) - 1)]
                    img_path = Path(rel)
                    if not img_path.is_absolute():
                        img_path = data.episode_dir / rel
                    frame = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
                    if frame is None:
                        raise FileNotFoundError(str(img_path))
                else:
                    video_paths = record.video_paths or {}
                    if camera not in video_paths:
                        raise FileNotFoundError(f"unknown camera: {camera}")
                    frame = video_cache.read(Path(video_paths[camera]), frame_index)
                body = resize_and_encode(frame, width=width, quality=quality)
            except Exception as exc:
                blank = np.full((480, 640, 3), 48, dtype=np.uint8)
                cv2.putText(blank, str(exc)[:80], (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                body = resize_and_encode(blank, width=width, quality=quality)
            self.write_bytes(200, body, "image/jpeg", cache="no-store")

        def serve_series(self, query: str) -> None:
            parsed = self.parse_index(query)
            if parsed is None:
                return
            index, _ = parsed
            try:
                signals = self.signal_data(index)
            except Exception as exc:
                self.write_json({"ok": False, "error": str(exc)}, 500)
                return
            state_dim = int(signals.state.shape[1]) if signals.state is not None else 0
            action_dim = int(signals.action.shape[1]) if signals.action is not None else 0
            self.write_json(
                {
                    "ok": True,
                    "state_dim": state_dim,
                    "action_dim": action_dim,
                    "state_labels": signals.state_labels,
                    "action_labels": signals.action_labels,
                    "state_column": signals.state_column,
                    "action_column": signals.action_column,
                    "sample_indices": sample_indices(max(signals.timestamps.size, signals.frame_indices.size)),
                    "state_series": sample_series(signals.state),
                    "action_series": sample_series(signals.action),
                }
            )

        def serve_frame_signal(self, query: str) -> None:
            parsed = self.parse_index(query)
            if parsed is None:
                return
            index, params = parsed
            record = records[index]
            try:
                frame_index = int(params.get("frame", ["0"])[0])
            except ValueError:
                self.write_json({"ok": False, "error": "invalid frame"}, 400)
                return
            try:
                signals = self.signal_data(index)
            except Exception as exc:
                self.write_json({"ok": False, "error": str(exc)}, 500)
                return
            row = row_for_frame(signals, frame_index)
            timestamp = float(signals.timestamps[row]) if row < signals.timestamps.size else None
            self.write_json(
                {
                    "ok": True,
                    "frame": frame_index,
                    "row": row,
                    "timestamp": round(timestamp, 6) if timestamp is not None else None,
                    "stage": current_stage(record, frame_index),
                    "state_labels": signals.state_labels,
                    "action_labels": signals.action_labels,
                    "state_values": vector_at(signals.state, row),
                    "action_values": vector_at(signals.action, row),
                    "state_delta": vector_delta(signals.state, row),
                    "action_delta": vector_delta(signals.action, row),
                }
            )

        def write_bytes(self, status: int, body: bytes, content_type: str, cache: str = "no-store") -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", cache)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def write_json(self, payload: dict[str, Any], status: int = 200) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.write_bytes(status, body, "application/json; charset=utf-8")

        def log_message(self, fmt: str, *args: Any) -> None:
            sys.stderr.write("[%s] %s\n" % (time.strftime("%F %T"), fmt % args))

    return Handler


def choose_initial_index(records: list[EpisodeRecord], start_format: str, start_episode: int) -> int:
    if start_format == "auto":
        for idx, record in enumerate(records):
            if record.episode_index >= start_episode:
                return idx
        return 0
    for idx, record in enumerate(records):
        if record.format == start_format and record.episode_index >= start_episode:
            return idx
    return 0


def main() -> int:
    args = parse_args()
    data_dir = args.data_dir.expanduser().resolve() if args.data_dir else None
    hdf5_root = args.hdf5_root.expanduser().resolve() if args.hdf5_root else None
    lerobot_root = args.lerobot_root.expanduser().resolve() if args.lerobot_root else None
    if data_dir and hdf5_root is None:
        hdf5_root = data_dir / "aloha"
    if data_dir and lerobot_root is None:
        preferred = data_dir / "lerobot" / data_dir.name
        lerobot_root = preferred if preferred.exists() else data_dir / "lerobot"

    records = discover_records_from_roots(hdf5_root, lerobot_root)
    if args.list:
        for record in records:
            print(f"{record.format:7s} {record.dataset_name:24s} episode{record.episode_index:<6d} frames={record.frame_count}")
        return 0
    if not records:
        print(
            f"No HDF5/LeRobot episodes found under {hdf5_root or '-'} or {lerobot_root or '-'}",
            file=sys.stderr,
        )

    initial_index = choose_initial_index(records, args.start_format, args.start_episode)
    hdf5_cache = Hdf5Cache(records)
    lerobot_cache = LeRobotSignalCache(records)
    video_cache = VideoCache()
    handler = make_handler(
        records=records,
        hdf5_cache=hdf5_cache,
        lerobot_cache=lerobot_cache,
        video_cache=video_cache,
        initial_index=initial_index,
        image_width=max(args.image_width, 64),
        jpeg_quality=min(max(args.jpeg_quality, 30), 95),
        hdf5_root=hdf5_root,
        lerobot_root=lerobot_root,
    )

    server: VisualizerServer | None = None
    last_error: Exception | None = None
    for port in [args.port] + [args.port + offset for offset in range(1, 10)]:
        try:
            server = VisualizerServer((args.host, port), handler)
            break
        except OSError as exc:
            last_error = exc
    if server is None:
        print(f"Failed to start visualizer: {last_error}", file=sys.stderr)
        return 1

    local_url = f"http://127.0.0.1:{server.server_port}/"
    print("ALOHA dataset visualizer started.")
    print(f"Local: {local_url}")
    if args.host in {"0.0.0.0", "::"}:
        ip = get_local_ip()
        if ip:
            print(f"LAN:   http://{ip}:{server.server_port}/")
    else:
        print(f"Host:  http://{args.host}:{server.server_port}/")
    print(f"HDF5 root:    {hdf5_root}")
    print(f"LeRobot root: {lerobot_root}")
    print(f"Episodes:     {len(records)}")
    if args.open_browser:
        webbrowser.open(local_url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        video_cache.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
