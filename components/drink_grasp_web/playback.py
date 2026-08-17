from __future__ import annotations

import json
import os
import threading
import time
from collections import OrderedDict
from pathlib import Path
from typing import Any

import cv2
import rosbag2_py
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, JointState


PLAYBACK_CAMERAS = {
    "cam_high": {"label": "顶部相机", "topic": "/cam_high/color/image_raw"},
    "cam_left": {"label": "左腕相机", "topic": "/cam_left/color/image_raw"},
    "cam_right": {"label": "右腕相机", "topic": "/cam_right/color/image_raw"},
}

PLAYBACK_POSE_TOPICS = {
    "left": ("/end_pose_stamped_left", "/end_pose_left"),
    "right": ("/end_pose_stamped_right", "/end_pose_right"),
}

PLAYBACK_JOINT_TOPICS = {
    "left": ("/joint_left", "/joint_states_left"),
    "right": ("/joint_right", "/joint_states_right"),
}


class EpisodePlaybackLibrary:
    """Read-only, lazily decoded rosbag data used by the browser viewer."""

    def __init__(self, target_dir: Path) -> None:
        self.target_dir = target_dir
        self.max_cached_episodes = bounded_int(
            os.environ.get("DRINK_GRASP_PLAYBACK_CACHE_EPISODES"), 2, 1, 4
        )
        self.max_trajectory_samples = bounded_int(
            os.environ.get("DRINK_GRASP_PLAYBACK_MAX_TRAJECTORY_SAMPLES"),
            1400,
            200,
            5000,
        )
        self.jpeg_quality = bounded_int(
            os.environ.get("DRINK_GRASP_PLAYBACK_JPEG_QUALITY"), 76, 45, 95
        )
        self._lock = threading.RLock()
        self._cache: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._bridge = CvBridge()

    def catalog(self) -> list[dict[str, Any]]:
        entries: list[dict[str, Any]] = []
        if not self.target_dir.exists():
            return entries
        for group_dir in sorted(self.target_dir.iterdir(), reverse=True):
            if not group_dir.is_dir() or group_dir.name.startswith("."):
                continue
            for episode_dir in sorted(group_dir.glob("episode_*"), reverse=True):
                if not episode_dir.is_dir():
                    continue
                try:
                    entries.append(self._catalog_entry(episode_dir))
                except (OSError, ValueError, yaml.YAMLError) as exc:
                    entries.append(
                        {
                            "id": f"{group_dir.name}/{episode_dir.name}",
                            "group": group_dir.name,
                            "name": episode_dir.name,
                            "path": str(episode_dir),
                            "status": "error",
                            "error": f"{exc.__class__.__name__}: {exc}",
                        }
                    )
        entries.sort(
            key=lambda item: (
                float(item.get("started_at") or 0.0),
                str(item.get("id") or ""),
            ),
            reverse=True,
        )
        return entries

    def manifest(self, episode_id: str) -> dict[str, Any]:
        return self._loaded_episode(episode_id)["manifest"]

    def frame(self, episode_id: str, camera_id: str, index: int) -> bytes:
        loaded = self._loaded_episode(episode_id)
        frames = loaded["frames"].get(camera_id)
        if frames is None:
            raise KeyError(f"camera not recorded: {camera_id}")
        if index < 0 or index >= len(frames):
            raise IndexError(f"frame index out of range: {index}")
        return frames[index]

    def _loaded_episode(self, episode_id: str) -> dict[str, Any]:
        normalized = self._normalize_episode_id(episode_id)
        with self._lock:
            cached = self._cache.pop(normalized, None)
            if cached is not None:
                self._cache[normalized] = cached
                return cached
            entry = next(
                (item for item in self.catalog() if item.get("id") == normalized),
                None,
            )
            if entry is None or entry.get("status") == "error":
                raise FileNotFoundError(
                    f"episode not found or unreadable: {normalized}"
                )
            loaded = self._decode_episode(entry)
            self._cache[normalized] = loaded
            while len(self._cache) > self.max_cached_episodes:
                self._cache.popitem(last=False)
            return loaded

    def _catalog_entry(self, episode_dir: Path) -> dict[str, Any]:
        metadata_path = episode_dir / "metadata.yaml"
        if not metadata_path.is_file():
            raise ValueError("metadata.yaml is missing")
        metadata_raw = yaml.safe_load(
            metadata_path.read_text(encoding="utf-8")
        ) or {}
        bag_info = metadata_raw.get("rosbag2_bagfile_information") or {}
        topics: dict[str, dict[str, Any]] = {}
        for item in bag_info.get("topics_with_message_count") or []:
            topic_metadata = item.get("topic_metadata") or {}
            name = str(topic_metadata.get("name") or "")
            if not name:
                continue
            topics[name] = {
                "type": str(topic_metadata.get("type") or ""),
                "count": int(item.get("message_count") or 0),
            }

        recording: dict[str, Any] = {}
        recording_path = episode_dir / "drink_recording.json"
        if recording_path.is_file():
            try:
                value = json.loads(recording_path.read_text(encoding="utf-8"))
                if isinstance(value, dict):
                    recording = value
            except (OSError, json.JSONDecodeError):
                recording = {}

        duration_ns = int(
            (bag_info.get("duration") or {}).get("nanoseconds") or 0
        )
        started_ns = int(
            (bag_info.get("starting_time") or {}).get(
                "nanoseconds_since_epoch"
            )
            or 0
        )
        mcap_files = sorted(episode_dir.glob("*.mcap"))
        mcap_bytes = sum(
            path.stat().st_size for path in mcap_files if path.is_file()
        )
        episode_id = f"{episode_dir.parent.name}/{episode_dir.name}"
        cameras = [
            camera_id
            for camera_id, config in PLAYBACK_CAMERAS.items()
            if topics.get(config["topic"], {}).get("count", 0) > 0
        ]
        pose_arms = [
            side
            for side, candidates in PLAYBACK_POSE_TOPICS.items()
            if any(
                topics.get(topic, {}).get("count", 0) > 0
                for topic in candidates
            )
        ]
        joint_arms = [
            side
            for side, candidates in PLAYBACK_JOINT_TOPICS.items()
            if any(
                topics.get(topic, {}).get("count", 0) > 0
                for topic in candidates
            )
        ]
        return {
            "id": episode_id,
            "group": episode_dir.parent.name,
            "name": episode_dir.name,
            "path": str(episode_dir),
            "task_name": recording.get("task_name") or episode_dir.parent.name,
            "note": recording.get("note") or "",
            "sequence": recording.get("sequence"),
            "status": "ready",
            "started_at": (
                started_ns / 1_000_000_000
                if started_ns
                else recording.get("started_at")
            ),
            "started_ns": started_ns,
            "duration_sec": duration_ns / 1_000_000_000,
            "message_count": int(bag_info.get("message_count") or 0),
            "storage": str(
                bag_info.get("storage_identifier") or "mcap"
            ),
            "mcap_bytes": mcap_bytes,
            "cameras": cameras,
            "pose_arms": pose_arms,
            "joint_arms": joint_arms,
            "topic_counts": {
                name: data["count"] for name, data in topics.items()
            },
        }

    def _decode_episode(self, entry: dict[str, Any]) -> dict[str, Any]:
        episode_dir = Path(str(entry["path"]))
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(episode_dir),
                storage_id=str(entry.get("storage") or "mcap"),
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
        available_types = {
            item.name: item.type
            for item in reader.get_all_topics_and_types()
        }
        topic_counts = entry.get("topic_counts") or {}
        selected_pose_topics = {
            side: next(
                (
                    topic
                    for topic in candidates
                    if topic in available_types
                    and int(topic_counts.get(topic) or 0) > 0
                ),
                "",
            )
            for side, candidates in PLAYBACK_POSE_TOPICS.items()
        }
        selected_joint_topics = {
            side: next(
                (
                    topic
                    for topic in candidates
                    if topic in available_types
                    and int(topic_counts.get(topic) or 0) > 0
                ),
                "",
            )
            for side, candidates in PLAYBACK_JOINT_TOPICS.items()
        }
        camera_by_topic = {
            config["topic"]: camera_id
            for camera_id, config in PLAYBACK_CAMERAS.items()
            if config["topic"] in available_types
            and int(topic_counts.get(config["topic"]) or 0) > 0
        }
        pose_by_topic = {
            topic: side
            for side, topic in selected_pose_topics.items()
            if topic
        }
        joint_by_topic = {
            topic: side
            for side, topic in selected_joint_topics.items()
            if topic
        }

        camera_times: dict[str, list[float]] = {
            camera_id: [] for camera_id in camera_by_topic.values()
        }
        camera_frames: dict[str, list[bytes]] = {
            camera_id: [] for camera_id in camera_by_topic.values()
        }
        camera_shapes: dict[str, tuple[int, int]] = {}
        pose_samples: dict[str, list[list[float]]] = {
            side: [] for side in pose_by_topic.values()
        }
        joint_samples: dict[str, list[list[float]]] = {
            side: [] for side in joint_by_topic.values()
        }
        joint_names: dict[str, list[str]] = {
            side: [] for side in joint_by_topic.values()
        }
        warnings: list[str] = []
        warning_topics: set[str] = set()
        started_ns = int(entry.get("started_ns") or 0)
        last_relevant_ns = started_ns

        while reader.has_next():
            topic, serialized, timestamp_ns = reader.read_next()
            if (
                topic not in camera_by_topic
                and topic not in pose_by_topic
                and topic not in joint_by_topic
            ):
                continue
            if not started_ns:
                started_ns = timestamp_ns
            last_relevant_ns = max(last_relevant_ns, timestamp_ns)
            relative_time = round(
                max(
                    0.0,
                    (timestamp_ns - started_ns) / 1_000_000_000,
                ),
                4,
            )
            try:
                if topic in camera_by_topic:
                    message = deserialize_message(serialized, Image)
                    frame = self._bridge.imgmsg_to_cv2(
                        message, desired_encoding="bgr8"
                    )
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        frame,
                        [
                            int(cv2.IMWRITE_JPEG_QUALITY),
                            self.jpeg_quality,
                        ],
                    )
                    if not ok:
                        raise ValueError("OpenCV JPEG encoding failed")
                    camera_id = camera_by_topic[topic]
                    camera_times[camera_id].append(relative_time)
                    camera_frames[camera_id].append(encoded.tobytes())
                    camera_shapes[camera_id] = (
                        int(message.width),
                        int(message.height),
                    )
                elif topic in pose_by_topic:
                    message_type = (
                        PoseStamped
                        if available_types[topic].endswith("PoseStamped")
                        else Pose
                    )
                    message = deserialize_message(
                        serialized, message_type
                    )
                    pose = (
                        message.pose
                        if isinstance(message, PoseStamped)
                        else message
                    )
                    side = pose_by_topic[topic]
                    pose_samples[side].append(
                        [
                            relative_time,
                            round(float(pose.position.x), 6),
                            round(float(pose.position.y), 6),
                            round(float(pose.position.z), 6),
                            round(float(pose.orientation.x), 6),
                            round(float(pose.orientation.y), 6),
                            round(float(pose.orientation.z), 6),
                            round(float(pose.orientation.w), 6),
                        ]
                    )
                else:
                    message = deserialize_message(
                        serialized, JointState
                    )
                    side = joint_by_topic[topic]
                    if not joint_names[side] and message.name:
                        joint_names[side] = [
                            str(name) for name in message.name
                        ]
                    joint_samples[side].append(
                        [
                            relative_time,
                            *[
                                round(float(value), 6)
                                for value in message.position
                            ],
                        ]
                    )
            except Exception as exc:  # noqa: BLE001
                if topic not in warning_topics:
                    warning_topics.add(topic)
                    warnings.append(
                        f"{topic}: {exc.__class__.__name__}: {exc}"
                    )

        cameras: dict[str, Any] = {}
        for camera_id, times in camera_times.items():
            width, height = camera_shapes.get(camera_id, (0, 0))
            cameras[camera_id] = {
                "id": camera_id,
                "label": PLAYBACK_CAMERAS[camera_id]["label"],
                "topic": PLAYBACK_CAMERAS[camera_id]["topic"],
                "width": width,
                "height": height,
                "frame_count": len(times),
                "times": times,
            }

        poses = {
            side: {
                "topic": selected_pose_topics[side],
                "fields": [
                    "time",
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                ],
                "samples": downsample_rows(
                    samples, self.max_trajectory_samples
                ),
                "sample_count": len(samples),
            }
            for side, samples in pose_samples.items()
        }
        joints = {
            side: {
                "topic": selected_joint_topics[side],
                "names": joint_names.get(side) or [],
                "samples": downsample_rows(
                    samples, self.max_trajectory_samples
                ),
                "sample_count": len(samples),
            }
            for side, samples in joint_samples.items()
        }
        decoded_duration = (
            max(
                0.0,
                (last_relevant_ns - started_ns) / 1_000_000_000,
            )
            if started_ns
            else 0.0
        )
        duration_sec = max(
            float(entry.get("duration_sec") or 0.0),
            decoded_duration,
        )
        public_entry = {
            key: value
            for key, value in entry.items()
            if key not in {"path", "started_ns", "topic_counts"}
        }
        manifest = {
            "ok": True,
            "episode": public_entry,
            "duration_sec": round(duration_sec, 4),
            "cameras": cameras,
            "trajectories": {
                "poses": poses,
                "joints": joints,
            },
            "warnings": warnings,
        }
        return {
            "manifest": manifest,
            "frames": camera_frames,
            "loaded_at": time.time(),
        }

    def _normalize_episode_id(self, episode_id: str) -> str:
        raw = str(episode_id or "").strip().strip("/")
        parts = Path(raw).parts
        if (
            len(parts) != 2
            or any(part in {"", ".", ".."} for part in parts)
        ):
            raise ValueError(
                "episode must be group/episode_NNN"
            )
        if not parts[1].startswith("episode_"):
            raise ValueError("invalid episode name")
        return f"{parts[0]}/{parts[1]}"


def downsample_rows(
    rows: list[list[float]], limit: int
) -> list[list[float]]:
    if len(rows) <= limit:
        return rows
    if limit <= 1:
        return [rows[0]]
    scale = (len(rows) - 1) / (limit - 1)
    indices = [round(index * scale) for index in range(limit)]
    return [rows[index] for index in indices]


def bounded_int(
    value: Any,
    default: int,
    minimum: int,
    maximum: int,
) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        number = default
    return max(minimum, min(maximum, number))
