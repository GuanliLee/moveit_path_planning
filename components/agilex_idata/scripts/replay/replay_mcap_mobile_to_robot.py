#!/usr/bin/env python3
"""Replay one raw collection MCAP episode to arms, base, and lifting column.

The adapter reads only typed action/state records from the existing episode.
It never republishes an entire rosbag and therefore cannot replay camera or
collection state-machine topics by accident.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

from replay_agibot_mobile_to_robot import (
    MobileReplayNode,
    replay_schedule,
    sample_at_time,
    smooth_move_arms,
)


@dataclasses.dataclass
class McapReplayData:
    episode_dir: Path
    timestamps: np.ndarray
    arms: np.ndarray | None
    base_cmd: np.ndarray | None
    lift_height: np.ndarray | None
    arm_source: str


def resolve_episode_dir(path_text: str, episode: str) -> Path:
    path = Path(path_text).expanduser().resolve()
    if path.is_file() and path.suffix == ".mcap":
        return path.parent
    if not path.is_dir():
        raise FileNotFoundError(path)
    if list(path.glob("*.mcap")) and (path / "metadata.yaml").is_file():
        return path
    name = episode if episode.startswith("episode") else f"episode{episode}"
    candidate = path / name
    if candidate.is_dir() and list(candidate.glob("*.mcap")):
        return candidate
    raise FileNotFoundError(f"MCAP episode not found: {candidate}")


def stream_array(records: list[tuple[float, list[float]]]) -> tuple[np.ndarray, np.ndarray]:
    if not records:
        return np.zeros(0), np.zeros((0, 0))
    records.sort(key=lambda item: item[0])
    times = np.asarray([item[0] for item in records], dtype=np.float64)
    values = np.asarray([item[1] for item in records], dtype=np.float64)
    keep = np.concatenate(([True], np.diff(times) > 1e-9))
    return times[keep], values[keep]


def interpolate(values: np.ndarray, times: np.ndarray, target: np.ndarray) -> np.ndarray | None:
    if len(times) == 0 or values.size == 0:
        return None
    return np.column_stack(
        [np.interp(target, times, values[:, index]) for index in range(values.shape[1])]
    )


def load_episode(episode_dir: Path, arm_source: str) -> McapReplayData:
    prefix = "/master" if arm_source == "master" else "/puppet"
    left_topic = f"{prefix}/joint_left"
    right_topic = f"{prefix}/joint_right"
    selected = {
        left_topic,
        right_topic,
        "/action/chassis",
        "/LiftMotorStatePub",
    }

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(episode_dir), storage_id="mcap"),
        ConverterOptions("", ""),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    missing = [topic for topic in (left_topic, right_topic) if topic not in topic_types]
    if missing:
        raise ValueError(f"{episode_dir}: missing required topics: {', '.join(missing)}")

    records: dict[str, list[tuple[float, list[float]]]] = {
        topic: [] for topic in selected
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in selected
        if topic in topic_types
    }
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        timestamp = float(timestamp_ns) / 1e9
        if topic in {left_topic, right_topic}:
            if len(message.position) >= 7:
                records[topic].append((timestamp, [float(v) for v in message.position[:7]]))
        elif topic == "/action/chassis":
            payload = json.loads(message.data)
            records[topic].append(
                (
                    timestamp,
                    [
                        float(payload.get("linearX", payload.get("linear_x", 0.0))),
                        float(payload.get("linearY", payload.get("linear_y", 0.0))),
                        float(payload.get("angularZ", payload.get("angular_z", 0.0))),
                    ],
                )
            )
        elif topic == "/LiftMotorStatePub":
            height = getattr(message, "back_height", None)
            if height is None:
                height = getattr(message, "backHeight", None)
            if height is not None:
                records[topic].append((timestamp, [float(height)]))

    left_times, left_values = stream_array(records[left_topic])
    right_times, right_values = stream_array(records[right_topic])
    if len(left_times) == 0 or len(right_times) == 0:
        raise ValueError(f"{episode_dir}: no usable {arm_source} arm JointState samples")

    overlap = (left_times >= right_times[0]) & (left_times <= right_times[-1])
    timeline = left_times[overlap]
    left = left_values[overlap]
    if len(timeline) < 2:
        raise ValueError(f"{episode_dir}: left/right arm streams have no usable overlap")
    right = interpolate(right_values, right_times, timeline)
    assert right is not None
    arms = np.concatenate([left, right], axis=1)

    base_times, base_values = stream_array(records["/action/chassis"])
    lift_times, lift_values = stream_array(records["/LiftMotorStatePub"])
    base = interpolate(base_values, base_times, timeline)
    lift_matrix = interpolate(lift_values, lift_times, timeline)
    lift = lift_matrix[:, 0] if lift_matrix is not None else None
    timeline = timeline - timeline[0]
    return McapReplayData(
        episode_dir=episode_dir,
        timestamps=timeline,
        arms=arms,
        base_cmd=base,
        lift_height=lift,
        arm_source=f"{left_topic} + {right_topic}",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("path", help="dataset root, episode directory, or episodeN_0.mcap")
    parser.add_argument("--episode", default="0", help="episode number/name when path is a dataset root")
    parser.add_argument("--source", choices=("master", "puppet"), default="master")
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--publish-hz", type=float, default=30.0)
    parser.add_argument("--skip", type=int, default=1)
    parser.add_argument("--arm-interp", choices=("hold", "linear"), default="linear")
    parser.add_argument("--base-interp", choices=("hold", "linear"), default="linear")
    parser.add_argument("--vel", type=float, default=70.0)
    parser.add_argument("--init-vel", type=float, default=35.0)
    parser.add_argument("--init-duration", type=float, default=5.0)
    parser.add_argument("--base-scale", type=float, default=None)
    parser.add_argument("--max-linear", type=float, default=0.3)
    parser.add_argument("--max-angular", type=float, default=0.8)
    parser.add_argument("--lift-mode", type=int, default=0)
    parser.add_argument("--lift-min-delta", type=float, default=2.0)
    parser.add_argument("--lift-min-period", type=float, default=0.2)
    parser.add_argument("--left-state-topic", default="/puppet/joint_left")
    parser.add_argument("--right-state-topic", default="/puppet/joint_right")
    parser.add_argument("--left-command-topic", default="/joint_left_states")
    parser.add_argument("--right-command-topic", default="/joint_right_states")
    parser.add_argument("--base-command-topic", default="/cmd_vel")
    parser.add_argument("--lift-state-topic", default="/LiftMotorStatePub")
    parser.add_argument("--lift-service", default="/LiftingMotorService")
    parser.add_argument("--disable-arms", action="store_true")
    parser.add_argument("--disable-base", action="store_true")
    parser.add_argument("--disable-lift", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-confirm", action="store_true")
    return parser.parse_args()


def print_summary(data: McapReplayData, args: argparse.Namespace) -> None:
    print(f"MCAP episode: {data.episode_dir}")
    print(f"  frames: {len(data.timestamps)}")
    print(f"  duration: {data.timestamps[-1]:.3f}s")
    print(f"  arms: {None if data.arms is None else data.arms.shape}, source={data.arm_source}")
    print(f"  base: {None if data.base_cmd is None else data.base_cmd.shape}, source=/action/chassis")
    print(f"  lift: {None if data.lift_height is None else data.lift_height.shape}, source=/LiftMotorStatePub.back_height")
    print(f"  command endpoints: {args.left_command_topic}, {args.right_command_topic}, {args.base_command_topic}, {args.lift_service}")


def main() -> int:
    args = parse_args()
    if args.rate <= 0 or args.publish_hz > 200 or args.skip <= 0:
        raise SystemExit("rate/skip must be positive and publish-hz must be <= 200")
    episode_dir = resolve_episode_dir(args.path, str(args.episode))
    data = load_episode(episode_dir, args.source)
    if args.disable_arms:
        data.arms = None
    if args.disable_base:
        data.base_cmd = None
    if args.disable_lift:
        data.lift_height = None
    print_summary(data, args)
    if args.dry_run:
        print("Dry run only; no ROS commands published.")
        return 0

    if not args.no_confirm:
        answer = input("Keep workspace clear. Press Enter to continue, or q to cancel: ").strip().lower()
        if answer == "q":
            return 0

    base_scale = args.rate if args.base_scale is None else args.base_scale
    rclpy.init()
    node = None
    try:
        node = MobileReplayNode(
            args,
            enable_arms=data.arms is not None,
            enable_base=data.base_cmd is not None,
            enable_lift=data.lift_height is not None,
        )
        if data.arms is not None:
            if not node.wait_for_current_arms():
                raise RuntimeError("Did not receive current left/right arm JointState within 5s")
        if data.lift_height is not None and not node.wait_for_lift_service():
            raise RuntimeError(f"Lift service not available: {args.lift_service}")
        node.stop_base()
        if data.arms is not None:
            smooth_move_arms(
                node,
                data.arms[0, :7],
                data.arms[0, 7:14],
                duration=args.init_duration,
                vel_pct=args.init_vel,
            )

        schedule = replay_schedule(data.timestamps, args.publish_hz, args.skip)
        start_time = time.monotonic()
        last_lift_height = None
        last_lift_time = 0.0
        for index, source_time in enumerate(schedule):
            if not rclpy.ok():
                break
            sleep_seconds = source_time / args.rate - (time.monotonic() - start_time)
            if sleep_seconds > 0.0005:
                time.sleep(sleep_seconds)
            elif sleep_seconds < -0.05:
                continue
            if data.arms is not None:
                arms = sample_at_time(data.arms, data.timestamps, source_time, args.arm_interp)
                assert arms is not None
                node.publish_arms(arms[:7], arms[7:14], args.vel)
            if data.base_cmd is not None:
                base = sample_at_time(data.base_cmd, data.timestamps, source_time, args.base_interp)
                assert base is not None
                node.publish_base(base, args.max_linear, args.max_angular, base_scale)
            if data.lift_height is not None:
                sampled = sample_at_time(data.lift_height, data.timestamps, source_time, "linear")
                height = float(np.asarray(sampled).reshape(-1)[0])
                now = time.monotonic()
                if (
                    (last_lift_height is None or abs(height - last_lift_height) >= args.lift_min_delta)
                    and now - last_lift_time >= args.lift_min_period
                ):
                    node.send_lift_height(height, args.lift_mode)
                    last_lift_height = height
                    last_lift_time = now
            rclpy.spin_once(node, timeout_sec=0.0)
            if index % max(1, len(schedule) // 20) == 0:
                print(f"progress {index + 1}/{len(schedule)}", flush=True)

        if data.arms is not None:
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline and rclpy.ok():
                node.publish_arms(data.arms[-1, :7], data.arms[-1, 7:14], args.vel)
                node.stop_base()
                time.sleep(0.05)
        else:
            node.stop_base()
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.stop_base()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
