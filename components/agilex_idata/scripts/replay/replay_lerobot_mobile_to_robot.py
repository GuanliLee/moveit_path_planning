#!/usr/bin/env python3
"""Replay a mobile LeRobot episode to arms, base and lifting column."""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from lifting_msg_pkg.msg import LiftMotorMsg
    from lifting_msg_pkg.srv import LiftMotorSrv
except ImportError:
    try:
        from bt_task_msgs.msg import LiftMotorMsg
        from bt_task_msgs.srv import LiftMotorSrv
    except ImportError:
        LiftMotorMsg = None
        LiftMotorSrv = None


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]
LEROBOT_COLUMNS = {"action": "action", "state": "observation.state"}


@dataclasses.dataclass
class EpisodeData:
    arms: np.ndarray | None
    base_cmd: np.ndarray | None
    base_state: np.ndarray | None
    lift_height: np.ndarray | None
    timestamps: np.ndarray
    episode_path: Path
    dataset_root: Path | None
    info: dict
    arm_source_column: str
    base_source: str | None
    lift_source: str | None


def import_pandas():
    try:
        import pandas as pd
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "pandas/pyarrow is required to read LeRobot parquet files. "
            "Run this script in the LeRobot Python environment."
        ) from exc
    return pd


def read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def read_jsonl(path: Path) -> list[dict]:
    rows = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def find_dataset_roots(start: Path) -> list[Path]:
    if start.is_file():
        return []
    roots = []
    for info in start.rglob("meta/info.json"):
        root = info.parent.parent
        if (root / "data").exists():
            roots.append(root)
    return sorted(roots)


def resolve_episode_path(dataset_or_file: Path, episode_index: int) -> tuple[Path, Path | None, dict]:
    if dataset_or_file.is_file():
        return dataset_or_file, None, {}

    dataset_root = dataset_or_file
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.exists():
        roots = find_dataset_roots(dataset_root)
        if len(roots) == 1:
            dataset_root = roots[0]
            info_path = dataset_root / "meta" / "info.json"
        elif roots:
            raise FileNotFoundError(
                "Multiple LeRobot datasets found; pass one exact dataset root:\n"
                + "\n".join(f"  {root}" for root in roots)
            )
        else:
            raise FileNotFoundError(f"LeRobot meta/info.json not found under: {dataset_or_file}")

    info = read_json(info_path)
    chunks_size = int(info.get("chunks_size", 1000))
    episode_chunk = episode_index // chunks_size
    data_path_template = info.get(
        "data_path",
        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    )
    rel_path = data_path_template.format(episode_chunk=episode_chunk, episode_index=episode_index)
    episode_path = dataset_root / rel_path
    if not episode_path.exists():
        matches = sorted(dataset_root.glob(f"data/**/episode_{episode_index:06d}.parquet"))
        if matches:
            episode_path = matches[0]
        else:
            raise FileNotFoundError(f"Episode parquet not found: {episode_path}")
    return episode_path, dataset_root, info


def list_dataset(dataset_root: Path) -> None:
    roots = [dataset_root] if (dataset_root / "meta" / "info.json").exists() else find_dataset_roots(dataset_root)
    if not roots:
        raise FileNotFoundError(f"No LeRobot dataset found under: {dataset_root}")
    for root in roots:
        info = read_json(root / "meta" / "info.json")
        episodes = read_jsonl(root / "meta" / "episodes.jsonl")
        print(root)
        print(
            f"  episodes={info.get('total_episodes')} frames={info.get('total_frames')} "
            f"fps={info.get('fps')} robot_type={info.get('robot_type')}"
        )
        for row in episodes[:10]:
            tasks = ", ".join(row.get("tasks", []))
            print(f"  episode {row.get('episode_index')}: length={row.get('length')} task={tasks}")
        if len(episodes) > 10:
            print(f"  ... {len(episodes) - 10} more episodes")


def feature_names(info: dict, column: str) -> list[str]:
    feature = info.get("features", {}).get(column, {})
    names = feature.get("names", [])
    if isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    if isinstance(names, list):
        parsed = [str(name) for name in names]
        shape = feature.get("shape", [])
        width = int(shape[0]) if isinstance(shape, list) and shape and str(shape[0]).isdigit() else 0
        generic_vector_names = {"action", "actions", "state", "states"}
        if width > 1 and len(parsed) == 1 and parsed[0].lower() in generic_vector_names:
            return []
        return parsed
    return []


def stack_column(df, column: str) -> np.ndarray:
    if column not in df.columns:
        raise KeyError(f"Column {column!r} not found. Available columns: {list(df.columns)}")
    values = [np.asarray(value, dtype=np.float64).reshape(-1) for value in df[column].to_numpy()]
    if not values:
        raise ValueError(f"Column {column!r} is empty.")
    return np.stack(values, axis=0)


def normalize_timestamps(df, fps: float) -> np.ndarray:
    if "timestamp" in df.columns:
        timestamps = np.asarray(df["timestamp"].to_numpy(), dtype=np.float64).reshape(-1)
        if len(timestamps) > 1 and np.all(np.diff(timestamps) >= 0):
            return timestamps
        print("[Warn] Invalid timestamp column; falling back to FPS.")
    return np.arange(len(df), dtype=np.float64) / float(fps)


def exact_indices(names: list[str], ordered_names: list[str]) -> list[int] | None:
    if not names:
        return None
    index_by_name = {name: i for i, name in enumerate(names)}
    if all(name in index_by_name for name in ordered_names):
        return [index_by_name[name] for name in ordered_names]
    return None


def joint_indices_from_names(names: list[str], source: str, width: int) -> tuple[list[int], list[int]] | None:
    if source == "action":
        left_prefixes = ["arm.jointStatePosition.masterLeft", "arm.endPose.masterLeft"]
        right_prefixes = ["arm.jointStatePosition.masterRight", "arm.endPose.masterRight"]
    else:
        left_prefixes = ["arm.jointStatePosition.puppetLeft", "arm.endPose.puppetLeft"]
        right_prefixes = ["arm.jointStatePosition.puppetRight", "arm.endPose.puppetRight"]

    for left_prefix, right_prefix in zip(left_prefixes, right_prefixes):
        left = exact_indices(names, [f"{left_prefix}.joint{i}" for i in range(7)])
        right = exact_indices(names, [f"{right_prefix}.joint{i}" for i in range(7)])
        if left is not None and right is not None:
            return left, right

    if names:
        left = [i for i, name in enumerate(names) if name.startswith("left_")]
        right = [i for i, name in enumerate(names) if name.startswith("right_")]
        if len(left) >= 7 and len(right) >= 7:
            return left[:7], right[:7]

    if width >= 14:
        return list(range(7)), list(range(7, 14))
    return None


def extract_arms(data: np.ndarray, names: list[str], source: str) -> np.ndarray:
    indices = joint_indices_from_names(names, source, data.shape[1])
    if indices is None:
        raise ValueError(f"Could not locate left/right 7-DoF arm data in {LEROBOT_COLUMNS[source]}.")
    left_indices, right_indices = indices
    return np.concatenate([data[:, left_indices], data[:, right_indices]], axis=1)


def extract_base_action(action: np.ndarray | None, names: list[str]) -> tuple[np.ndarray | None, str | None]:
    if action is None:
        return None, None

    prefixes = []
    for name in names:
        if name.startswith("robotBase.action.") and name.count(".") >= 3:
            prefixes.append(name.rsplit(".", 1)[0])
    for prefix in sorted(set(prefixes)):
        idx = exact_indices(names, [f"{prefix}.vx_cmd", f"{prefix}.vy_cmd", f"{prefix}.wz_cmd"])
        if idx is not None:
            return action[:, idx], f"action names {idx}"

    if not names:
        if action.shape[1] >= 18:
            return action[:, 15:18], "action joints_base fallback columns 15:18"
        if action.shape[1] >= 17:
            return action[:, 14:17], "action fallback columns 14:17"
    return None, None


def extract_base_state(state: np.ndarray | None, names: list[str]) -> np.ndarray | None:
    if state is None:
        return None
    prefixes = []
    for name in names:
        if name.startswith("robotBase.state.") and name.count(".") >= 3:
            prefixes.append(name.rsplit(".", 1)[0])
    for prefix in sorted(set(prefixes)):
        idx = exact_indices(names, [f"{prefix}.x", f"{prefix}.y", f"{prefix}.yaw",
                                    f"{prefix}.vx", f"{prefix}.vy", f"{prefix}.wz"])
        if idx is not None:
            return state[:, idx]
    if not names:
        if state.shape[1] >= 21:
            return state[:, 15:21]
        if state.shape[1] >= 20:
            return state[:, 14:20]
    return None


def extract_lift_height(state: np.ndarray | None, state_names: list[str],
                        action: np.ndarray | None, action_names: list[str]) -> tuple[np.ndarray | None, str | None]:
    for column_name, data, names, prefixes in [
        ("action", action, action_names, ("action.lifting.", "lift.action.")),
        ("observation.state", state, state_names, ("lift.motor.",)),
    ]:
        if data is None:
            continue
        for i, name in enumerate(names):
            lower = name.lower()
            if lower.startswith(prefixes) and ("height" in lower or "back_height" in lower):
                source = name if name.startswith(f"{column_name}.") else f"{column_name}.{name}"
                return data[:, i], source

    if action is not None and not action_names:
        if action.shape[1] >= 18:
            return action[:, 14], "action joints_base fallback column 14"
        if action.shape[1] == 15:
            return action[:, 14], "action fallback column 14"
    if state is not None and not state_names:
        if state.shape[1] >= 21:
            return state[:, 14], "observation.state joints_base fallback column 14"
        if state.shape[1] == 15:
            return state[:, 14], "observation.state fallback column 14"
    return None, None


def denormalize_grippers(arms: np.ndarray, gripper_min: float,
                         gripper_max: float, gripper_unit: str) -> np.ndarray:
    commands = np.array(arms, dtype=np.float64, copy=True)
    if gripper_unit == "normalized":
        for index in (6, 13):
            commands[:, index] = gripper_min + np.clip(commands[:, index], 0.0, 1.0) * (
                gripper_max - gripper_min
            )
    elif gripper_unit != "raw":
        raise ValueError(f"Unknown gripper unit: {gripper_unit}")
    return commands


def slice_data(data: np.ndarray | None, start: int, end: int | None) -> np.ndarray | None:
    if data is None:
        return None
    return data[start:end]


def load_lerobot_episode(args: argparse.Namespace) -> EpisodeData:
    pd = import_pandas()
    episode_path, dataset_root, info = resolve_episode_path(Path(args.dataset), args.episode)
    df = pd.read_parquet(episode_path)

    fps = float(args.fps or info.get("fps") or 30.0)
    timestamps = normalize_timestamps(df, fps)
    state_names = feature_names(info, "observation.state")
    action_names = feature_names(info, "action")

    state = stack_column(df, "observation.state") if "observation.state" in df.columns else None
    action = stack_column(df, "action") if "action" in df.columns else None

    arm_source_column = LEROBOT_COLUMNS[args.source]
    arm_source = action if args.source == "action" else state
    arm_names = action_names if args.source == "action" else state_names
    arms = None
    if arm_source is not None:
        try:
            arms = extract_arms(arm_source, arm_names, args.source)
            arms = denormalize_grippers(arms, args.gripper_min, args.gripper_max, args.gripper_unit)
        except ValueError:
            if not args.disable_arms:
                raise

    base_cmd, base_source = extract_base_action(action, action_names)
    base_state = extract_base_state(state, state_names)
    lift_height, lift_source = extract_lift_height(state, state_names, action, action_names)

    start = max(args.start_frame, 0)
    end = args.end_frame if args.end_frame is not None else len(timestamps)
    timestamps = timestamps[start:end]
    if len(timestamps):
        timestamps = timestamps - timestamps[0]
    arms = slice_data(arms, start, end)
    base_cmd = slice_data(base_cmd, start, end)
    base_state = slice_data(base_state, start, end)
    lift_height = slice_data(lift_height, start, end)

    lengths = [len(timestamps)]
    for item in (arms, base_cmd, base_state, lift_height):
        if item is not None:
            lengths.append(len(item))
    size = min(lengths)
    if size <= 0:
        raise ValueError("Selected frame range is empty.")

    timestamps = timestamps[:size]
    arms = slice_data(arms, 0, size)
    base_cmd = slice_data(base_cmd, 0, size)
    base_state = slice_data(base_state, 0, size)
    lift_height = slice_data(lift_height, 0, size)

    for name, item in [("arms", arms), ("base_cmd", base_cmd), ("base_state", base_state), ("lift_height", lift_height)]:
        if item is not None and not np.all(np.isfinite(item)):
            raise ValueError(f"{name} contains NaN or inf.")

    if arms is not None and args.reject_joint_abs > 0:
        joints = np.concatenate([arms[:, :6], arms[:, 7:13]], axis=1)
        max_joint_abs = float(np.max(np.abs(joints)))
        if max_joint_abs > args.reject_joint_abs:
            raise ValueError(
                f"Joint abs {max_joint_abs:.3f} exceeds --reject-joint-abs {args.reject_joint_abs:.3f}."
            )

    return EpisodeData(
        arms=arms,
        base_cmd=base_cmd,
        base_state=base_state,
        lift_height=lift_height,
        timestamps=timestamps,
        episode_path=episode_path,
        dataset_root=dataset_root,
        info=info,
        arm_source_column=arm_source_column,
        base_source=base_source,
        lift_source=lift_source,
    )


def clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return float(value)
    return float(max(-limit, min(limit, value)))


class MobileReplayNode(Node):
    def __init__(self, args: argparse.Namespace, enable_arms: bool,
                 enable_base: bool, enable_lift: bool):
        super().__init__("lerobot_mobile_replay_to_robot")
        self.enable_arms = enable_arms
        self.enable_base = enable_base
        self.enable_lift = enable_lift

        self.cur_left = None
        self.cur_right = None
        self.pub_left = None
        self.pub_right = None
        if self.enable_arms:
            self.pub_left = self.create_publisher(JointState, args.left_command_topic, 10)
            self.pub_right = self.create_publisher(JointState, args.right_command_topic, 10)
            self.create_subscription(
                JointState, args.left_state_topic,
                lambda m: setattr(self, "cur_left", np.asarray(m.position[:7], dtype=np.float64)), 10)
            self.create_subscription(
                JointState, args.right_state_topic,
                lambda m: setattr(self, "cur_right", np.asarray(m.position[:7], dtype=np.float64)), 10)

        self.pub_base = None
        if self.enable_base:
            self.pub_base = self.create_publisher(Twist, args.base_command_topic, 10)

        self.cur_lift_height = None
        self.lift_client = None
        if self.enable_lift:
            if LiftMotorMsg is None or LiftMotorSrv is None:
                raise RuntimeError(
                    "Lift message/service package is not available. Source the lift driver workspace "
                    "or run with --disable-lift."
                )
            self.lift_client = self.create_client(LiftMotorSrv, args.lift_service)
            self.create_subscription(LiftMotorMsg, args.lift_state_topic, self._on_lift_state, 10)

    def _on_lift_state(self, msg) -> None:
        height = getattr(msg, "back_height", None)
        if height is None:
            height = getattr(msg, "backHeight", None)
        if height is not None:
            self.cur_lift_height = int(height)

    def wait_for_current_arms(self, timeout: float = 5.0) -> bool:
        if not self.enable_arms:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.cur_left is not None and self.cur_right is not None:
                return True
        return False

    def wait_for_lift_service(self, timeout: float = 3.0) -> bool:
        if not self.enable_lift:
            return True
        return self.lift_client.wait_for_service(timeout_sec=timeout)

    def publish_arms(self, left: np.ndarray, right: np.ndarray, vel_pct: float) -> None:
        if not self.enable_arms:
            return
        stamp = self.get_clock().now().to_msg()
        velocity = [0.0] * 6 + [float(vel_pct)]

        msg_l = JointState()
        msg_l.header.stamp = stamp
        msg_l.name = JOINT_NAMES
        msg_l.position = [float(x) for x in left[:7]]
        msg_l.velocity = list(velocity)
        msg_l.effort = [0.0] * 7

        msg_r = JointState()
        msg_r.header.stamp = stamp
        msg_r.name = JOINT_NAMES
        msg_r.position = [float(x) for x in right[:7]]
        msg_r.velocity = list(velocity)
        msg_r.effort = [0.0] * 7

        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def publish_base(self, vel_xyz: np.ndarray, max_linear: float,
                     max_angular: float, scale: float) -> None:
        if not self.enable_base:
            return
        msg = Twist()
        msg.linear.x = clamp(float(vel_xyz[0]) * scale, max_linear)
        msg.linear.y = clamp(float(vel_xyz[1]) * scale, max_linear)
        msg.angular.z = clamp(float(vel_xyz[2]) * scale, max_angular)
        self.pub_base.publish(msg)

    def stop_base(self) -> None:
        if self.enable_base:
            self.pub_base.publish(Twist())

    def send_lift_height(self, height: float, mode: int) -> None:
        if not self.enable_lift:
            return
        req = LiftMotorSrv.Request()
        req.val = int(round(float(height)))
        req.mode = int(mode)
        self.lift_client.call_async(req)


def smooth_move_arms(node: MobileReplayNode, target_l: np.ndarray, target_r: np.ndarray,
                     duration: float, vel_pct: float) -> None:
    if not node.enable_arms:
        return
    print(f"\n>>> Move arms to initial pose ({duration:.1f}s, {vel_pct:.0f}%)")
    rate_hz = 20.0
    steps = max(int(duration * rate_hz), 2)
    start_l = node.cur_left.copy()
    start_r = node.cur_right.copy()
    for i in range(steps + 1):
        alpha = i / steps
        s = 0.5 - 0.5 * np.cos(np.pi * alpha)
        left = start_l * (1 - s) + target_l * s
        right = start_r * (1 - s) + target_r * s
        node.publish_arms(left, right, vel_pct=vel_pct)
        node.stop_base()
        time.sleep(1.0 / rate_hz)
    print(">>> Initial arm pose reached")


def sample_at_time(values: np.ndarray | None, timestamps: np.ndarray, timestamp: float) -> np.ndarray | None:
    if values is None:
        return None
    if timestamp <= timestamps[0]:
        return values[0]
    if timestamp >= timestamps[-1]:
        return values[-1]
    right = int(np.searchsorted(timestamps, timestamp, side="right"))
    left = max(0, right - 1)
    right = min(right, len(timestamps) - 1)
    t0 = float(timestamps[left])
    t1 = float(timestamps[right])
    if t1 <= t0:
        return values[right]
    alpha = (float(timestamp) - t0) / (t1 - t0)
    return values[left] * (1.0 - alpha) + values[right] * alpha


def replay_schedule(timestamps: np.ndarray, publish_hz: float, skip: int) -> list[tuple[int | None, float]]:
    if len(timestamps) == 0:
        return []
    if publish_hz > 0:
        duration = float(timestamps[-1] - timestamps[0])
        step = 1.0 / publish_hz
        count = max(1, int(np.floor(duration / step)) + 1)
        schedule = [(None, float(timestamps[0] + i * step)) for i in range(count)]
        if schedule[-1][1] < float(timestamps[-1]):
            schedule.append((None, float(timestamps[-1])))
        return schedule
    indices = list(range(0, len(timestamps), skip))
    if indices[-1] != len(timestamps) - 1:
        indices.append(len(timestamps) - 1)
    return [(frame_index, float(timestamps[frame_index])) for frame_index in indices]


def print_summary(data: EpisodeData, args: argparse.Namespace) -> None:
    duration = float(data.timestamps[-1] - data.timestamps[0]) if len(data.timestamps) > 1 else 0.0
    print(f"LeRobot episode: {data.episode_path}")
    if data.dataset_root is not None:
        print(f"Dataset root:    {data.dataset_root}")
    print(f"  frames:        {len(data.timestamps)}")
    print(f"  duration:      {duration:.2f}s")
    print(f"  dataset fps:   {data.info.get('fps', args.fps or 'unknown')}")
    print(f"  replay rate:   {args.rate}x, skip={args.skip}, publish_hz={args.publish_hz}")
    if data.arms is not None:
        left = data.arms[:, :7]
        right = data.arms[:, 7:14]
        print(f"  arms source:   {data.arm_source_column}")
        print(f"  first left:    {np.array2string(left[0], precision=4, suppress_small=True)}")
        print(f"  first right:   {np.array2string(right[0], precision=4, suppress_small=True)}")
        print(f"  last left:     {np.array2string(left[-1], precision=4, suppress_small=True)}")
        print(f"  last right:    {np.array2string(right[-1], precision=4, suppress_small=True)}")
    if data.base_cmd is not None:
        max_base = np.max(np.abs(data.base_cmd), axis=0)
        print(f"  base command:  {data.base_cmd.shape}, source={data.base_source}")
        print(f"  max base abs:  {np.array2string(max_base, precision=3)} [vx_cmd, vy_cmd, wz_cmd]")
    if data.base_state is not None:
        print(f"  base state:    {data.base_state.shape} [x, y, yaw, vx, vy, wz]")
    if data.lift_height is not None:
        print(
            f"  lift height:   {data.lift_height.shape}, source={data.lift_source}, "
            f"first={data.lift_height[0]:.0f}mm, last={data.lift_height[-1]:.0f}mm"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", help="LeRobot dataset root or a single episode_XXXXXX.parquet file")
    parser.add_argument("--episode", type=int, default=0, help="Episode index when dataset root is given")
    parser.add_argument("--list", action="store_true", help="List local LeRobot episodes and exit")
    parser.add_argument("--source", choices=sorted(LEROBOT_COLUMNS), default="action",
                        help="Arm trajectory source. Base action always uses LeRobot action")
    parser.add_argument("--fps", type=float, default=None, help="Override FPS if timestamp is unavailable")
    parser.add_argument("--rate", type=float, default=0.5, help="Replay speed multiplier")
    parser.add_argument("--skip", type=int, default=1, help="Publish every N frames")
    parser.add_argument("--publish-hz", type=float, default=30.0,
                        help="Uniform robot command publish rate. Set <=0 to publish only source frames.")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--vel", type=float, default=70.0, help="Arm tracking velocity percentage")
    parser.add_argument("--init-vel", type=float, default=35.0, help="Initial arm move velocity percentage")
    parser.add_argument("--init-duration", type=float, default=5.0, help="Seconds to move arms to first frame")
    parser.add_argument("--hold-sec", type=float, default=1.0, help="Hold final arm pose after replay")
    parser.add_argument("--gripper-unit", choices=("raw", "normalized"), default="raw")
    parser.add_argument("--gripper-min", type=float, default=-0.0026)
    parser.add_argument("--gripper-max", type=float, default=0.1043)
    parser.add_argument("--reject-joint-abs", type=float, default=3.5)
    parser.add_argument("--base-scale", type=float, default=None,
                        help="Scale recorded base velocity. Default follows --rate "
                             "so slow/fast replay preserves displacement")
    parser.add_argument("--max-linear", type=float, default=0.4,
                        help="Clamp abs(linear x/y), m/s. <=0 disables clamp")
    parser.add_argument("--max-angular", type=float, default=0.8,
                        help="Clamp abs(angular z), rad/s. <=0 disables clamp")
    parser.add_argument("--lift-mode", type=int, default=0,
                        help="Lift service mode, default 0 for height position control")
    parser.add_argument("--lift-min-delta", type=float, default=2.0,
                        help="Minimum lift height change in mm before sending another command")
    parser.add_argument("--lift-min-period", type=float, default=0.2,
                        help="Minimum seconds between lift service calls")
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
    parser.add_argument("--dry-run", action="store_true", help="Load and schedule data without publishing commands")
    parser.add_argument("--no-confirm", action="store_true", help="Skip safety confirmation")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise SystemExit("--rate must be positive.")
    if args.skip <= 0:
        raise SystemExit("--skip must be positive.")
    if args.publish_hz > 200:
        raise SystemExit("--publish-hz must be <= 200.")
    if args.gripper_max <= args.gripper_min:
        raise SystemExit("--gripper-max must be greater than --gripper-min.")
    base_scale = args.rate if args.base_scale is None else args.base_scale

    if args.list:
        list_dataset(Path(args.dataset))
        return 0

    data = load_lerobot_episode(args)
    print_summary(data, args)
    if data.base_cmd is not None:
        print(f"  base replay scale: {base_scale:.3f} ({'auto from --rate' if args.base_scale is None else 'explicit --base-scale'})")

    enable_arms = not args.disable_arms and data.arms is not None
    enable_base = not args.disable_base and data.base_cmd is not None
    enable_lift = not args.disable_lift and data.lift_height is not None

    if not enable_arms and not args.disable_arms:
        raise RuntimeError(f"Missing arm data in {data.arm_source_column}.")
    if not enable_base and not args.disable_base:
        print("[Warn] Missing robotBase.action.*.[vx_cmd,vy_cmd,wz_cmd]; base replay disabled")
    if not enable_lift and not args.disable_lift:
        print("[Warn] Missing action.lifting.*.target_height and lift.motor.*.back_height; lift replay disabled")

    if args.dry_run:
        print("\nDry run only; no ROS commands published.")
        return 0
    if not any([enable_arms, enable_base, enable_lift]):
        raise RuntimeError("No replay outputs are enabled.")

    print("\nSafety:")
    print("  - Start piper_ros and ranger/lift drivers first")
    print("  - Keep the workspace clear")
    print("  - Ctrl+C stops command publishing and sends zero /cmd_vel")
    if not args.no_confirm:
        ans = input("\nPress Enter to continue, or q to cancel: ").strip().lower()
        if ans == "q":
            print("Canceled.")
            return 0

    rclpy.init()
    node = None
    try:
        node = MobileReplayNode(args, enable_arms, enable_base, enable_lift)

        if enable_arms:
            print("\n>>> Waiting for current arm states ...")
            if not node.wait_for_current_arms():
                raise RuntimeError("Did not receive current left/right arm JointState within 5s.")
            print(f"  current left : {np.array2string(node.cur_left, precision=3, suppress_small=True)}")
            print(f"  current right: {np.array2string(node.cur_right, precision=3, suppress_small=True)}")

        if enable_lift:
            print("\n>>> Waiting for lift service ...")
            if not node.wait_for_lift_service():
                raise RuntimeError(f"Lift service not available: {args.lift_service}")
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.cur_lift_height is not None:
                    break
            if node.cur_lift_height is not None:
                print(f"  current lift height: {node.cur_lift_height}mm")

        node.stop_base()

        if enable_arms:
            smooth_move_arms(node, data.arms[0, :7], data.arms[0, 7:14],
                             args.init_duration, args.init_vel)

        if not args.no_confirm:
            ans = input("\n>>> Ready. Press Enter to start replay, or q to cancel: ").strip().lower()
            if ans == "q":
                print("Canceled.")
                return 0

        schedule = replay_schedule(data.timestamps, args.publish_hz, args.skip)
        schedule_mode = f"{args.publish_hz:.1f}Hz interpolated" if args.publish_hz > 0 else f"source frames, skip={args.skip}"
        print(f"\n>>> Start replay: {len(schedule)} commands, rate={args.rate}x, mode={schedule_mode}")
        start_time = time.monotonic()
        last_log_time = start_time
        published = 0
        late = 0
        last_lift_height = None
        last_lift_t = 0.0

        for k, (frame_index, source_timestamp) in enumerate(schedule):
            if not rclpy.ok():
                break
            target_elapsed = (source_timestamp - data.timestamps[0]) / args.rate
            now_elapsed = time.monotonic() - start_time
            sleep_s = target_elapsed - now_elapsed
            if sleep_s > 0.0005:
                time.sleep(sleep_s)
            elif sleep_s < -0.05:
                late += 1
                continue

            if frame_index is None:
                arms = sample_at_time(data.arms, data.timestamps, source_timestamp)
                base_cmd = sample_at_time(data.base_cmd, data.timestamps, source_timestamp)
                lift_height = sample_at_time(data.lift_height, data.timestamps, source_timestamp)
            else:
                arms = data.arms[frame_index] if data.arms is not None else None
                base_cmd = data.base_cmd[frame_index] if data.base_cmd is not None else None
                lift_height = data.lift_height[frame_index] if data.lift_height is not None else None

            if enable_arms:
                node.publish_arms(arms[:7], arms[7:14], args.vel)
            if enable_base:
                node.publish_base(base_cmd, args.max_linear, args.max_angular, base_scale)
            if enable_lift:
                height = float(lift_height)
                now = time.monotonic()
                changed = last_lift_height is None or abs(height - last_lift_height) >= args.lift_min_delta
                enough_time = now - last_lift_t >= args.lift_min_period
                if changed and enough_time:
                    node.send_lift_height(height, args.lift_mode)
                    last_lift_height = height
                    last_lift_t = now

            rclpy.spin_once(node, timeout_sec=0.0)
            published += 1

            now = time.monotonic()
            if now - last_log_time > 0.5:
                pct = (k + 1) / len(schedule) * 100.0
                sys.stdout.write(
                    f"\r  progress: {k + 1}/{len(schedule)} ({pct:5.1f}%) "
                    f"published={published} late={late}"
                )
                sys.stdout.flush()
                last_log_time = now

        sys.stdout.write(
            f"\r  progress: {len(schedule)}/{len(schedule)} (100.0%) "
            f"published={published} late={late}\n"
        )
        print(f">>> Replay done in {time.monotonic() - start_time:.2f}s")

        if enable_arms and args.hold_sec > 0:
            print(f">>> Hold final arm pose for {args.hold_sec:.1f}s")
            deadline = time.monotonic() + args.hold_sec
            while time.monotonic() < deadline and rclpy.ok():
                node.publish_arms(data.arms[-1, :7], data.arms[-1, 7:14], args.vel)
                node.stop_base()
                time.sleep(0.05)
        else:
            node.stop_base()

    except KeyboardInterrupt:
        print("\n[Interrupted] Stop publishing commands.")
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
