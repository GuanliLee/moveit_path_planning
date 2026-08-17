#!/usr/bin/env python3
"""Replay hdf5_episodes aligned_joints.h5 to arms, mobile base, and lift."""

from __future__ import annotations

import argparse
import dataclasses
import sys
import time
from pathlib import Path

import h5py
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


@dataclasses.dataclass
class EpisodeData:
    h5_path: Path
    timestamps: np.ndarray
    arms: np.ndarray | None
    base_cmd: np.ndarray | None
    lift_height: np.ndarray | None
    arm_source: str | None
    base_source: str | None
    lift_source: str | None


def natural_key(path: Path) -> list[object]:
    import re

    out: list[object] = []
    for part in re.split(r"(\d+)", path.name):
        if not part:
            continue
        out.append(int(part) if part.isdigit() else part.lower())
    return out


def numeric_hdf5_keys(keys) -> list[str]:
    return sorted((str(key) for key in keys if str(key).isdigit()), key=lambda key: int(key))


def resolve_h5_path(path_text: str, episode: str | None = None) -> Path:
    path = Path(path_text).expanduser().resolve()
    if path.is_file():
        return path
    if not path.is_dir():
        raise FileNotFoundError(path)

    direct_candidates = (
        path / "states" / "aligned_joints.h5",
        path / "states" / "aligned_joints.hdf5",
        path / "aligned_joints.h5",
        path / "aligned_joints.hdf5",
    )
    for candidate in direct_candidates:
        if candidate.is_file():
            return candidate

    episode_dirs = sorted(
        [item for item in path.iterdir() if item.is_dir() and (item / "states" / "aligned_joints.h5").is_file()],
        key=natural_key,
    )
    if not episode_dirs:
        raise FileNotFoundError(f"No episode*/states/aligned_joints.h5 found under: {path}")

    selected = "0" if episode in (None, "") else str(episode)
    if selected.isdigit():
        index = int(selected)
        if index < 0 or index >= len(episode_dirs):
            raise IndexError(f"Episode index {index} out of range, found {len(episode_dirs)} episodes")
        return episode_dirs[index] / "states" / "aligned_joints.h5"

    by_name = {item.name: item for item in episode_dirs}
    if selected not in by_name:
        raise FileNotFoundError(
            f"Episode {selected!r} not found under {path}. "
            f"Available: {', '.join(item.name for item in episode_dirs[:20])}"
        )
    return by_name[selected] / "states" / "aligned_joints.h5"


def list_episodes(path_text: str) -> None:
    root = Path(path_text).expanduser().resolve()
    if not root.is_dir():
        h5_path = resolve_h5_path(path_text)
        print_episode_info(0, h5_path)
        return
    episode_dirs = sorted(
        [item for item in root.iterdir() if item.is_dir() and (item / "states" / "aligned_joints.h5").is_file()],
        key=natural_key,
    )
    if not episode_dirs:
        print_episode_info(0, resolve_h5_path(path_text))
        return
    for index, episode_dir in enumerate(episode_dirs):
        print_episode_info(index, episode_dir / "states" / "aligned_joints.h5")


def print_episode_info(index: int, h5_path: Path) -> None:
    with h5py.File(h5_path, "r") as file_obj:
        frame_keys = numeric_hdf5_keys(file_obj.keys())
        if not frame_keys:
            print(f"{index}: {h5_path} frames=0")
            return
        ts = np.array([timestamp_seconds(file_obj[key]["main_timestamp"][()]) for key in frame_keys])
    duration = float(ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    print(f"{index}: {h5_path.parent.parent.name} frames={len(frame_keys)} duration={duration:.2f}s {h5_path}")


def timestamp_seconds(raw_value) -> float:
    value = float(np.asarray(raw_value).reshape(-1)[0])
    magnitude = abs(value)
    if magnitude > 1e17:
        return value / 1e9
    if magnitude > 1e14:
        return value / 1e6
    if magnitude > 1e11:
        return value / 1e3
    return value


def read_vector(group, path: str, dim: int | None = None) -> np.ndarray | None:
    if path not in group:
        return None
    value = np.asarray(group[path][()], dtype=np.float64).reshape(-1)
    if dim is not None and len(value) < dim:
        raise ValueError(f"{group.name}/{path} expected at least {dim} values, got {value.shape}")
    return value


def select_base_path(file_obj, first_key: str, source: str) -> tuple[str, str] | None:
    first = file_obj[first_key]
    if source in ("auto", "action") and "action/robot/velocity" in first:
        return "action/robot/velocity", "action/robot/velocity"
    if source == "action":
        raise ValueError("Requested --base-source action, but action/robot/velocity is missing")
    if source in ("auto", "state"):
        if "state/robot/velocity" in first:
            return "state/robot/velocity", "state/robot/velocity"
        if "state/robot/base_state" in first:
            return "state/robot/base_state", "state/robot/base_state[3:6]"
    if source == "state":
        raise ValueError("Requested --base-source state, but state/robot/velocity/base_state is missing")
    return None


def select_lift_path(file_obj, first_key: str, source: str) -> tuple[str, str] | None:
    first = file_obj[first_key]
    if source in ("auto", "action") and "action/waist/position" in first:
        return "action/waist/position", "action/waist/position"
    if source == "action":
        raise ValueError("Requested --lift-source action, but action/waist/position is missing")
    if source in ("auto", "state") and "state/waist/position" in first:
        return "state/waist/position", "state/waist/position"
    if source == "state":
        raise ValueError("Requested --lift-source state, but state/waist/position is missing")
    return None


def load_episode(
    h5_path: Path,
    *,
    arm_source: str,
    base_source: str,
    lift_source: str,
    start_frame: int,
    end_frame: int | None,
) -> EpisodeData:
    with h5py.File(h5_path, "r") as file_obj:
        frame_keys = numeric_hdf5_keys(file_obj.keys())
        if not frame_keys:
            raise ValueError(f"{h5_path}: missing numeric frame groups")

        start = max(0, int(start_frame))
        end = len(frame_keys) if end_frame is None else min(len(frame_keys), int(end_frame))
        if start >= end:
            raise ValueError(f"Empty frame range: start={start}, end={end}")
        frame_keys = frame_keys[start:end]
        first_key = frame_keys[0]

        arm_path = f"{arm_source}/joint/position"
        if arm_path not in file_obj[first_key]:
            raise ValueError(f"{h5_path}: missing {first_key}/{arm_path}")
        base_selection = select_base_path(file_obj, first_key, base_source)
        lift_selection = select_lift_path(file_obj, first_key, lift_source)

        timestamps = np.zeros(len(frame_keys), dtype=np.float64)
        arms = np.zeros((len(frame_keys), 14), dtype=np.float64)
        base_cmd = np.zeros((len(frame_keys), 3), dtype=np.float64) if base_selection else None
        lift_height = np.zeros(len(frame_keys), dtype=np.float64) if lift_selection else None

        for index, key in enumerate(frame_keys):
            group = file_obj[key]
            timestamps[index] = timestamp_seconds(group["main_timestamp"][()])

            arm_values = read_vector(group, arm_path, 14)
            arms[index] = arm_values[:14]

            if base_selection is not None and base_cmd is not None:
                base_path, base_name = base_selection
                base_values = read_vector(group, base_path, 3 if base_path.endswith("velocity") else 6)
                if base_name.endswith("[3:6]"):
                    base_cmd[index] = base_values[3:6]
                else:
                    base_cmd[index] = base_values[:3]

            if lift_selection is not None and lift_height is not None:
                lift_path, _ = lift_selection
                lift_values = read_vector(group, lift_path, 1)
                lift_height[index] = float(lift_values[0])

    keep = np.concatenate(([True], np.diff(timestamps) > 1e-9))
    if not np.all(keep):
        removed = int(len(keep) - np.sum(keep))
        print(f"[Warn] Drop {removed} non-increasing timestamp frames from {h5_path}")
    timestamps = timestamps[keep]
    timestamps = timestamps - timestamps[0]
    arms = arms[keep]
    if base_cmd is not None:
        base_cmd = base_cmd[keep]
    if lift_height is not None:
        lift_height = lift_height[keep]

    return EpisodeData(
        h5_path=h5_path,
        timestamps=timestamps,
        arms=arms,
        base_cmd=base_cmd,
        lift_height=lift_height,
        arm_source=arm_path,
        base_source=base_selection[1] if base_selection else None,
        lift_source=lift_selection[1] if lift_selection else None,
    )


def clamp(value: float, limit: float) -> float:
    if limit <= 0:
        return float(value)
    return float(max(-limit, min(limit, value)))


def sample_at_time(values: np.ndarray | None, timestamps: np.ndarray, target_time: float, mode: str) -> np.ndarray | None:
    if values is None:
        return None
    if target_time <= timestamps[0]:
        return values[0]
    if target_time >= timestamps[-1]:
        return values[-1]
    right = int(np.searchsorted(timestamps, target_time, side="right"))
    left = max(0, right - 1)
    right = min(right, len(timestamps) - 1)
    if mode == "hold":
        return values[left]
    t0 = float(timestamps[left])
    t1 = float(timestamps[right])
    if t1 <= t0:
        return values[right]
    alpha = (target_time - t0) / (t1 - t0)
    return values[left] * (1.0 - alpha) + values[right] * alpha


def replay_schedule(timestamps: np.ndarray, publish_hz: float, skip: int) -> list[float]:
    if len(timestamps) == 0:
        return []
    if publish_hz > 0:
        step = 1.0 / float(publish_hz)
        count = max(1, int(np.floor(float(timestamps[-1]) / step)) + 1)
        schedule = [i * step for i in range(count)]
        if schedule[-1] < float(timestamps[-1]):
            schedule.append(float(timestamps[-1]))
        return schedule
    indices = list(range(0, len(timestamps), skip))
    if indices[-1] != len(timestamps) - 1:
        indices.append(len(timestamps) - 1)
    return [float(timestamps[index]) for index in indices]


class MobileReplayNode(Node):
    def __init__(self, args, enable_arms: bool, enable_base: bool, enable_lift: bool):
        super().__init__("agibot_mobile_replay_to_robot")
        self.enable_arms = enable_arms
        self.enable_base = enable_base
        self.enable_lift = enable_lift
        self.cur_left = None
        self.cur_right = None
        self.cur_lift_height = None

        self.pub_left = None
        self.pub_right = None
        if self.enable_arms:
            self.pub_left = self.create_publisher(JointState, args.left_command_topic, 10)
            self.pub_right = self.create_publisher(JointState, args.right_command_topic, 10)
            self.create_subscription(
                JointState,
                args.left_state_topic,
                lambda msg: setattr(self, "cur_left", np.array(msg.position[:7], dtype=np.float64)),
                10,
            )
            self.create_subscription(
                JointState,
                args.right_state_topic,
                lambda msg: setattr(self, "cur_right", np.array(msg.position[:7], dtype=np.float64)),
                10,
            )

        self.pub_base = None
        if self.enable_base:
            self.pub_base = self.create_publisher(Twist, args.base_command_topic, 10)

        self.lift_client = None
        if self.enable_lift:
            if LiftMotorMsg is None or LiftMotorSrv is None:
                raise RuntimeError(
                    "Lift message/service package is not available. "
                    "Source the lift driver workspace or run with --disable-lift."
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

    def publish_base(self, vel_xyz: np.ndarray, max_linear: float, max_angular: float, scale: float) -> None:
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
    for index in range(steps + 1):
        alpha = index / steps
        blend = 0.5 - 0.5 * np.cos(np.pi * alpha)
        left = start_l * (1.0 - blend) + target_l * blend
        right = start_r * (1.0 - blend) + target_r * blend
        node.publish_arms(left, right, vel_pct)
        node.stop_base()
        time.sleep(1.0 / rate_hz)
    print(">>> Initial arm pose reached")


def print_summary(data: EpisodeData, args, base_scale: float) -> None:
    print(f"Episode HDF5: {data.h5_path}")
    print(f"  frames:       {len(data.timestamps)}")
    print(f"  duration:     {data.timestamps[-1]:.2f}s")
    if len(data.timestamps) > 1:
        diffs = np.diff(data.timestamps)
        print(
            f"  source rate:  mean={(len(data.timestamps) - 1) / data.timestamps[-1]:.2f}Hz, "
            f"median={1.0 / np.median(diffs):.2f}Hz, max_dt={np.max(diffs):.3f}s"
        )
    print(f"  replay:       rate={args.rate}x, publish_hz={args.publish_hz}, base_scale={base_scale:.3f}")
    if data.arms is not None:
        print(f"  arms:         {data.arms.shape}, source={data.arm_source}")
        print(f"  first left:   {np.array2string(data.arms[0, :7], precision=3, suppress_small=True)}")
        print(f"  first right:  {np.array2string(data.arms[0, 7:14], precision=3, suppress_small=True)}")
    if data.base_cmd is not None:
        max_base = np.max(np.abs(data.base_cmd), axis=0)
        print(f"  base command: {data.base_cmd.shape}, source={data.base_source}")
        print(f"  max base abs: {np.array2string(max_base, precision=3)} [vx_cmd, vy_cmd, wz_cmd]")
    if data.lift_height is not None:
        print(
            f"  lift height:  {data.lift_height.shape}, source={data.lift_source}, "
            f"first={data.lift_height[0]:.0f}mm, last={data.lift_height[-1]:.0f}mm"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("path", help="hdf5_episodes root, episode dir, or aligned_joints.h5")
    parser.add_argument("--episode", default="0", help="Episode index/name when path is a hdf5_episodes root")
    parser.add_argument("--list", action="store_true", help="List episodes under path and exit")
    parser.add_argument("--source", choices=("action", "state"), default="action",
                        help="Arm trajectory source")
    parser.add_argument("--base-source", choices=("auto", "action", "state"), default="action",
                        help="Base command source")
    parser.add_argument("--lift-source", choices=("auto", "action", "state"), default="action",
                        help="Lift command source")
    parser.add_argument("--base-interp", choices=("hold", "linear"), default="linear")
    parser.add_argument("--arm-interp", choices=("hold", "linear"), default="linear")
    parser.add_argument("--rate", type=float, default=0.5, help="Replay speed multiplier")
    parser.add_argument("--publish-hz", type=float, default=30.0,
                        help="Uniform command publish rate; <=0 publishes source frames only")
    parser.add_argument("--skip", type=int, default=1,
                        help="Publish every N source frames when --publish-hz <= 0")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--vel", type=float, default=70.0, help="Arm tracking velocity percentage")
    parser.add_argument("--init-vel", type=float, default=35.0)
    parser.add_argument("--init-duration", type=float, default=5.0)
    parser.add_argument("--base-scale", type=float, default=None,
                        help="Scale recorded base velocity. Default follows --rate")
    parser.add_argument("--max-linear", type=float, default=0.3,
                        help="Clamp abs(linear x/y), m/s. <=0 disables clamp")
    parser.add_argument("--max-angular", type=float, default=0.8,
                        help="Clamp abs(angular z), rad/s. <=0 disables clamp")
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
    parser.add_argument("--dry-run", action="store_true", help="Load data and print summary only")
    parser.add_argument("--no-confirm", action="store_true", help="Skip safety confirmations")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list:
        list_episodes(args.path)
        return 0
    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.skip <= 0:
        raise SystemExit("--skip must be positive")
    if args.publish_hz > 200:
        raise SystemExit("--publish-hz must be <= 200")

    h5_path = resolve_h5_path(args.path, args.episode)
    data = load_episode(
        h5_path,
        arm_source=args.source,
        base_source=args.base_source,
        lift_source=args.lift_source,
        start_frame=args.start_frame,
        end_frame=args.end_frame,
    )
    base_scale = args.rate if args.base_scale is None else args.base_scale

    if args.disable_arms:
        data.arms = None
    if args.disable_base:
        data.base_cmd = None
    if args.disable_lift:
        data.lift_height = None

    print_summary(data, args, base_scale)
    if data.base_cmd is None and not args.disable_base:
        print("[Warn] Missing base command; base replay disabled")
    if data.lift_height is None and not args.disable_lift:
        print("[Warn] Missing lift height; lift replay disabled")

    if args.dry_run:
        print("\nDry run only; no ROS commands published.")
        return 0

    print("\nSafety:")
    print("  - Start piper_ros, base driver, and lift driver first")
    print("  - Keep the workspace clear")
    print("  - Ctrl+C sends zero /cmd_vel and exits")
    if not args.no_confirm:
        answer = input("\nPress Enter to continue, or q to cancel: ").strip().lower()
        if answer == "q":
            print("Canceled.")
            return 0

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
            print("\n>>> Waiting for current arm states ...")
            if not node.wait_for_current_arms():
                raise RuntimeError("Did not receive current left/right arm JointState within 5s")
            print(f"  current left : {np.array2string(node.cur_left, precision=3, suppress_small=True)}")
            print(f"  current right: {np.array2string(node.cur_right, precision=3, suppress_small=True)}")

        if data.lift_height is not None:
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
        if data.arms is not None:
            smooth_move_arms(
                node,
                data.arms[0, :7],
                data.arms[0, 7:14],
                duration=args.init_duration,
                vel_pct=args.init_vel,
            )

        if not args.no_confirm:
            answer = input("\n>>> Ready. Press Enter to start replay, or q to cancel: ").strip().lower()
            if answer == "q":
                print("Canceled.")
                return 0

        schedule = replay_schedule(data.timestamps, args.publish_hz, args.skip)
        mode = f"{args.publish_hz:.1f}Hz interpolated" if args.publish_hz > 0 else f"source frames, skip={args.skip}"
        print(f"\n>>> Start replay: {len(schedule)} commands, rate={args.rate}x, mode={mode}")
        start_time = time.monotonic()
        last_log_time = start_time
        published = 0
        late = 0
        last_lift_height = None
        last_lift_time = 0.0

        for index, source_time in enumerate(schedule):
            if not rclpy.ok():
                break
            target_elapsed = source_time / args.rate
            now_elapsed = time.monotonic() - start_time
            sleep_s = target_elapsed - now_elapsed
            if sleep_s > 0.0005:
                time.sleep(sleep_s)
            elif sleep_s < -0.05:
                late += 1
                continue

            if data.arms is not None:
                arms = sample_at_time(data.arms, data.timestamps, source_time, args.arm_interp)
                node.publish_arms(arms[:7], arms[7:14], args.vel)
            if data.base_cmd is not None:
                base_cmd = sample_at_time(data.base_cmd, data.timestamps, source_time, args.base_interp)
                node.publish_base(base_cmd, args.max_linear, args.max_angular, base_scale)
            if data.lift_height is not None:
                height = float(sample_at_time(data.lift_height, data.timestamps, source_time, "linear"))
                now = time.monotonic()
                changed = last_lift_height is None or abs(height - last_lift_height) >= args.lift_min_delta
                enough_time = now - last_lift_time >= args.lift_min_period
                if changed and enough_time:
                    node.send_lift_height(height, args.lift_mode)
                    last_lift_height = height
                    last_lift_time = now

            rclpy.spin_once(node, timeout_sec=0.0)
            published += 1

            now = time.monotonic()
            if now - last_log_time > 0.5:
                pct = (index + 1) / len(schedule) * 100.0
                sys.stdout.write(
                    f"\r  progress: {index + 1}/{len(schedule)} ({pct:5.1f}%) "
                    f"published={published} late={late}"
                )
                sys.stdout.flush()
                last_log_time = now

        sys.stdout.write(
            f"\r  progress: {len(schedule)}/{len(schedule)} (100.0%) "
            f"published={published} late={late}\n"
        )
        print(f">>> Replay done in {time.monotonic() - start_time:.2f}s")

        if data.arms is not None:
            print(">>> Hold final arm pose for 1s")
            deadline = time.monotonic() + 1.0
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
