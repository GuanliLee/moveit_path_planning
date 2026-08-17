#!/usr/bin/env python3
"""Move AgileX Piper arm(s) slowly back to joint zero through ROS2 topics."""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


@dataclass(frozen=True)
class ArmSpec:
    name: str
    state_topic: str
    command_topic: str


PROFILES = {
    "two": [
        ArmSpec("left", "/puppet/joint_left", "/joint_left_states"),
        ArmSpec("right", "/puppet/joint_right", "/joint_right_states"),
    ],
    "double": [
        ArmSpec("left", "/joint_states_single_l", "/joint_states_gripper_l"),
        ArmSpec("right", "/joint_states_single_r", "/joint_states_gripper_r"),
    ],
    "single": [
        ArmSpec("single", "/joint_states_single", "/joint_states_gripper"),
    ],
}


class SlowReturnZeroNode(Node):
    def __init__(self, arms: Sequence[ArmSpec], speed_percent: float):
        super().__init__("piper_slow_return_zero")
        self.arms = list(arms)
        self.speed_percent = speed_percent
        self.current: Dict[str, List[float]] = {}
        self.command_publishers = {}

        for arm in self.arms:
            self.command_publishers[arm.name] = self.create_publisher(JointState, arm.command_topic, 10)
            self.create_subscription(
                JointState,
                arm.state_topic,
                lambda msg, arm_name=arm.name: self._joint_state_cb(arm_name, msg),
                10,
            )

    def _joint_state_cb(self, arm_name: str, msg: JointState) -> None:
        if len(msg.position) >= 7:
            self.current[arm_name] = [float(v) for v in msg.position[:7]]

    def wait_for_current_states(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        required = {arm.name for arm in self.arms}
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if required.issubset(self.current):
                return True
        return False

    def publish_positions(self, positions_by_arm: Dict[str, Sequence[float]]) -> None:
        stamp = self.get_clock().now().to_msg()
        velocity = [0.0] * 6 + [self.speed_percent]

        for arm in self.arms:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = list(JOINT_NAMES)
            msg.position = [float(v) for v in positions_by_arm[arm.name]]
            msg.velocity = list(velocity)
            msg.effort = [0.0] * 7
            self.command_publishers[arm.name].publish(msg)


def parse_target(value: str) -> List[float]:
    try:
        parts = [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--target must be 7 comma-separated floats") from exc
    if len(parts) != 7:
        raise argparse.ArgumentTypeError("--target must contain exactly 7 values")
    return parts


def detect_profile(node: Node) -> str:
    deadline = time.time() + 1.0
    topics = set()
    while time.time() < deadline:
        topics = {name for name, _types in node.get_topic_names_and_types()}
        if topics:
            break
        time.sleep(0.05)

    if {
        "/puppet/joint_left",
        "/puppet/joint_right",
        "/joint_left_states",
        "/joint_right_states",
    }.issubset(topics):
        return "two"
    if {
        "/joint_states_single_l",
        "/joint_states_single_r",
        "/joint_states_gripper_l",
        "/joint_states_gripper_r",
    }.issubset(topics):
        return "double"
    if {"/joint_states_single", "/joint_states_gripper"}.issubset(topics):
        return "single"
    return "two"


def format_positions(values: Sequence[float]) -> str:
    return "[" + ", ".join(f"{v:.4f}" for v in values) + "]"


def interpolate(start: Sequence[float], target: Sequence[float], alpha: float) -> List[float]:
    eased = 0.5 - 0.5 * math.cos(math.pi * alpha)
    return [s * (1.0 - eased) + t * eased for s, t in zip(start, target)]


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def build_targets(
    arms: Iterable[ArmSpec],
    current: Dict[str, Sequence[float]],
    base_target: Sequence[float],
    keep_gripper: bool,
) -> Dict[str, List[float]]:
    targets = {}
    for arm in arms:
        target = [float(v) for v in base_target]
        if keep_gripper:
            target[6] = float(current[arm.name][6])
        targets[arm.name] = target
    return targets


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=["auto", "two", "double", "single"],
        default="auto",
        help="Topic profile. auto supports the local start_two/start_double/start_single launch files.",
    )
    parser.add_argument("--arms", choices=["both", "left", "right"], default="both")
    parser.add_argument("--duration", type=float, default=12.0, help="Move duration in seconds.")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Command publish rate.")
    parser.add_argument(
        "--speed",
        type=float,
        default=30.0,
        help="Piper servo speed percent. The local driver clamps below 30 to 30.",
    )
    parser.add_argument(
        "--target",
        type=parse_target,
        default=parse_target("0,0,0,0,0,0,0"),
        help="Seven comma-separated targets: joint1..joint6 in rad, gripper in meters.",
    )
    parser.add_argument(
        "--keep-gripper",
        action="store_true",
        help="Keep each gripper at its current opening instead of moving it to target[6].",
    )
    parser.add_argument("--state-timeout", type=float, default=5.0)
    parser.add_argument("--settle", type=float, default=1.0, help="Keep publishing final target for this many seconds.")
    parser.add_argument("--yes", action="store_true", help="Skip the interactive confirmation.")
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")

    rclpy.init()
    probe_node = None
    node = None
    try:
        if args.profile == "auto":
            probe_node = Node("piper_slow_return_zero_probe")
            profile = detect_profile(probe_node)
            probe_node.destroy_node()
            probe_node = None
        else:
            profile = args.profile

        arms = list(PROFILES[profile])
        if profile != "single" and args.arms != "both":
            arms = [arm for arm in arms if arm.name == args.arms]
        elif profile == "single" and args.arms != "both":
            print("--arms is ignored for single profile")

        speed = clamp(args.speed, 30.0, 100.0)
        node = SlowReturnZeroNode(arms, speed)

        print(f"Profile: {profile}")
        for arm in arms:
            print(f"  {arm.name}: state={arm.state_topic}, command={arm.command_topic}")

        print("\nWaiting for current joint states...")
        if not node.wait_for_current_states(args.state_timeout):
            print("ERROR: current joint state was not received before timeout.")
            print("Check that piper_ros is running and that --profile matches the launch file.")
            print("Profiles: two=start_two_piper, double=start_double_piper, single=start_single_piper")
            return 2

        targets = build_targets(arms, node.current, args.target, args.keep_gripper)
        starts = {arm.name: list(node.current[arm.name]) for arm in arms}

        print("\nCurrent -> target:")
        for arm in arms:
            print(f"  {arm.name}: {format_positions(starts[arm.name])}")
            print(f"       -> {format_positions(targets[arm.name])}")
        print(f"\nDuration: {args.duration:.1f}s, publish rate: {args.rate_hz:.1f}Hz, speed: {speed:.0f}%")

        if not args.yes:
            answer = input("Type YES to start moving to zero: ").strip()
            if answer != "YES":
                print("Canceled.")
                return 1

        step_count = max(int(args.duration * args.rate_hz), 2)
        period = 1.0 / args.rate_hz
        last_command = starts

        print("\nMoving...")
        for step in range(step_count + 1):
            if not rclpy.ok():
                break
            alpha = step / step_count
            last_command = {
                arm.name: interpolate(starts[arm.name], targets[arm.name], alpha)
                for arm in arms
            }
            node.publish_positions(last_command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

        settle_deadline = time.time() + max(0.0, args.settle)
        while time.time() < settle_deadline and rclpy.ok():
            node.publish_positions(targets)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)

        print("Done.")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted. Holding the last command briefly; run piper_emergency_stop.py for disable-stop.")
        if node is not None:
            for _ in range(5):
                try:
                    node.publish_positions(last_command)  # type: ignore[name-defined]
                    time.sleep(0.05)
                except Exception:
                    break
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if probe_node is not None:
            probe_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
