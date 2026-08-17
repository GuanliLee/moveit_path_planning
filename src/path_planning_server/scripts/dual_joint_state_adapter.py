#!/usr/bin/env python3

"""Merge the two normalized Piper joint-state streams for MoveIt."""

from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 9))
JOINT_STATE_PUBLISH_RATE_HZ = 180.0


@dataclass(frozen=True)
class ArmState:
    position: Tuple[float, ...]
    velocity: Tuple[float, ...]
    effort: Tuple[float, ...]
    received_at: float


@dataclass(frozen=True)
class MergedState:
    name: Tuple[str, ...]
    position: Tuple[float, ...]
    velocity: Tuple[float, ...]
    effort: Tuple[float, ...]


def validate_arm_state(
    names: Sequence[str],
    position: Sequence[float],
    velocity: Sequence[float],
    effort: Sequence[float],
    received_at: float,
) -> ArmState:
    """Validate one bridge message and normalize it to joint1..joint8 order."""
    if len(names) != len(JOINT_NAMES) or set(names) != set(JOINT_NAMES):
        raise ValueError("joint names must contain joint1..joint8 exactly once")
    if len(position) != len(names):
        raise ValueError("position must contain one value for every joint")

    indices = {name: index for index, name in enumerate(names)}

    def ordered(values: Sequence[float]) -> Tuple[float, ...]:
        if len(values) != len(names):
            return ()
        return tuple(float(values[indices[name]]) for name in JOINT_NAMES)

    return ArmState(
        position=ordered(position),
        velocity=ordered(velocity),
        effort=ordered(effort),
        received_at=float(received_at),
    )


class DualJointStateMerger:
    """Pure state/timeout logic shared by the ROS node and unit tests."""

    def __init__(self, state_timeout: float = 1.0) -> None:
        if state_timeout <= 0.0:
            raise ValueError("state_timeout must be greater than zero")
        self.state_timeout = float(state_timeout)
        self._states: Dict[str, ArmState] = {}

    def update(
        self,
        side: str,
        names: Sequence[str],
        position: Sequence[float],
        velocity: Sequence[float],
        effort: Sequence[float],
        received_at: float,
    ) -> Optional[MergedState]:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._states[side] = validate_arm_state(
            names, position, velocity, effort, received_at
        )
        return self.merge(received_at)

    def merge(self, now: float) -> Optional[MergedState]:
        if "left" not in self._states or "right" not in self._states:
            return None

        left = self._states["left"]
        right = self._states["right"]
        if any(
            now < state.received_at
            or now - state.received_at > self.state_timeout
            for state in (left, right)
        ):
            return None

        names = tuple(f"left_{name}" for name in JOINT_NAMES) + tuple(
            f"right_{name}" for name in JOINT_NAMES
        )
        velocity = (
            left.velocity + right.velocity
            if left.velocity and right.velocity
            else ()
        )
        effort = left.effort + right.effort if left.effort and right.effort else ()
        return MergedState(
            name=names,
            position=left.position + right.position,
            velocity=velocity,
            effort=effort,
        )


try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:  # The pure merger remains importable without a ROS environment.
    rclpy = None
    Node = object
    JointState = None


class DualJointStateAdapter(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy and sensor_msgs are required to run this node")
        super().__init__("dual_joint_state_adapter")
        timeout = float(self.declare_parameter("state_timeout", 1.0).value)
        self._merger = DualJointStateMerger(timeout)
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointState,
            "/left/joint_states",
            lambda message: self._on_state("left", message),
            10,
        )
        self.create_subscription(
            JointState,
            "/right/joint_states",
            lambda message: self._on_state("right", message),
            10,
        )
        self._publish_timer = self.create_timer(
            1.0 / JOINT_STATE_PUBLISH_RATE_HZ,
            self._publish,
        )

    def _on_state(self, side: str, message: JointState) -> None:
        now = self.get_clock().now()
        try:
            self._merger.update(
                side,
                message.name,
                message.position,
                message.velocity,
                message.effort,
                now.nanoseconds / 1_000_000_000.0,
            )
        except ValueError as error:
            self.get_logger().warning(f"Ignoring invalid {side} joint state: {error}")

    def _publish(self) -> None:
        now = self.get_clock().now()
        merged = self._merger.merge(now.nanoseconds / 1_000_000_000.0)
        if merged is None:
            return

        output = JointState()
        output.header.stamp = now.to_msg()
        output.name = list(merged.name)
        output.position = list(merged.position)
        output.velocity = list(merged.velocity)
        output.effort = list(merged.effort)
        self._publisher.publish(output)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy and sensor_msgs are required to run this node")
    rclpy.init(args=args)
    node = DualJointStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
