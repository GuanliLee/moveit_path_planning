"""Pure helpers for OpenPI AgileX mobile inference clients."""

from __future__ import annotations

import numpy as np


REFERENCE_LEROBOT_DATASET = "/home/agilex/data/market_peachyogurt/lerobot/market_peachyogurt"
DEFAULT_PROMPT = "Target: Grape Juice. Pick the Grape Juice from the shelf and place it into the cart."

MOBILE_STATE_LIFT_INDEX = 14
MOBILE_STATE_BASE_SLICE = slice(15, 21)
MOBILE_ACTION_LIFT_INDEX = 14
MOBILE_ACTION_BASE_SLICE = slice(15, 18)


def normalize_gripper(value: float, gripper_min: float, gripper_max: float) -> float:
    return float(np.clip((value - gripper_min) / (gripper_max - gripper_min), 0.0, 1.0))


def denormalize_gripper(value: float, gripper_min: float, gripper_max: float) -> float:
    raw = gripper_min + float(np.clip(value, 0.0, 1.0)) * (gripper_max - gripper_min)
    return float(np.clip(raw, 0.0, gripper_max))


def piper_feedback_to_policy_state(
    left: np.ndarray,
    right: np.ndarray,
    gripper_min: float,
    gripper_max: float,
    state_gripper_unit: str,
) -> np.ndarray:
    state = np.zeros(14, dtype=np.float32)
    left = np.asarray(left, dtype=np.float32).reshape(-1)
    right = np.asarray(right, dtype=np.float32).reshape(-1)
    if left.size < 7 or right.size < 7:
        raise ValueError(f"Expected 7D left/right feedback, got {left.size}D/{right.size}D.")

    state[0:6] = left[:6]
    state[7:13] = right[:6]
    if state_gripper_unit == "normalized":
        state[6] = normalize_gripper(float(left[6]), gripper_min, gripper_max)
        state[13] = normalize_gripper(float(right[6]), gripper_min, gripper_max)
    elif state_gripper_unit == "meters":
        state[6] = float(left[6])
        state[13] = float(right[6])
    else:
        raise ValueError(f"Unsupported state gripper unit: {state_gripper_unit}")
    return state


def compose_policy_state(
    left: np.ndarray,
    right: np.ndarray,
    *,
    base_state: np.ndarray | None,
    lift_height: float | None,
    state_layout: str,
    gripper_min: float,
    gripper_max: float,
    state_gripper_unit: str,
) -> np.ndarray:
    arm_state = piper_feedback_to_policy_state(
        left,
        right,
        gripper_min,
        gripper_max,
        state_gripper_unit,
    )
    if state_layout == "aloha14":
        return arm_state
    if state_layout != "mobile21":
        raise ValueError(f"Unsupported state layout: {state_layout}")
    if base_state is None or lift_height is None:
        raise ValueError("mobile21 observation requires odometry and lift height.")
    base = np.asarray(base_state, dtype=np.float32).reshape(-1)
    if base.size != 6:
        raise ValueError(f"mobile21 base state must be 6D, got {base.size}D.")
    return np.concatenate([arm_state, np.asarray([lift_height], dtype=np.float32), base])


def clamp_value(value: float, limit: float) -> float:
    if limit <= 0:
        return float(value)
    return float(max(-limit, min(limit, value)))


def expected_action_dim(
    state_layout: str,
    *,
    enable_base: bool,
    enable_lift: bool,
    expected_action_dim_override: int = 0,
) -> int:
    if expected_action_dim_override > 0:
        return int(expected_action_dim_override)
    if enable_lift or enable_base or state_layout == "mobile21":
        return 18
    return 14


def validate_actions(actions: np.ndarray, reject_joint_abs: float, expected_width: int) -> np.ndarray:
    actions = np.asarray(actions, dtype=np.float64)
    if actions.ndim == 1:
        actions = actions[None, :]
    width = max(14, int(expected_width))
    if actions.ndim != 2 or actions.shape[1] < width:
        raise ValueError(f"Expected actions with shape [N, >={width}], got {actions.shape}.")
    if not np.all(np.isfinite(actions[:, :width])):
        raise ValueError("Policy returned NaN or inf.")

    joints = np.concatenate([actions[:, :6], actions[:, 7:13]], axis=1)
    max_abs = float(np.max(np.abs(joints)))
    if reject_joint_abs > 0 and max_abs > reject_joint_abs:
        raise ValueError(f"Policy joint action abs {max_abs:.3f} exceeds limit {reject_joint_abs:.3f}.")
    return actions


def policy_action_to_piper_commands(
    action: np.ndarray,
    *,
    gripper_min: float,
    gripper_max: float,
    action_gripper_unit: str,
) -> tuple[np.ndarray, np.ndarray]:
    action = np.asarray(action, dtype=np.float64).reshape(-1)
    if action.size < 14:
        raise ValueError(f"Expected action with at least 14 values, got {action.size}.")

    left = np.zeros(7, dtype=np.float64)
    right = np.zeros(7, dtype=np.float64)
    left[:6] = action[:6]
    right[:6] = action[7:13]
    if action_gripper_unit == "normalized":
        left[6] = denormalize_gripper(float(action[6]), gripper_min, gripper_max)
        right[6] = denormalize_gripper(float(action[13]), gripper_min, gripper_max)
    elif action_gripper_unit == "meters":
        left[6] = np.clip(float(action[6]), 0.0, gripper_max)
        right[6] = np.clip(float(action[13]), 0.0, gripper_max)
    else:
        raise ValueError(f"Unsupported action gripper unit: {action_gripper_unit}")
    return left, right


def policy_action_to_base_command(
    action: np.ndarray,
    *,
    base_scale: float,
    max_linear: float,
    max_angular: float,
    base_component_deadband: float = 0.0,
    swap_base_xy: bool = False,
    invert_base_x: bool = False,
    invert_base_y: bool = False,
    invert_base_wz: bool = False,
) -> np.ndarray:
    action = np.asarray(action, dtype=np.float64).reshape(-1)
    if action.size < MOBILE_ACTION_BASE_SLICE.stop:
        raise ValueError(f"Expected base action with at least 18 values, got {action.size}.")
    base_action = action[MOBILE_ACTION_BASE_SLICE]
    vx = clamp_value(float(base_action[0]) * base_scale, max_linear)
    vy = clamp_value(float(base_action[1]) * base_scale, max_linear)
    wz = clamp_value(float(base_action[2]) * base_scale, max_angular)

    if base_component_deadband > 0.0:
        if abs(vx) < base_component_deadband:
            vx = 0.0
        if abs(vy) < base_component_deadband:
            vy = 0.0
        if abs(wz) < base_component_deadband:
            wz = 0.0
    if swap_base_xy:
        vx, vy = vy, vx
    if invert_base_x:
        vx = -vx
    if invert_base_y:
        vy = -vy
    if invert_base_wz:
        wz = -wz
    return np.asarray([vx, vy, wz], dtype=np.float64)


def policy_action_to_lift_target(
    action: np.ndarray,
    *,
    lift_min_height: float | None,
    lift_max_height: float | None,
) -> float:
    action = np.asarray(action, dtype=np.float64).reshape(-1)
    if action.size < 18:
        raise ValueError(f"Expected lift action with at least 18 values, got {action.size}.")
    target = float(action[MOBILE_ACTION_LIFT_INDEX])
    if lift_min_height is not None:
        target = max(float(lift_min_height), target)
    if lift_max_height is not None:
        target = min(float(lift_max_height), target)
    return target


def clamp_command_step(
    target_left: np.ndarray,
    target_right: np.ndarray,
    last_left: np.ndarray,
    last_right: np.ndarray,
    *,
    max_joint_step: float,
    max_gripper_step: float,
    gripper_max: float,
) -> tuple[np.ndarray, np.ndarray]:
    target_left = np.asarray(target_left, dtype=np.float64)
    target_right = np.asarray(target_right, dtype=np.float64)
    last_left = np.asarray(last_left, dtype=np.float64)
    last_right = np.asarray(last_right, dtype=np.float64)

    limits = np.asarray([max_joint_step] * 6 + [max_gripper_step], dtype=np.float64)
    limits = np.where(limits > 0, limits, np.inf)
    next_left = last_left + np.clip(target_left - last_left, -limits, limits)
    next_right = last_right + np.clip(target_right - last_right, -limits, limits)
    next_left[6] = np.clip(next_left[6], 0.0, gripper_max)
    next_right[6] = np.clip(next_right[6], 0.0, gripper_max)
    return next_left, next_right


def base_commands_from_actions(
    actions: np.ndarray,
    *,
    base_scale: float,
    max_linear: float,
    max_angular: float,
    base_component_deadband: float = 0.0,
    swap_base_xy: bool = False,
    invert_base_x: bool = False,
    invert_base_y: bool = False,
    invert_base_wz: bool = False,
) -> np.ndarray:
    return np.stack(
        [
            policy_action_to_base_command(
                action,
                base_scale=base_scale,
                max_linear=max_linear,
                max_angular=max_angular,
                base_component_deadband=base_component_deadband,
                swap_base_xy=swap_base_xy,
                invert_base_x=invert_base_x,
                invert_base_y=invert_base_y,
                invert_base_wz=invert_base_wz,
            )
            for action in np.asarray(actions)
        ],
        axis=0,
    )


def effective_base_mask(
    base_commands: np.ndarray,
    *,
    base_linear_deadband: float,
    base_angular_deadband: float,
) -> np.ndarray:
    base_commands = np.asarray(base_commands, dtype=np.float64)
    if base_commands.size == 0:
        return np.asarray([], dtype=bool)
    return (
        (np.abs(base_commands[:, 0]) > base_linear_deadband)
        | (np.abs(base_commands[:, 1]) > base_linear_deadband)
        | (np.abs(base_commands[:, 2]) > base_angular_deadband)
    )


def base_peak(base_commands: np.ndarray) -> tuple[int, np.ndarray]:
    base_commands = np.asarray(base_commands, dtype=np.float64)
    base_norm = np.max(np.abs(base_commands), axis=1)
    peak_index = int(np.argmax(base_norm))
    return peak_index, base_commands[peak_index]
