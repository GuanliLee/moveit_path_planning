from __future__ import annotations

from typing import Any


TARGET_OPTIONS = (
    "Coca-Cola",
    "Daily C Grape Juice",
    "Guangming Probiotic Milk",
    "Daily C Orange Juice",
    "AD Calcium Milk",
    "Robuk Velvet Latte",
    "Aojiru",
    "HK Orange Fanta",
    "Taro Milk",
    "Yili Peach Yogurt",
    "NEVER Coconut Latte",
    "Yili Strawberry Yogurt",
    "Wanglaoji",
    "Sprite",
    "Yakult",
    "Dahongpao Milk Tea",
)

GRASP_PROMPT_TOKEN = "{collection_grasp_targets}"
PLACE_PROMPT_TOKEN = "{collection_place_targets}"


def validate_target(value: Any) -> str:
    target = str(value or "").strip()
    if target and target not in TARGET_OPTIONS:
        raise ValueError(f"unsupported collection target: {target}")
    return target


def grasp_instruction(left: Any, right: Any) -> str:
    left_target = validate_target(left)
    right_target = validate_target(right)
    parts: list[str] = []
    if left_target:
        parts.append(f"Grasp {left_target} with the left hand.")
    if right_target:
        parts.append(f"Grasp {right_target} with the right hand.")
    return " and ".join(parts)


def place_instruction(left: Any, right: Any) -> str:
    left_target = validate_target(left)
    right_target = validate_target(right)
    if left_target and right_target:
        return (
            f"Place {left_target} into the cart with the left hand, "
            f"then place {right_target} into the cart with the right hand."
        )
    if left_target:
        return f"Place {left_target} into the cart with the left hand."
    if right_target:
        return f"Place {right_target} into the cart with the right hand."
    return ""


def render_target_prompts(text: str, left: Any, right: Any) -> str:
    rendered = str(text or "")
    rendered = rendered.replace(GRASP_PROMPT_TOKEN, grasp_instruction(left, right))
    rendered = rendered.replace(PLACE_PROMPT_TOKEN, place_instruction(left, right))
    return rendered


def stage_instruction_templates() -> list[str]:
    return [
        "Move chassis to shelf.",
        GRASP_PROMPT_TOKEN,
        "Move chassis to cart.",
        PLACE_PROMPT_TOKEN,
        "Move chassis to start.",
    ]
