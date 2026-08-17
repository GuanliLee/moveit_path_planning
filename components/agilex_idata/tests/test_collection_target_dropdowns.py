from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
TARGETS_MODULE_PATH = REPO_ROOT / "scripts" / "collection" / "collection_targets.py"
METADATA_MODULE_PATH = REPO_ROOT / "scripts" / "collection" / "collection_episode_metadata.py"
WEB_CONTROLLER_PATH = REPO_ROOT / "scripts" / "collection" / "collect_mobile_episode_web.sh"
STAGED_LAUNCHER_PATH = REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_staged.sh"
FIXED_LAUNCHER_PATH = REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_fixed_stage.sh"


def load_targets_module():
    spec = importlib.util.spec_from_file_location("collection_targets_test", TARGETS_MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_metadata_module():
    spec = importlib.util.spec_from_file_location("collection_episode_metadata_targets_test", METADATA_MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_target_catalog_has_exact_approved_names_and_order():
    module = load_targets_module()

    assert module.TARGET_OPTIONS == (
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


def test_target_validation_accepts_empty_and_approved_values_only():
    module = load_targets_module()

    assert module.validate_target(None) == ""
    assert module.validate_target("  ") == ""
    assert module.validate_target("  Daily C Grape Juice  ") == "Daily C Grape Juice"
    with pytest.raises(ValueError, match="unsupported collection target"):
        module.validate_target("Apple")


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        ("", "", ""),
        ("Daily C Grape Juice", "", "Grasp Daily C Grape Juice with the left hand."),
        ("", "Daily C Grape Juice", "Grasp Daily C Grape Juice with the right hand."),
        (
            "Coca-Cola",
            "Sprite",
            "Grasp Coca-Cola with the left hand. and Grasp Sprite with the right hand.",
        ),
    ],
)
def test_grasp_prompt_covers_empty_single_and_dual_hands(left, right, expected):
    module = load_targets_module()

    assert module.grasp_instruction(left, right) == expected


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        ("", "", ""),
        ("Daily C Grape Juice", "", "Place Daily C Grape Juice into the cart with the left hand."),
        ("", "Daily C Grape Juice", "Place Daily C Grape Juice into the cart with the right hand."),
        (
            "Coca-Cola",
            "Sprite",
            "Place Coca-Cola into the cart with the left hand, then place Sprite into the cart with the right hand.",
        ),
    ],
)
def test_place_prompt_covers_empty_single_and_dual_hands(left, right, expected):
    module = load_targets_module()

    assert module.place_instruction(left, right) == expected


def test_stage_templates_keep_other_stages_and_render_runtime_targets():
    module = load_targets_module()

    templates = module.stage_instruction_templates()
    assert templates == [
        "Move chassis to shelf.",
        module.GRASP_PROMPT_TOKEN,
        "Move chassis to cart.",
        module.PLACE_PROMPT_TOKEN,
        "Move chassis to start.",
    ]
    rendered = [
        module.render_target_prompts(text, "Coca-Cola", "Sprite")
        for text in templates
    ]
    assert rendered == [
        "Move chassis to shelf.",
        "Grasp Coca-Cola with the left hand. and Grasp Sprite with the right hand.",
        "Move chassis to cart.",
        "Place Coca-Cola into the cart with the left hand, then place Sprite into the cart with the right hand.",
        "Move chassis to start.",
    ]


@pytest.mark.parametrize(
    ("left", "right", "expected"),
    [
        ("", "", ""),
        ("Coca-Cola", "", "Grasp Coca-Cola with the left hand."),
        ("", "Sprite", "Grasp Sprite with the right hand."),
        (
            "Coca-Cola",
            "Sprite",
            "Grasp Coca-Cola with the left hand. and Grasp Sprite with the right hand.",
        ),
    ],
)
def test_metadata_prepare_uses_explicit_hand_targets(tmp_path, left, right, expected):
    module = load_metadata_module()

    module.prepare_episode(
        data_dir=tmp_path,
        episode=7,
        targets=[target for target in (left, right) if target],
        left_target=left,
        right_target=right,
    )

    info_path = tmp_path / "episode7" / "episode7_0_info.json"
    payload = json.loads(info_path.read_text(encoding="utf-8"))
    assert payload["mark"]["full-instructions-en"] == [expected]
    assert payload["mark"]["full-instructions-zh"] == [expected]


def test_web_controller_renders_left_and_right_target_dropdowns():
    source = WEB_CONTROLLER_PATH.read_text(encoding="utf-8")

    assert "左手目标物品" in source
    assert "右手目标物品" in source
    assert '<select id="target_bottle_a">' in source
    assert '<select id="target_bottle_b">' in source
    assert '<input id="target_bottle_a" type="text">' not in source
    assert '<input id="target_bottle_b" type="text">' not in source


def test_web_controller_exposes_and_validates_target_options():
    source = WEB_CONTROLLER_PATH.read_text(encoding="utf-8")

    assert '"target_options": json.loads(env("STATUS_TARGET_OPTIONS_JSON", "[]"))' in source
    assert "from collection_targets import TARGET_OPTIONS" in source
    assert "validate_target" in source
    assert '"TARGET_BOTTLE_A": validate_target(text("target_bottle_a", current_target_a))' in source
    assert '"TARGET_BOTTLE_B": validate_target(text("target_bottle_b", current_target_b))' in source


def test_web_controller_persists_target_changes_before_automatic_collection():
    source = WEB_CONTROLLER_PATH.read_text(encoding="utf-8")

    assert "async function persistTargetSelection()" in source
    assert "async function waitForTargetSelection" in source
    assert "el.target_bottle_a.addEventListener('change', persistTargetSelection);" in source
    assert "el.target_bottle_b.addEventListener('change', persistTargetSelection);" in source


def test_web_controller_passes_hand_targets_and_renders_stage_tokens():
    source = WEB_CONTROLLER_PATH.read_text(encoding="utf-8")

    assert 'METADATA_HAND_TARGET_ARGS=(--left-target "${TARGET_BOTTLE_A}" --right-target "${TARGET_BOTTLE_B}")' in source
    assert '"${METADATA_HAND_TARGET_ARGS[@]}"' in source
    assert "from collection_targets import render_target_prompts" in source
    assert "text = render_target_prompts(text, left_target, right_target)" in source
    assert "segment_en[i] if i < len(segment_en) else f\"Stage {i + 1}\"" in source


def test_web_controller_defaults_both_targets_to_empty():
    source = WEB_CONTROLLER_PATH.read_text(encoding="utf-8")

    assert 'TARGET_BOTTLE_A=""' in source
    assert 'TARGET_BOTTLE_B=""' in source


def test_launcher_help_removes_terminal_target_arguments():
    for script in (STAGED_LAUNCHER_PATH, FIXED_LAUNCHER_PATH):
        result = subprocess.run(
            ["bash", str(script), "--help"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        assert "left_target" not in result.stdout
        assert "right_target" not in result.stdout
        assert "--left-target" not in result.stdout
        assert "--right-target" not in result.stdout


def test_staged_launcher_uses_empty_runtime_targets_and_template_prompts():
    source = STAGED_LAUNCHER_PATH.read_text(encoding="utf-8")

    assert "LEFT_TARGET_ARG" not in source
    assert "RIGHT_TARGET_ARG" not in source
    assert "--left-target" not in source
    assert "--right-target" not in source
    assert 'TARGET_BOTTLE_A=""' in source
    assert 'TARGET_BOTTLE_B=""' in source
    assert "from collection_targets import stage_instruction_templates" in source
    assert "stages = stage_instruction_templates()" in source


def test_fixed_launcher_forwards_only_supported_staged_arguments(tmp_path):
    stub = tmp_path / "staged_stub.sh"
    output = tmp_path / "args.txt"
    stub.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$*\" > \"${FIXED_STAGE_TEST_OUTPUT}\"\n",
        encoding="utf-8",
    )
    stub.chmod(0o755)
    env = os.environ.copy()
    env["COLLECT_MOBILE_PIPELINE_WEB_STAGED_SCRIPT"] = str(stub)
    env["FIXED_STAGE_TEST_OUTPUT"] = str(output)

    subprocess.run(
        ["bash", str(FIXED_LAUNCHER_PATH), "/tmp/dataset", "7", "--grade", "A"],
        cwd=REPO_ROOT,
        env=env,
        check=True,
    )

    assert output.read_text(encoding="utf-8") == "/tmp/dataset 7 --grade A\n"


def test_automation_bridge_mode_config_does_not_overwrite_runtime_targets():
    source = STAGED_LAUNCHER_PATH.read_text(encoding="utf-8")
    payload_function = source.split("def collection_config_payload", 1)[1].split(
        "def configure_collection_for_mode",
        1,
    )[0]

    assert '"target_bottle_a"' not in payload_function
    assert '"target_bottle_b"' not in payload_function
