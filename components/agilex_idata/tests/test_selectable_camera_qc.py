from __future__ import annotations

from pathlib import Path
import importlib.util
import json
import os
import re
import subprocess
import sys

import h5py
import numpy as np
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
COLLECTION_SCRIPTS = REPO_ROOT / "scripts" / "collection"
GENERIC_ENTRY = COLLECTION_SCRIPTS / "collect_mobile_pipeline_qc_web.sh"
SELECTABLE_ENTRY = COLLECTION_SCRIPTS / "collect_mobile_pipeline_qc_web_four_camera.sh"
DRINK_GRASP_ENTRY = COLLECTION_SCRIPTS / "collect_drink_grasp_pipeline_qc_web.sh"
PIPELINE_ROOT = REPO_ROOT / "scripts" / "embodied_data_pipeline-main"
MCAP_SCRIPTS = PIPELINE_ROOT / "mcap_conversion" / "scripts"
PIPELINE_SCRIPTS = PIPELINE_ROOT / "scripts"
FOUR_CAMERA_PROFILE = PIPELINE_ROOT / "robot_profiles" / "aloha_four_camera.yaml"
FOUR_CAMERA_TOPIC_YAML = (
    PIPELINE_ROOT / "mcap_conversion" / "topic_configs" / "aloha_four_camera_data_params.yaml"
)
DRINK_GRASP_TOPIC_YAML = (
    PIPELINE_ROOT
    / "mcap_conversion"
    / "topic_configs"
    / "drink_grasp_three_camera_data_params.yaml"
)
PIPELINE_WEB_APP = PIPELINE_SCRIPTS / "pipeline_web_app.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def extract_javascript_function(html: str, name: str) -> str:
    match = re.search(rf"(?:async\s+)?function\s+{re.escape(name)}\s*\(", html)
    assert match, f"missing JavaScript function {name}"
    brace_start = html.index("{", match.end())
    depth = 0
    quote = ""
    escaped = False
    for index in range(brace_start, len(html)):
        char = html[index]
        if quote:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == quote:
                quote = ""
            continue
        if char in {'"', "'", "`"}:
            quote = char
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return html[match.start() : index + 1]
    raise AssertionError(f"unterminated JavaScript function {name}")


def run_camera_ui_bootstrap(html: str, config: dict) -> dict:
    functions = "\n".join(
        extract_javascript_function(html, name)
        for name in ("syncCameraVariantControls", "bootstrapCameraVariantControls")
    )
    script = f"""
const makeClassList = (initial) => {{
  const values = new Set(initial);
  return {{
    contains: value => values.has(value),
    toggle: (value, force) => force ? values.add(value) : values.delete(value),
  }};
}};
const elements = {{
  cameraVariantControls: {{classList: makeClassList(["hidden"])}},
  cameraVariantHint: {{classList: makeClassList(["hidden"])}},
  cameraCount: {{value: ""}},
  headCameraSource: {{value: "", disabled: false}},
  profile: {{value: "", readOnly: false}},
  robotType: {{value: "aloha"}},
}};
globalThis.document = {{getElementById: id => elements[id]}};
globalThis.fetch = async () => ({{
  ok: true,
  json: async () => ({{config: {json.dumps(config, ensure_ascii=False)}}}),
}});
{functions}
(async () => {{
  await bootstrapCameraVariantControls();
  console.log(JSON.stringify({{
    hidden: elements.cameraVariantControls.classList.contains("hidden"),
    hintHidden: elements.cameraVariantHint.classList.contains("hidden"),
    count: elements.cameraCount.value,
    head: elements.headCameraSource.value,
    headDisabled: elements.headCameraSource.disabled,
    profileReadOnly: elements.profile.readOnly,
  }}));
}})().catch(error => {{ console.error(error); process.exit(1); }});
"""
    result = subprocess.run(
        ["node", "-e", script],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(result.stdout)


def run_launcher(*args: str, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    command_env = os.environ.copy()
    if env:
        command_env.update(env)
    return subprocess.run(
        ["bash", str(SELECTABLE_ENTRY), *args],
        check=False,
        capture_output=True,
        text=True,
        env=command_env,
    )


def test_launcher_defaults_to_three_camera_global_with_six_workers_and_gpu_zero():
    generic_text = GENERIC_ENTRY.read_text(encoding="utf-8")
    selectable_text = SELECTABLE_ENTRY.read_text(encoding="utf-8")
    generic_result = subprocess.run(
        ["bash", str(GENERIC_ENTRY), "--help"],
        check=False,
        capture_output=True,
        text=True,
        env=os.environ.copy(),
    )
    selectable_result = run_launcher("--help")

    assert generic_result.returncode == 0
    assert selectable_result.returncode == 0
    assert "Default: 8012" in generic_result.stdout
    assert 'WEB_PORT="${WEB_PORT:-8012}"' in generic_text
    assert "Default: 8001" in selectable_result.stdout
    assert 'WEB_PORT="${WEB_PORT:-8001}"' in selectable_text
    assert 'PIPELINE_CAMERA_VARIANT_SELECTABLE="${PIPELINE_CAMERA_VARIANT_SELECTABLE:-1}"' in selectable_text
    assert 'PIPELINE_CAMERA_COUNT="${PIPELINE_CAMERA_COUNT:-3}"' in selectable_text
    assert 'PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-global}"' in selectable_text
    assert 'PIPELINE_DEFAULT_CONVERT_JOBS="${PIPELINE_DEFAULT_CONVERT_JOBS:-6}"' in selectable_text
    assert 'PIPELINE_DEFAULT_GPU_DEVICE="${PIPELINE_DEFAULT_GPU_DEVICE:-0}"' in selectable_text
    assert "PIPELINE_OUTPUT_NAMESPACE" not in selectable_text


def test_selectable_launcher_help_documents_camera_mode_options():
    result = run_launcher("--help")

    assert result.returncode == 0
    assert "--camera-count 3|4" in result.stdout
    assert "--head-camera front|global" in result.stdout


def test_drink_grasp_launcher_selects_matching_topic_yaml_and_camera_layout():
    text = DRINK_GRASP_ENTRY.read_text(encoding="utf-8")

    assert "drink_grasp_three_camera_data_params.yaml" in text
    assert 'PIPELINE_CAMERA_LAYOUT="${PIPELINE_CAMERA_LAYOUT:-three_camera_global}"' in text
    assert 'PIPELINE_CAMERA_COUNT="${PIPELINE_CAMERA_COUNT:-3}"' in text
    assert 'PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-global}"' in text


@pytest.mark.parametrize(
    ("args", "message"),
    [
        (("--camera-count", "2"), "--camera-count must be 3 or 4"),
        (("--head-camera=side",), "--head-camera must be front or global"),
    ],
)
def test_selectable_launcher_rejects_invalid_camera_options(
    args: tuple[str, ...], message: str
):
    result = run_launcher(*args, "--no-deps-check")

    assert result.returncode != 0
    assert message in result.stderr


def test_generic_launcher_forwards_camera_selection_environment():
    text = GENERIC_ENTRY.read_text(encoding="utf-8")

    for name in (
        "PIPELINE_CAMERA_VARIANT_SELECTABLE",
        "PIPELINE_CAMERA_COUNT",
        "PIPELINE_HEAD_CAMERA_SOURCE",
    ):
        assert f'{name}="${{{name}_VALUE}}"' in text


def test_selectable_camera_layouts_choose_front_or_global_head(monkeypatch):
    monkeypatch.syspath_prepend(str(MCAP_SCRIPTS))
    layouts = load_module(MCAP_SCRIPTS / "camera_layouts.py", "selectable_camera_layouts")

    legacy = layouts.get_camera_layout("three_camera")
    front = layouts.get_camera_layout("three_camera_front")
    global_head = layouts.get_camera_layout("three_camera_global")
    four = layouts.get_camera_layout("four_camera")

    assert legacy.required_video_names == ()
    assert front.video_sources == {
        "head_color.mp4": Path("camera/color/front/sync.txt"),
        "hand_left_color.mp4": Path("camera/color/left/sync.txt"),
        "hand_right_color.mp4": Path("camera/color/right/sync.txt"),
    }
    assert front.required_video_names == tuple(front.video_sources)
    assert front.required_timestamp_keys == (
        "head_color",
        "head_stereo_left",
        "head_stereo_right",
    )
    assert global_head.video_sources == {
        "head.mp4": Path("camera/color/head/sync.txt"),
        "hand_left_color.mp4": Path("camera/color/left/sync.txt"),
        "hand_right_color.mp4": Path("camera/color/right/sync.txt"),
    }
    assert global_head.required_video_names == tuple(global_head.video_sources)
    assert global_head.required_timestamp_keys == (
        "head",
        "head_stereo_left",
        "head_stereo_right",
    )
    assert tuple(four.video_sources) == (
        "head_color.mp4",
        "hand_left_color.mp4",
        "hand_right_color.mp4",
        "head.mp4",
    )
    assert four.required_timestamp_keys == (
        "head_color",
        "head_stereo_left",
        "head_stereo_right",
        "head",
    )


@pytest.mark.parametrize(
    ("layout_name", "timestamp_keys"),
    [
        (
            "three_camera_front",
            ("head_color", "head_stereo_left", "head_stereo_right"),
        ),
        (
            "three_camera_global",
            ("head", "head_stereo_left", "head_stereo_right"),
        ),
        (
            "four_camera",
            ("head_color", "head_stereo_left", "head_stereo_right", "head"),
        ),
    ],
)
def test_strict_layout_completion_rejects_each_missing_selected_timestamp(
    monkeypatch, tmp_path, layout_name, timestamp_keys
):
    import cv2

    monkeypatch.syspath_prepend(str(MCAP_SCRIPTS))
    layouts = load_module(
        MCAP_SCRIPTS / "camera_layouts.py",
        f"strict_timestamp_layouts_{layout_name}",
    )
    layout = layouts.get_camera_layout(layout_name)
    episode_dir = tmp_path / layout_name
    h5_path = episode_dir / "states" / "aligned_joints.h5"
    h5_path.parent.mkdir(parents=True)
    with h5py.File(h5_path, "w") as root:
        frame = root.create_group("0")
        frame.create_dataset("main_timestamp", data=np.uint64(1_000_000_000))
        for timestamp_key in timestamp_keys:
            frame.create_dataset(
                f"timestamp/camera/{timestamp_key}",
                data=[np.uint64(1_000_000_000)],
            )

    videos_dir = episode_dir / "videos"
    videos_dir.mkdir(parents=True)
    available_videos = []
    for video_name in layout.required_video_names:
        writer = cv2.VideoWriter(
            str(videos_dir / video_name),
            cv2.VideoWriter_fourcc(*"mp4v"),
            30.0,
            (16, 16),
        )
        assert writer.isOpened()
        writer.write(np.zeros((16, 16, 3), dtype=np.uint8))
        writer.release()
        available_videos.append({"file": video_name, "frame_count": 1})
    meta_path = episode_dir / "meta" / "episode_meta.json"
    meta_path.parent.mkdir(parents=True)
    meta_path.write_text(
        json.dumps({"available_videos": available_videos}) + "\n",
        encoding="utf-8",
    )

    assert layouts.episode_artifacts_complete(episode_dir, layout_name)
    for timestamp_key in timestamp_keys:
        dataset_path = f"0/timestamp/camera/{timestamp_key}"
        with h5py.File(h5_path, "a") as root:
            del root[dataset_path]
        assert not layouts.episode_artifacts_complete(episode_dir, layout_name)
        with h5py.File(h5_path, "a") as root:
            root.create_dataset(dataset_path, data=[np.uint64(1_000_000_000)])


def test_four_camera_profile_uses_requested_lerobot_feature_names():
    profile = yaml.safe_load(FOUR_CAMERA_PROFILE.read_text(encoding="utf-8"))
    mappings = {item["raw_key"]: item["lerobot_key"] for item in profile["cameras"]}

    assert mappings == {
        "head_color": "observation.images.hand_head_color",
        "hand_left_color": "observation.images.hand_left_color",
        "hand_right_color": "observation.images.hand_right_color",
        "head": "observation.images.global_color",
    }


def test_drink_grasp_topic_yaml_matches_recorded_three_camera_dataset():
    params = yaml.safe_load(DRINK_GRASP_TOPIC_YAML.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]["dataInfo"]

    assert params["camera"]["color"]["names"] == ["left", "right", "head"]
    assert params["camera"]["color"]["topics"] == [
        "/cam_left/color/image_raw",
        "/cam_right/color/image_raw",
        "/cam_high/color/image_raw",
    ]
    assert params["camera"]["color"]["configTopics"] == [
        "/cam_left/color/camera_info",
        "/cam_right/color/camera_info",
        "/cam_high/color/camera_info",
    ]
    assert params["arm"]["jointState"]["topics"] == [
        "/joint_left",
        "/joint_right",
        "/joint_states_left",
        "/joint_states_right",
    ]
    assert params["localization"]["pose"]["topics"] == [
        "/end_pose_stamped_left",
        "/end_pose_stamped_right",
    ]


def test_mcap_conversion_message_count_does_not_require_mcap_cli():
    converter_text = (MCAP_SCRIPTS / "mcap_to_aloha_data.py").read_text(encoding="utf-8")

    assert "rosbag2_py.Info().read_metadata" in converter_text
    assert 'run_shell_command("mcap info "' not in converter_text


@pytest.mark.parametrize("layout", ["three_camera_front", "three_camera_global"])
def test_batch_converter_parser_accepts_selectable_camera_layouts(monkeypatch, layout):
    monkeypatch.syspath_prepend(str(PIPELINE_SCRIPTS))
    converter = load_module(
        PIPELINE_SCRIPTS / "convert_mcap_dataset.py",
        f"selectable_batch_converter_{layout}",
    )

    args = converter.parse_args(["--dataset-name", "demo", "--camera-layout", layout])

    assert args.camera_layout == layout


@pytest.mark.parametrize("layout", ["three_camera_front", "three_camera_global"])
def test_episode_converter_parser_accepts_selectable_camera_layouts(monkeypatch, layout):
    monkeypatch.syspath_prepend(str(MCAP_SCRIPTS))
    converter = load_module(
        MCAP_SCRIPTS / "mcap_to_icra_episode.py",
        f"selectable_episode_converter_{layout}",
    )

    args = converter.parse_args(
        ["--mcapPath", "/tmp/input", "--output", "/tmp/output", "--cameraLayout", layout]
    )

    assert args.cameraLayout == layout


def set_selectable_web_environment(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setenv("PIPELINE_CAMERA_VARIANT_SELECTABLE", "1")
    monkeypatch.setenv("PIPELINE_CAMERA_COUNT", "3")
    monkeypatch.setenv("PIPELINE_HEAD_CAMERA_SOURCE", "global")
    monkeypatch.setenv("PIPELINE_DEFAULT_PROFILE", str(FOUR_CAMERA_PROFILE))
    monkeypatch.setenv("PIPELINE_ALOHA_YAML", str(FOUR_CAMERA_TOPIC_YAML))
    monkeypatch.setenv("PIPELINE_CAMERA_LAYOUT", "four_camera")
    monkeypatch.delenv("PIPELINE_OUTPUT_NAMESPACE", raising=False)
    monkeypatch.setenv("WEB_LOG_ROOT", str(tmp_path / "web_logs"))


def derive_selectable_config(monkeypatch, tmp_path: Path, **payload):
    set_selectable_web_environment(monkeypatch, tmp_path)
    app = load_module(PIPELINE_WEB_APP, f"selectable_web_{len(sys.modules)}")
    cfg = app.derive_paths(
        {
            "robot_type": "aloha",
            "data_root": str(tmp_path / "data"),
            "dataset_name": "demo",
            **payload,
        }
    )
    return app, cfg


def test_derive_selectable_camera_defaults_to_three_camera_global_paths(monkeypatch, tmp_path):
    _, cfg = derive_selectable_config(monkeypatch, tmp_path)

    assert cfg["camera_variant_selectable"] is True
    assert cfg["camera_count"] == 3
    assert cfg["head_camera_source"] == "global"
    assert cfg["camera_layout"] == "three_camera_global"
    assert cfg["output_namespace"] == "three_camera_global"
    assert cfg["profile"].name == "aloha_three_camera_global.json"
    assert cfg["hdf5_root"] == tmp_path / "data" / "three_camera_global" / "hdf5_episodes" / "demo"
    assert cfg["camera_feature_keys"] == [
        "observation.images.hand_head_color",
        "observation.images.hand_left_color",
        "observation.images.hand_right_color",
    ]


def test_derive_three_camera_front_profile_filters_global_camera(monkeypatch, tmp_path):
    _, cfg = derive_selectable_config(
        monkeypatch,
        tmp_path,
        camera_count=3,
        head_camera_source="front",
    )
    profile = yaml.safe_load(cfg["profile"].read_text(encoding="utf-8"))
    mappings = {item["raw_key"]: item["lerobot_key"] for item in profile["cameras"]}

    assert cfg["camera_layout"] == "three_camera_front"
    assert cfg["output_namespace"] == "three_camera_front"
    assert cfg["profile"].parent == (tmp_path / "web_logs" / "camera_profiles").resolve()
    assert mappings == {
        "head_color": "observation.images.hand_head_color",
        "hand_left_color": "observation.images.hand_left_color",
        "hand_right_color": "observation.images.hand_right_color",
    }
    assert profile["quality_checks"]["required_cameras"] == list(mappings)
    annotations = profile["processing"]["annotations"]
    assert annotations["vision_pipeline"]["camera"] == "head_color"
    assert annotations["hand_target_assignment"]["cameras"] == ["head_color"]


def test_derive_three_camera_global_profile_maps_wide_angle_to_hand_head(monkeypatch, tmp_path):
    _, cfg = derive_selectable_config(
        monkeypatch,
        tmp_path,
        camera_count="3",
        head_camera_source="global",
    )
    profile = yaml.safe_load(cfg["profile"].read_text(encoding="utf-8"))
    mappings = {item["raw_key"]: item["lerobot_key"] for item in profile["cameras"]}

    assert cfg["camera_count"] == 3
    assert cfg["head_camera_source"] == "global"
    assert cfg["camera_layout"] == "three_camera_global"
    assert cfg["output_namespace"] == "three_camera_global"
    assert mappings == {
        "hand_left_color": "observation.images.hand_left_color",
        "hand_right_color": "observation.images.hand_right_color",
        "head": "observation.images.hand_head_color",
    }
    assert profile["quality_checks"]["required_cameras"] == list(mappings)
    annotations = profile["processing"]["annotations"]
    assert annotations["vision_pipeline"]["camera"] == "head"
    assert annotations["hand_target_assignment"]["cameras"] == ["head"]


def test_selectable_camera_explicit_output_namespace_wins(monkeypatch, tmp_path):
    _, cfg = derive_selectable_config(
        monkeypatch,
        tmp_path,
        camera_count=3,
        head_camera_source="global",
        output_namespace="custom/cameras",
    )

    assert cfg["output_namespace"] == "custom/cameras"
    assert cfg["hdf5_root"] == tmp_path / "data" / "custom" / "cameras" / "hdf5_episodes" / "demo"


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        ({"camera_count": "2"}, "Invalid camera count"),
        ({"head_camera_source": "side"}, "Invalid head camera source"),
    ],
)
def test_derive_selectable_camera_rejects_invalid_values(monkeypatch, tmp_path, payload, message):
    set_selectable_web_environment(monkeypatch, tmp_path)
    app = load_module(PIPELINE_WEB_APP, f"invalid_selectable_web_{len(sys.modules)}")

    with pytest.raises(ValueError, match=message):
        app.derive_paths(
            {
                "data_root": str(tmp_path / "data"),
                "dataset_name": "demo",
                **payload,
            }
        )


def test_docker_conversion_mounts_generated_camera_profile(monkeypatch, tmp_path):
    app, cfg = derive_selectable_config(
        monkeypatch,
        tmp_path,
        camera_count=3,
        head_camera_source="global",
        mcap_path=str(tmp_path / "data" / "raw_mcap" / "demo"),
        use_docker=True,
    )

    command, _, _ = app.convert_command(cfg)
    profile_container = "/workspace/variant/aloha_three_camera_global.json"

    assert f"PROFILE_CONTAINER={profile_container}" in command
    assert f"{cfg['profile']}:{profile_container}:ro" in command
    assert "CAMERA_LAYOUT=three_camera_global" in command


def test_selectable_camera_ui_exposes_server_enabled_mode_controls(monkeypatch, tmp_path):
    set_selectable_web_environment(monkeypatch, tmp_path)
    app = load_module(PIPELINE_WEB_APP, f"selectable_camera_ui_{len(sys.modules)}")
    html = app.HTML

    assert 'id="cameraVariantControls" class="row hidden"' in html
    assert 'id="cameraCount"' in html
    assert 'id="headCameraSource"' in html
    assert 'cameraControls.classList.toggle("hidden", !selectable)' in html
    assert 'headCamera.disabled = !selectable || cameraCount.value !== "3"' in html
    assert 'if (cameraControls.classList.contains("hidden")) return {}' in html
    assert "camera_count: Number(cameraCount.value)" in html
    assert "head_camera_source: headCamera.value" in html
    assert 'id="cameraCount"' in html
    assert '<option value="3" selected>三路</option>' in html
    assert '<option value="global" selected>广角头部 /camera_h</option>' in html


def test_empty_page_bootstraps_selectable_controls_but_keeps_generic_page_hidden(
    monkeypatch, tmp_path
):
    set_selectable_web_environment(monkeypatch, tmp_path)
    app = load_module(PIPELINE_WEB_APP, f"camera_ui_bootstrap_{len(sys.modules)}")
    selectable_cfg = app.stringify_config(app.derive_paths({"robot_type": "aloha"}))

    selectable_state = run_camera_ui_bootstrap(app.HTML, selectable_cfg)

    assert selectable_state == {
        "hidden": False,
        "hintHidden": False,
        "count": "3",
        "head": "global",
        "headDisabled": False,
        "profileReadOnly": True,
    }

    monkeypatch.setenv("PIPELINE_CAMERA_VARIANT_SELECTABLE", "0")
    generic_cfg = app.stringify_config(app.derive_paths({"robot_type": "aloha"}))

    generic_state = run_camera_ui_bootstrap(app.HTML, generic_cfg)

    assert generic_state["hidden"] is True
    assert generic_state["hintHidden"] is True
    assert generic_state["profileReadOnly"] is False
