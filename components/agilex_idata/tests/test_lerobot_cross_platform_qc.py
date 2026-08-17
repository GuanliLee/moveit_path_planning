from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
PIPELINE_ROOT = REPO_ROOT / "scripts" / "embodied_data_pipeline-main"
PIPELINE_WEB_APP = PIPELINE_ROOT / "scripts" / "pipeline_web_app.py"
if str(PIPELINE_ROOT) not in sys.path:
    sys.path.insert(0, str(PIPELINE_ROOT))

import lerobot_cross_platform as cross_platform


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows), encoding="utf-8")


def make_lerobot_dataset(path: Path, task: str, offset: float = 0.0) -> Path:
    info = {
        "codebase_version": "v2.1",
        "robot_type": "aloha",
        "total_episodes": 2,
        "total_frames": 6,
        "total_videos": 2,
        "total_chunks": 1,
        "chunks_size": 1000,
        "fps": 30,
        "splits": {"train": "0:2"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "observation.state": {"dtype": "float32", "shape": [21], "names": cross_platform.STATE_LAYOUT},
            "action": {"dtype": "float32", "shape": [18], "names": cross_platform.ACTION_LAYOUT},
            "observation.images.head": {"dtype": "video", "shape": [480, 640, 3]},
        },
    }
    write_json(path / "meta" / "info.json", info)
    write_jsonl(path / "meta" / "tasks.jsonl", [{"task_index": 0, "task": task}])
    write_jsonl(
        path / "meta" / "episodes.jsonl",
        [
            {"episode_index": index, "task_index": 0, "tasks": [task], "length": 3, "quality_grade": "A"}
            for index in range(2)
        ],
    )
    write_jsonl(
        path / "meta" / "episodes_stats.jsonl",
        [{"episode_index": index, "stats": {"marker": index}} for index in range(2)],
    )
    mapping_rows = []
    global_index = 0
    for episode_index in range(2):
        state_rows = []
        action_rows = []
        for frame_index in range(3):
            state = np.arange(21, dtype=np.float32) * 0.01 + offset
            action = np.arange(18, dtype=np.float32) * 0.01 + offset
            binary = float(frame_index % 2) * 0.1
            state[6] = binary
            state[13] = binary
            action[6] = binary
            action[13] = binary
            state[14] = 200.0
            action[14] = 200.0
            state_rows.append(state)
            action_rows.append(action)
        frame = pd.DataFrame(
            {
                "observation.state": state_rows,
                "action": action_rows,
                "episode_index": [episode_index] * 3,
                "frame_index": list(range(3)),
                "index": list(range(global_index, global_index + 3)),
                "timestamp": [0.0, 1 / 30, 2 / 30],
                "task_index": [0] * 3,
            }
        )
        global_index += 3
        parquet = path / "data" / "chunk-000" / f"episode_{episode_index:06d}.parquet"
        parquet.parent.mkdir(parents=True, exist_ok=True)
        frame.to_parquet(parquet, index=False)
        video_rel = f"videos/chunk-000/observation.images.head/episode_{episode_index:06d}.mp4"
        video = path / video_rel
        video.parent.mkdir(parents=True, exist_ok=True)
        video.write_bytes(f"video-{episode_index}".encode())
        mapping_rows.append(
            {
                "source_episode_name": f"episode{episode_index}",
                "lerobot_episode_index": episode_index,
                "lerobot_episode_name": f"episode_{episode_index:06d}",
                "lerobot_data_file": str(parquet.relative_to(path)),
                "lerobot_video_files": {"observation.images.head": video_rel},
                "quality_grade": "A",
            }
        )
    write_json(path / "meta" / "episode_name_mapping.json", {"episodes": mapping_rows})
    return path


def tree_hashes(root: Path) -> dict[str, str]:
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def load_web_app(name: str):
    spec = importlib.util.spec_from_file_location(name, PIPELINE_WEB_APP)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_recursive_discovery_and_cross_platform_analysis(tmp_path: Path) -> None:
    sim = make_lerobot_dataset(tmp_path / "sim" / "task_a", "Grasp Sprite with the left hand.")
    real = make_lerobot_dataset(tmp_path / "real" / "task_a", "Grasp Wanglaoji with the right hand.")

    discovered = cross_platform.discover_datasets(
        [
            {"path": str(sim.parent), "platform": "simulation", "recursive": True},
            {"path": str(real), "platform": "real", "recursive": False},
        ]
    )

    assert discovered["summary"] == {"total": 2, "simulation": 1, "real": 1}
    report = cross_platform.analyze_datasets(
        [{"path": item["path"], "platform": item["platform"]} for item in discovered["datasets"]],
        max_episodes=2,
        max_frames=50,
    )
    assert report["contract"]["action_dim"] == 18
    assert report["contract"]["state_dim"] == 21
    assert report["summary"]["dataset_count"] == 2
    assert report["summary"]["episode_count"] == 4
    assert report["prompts"]["status"] == "pass"
    assert report["prompts"]["only_simulation"] == []
    assert report["prompts"]["only_real"] == []
    assert len(report["dimensions"]) == 39
    assert len(report["grippers"]) == 4
    assert len(report["lift"]) == 2
    assert all(row["status"] == "pass" for row in report["dimensions"])
    assert all(row["status"] == "pass" for row in report["lift"])
    assert all(row["simulation"]["binary"] is True for row in report["grippers"])
    assert all(row["real"]["binary"] is True for row in report["grippers"])


def test_grade_directories_are_discovered_as_one_logical_group_source(tmp_path: Path) -> None:
    grade_root = tmp_path / "scene" / "lerobot" / "dataset"
    for grade in ("A", "B", "F"):
        make_lerobot_dataset(grade_root / grade, f"Grasp Sprite with the left hand. {grade}")

    discovered = cross_platform.discover_datasets(
        [{"path": str(grade_root / "A"), "platform": "real", "recursive": False}]
    )

    assert {item["source_grade"] for item in discovered["datasets"]} == {"A", "B", "F"}
    assert {item["grade_group_path"] for item in discovered["datasets"]} == {str(grade_root.resolve())}
    report = cross_platform.analyze_datasets(
        [{
            "path": str(grade_root / "A"),
            "platform": "real",
            "logical_name": "dataset",
            "logical_path": str(grade_root),
            "source_grade": "A",
        }],
        max_episodes=1,
        max_frames=50,
    )
    assert report["summary"]["dataset_count"] == 1
    assert report["datasets"][0]["name"] == "dataset"
    assert report["datasets"][0]["source_grade"] == "A"


def test_lift_qc_requires_action_and_state_to_stay_at_200_mm(tmp_path: Path) -> None:
    dataset = make_lerobot_dataset(tmp_path / "source", "Grasp Sprite with the left hand.")
    parquet = dataset / "data" / "chunk-000" / "episode_000000.parquet"
    frame = pd.read_parquet(parquet)
    frame["observation.state"] = frame["observation.state"].map(
        lambda values: np.asarray(values).copy()
    )
    frame["action"] = frame["action"].map(lambda values: np.asarray(values).copy())
    for values in frame["observation.state"]:
        values[14] = 198.0
    for values in frame["action"]:
        values[14] = 205.0
    frame.to_parquet(parquet, index=False)

    report = cross_platform.analyze_datasets(
        [{"path": str(dataset), "platform": "real"}],
        max_episodes=2,
        max_frames=50,
    )

    lift = {row["vector"]: row for row in report["lift"]}
    assert lift["state"]["status"] == "fail"
    assert lift["action"]["status"] == "fail"
    assert "200±0.5 mm" in lift["state"]["warnings"][0]
    assert report["summary"]["lift_failures"] == 2


def test_gripper_binary_and_continuous_absolute_value_rules() -> None:
    valid_binary = cross_platform._binary_profile(np.array([0.0, 0.1, 0.0, 0.1]))
    assert valid_binary["binary"] is True
    assert cross_platform._gripper_profile_status(valid_binary, "真机") == ("pass", [])

    rare_high_level = cross_platform._binary_profile(np.array([0.0] * 100 + [0.1]))
    assert rare_high_level["binary"] is True
    assert rare_high_level["low_level"] == 0.0
    assert rare_high_level["high_level"] == 0.1
    assert cross_platform._gripper_profile_status(rare_high_level, "真机") == ("pass", [])

    invalid_binary = cross_platform._binary_profile(np.array([0.0, 0.2, 0.0, 0.2]))
    status, warnings = cross_platform._gripper_profile_status(invalid_binary, "真机")
    assert status == "fail"
    assert "档位为" in warnings[0]
    assert "期望 0/0.1" in warnings[0]

    soft_continuous = cross_platform._binary_profile(np.array([-0.01, 0.02, 0.04, 0.08, 0.11]))
    assert soft_continuous["binary"] is False
    status, warnings = cross_platform._gripper_profile_status(soft_continuous, "仿真")
    assert status == "warn"
    assert "超出标准 0～0.1" in warnings[0]
    assert "允许范围 -0.05～0.15" in warnings[0]

    invalid_continuous = cross_platform._binary_profile(np.array([-0.06, 0.02, 0.04, 0.08, 0.16]))
    assert invalid_continuous["binary"] is False
    status, warnings = cross_platform._gripper_profile_status(invalid_continuous, "仿真")
    assert status == "fail"
    assert "超出允许范围 -0.05～0.15" in warnings[0]


def test_gripper_and_lift_do_not_use_generic_distribution_overlap(tmp_path: Path) -> None:
    sim = make_lerobot_dataset(tmp_path / "sim", "Grasp Sprite with the left hand.")
    real = make_lerobot_dataset(tmp_path / "real", "Grasp Wanglaoji with the right hand.")
    report = cross_platform.analyze_datasets(
        [
            {"path": str(sim), "platform": "simulation"},
            {"path": str(real), "platform": "real"},
        ],
        max_episodes=2,
        max_frames=50,
    )

    specialised_names = {"left_gripper", "right_gripper", "lift_mm"}
    specialised_dimensions = [row for row in report["dimensions"] if row["name"] in specialised_names]
    assert len(specialised_dimensions) == 6
    assert all(row["range_status"] == "pass" for row in specialised_dimensions)
    assert all(row["warnings"] == [] for row in specialised_dimensions)
    assert all(row["status"] == "pass" for row in report["grippers"])
    assert all(row["status"] == "pass" for row in report["lift"])


def test_identical_constant_ranges_pass_without_false_overlap_warning() -> None:
    constant_zero = {"count": 100, "p01": 0.0, "p99": 0.0}
    status, warnings, metrics = cross_platform.compare_ranges(constant_zero, constant_zero)
    assert status == "pass"
    assert warnings == []
    assert metrics["overlap_ratio"] == 1.0

    constant_one = {"count": 100, "p01": 1.0, "p99": 1.0}
    status, warnings, metrics = cross_platform.compare_ranges(constant_zero, constant_one)
    assert status == "warn"
    assert "均为常量，但数值不同" in warnings[0]
    assert metrics["constant_values"] == {"simulation": 0.0, "real": 1.0}


def test_review_rebuild_is_copy_on_write_and_renumbers_everything(tmp_path: Path) -> None:
    source = make_lerobot_dataset(tmp_path / "source", "Grasp Sprite with the left hand.")
    output = tmp_path / "reviewed"
    before = tree_hashes(source)

    result = cross_platform.rebuild_reviewed_dataset(
        source,
        output,
        [
            {"episode_index": 0, "exclude": True, "reason": "bad trajectory"},
            {"episode_index": 1, "quality_grade": "B", "reason": "minor issue"},
        ],
    )

    assert tree_hashes(source) == before
    assert result["source_episodes"] == 2
    assert result["output_episodes"] == 1
    assert result["excluded"] == 1
    info = json.loads((output / "meta" / "info.json").read_text())
    assert info["total_episodes"] == 1
    assert info["total_frames"] == 3
    episodes = [json.loads(line) for line in (output / "meta" / "episodes.jsonl").read_text().splitlines()]
    assert episodes[0]["episode_index"] == 0
    assert episodes[0]["quality_grade"] == "B"
    assert episodes[0]["manual_review_reason"] == "minor issue"
    frame = pd.read_parquet(output / "data" / "chunk-000" / "episode_000000.parquet")
    assert frame["episode_index"].tolist() == [0, 0, 0]
    assert frame["frame_index"].tolist() == [0, 1, 2]
    assert frame["index"].tolist() == [0, 1, 2]
    assert (output / "videos" / "chunk-000" / "observation.images.head" / "episode_000000.mp4").read_bytes() == b"video-1"
    mapping = json.loads((output / "meta" / "episode_name_mapping.json").read_text())["episodes"][0]
    assert mapping["source_lerobot_episode_index"] == 1
    assert mapping["lerobot_episode_index"] == 0
    assert mapping["quality_grade"] == "B"
    review = json.loads((output / "meta" / "manual_review.json").read_text())
    assert review["excluded_episode_indices"] == [0]
    assert review["index_mapping"][0]["source_episode_index"] == 1
    assert review["index_mapping"][0]["reviewed_episode_index"] == 0


def test_review_rejects_source_overwrite_or_nested_output(tmp_path: Path) -> None:
    source = make_lerobot_dataset(tmp_path / "source", "Grasp Sprite with the left hand.")
    with pytest.raises(ValueError, match="相同"):
        cross_platform.validate_review_request(source, source, [])
    with pytest.raises(ValueError, match="内部"):
        cross_platform.validate_review_request(source, source / "reviewed", [])


def test_incomplete_dimension_names_are_not_reported_when_positions_and_sizes_are_valid(tmp_path: Path) -> None:
    dataset = make_lerobot_dataset(tmp_path / "source", "Grasp Sprite with the left hand.")
    info_path = dataset / "meta" / "info.json"
    info = json.loads(info_path.read_text())
    info["features"]["observation.state"]["names"] = ["state"]
    info["features"]["action"]["names"] = ["action"]
    write_json(info_path, info)

    descriptor = cross_platform.describe_dataset(dataset, "real")

    assert descriptor["state_dim"] == 21
    assert descriptor["action_dim"] == 18
    assert descriptor["state_names"] == []
    assert descriptor["action_names"] == []
    assert not any("每维名称" in issue for issue in descriptor["issues"])
    status, warnings = cross_platform._definition_status([descriptor], "state", 0, "left_joint_1")
    assert status == "pass"
    assert warnings == []

    alias_descriptor = {"name": "simulation", "state_names": ["left_waist", "left_shoulder"]}
    status, warnings = cross_platform._definition_status([alias_descriptor], "state", 0, "left_joint_1")
    assert status == "pass"
    assert warnings == []
    status, warnings = cross_platform._definition_status([alias_descriptor], "state", 1, "left_joint_1")
    assert status == "fail"
    assert "位置不正确" in warnings[0]


def test_cross_platform_workspace_is_additive_and_reuses_replay(tmp_path: Path) -> None:
    dataset = make_lerobot_dataset(tmp_path / "source", "Grasp Sprite with the left hand.")
    app = load_web_app("qc_web_cross_platform")

    assert 'id="collectionWorkspace"' in app.HTML
    assert 'id="crossPlatformFrame" src="/cross-platform/"' in app.HTML
    assert "采集数据质检与转换" in app.HTML
    assert "跨平台 LeRobot 质检与回放" in app.CROSS_PLATFORM_HTML
    assert 'data-tab="qc">LeRobot 质检</button>' in app.CROSS_PLATFORM_HTML
    assert 'data-tab="visual">可视化回放</button>' in app.CROSS_PLATFORM_HTML
    assert 'data-tab="prompt"' not in app.CROSS_PLATFORM_HTML
    assert 'data-tab="dimensions"' not in app.CROSS_PLATFORM_HTML
    assert 'id="episodeRows"' not in app.CROSS_PLATFORM_HTML
    assert 'id="visualEpisodeSelect"' in app.CROSS_PLATFORM_HTML
    assert 'id="visualReviewGrade"' in app.CROSS_PLATFORM_HTML
    assert 'class="visual-inspector"' not in app.CROSS_PLATFORM_HTML
    assert 'id="visualController" class="hidden"' in app.CROSS_PLATFORM_HTML
    assert 'id="episodePlatform"' not in app.CROSS_PLATFORM_HTML
    assert 'id="episodeTask"' not in app.CROSS_PLATFORM_HTML
    for label in (
        "Action 左臂", "Action 左夹爪", "Action 右臂", "Action 右夹爪", "Action 升降柱", "Action 底盘",
        "State 左臂", "State 左夹爪", "State 右臂", "State 右夹爪", "State 升降柱", "State 底盘定位", "State 底盘轮速",
    ):
        assert label in app.CROSS_PLATFORM_HTML
    assert "kind:'Action / State'" not in app.CROSS_PLATFORM_HTML
    assert "body.classList.toggle('visual-only'" not in app.CROSS_PLATFORM_HTML
    assert 'class="left-column"' in app.CROSS_PLATFORM_HTML
    assert 'id="qcResultRows"' in app.CROSS_PLATFORM_HTML
    assert 'class="qc-details"' not in app.CROSS_PLATFORM_HTML
    assert 'id="replayFrameMetric"' not in app.CROSS_PLATFORM_HTML
    assert 'id="embeddedDatasetSelect"' in app.LEROBOT_REPLAY_HTML
    assert 'id="embeddedEpisodeSelect"' in app.LEROBOT_REPLAY_HTML
    assert 'id="embeddedReviewStrip"' in app.LEROBOT_REPLAY_HTML
    assert app.LEROBOT_REPLAY_HTML.index('id="embeddedReviewStrip"') < app.LEROBOT_REPLAY_HTML.index('class="data-split"')
    assert "lerobot-review-event" in app.LEROBOT_REPLAY_HTML
    assert "lerobot-review-state" in app.CROSS_PLATFORM_HTML
    assert 'type: "lerobot-replay-state"' not in app.LEROBOT_REPLAY_HTML
    assert 'body.embedded-review .metrics' not in app.LEROBOT_REPLAY_HTML
    assert '仅 A（默认）' in app.CROSS_PLATFORM_HTML
    assert '读取 A/B/F 全部' in app.CROSS_PLATFORM_HTML
    replay = app.start_cross_platform_lerobot_replay({"dataset_path": str(dataset)})
    assert replay["url"].startswith("/lerobot-replay/?key=")
    assert len(replay["summary"]["episodes"]) == 2
