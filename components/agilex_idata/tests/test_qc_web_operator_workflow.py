from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
PIPELINE_ROOT = REPO_ROOT / "scripts" / "embodied_data_pipeline-main"
PIPELINE_WEB_APP = PIPELINE_ROOT / "scripts" / "pipeline_web_app.py"
FOUR_CAMERA_PROFILE = PIPELINE_ROOT / "robot_profiles" / "aloha_four_camera.yaml"


def load_app(name: str):
    spec = importlib.util.spec_from_file_location(name, PIPELINE_WEB_APP)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def make_mcap_dataset(path: Path) -> None:
    episode = path / "episode0"
    episode.mkdir(parents=True)
    (episode / "episode0_0.mcap").touch()


def test_machine_dataset_discovery_uses_expected_directory_depths(tmp_path, monkeypatch):
    app = load_app("qc_web_machine_discovery")
    agilex_root = tmp_path / "agilex"
    h200_root = tmp_path / "h200"
    agilex_dataset = agilex_root / "stage2_new" / "scene1"
    h200_dataset = h200_root / "public_dataset"
    make_mcap_dataset(agilex_dataset)
    make_mcap_dataset(h200_dataset)
    make_mcap_dataset(h200_dataset / "nested_dataset_must_not_be_listed")
    monkeypatch.setattr(app, "DATA_SCAN_ROOT", agilex_root)
    monkeypatch.setattr(app, "H200_DATA_SCAN_ROOT", h200_root)

    agilex = app.discover_machine_datasets("agilex")
    h200 = app.discover_machine_datasets("h200")

    assert agilex["scan_depth"] == 2
    assert [item["path"] for item in agilex["datasets"]] == [str(agilex_dataset.resolve())]
    assert h200["scan_depth"] == 1
    assert [item["path"] for item in h200["datasets"]] == [str(h200_dataset.resolve())]
    assert h200["datasets"][0]["parent_path"] == str(h200_root.resolve())


def test_machine_dataset_discovery_rejects_arbitrary_scan_targets():
    app = load_app("qc_web_machine_allowlist")

    with pytest.raises(ValueError, match="Unsupported machine"):
        app.discover_machine_datasets("/tmp/not-an-allowed-root")


def test_custom_dataset_descriptor_supports_processed_only_data(tmp_path):
    app = load_app("qc_web_custom_dataset")
    dataset = tmp_path / "scene12" / "20260728_scene12"
    episode = (
        dataset
        / "three_camera_global"
        / "hdf5_episodes"
        / dataset.name
        / "episode0"
        / "states"
    )
    episode.mkdir(parents=True)
    (episode / "aligned_joints.h5").touch()

    choice = app.dataset_choice_entry(dataset, dataset.parent)

    assert choice["has_mcap"] is False
    assert choice["episode_count"] == 1
    assert choice["parent_path"] == str(dataset.parent.resolve())
    assert choice["processed_variants"]["three_camera_global"][0]["dataset_name"] == dataset.name


def test_operator_defaults_are_three_camera_global_six_workers_and_gpu_zero(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("PIPELINE_CAMERA_VARIANT_SELECTABLE", "1")
    monkeypatch.delenv("PIPELINE_CAMERA_COUNT", raising=False)
    monkeypatch.delenv("PIPELINE_HEAD_CAMERA_SOURCE", raising=False)
    monkeypatch.delenv("PIPELINE_DEFAULT_CONVERT_JOBS", raising=False)
    monkeypatch.delenv("PIPELINE_DEFAULT_GPU_DEVICE", raising=False)
    monkeypatch.delenv("PIPELINE_DEFAULT_ALOHA_INCLUDE_BASE_ACTION", raising=False)
    monkeypatch.setenv("PIPELINE_DEFAULT_PROFILE", str(FOUR_CAMERA_PROFILE))
    monkeypatch.setenv("WEB_LOG_ROOT", str(tmp_path / "logs"))
    app = load_app("qc_web_operator_defaults")

    cfg = app.derive_paths(
        {
            "robot_type": "aloha",
            "data_root": str(tmp_path / "data"),
            "dataset_name": "demo",
        }
    )

    assert cfg["camera_count"] == 3
    assert cfg["head_camera_source"] == "global"
    assert cfg["camera_layout"] == "three_camera_global"
    assert cfg["convert_jobs"] == 6
    assert cfg["gpu_device"] == "0"
    assert cfg["aloha_include_base_action"] is False


def test_main_workspace_contains_new_operator_flow_without_lerobot_replay():
    app = load_app("qc_web_operator_html")
    html = app.HTML

    assert 'id="collectionWorkspaceBtn" class="workspace-btn active"' in html
    assert 'id="lerobotWorkspaceBtn"' in html
    assert 'id="hostMachine"' in html
    assert '<option value="agilex" selected>AgileX</option>' in html
    assert '<option value="h200">H200</option>' in html
    assert 'new Option("自定义目录…", CUSTOM_DATASET_VALUE)' in html
    assert 'id="mcapPath" type="hidden"' in html
    assert 'id="hdf5Root" type="hidden"' in html
    assert 'id="qcRoot" type="hidden"' in html
    assert 'id="lerobotRoot" type="hidden"' in html
    assert 'id="datasetName" type="hidden"' in html
    assert 'task_text: taskTextManuallyEdited ?' in html
    assert 'convert_jobs: Number(document.getElementById("convertJobs").value || 6)' in html
    for removed_id in (
        "lerobotReplayBtn",
        "lerobotReplayTab",
        "lerobotReplayPanel",
        "lerobotReplayFrame",
    ):
        assert f'id="{removed_id}"' not in html
