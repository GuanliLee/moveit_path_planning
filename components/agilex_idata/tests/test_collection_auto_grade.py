from __future__ import annotations

import os
import shlex
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest


REPO_ROOT = Path(os.environ.get("AUTO_GRADE_TEST_REPO", Path(__file__).resolve().parents[1]))
STAGED_SCRIPT = REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_staged.sh"
INFERENCE_SCRIPT = (
    REPO_ROOT / "scripts" / "collection" / "collect_mobile_pipeline_web_inference.sh"
)
UNIFIED_LAUNCHER = REPO_ROOT / "scripts" / "collection" / "collection_web_launcher.py"
COLLECTION_SCRIPT = REPO_ROOT / "scripts" / "collection" / "collect_mobile_episode_web.sh"


def run_staged(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(STAGED_SCRIPT), *args],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=10,
        check=False,
    )


def test_help_documents_optional_automatic_grade():
    result = run_staged("--help")

    assert result.returncode == 0
    assert "--grade A|B|F" in result.stdout
    assert "默认仍为人工选择" in result.stdout


def test_grade_parser_rejects_missing_empty_and_invalid_values():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")

    assert "--grade)" in text
    assert "--grade=*" in text
    assert "AUTO_GRADE_ARG_SEEN" in text
    assert "--grade 需要质量等级参数" in text
    assert "--grade 只能是 A、B 或 F" in text
    assert "[:lower:]" in text
    assert "[:upper:]" in text


def test_launcher_normalizes_and_passes_grade_to_collection_process():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")

    assert "AUTO_GRADE_ARG" in text
    assert "AUTO_GRADE_ARG_SEEN" in text
    assert "--grade=*" in text
    assert "AUTO_GRADE=" in text
    assert 'COLLECTION_AUTO_GRADE="${AUTO_GRADE}"' in text


def test_inference_collection_defaults_every_saved_episode_to_grade_a():
    text = INFERENCE_SCRIPT.read_text(encoding="utf-8")

    assert 'COLLECTION_AUTO_GRADE="${COLLECTION_AUTO_GRADE:-A}"' in text
    assert 'COLLECTION_AUTO_GRADE="${COLLECTION_AUTO_GRADE}"' in text


def test_unified_launcher_only_forces_grade_a_for_inference_mode():
    text = UNIFIED_LAUNCHER.read_text(encoding="utf-8")

    assert 'elif mode == "inference":\n                env["COLLECTION_AUTO_GRADE"] = "A"' in text
    assert "每条推理录制保存后自动标记为 A，无需手动选择等级。" in text


def test_collection_controller_validates_internal_automatic_grade():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")

    assert 'COLLECTION_AUTO_GRADE="${COLLECTION_AUTO_GRADE:-}"' in text
    assert "COLLECTION_AUTO_GRADE=\"$(printf '%s'" in text
    assert "COLLECTION_AUTO_GRADE 只能是 A、B 或 F" in text


def test_saved_episode_uses_existing_review_path_only_in_auto_mode():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    save_block = text.split("save_current_episode() {", 1)[1].split(
        "check_required_topics() {",
        1,
    )[0]

    pending_at = save_block.index("QUALITY_REVIEW_PENDING=1")
    auto_branch_at = save_block.index('if [ -n "${COLLECTION_AUTO_GRADE}" ]; then')
    finalize_at = save_block.index("finalize_quality_review_from_json")
    manual_at = save_block.index("已保存，请选择质量等级或放弃")
    assert pending_at < auto_branch_at < finalize_at < manual_at
    assert r'\"action\":\"review\"' in save_block
    assert r'\"reason_codes\":[]' in save_block
    assert r'\"reason_note\":\"\"' in save_block


def test_automatic_grade_does_not_publish_review_status_before_finalize():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    save_block = text.split("save_current_episode() {", 1)[1].split(
        "check_required_topics() {",
        1,
    )[0]
    auto_block = save_block.split(
        'if [ -n "${COLLECTION_AUTO_GRADE}" ]; then',
        1,
    )[1].split("    else", 1)[0]

    assert 'write_status "review"' not in auto_block
    assert "finalize_quality_review_from_json" in auto_block


def test_automatic_review_reuses_recoverable_metadata_failure_path():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    save_block = text.split("save_current_episode() {", 1)[1].split(
        "check_required_topics() {",
        1,
    )[0]
    review_block = text.split("finalize_quality_review_from_json() {", 1)[1].split(
        "qc_aloha_hdf5_episode() {",
        1,
    )[0]

    assert 'if [ -n "${COLLECTION_AUTO_GRADE}" ]; then' in save_block
    assert "finalize_quality_review_from_json" in save_block
    failure_block = review_block.split('if ! "${cmd[@]}"; then', 1)[1].split("fi", 1)[0]
    assert 'write_status "review"' in failure_block
    assert "QUALITY_REVIEW_PENDING=0" not in failure_block
    assert "CURRENT_EPISODE=$((CURRENT_EPISODE + 1))" not in failure_block


def free_ports(count: int) -> list[int]:
    sockets = []
    try:
        for _ in range(count):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.bind(("127.0.0.1", 0))
            sockets.append(sock)
        return [int(sock.getsockname()[1]) for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


def run_launcher_with_stub(tmp_path: Path, args: list[str]) -> tuple[int, str, list[str] | None]:
    capture_path = tmp_path / "collection-env.tsv"
    stub_path = tmp_path / "collection-stub.sh"
    stub_path.write_text(
        """#!/usr/bin/env bash
printf '%s\\t%s\\t%s\\t%s\\t%s\\n' \\
  "$COLLECTION_AUTO_GRADE" "$DATA_DIR" "$1" "$TARGET_BOTTLE_A" "$TARGET_BOTTLE_B" \\
  >"$COLLECTION_CAPTURE_PATH"
exec python3 -m http.server "$WEB_PORT" --bind 127.0.0.1
""",
        encoding="utf-8",
    )
    web_port, collection_port = free_ports(2)
    env = os.environ.copy()
    env.update(
        {
            "COLLECTION_SCRIPT": str(stub_path),
            "COLLECTION_CAPTURE_PATH": str(capture_path),
            "COLLECTION_WEB_PORT": str(collection_port),
            "WEB_HOST": "127.0.0.1",
            "WEB_PORT": str(web_port),
            "PIPELINE_ENABLE": "0",
            "VISUALIZER_ENABLE": "0",
            "STATE_MACHINE_BRIDGE_ENABLE": "0",
            "GENERATE_RUNTIME_STAGE_PRESET": "0",
        }
    )
    process = subprocess.Popen(
        ["bash", str(STAGED_SCRIPT), *args],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if capture_path.exists() or process.poll() is not None:
            break
        time.sleep(0.05)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        output, _ = process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        output, _ = process.communicate(timeout=5)
    captured = capture_path.read_text(encoding="utf-8").rstrip("\n").split("\t") if capture_path.exists() else None
    return process.returncode, output, captured


@pytest.mark.parametrize(
    "args_factory, expected_grade",
    [
        (lambda data_dir: ["--grade", "a", str(data_dir), "7"], "A"),
        (lambda data_dir: [str(data_dir), "--grade=A", "7"], "A"),
        (lambda data_dir: [str(data_dir), "--grade", "f", "7"], "F"),
        (lambda data_dir: [str(data_dir), "7", "--grade", "b"], "B"),
    ],
    ids=["before-positionals", "after-data-dir", "before-index", "after-positionals"],
)
def test_launcher_executes_grade_parsing_and_environment_propagation(
    tmp_path: Path,
    args_factory,
    expected_grade: str,
):
    data_dir = tmp_path / "dataset"
    returncode, output, captured = run_launcher_with_stub(tmp_path, args_factory(data_dir))

    assert captured == [expected_grade, str(data_dir), "7", "", ""], output
    assert returncode == 130


@pytest.mark.parametrize(
    "grade_args, expected_message",
    [
        (["--grade"], "--grade 需要质量等级参数"),
        (["--grade="], "--grade 需要质量等级参数"),
        (["--grade", "C"], "--grade 只能是 A、B 或 F: C"),
    ],
)
def test_invalid_grade_exits_before_collection_module_starts(
    tmp_path: Path,
    grade_args: list[str],
    expected_message: str,
):
    data_dir = tmp_path / "dataset"
    returncode, output, captured = run_launcher_with_stub(
        tmp_path,
        [str(data_dir), "7", *grade_args],
    )

    assert returncode == 1
    assert expected_message in output
    assert captured is None


def extract_shell_function(text: str, name: str, next_name: str) -> str:
    marker = f"{name}() {{"
    next_marker = f"{next_name}() {{"
    return marker + text.split(marker, 1)[1].split(next_marker, 1)[0]


def run_auto_review_harness(tmp_path: Path, metadata_exit: int) -> tuple[dict[str, str], list[str], list[str]]:
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    review_function = extract_shell_function(
        text,
        "finalize_quality_review_from_json",
        "qc_aloha_hdf5_episode",
    )
    save_function = extract_shell_function(
        text,
        "save_current_episode",
        "check_required_topics",
    )
    trace_path = tmp_path / "status-trace.tsv"
    metadata_args_path = tmp_path / "metadata-args.txt"
    metadata_script = tmp_path / "metadata-stub.py"
    metadata_script.write_text(
        """import os
import sys
from pathlib import Path

Path(os.environ["METADATA_ARGS_PATH"]).write_text("\\n".join(sys.argv[1:]), encoding="utf-8")
raise SystemExit(int(os.environ.get("METADATA_EXIT", "0")))
""",
        encoding="utf-8",
    )
    harness_path = tmp_path / "auto-review-harness.sh"
    harness_path.write_text(
        "\n".join(
            [
                "#!/usr/bin/env bash",
                "set -eo pipefail",
                review_function,
                save_function,
                'is_truthy() { [ "${1:-}" = "1" ]; }',
                'log() { :; }',
                'configure_profile() { :; }',
                'metadata_target_args() { METADATA_TARGET_ARGS=(); }',
                'reset_stage_state() { :; }',
                'capture_service_request() { :; }',
                'prepare_episode_metadata() { :; }',
                'start_saved_episode_processing() { PROCESS_CALLED="$1"; }',
                'write_status() { printf "%s\\t%s\\t%s\\t%s\\n" "$1" "$CURRENT_EPISODE" "$QUALITY_REVIEW_PENDING" "$LAST_SAVED_EPISODE" >>"$TRACE_PATH"; }',
                'CAPTURE_RUNNING=1',
                'QUALITY_REVIEW_PENDING=0',
                'QUALITY_PENDING_EPISODE=""',
                'CURRENT_EPISODE=7',
                'LAST_SAVED_EPISODE=""',
                'STAGED_CAPTURE=0',
                'COLLECTION_AUTO_GRADE=A',
                f'COLLECTION_METADATA_SCRIPT={shlex.quote(str(metadata_script))}',
                f'DATA_DIR={shlex.quote(str(tmp_path / "dataset"))}',
                f'ALOHA_DIR={shlex.quote(str(tmp_path / "aloha"))}',
                'COLLECTION_SCENE_INVENTORY_JSON=""',
                'PROCESS_CALLED=""',
                'save_current_episode 1',
                'printf "RESULT\\t%s\\t%s\\t%s\\t%s\\t%s\\n" "$CURRENT_EPISODE" "$QUALITY_REVIEW_PENDING" "$QUALITY_PENDING_EPISODE" "$LAST_SAVED_EPISODE" "$PROCESS_CALLED"',
            ]
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    env.update(
        {
            "TRACE_PATH": str(trace_path),
            "METADATA_ARGS_PATH": str(metadata_args_path),
            "METADATA_EXIT": str(metadata_exit),
        }
    )
    result = subprocess.run(
        ["bash", str(harness_path)],
        cwd=REPO_ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stdout
    result_line = next(line for line in result.stdout.splitlines() if line.startswith("RESULT\t"))
    values = result_line.split("\t")[1:]
    state = dict(zip(["episode", "pending", "pending_episode", "last_saved", "processed"], values))
    trace = trace_path.read_text(encoding="utf-8").splitlines()
    metadata_args = metadata_args_path.read_text(encoding="utf-8").splitlines()
    return state, trace, metadata_args


def test_automatic_review_success_advances_episode_without_review_status(tmp_path: Path):
    state, trace, metadata_args = run_auto_review_harness(tmp_path, metadata_exit=0)

    assert state == {
        "episode": "8",
        "pending": "0",
        "pending_episode": "",
        "last_saved": "7",
        "processed": "7",
    }
    assert not any(line.startswith("review\t") for line in trace)
    assert any(line.startswith("idle\t8\t0\t7") for line in trace)
    assert metadata_args[0] == "review"
    assert metadata_args[metadata_args.index("--grade") + 1] == "A"


def test_automatic_review_failure_stays_pending_for_manual_retry(tmp_path: Path):
    state, trace, metadata_args = run_auto_review_harness(tmp_path, metadata_exit=1)

    assert state == {
        "episode": "7",
        "pending": "1",
        "pending_episode": "7",
        "last_saved": "7",
        "processed": "",
    }
    assert any(line.startswith("review\t7\t1\t7") for line in trace)
    assert metadata_args[metadata_args.index("--grade") + 1] == "A"
