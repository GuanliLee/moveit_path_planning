from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
COLLECTION_SCRIPT = REPO_ROOT / "scripts" / "collection" / "collect_mobile_episode_web.sh"


def test_collection_page_removes_processing_controls_and_frontend_references():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")

    for control_id in (
        "run_convert",
        "run_qc",
        "run_lerobot",
        "background_processing",
    ):
        assert f'id="{control_id}"' not in text
        assert f"el.{control_id}" not in text


def test_failed_start_check_warns_but_does_not_cancel_capture_start():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    start_block = text.split("        start)\n", 1)[1].split("            ;;", 1)[0]
    failed_check_block = start_block.split(
        "                    if ! check_required_topics; then\n",
        1,
    )[1].split("                    else\n", 1)[0]

    assert "红色预警" in failed_check_block
    assert "仍继续启动" in failed_check_block
    assert "return 0" not in failed_check_block
    assert "启动已取消" not in start_block
    assert "start_capture_service_launch" in start_block
    assert "capture_service_request true false" in start_block


def test_failed_self_check_has_persistent_red_page_warning():
    text = COLLECTION_SCRIPT.read_text(encoding="utf-8")
    startup_block = text.split("run_startup_preflight_check() {", 1)[1].split(
        "cleanup_runtime() {",
        1,
    )[0]

    assert 'class="preflight-warning" id="preflightWarning" role="alert"' in text
    assert ".preflight-warning.visible { display: block; }" in text
    assert "const selfCheckFailed = !data.last_check_ok" in text
    assert "启动前自检未通过，但不会阻止采集" in text
    assert startup_block.index("if check_required_topics; then") < startup_block.index(
        "if start_capture_service_launch; then"
    )
    assert "仍会预热采集服务，并允许继续采集" in startup_block
    assert "启动自检失败；未通过前不会开始录制" not in text
