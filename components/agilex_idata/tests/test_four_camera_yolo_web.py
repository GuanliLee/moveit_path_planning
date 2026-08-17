import importlib.util
import json
import os
import re
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import pytest


ROOT = Path(__file__).parents[1]
INFERENCE_DIR = ROOT / "scripts" / "inference"
CORE_PATH = INFERENCE_DIR / "four_camera_yolo_core.py"
WEB_PATH = INFERENCE_DIR / "four_camera_yolo_web.py"
HTML_PATH = INFERENCE_DIR / "four_camera_yolo_web.html"
LAUNCHER = INFERENCE_DIR / "run_four_camera_yolo_web.sh"
OPENER = build_opener(ProxyHandler({}))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def core():
    return load_module("four_camera_yolo_core_web_test", CORE_PATH)


@pytest.fixture
def web_module(monkeypatch):
    monkeypatch.syspath_prepend(str(INFERENCE_DIR))
    return load_module("four_camera_yolo_web_test", WEB_PATH)


def request_bytes(url, method="GET", body=None, headers=None):
    request = Request(url, data=body, headers=headers or {}, method=method)
    with OPENER.open(request, timeout=2.0) as response:
        return SimpleNamespace(
            status=response.status,
            headers=response.headers,
            body=response.read(),
        )


def get_json(url):
    return json.loads(request_bytes(url).body)


def post_json(url, payload):
    response = request_bytes(
        url,
        method="POST",
        body=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    return json.loads(response.body)


def request_error(url, method="GET", body=None, headers=None):
    with pytest.raises(HTTPError) as captured:
        request_bytes(url, method=method, body=body, headers=headers)
    return captured.value.code, json.loads(captured.value.read())


@pytest.fixture
def web_server(core, web_module, tmp_path):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_health(
        True,
        {"ok": True, "api_version": "v1"},
        ({"name": "Wanglaoji", "supported_by_deployed_model": True},),
        "",
        1.0,
    )
    state.update_frame("left", b"latest-left", 10, time.monotonic())
    generation = state.apply_config("Wanglaoji", 3.0, False, {})
    frame = state.frame_snapshot("left")
    state.publish_result(
        "left",
        generation,
        frame,
        core.InferenceResult(
            overlay_jpeg=b"overlay-left",
            count=1,
            class_counts={"Wanglaoji": 1},
            service_latency_ms=8.0,
            round_trip_ms=10.0,
            completed_at=time.monotonic(),
        ),
    )
    html_path = tmp_path / "console.html"
    html_path.write_text("<!doctype html><title>YOLO Console</title>", encoding="utf-8")
    server = web_module.create_server("127.0.0.1", 0, state, html_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    fixture = SimpleNamespace(
        url=f"http://127.0.0.1:{server.server_port}", state=state
    )
    try:
        yield fixture
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2.0)


def test_home_status_classes_and_config_round_trip(web_server):
    home = request_bytes(web_server.url + "/")
    assert home.status == 200
    assert home.headers["Content-Type"] == "text/html; charset=utf-8"
    assert b"YOLO Console" in home.body

    status = get_json(web_server.url + "/api/status")
    assert list(status["cameras"]) == ["left", "front", "right", "head"]
    assert status["interval_sec"] == 3.0
    classes = get_json(web_server.url + "/api/classes")
    assert classes["classes"][0]["name"] == "Wanglaoji"

    updated = post_json(
        web_server.url + "/api/config",
        {
            "targets": ["Wanglaoji", "Sprite"],
            "interval_sec": 4,
            "paused": False,
            "cameras": {
                key: {"enabled": key != "head"} for key in status["cameras"]
            },
        },
    )
    assert updated["ok"] is True
    new_status = get_json(web_server.url + "/api/status")
    assert new_status["targets"] == ["Wanglaoji", "Sprite"]
    assert new_status["interval_sec"] == 4.0
    assert new_status["cameras"]["head"]["enabled"] is False


@pytest.mark.parametrize("kind, expected", [
    ("latest", b"latest-left"),
    ("source", b"latest-left"),
    ("overlay", b"overlay-left"),
])
def test_camera_jpeg_endpoints_are_uncached(web_server, kind, expected):
    response = request_bytes(
        web_server.url + f"/api/cameras/left/{kind}.jpg?seq=1"
    )
    assert response.body == expected
    assert response.headers["Cache-Control"] == "no-store"
    assert response.headers["Content-Type"] == "image/jpeg"
    assert int(response.headers["Content-Length"]) == len(expected)


def test_missing_image_and_unknown_routes_are_structured_errors(web_server):
    status, payload = request_error(
        web_server.url + "/api/cameras/front/overlay.jpg?seq=1"
    )
    assert status == 404
    assert payload == {"ok": False, "error": "image not available"}
    status, payload = request_error(web_server.url + "/api/not-found")
    assert status == 404
    assert payload["error"] == "not found"


@pytest.mark.parametrize(
    "payload, message",
    [
        (
            {"targets": ["x"], "interval_sec": 0.19, "paused": False, "cameras": {}},
            "between 0.2 and 60.0",
        ),
        (
            {"targets": ["x"], "interval_sec": 3, "paused": "no", "cameras": {}},
            "paused must be boolean",
        ),
        (
            {
                "targets": ["x"],
                "interval_sec": 3,
                "paused": False,
                "cameras": {"side": {"enabled": True}},
            },
            "unknown camera",
        ),
    ],
)
def test_config_rejects_invalid_payload_atomically(web_server, payload, message):
    before = get_json(web_server.url + "/api/status")
    status, error = request_error(
        web_server.url + "/api/config",
        method="POST",
        body=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    assert status == 400
    assert message in error["error"]
    assert get_json(web_server.url + "/api/status")["generation"] == before[
        "generation"
    ]


def test_config_rejects_invalid_json_and_large_bodies(web_server):
    status, payload = request_error(
        web_server.url + "/api/config",
        method="POST",
        body=b"{bad-json",
        headers={"Content-Type": "application/json"},
    )
    assert status == 400
    assert "JSON" in payload["error"]

    status, payload = request_error(
        web_server.url + "/api/config",
        method="POST",
        body=b"x" * (64 * 1024 + 1),
        headers={"Content-Type": "application/json"},
    )
    assert status == 413
    assert "large" in payload["error"]


def test_json_responses_are_uncached_and_have_exact_length(web_server):
    response = request_bytes(web_server.url + "/api/status")
    assert response.headers["Cache-Control"] == "no-store"
    assert response.headers["Content-Type"] == "application/json; charset=utf-8"
    assert int(response.headers["Content-Length"]) == len(response.body)


def test_home_page_contains_mask_first_controls_and_four_camera_cards():
    html = HTML_PATH.read_text(encoding="utf-8")
    assert '<html lang="zh-CN">' in html
    assert 'name="viewport"' in html
    assert 'id="target-picker"' in html
    assert 'id="target-picker-toggle"' in html
    assert 'id="target-menu"' in html
    assert 'id="target-search"' in html
    assert 'id="target-options"' in html
    assert 'id="target-empty"' in html
    assert 'id="clear-targets"' in html
    assert 'aria-multiselectable="true"' in html
    assert 'id="target-input"' not in html
    assert 'id="target-tags"' in html
    assert 'id="interval-sec"' in html
    assert 'min="0.2"' in html
    assert 'max="60"' in html
    assert 'step="0.1"' in html
    assert 'value="0.2"' in html
    assert 'id="apply-config"' in html
    assert 'id="pause-all"' in html
    assert 'id="yolo-health"' in html
    assert 'id="global-error"' in html
    for camera in ("left", "front", "right", "head"):
        assert f'data-camera="{camera}"' in html
        assert f'id="{camera}-image"' in html
        assert f'id="{camera}-mask-view"' in html
        assert f'id="{camera}-source-view"' in html
        assert f'id="{camera}-enabled"' in html
        assert f'id="{camera}-metrics"' in html
        assert f'id="{camera}-counts"' in html
    assert "setInterval(refreshStatus, 200)" in html


def test_home_page_implements_tags_sequence_images_and_safe_text_rendering():
    html = HTML_PATH.read_text(encoding="utf-8")
    assert "selectedTargets" in html
    assert "renderTargetOptions" in html
    assert "renderTargetSummary" in html
    assert "setTargetMenuOpen" in html
    assert 'event.key === "Escape"' in html
    assert "/api/classes" in html
    assert "/api/config" in html
    assert "/api/status" in html
    assert "result_sequence" in html
    assert "input_sequence" in html
    assert "source_stamp_sec" in html
    assert "check_age_sec" in html
    assert "源帧时间" in html
    assert "overlay.jpg?seq=" in html
    assert "source.jpg?seq=" in html
    assert "latest.jpg?seq=" in html
    assert ".textContent" in html
    assert "innerHTML" not in html
    assert "http://" not in html and "https://" not in html


def test_home_page_is_responsive_and_accessible():
    html = HTML_PATH.read_text(encoding="utf-8")
    assert "grid-template-columns: repeat(2" in html
    assert "@media (max-width: 900px)" in html
    assert 'role="status"' in html
    assert 'aria-live="polite"' in html
    assert 'aria-pressed="true"' in html
    assert ':focus-visible' in html


def test_inline_javascript_has_valid_syntax():
    html = HTML_PATH.read_text(encoding="utf-8")
    scripts = re.findall(r"<script>(.*?)</script>", html, flags=re.DOTALL)
    assert len(scripts) == 1
    result = subprocess.run(
        ["node", "--check"],
        input=scripts[0],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_web_module_imports_without_ros_and_cli_defaults_are_stable(web_module):
    assert "rclpy" not in web_module.__dict__
    args = web_module.parse_args([])
    assert args.host == "0.0.0.0"
    assert args.port == 7788
    assert args.ros_domain_id == 99
    assert args.yolo_url == "http://192.168.4.121:7881"
    assert args.default_interval_sec == 0.2
    assert args.request_timeout == 10.0


@pytest.mark.parametrize(
    "argv",
    [
        ["--port", "0"],
        ["--port", "65536"],
        ["--default-interval-sec", "0.19"],
        ["--request-timeout", "0"],
        ["--request-timeout", "nan"],
    ],
)
def test_cli_rejects_invalid_network_and_timing_values(web_module, argv):
    with pytest.raises(SystemExit):
        web_module.parse_args(argv)


def test_create_server_rejects_occupied_port_without_closing_owner(
    web_module, core, tmp_path
):
    owner = socket.socket()
    owner.bind(("127.0.0.1", 0))
    owner.listen()
    html_path = tmp_path / "index.html"
    html_path.write_text("ok", encoding="utf-8")
    try:
        with pytest.raises(OSError):
            web_module.create_server(
                "127.0.0.1",
                owner.getsockname()[1],
                core.StateStore(3.0),
                html_path,
            )
        assert owner.fileno() >= 0
    finally:
        owner.close()


def test_launcher_defaults_environment_and_port_safety():
    text = LAUNCHER.read_text(encoding="utf-8")
    assert 'WEB_PORT="${WEB_PORT:-7788}"' in text
    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"' in text
    assert 'YOLO_URL="${YOLO_URL:-http://192.168.4.121:7881}"' in text
    assert 'DEFAULT_INTERVAL_SEC="${DEFAULT_INTERVAL_SEC:-0.2}"' in text
    assert "端口 ${WEB_PORT} 已被占用" in text
    assert "--default-interval-sec" in text
    assert "kill " not in text and "fuser -k" not in text


def test_launcher_can_source_ros_setup_under_strict_shell_mode():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        free_port = probe.getsockname()[1]
    environment = os.environ.copy()
    environment.update(
        {
            "WEB_HOST": "127.0.0.1",
            "WEB_PORT": str(free_port),
            "PYTHON_BIN": "/bin/true",
        }
    )
    result = subprocess.run(
        ["bash", str(LAUNCHER)],
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_started_worker_group_runs_four_way_round_and_isolates_failure(core):
    state = core.StateStore(3.0)
    state.apply_config(["Wanglaoji", "Sprite"], 3.0, False, {})
    for index, key in enumerate(core.CAMERA_KEYS):
        state.update_frame(key, f"source-{key}".encode(), index, time.monotonic())

    barrier = threading.Barrier(4)
    first_round_complete = threading.Event()
    lock = threading.Lock()
    completions = []

    class Client:
        def predict(self, camera, frame, targets):
            assert targets == ("Wanglaoji", "Sprite")
            barrier.wait(timeout=2.0)
            try:
                if camera == "right":
                    raise RuntimeError("forced right failure")
                return core.InferenceResult(
                    overlay_jpeg=f"overlay-{camera}".encode(),
                    count=1,
                    class_counts={"Wanglaoji": 1, "Sprite": 0},
                    service_latency_ms=8.0,
                    round_trip_ms=10.0,
                    completed_at=time.monotonic(),
                )
            finally:
                with lock:
                    completions.append(camera)
                    if len(completions) == 4:
                        first_round_complete.set()

    stop_event = threading.Event()
    group = core.WorkerGroup(state, Client, stop_event, time.monotonic)
    group.start()
    try:
        assert first_round_complete.wait(2.0)
        assert group.wait_for_idle(2.0)
    finally:
        group.stop(timeout=2.0)

    status = state.status(time.monotonic())
    assert status["cameras"]["right"]["error"] == "forced right failure"
    for key in ("left", "front", "head"):
        assert state.image(key, "source") == f"source-{key}".encode()
        assert state.image(key, "overlay") == f"overlay-{key}".encode()
