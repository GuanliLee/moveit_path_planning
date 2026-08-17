import base64
import importlib.util
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace

import pytest


CORE_PATH = (
    Path(__file__).parents[1] / "scripts" / "inference" / "four_camera_yolo_core.py"
)


@pytest.fixture
def core():
    spec = importlib.util.spec_from_file_location("four_camera_yolo_core_http", CORE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def fake_jpeg(width: int, height: int) -> bytes:
    sof_payload = (
        b"\x08"
        + height.to_bytes(2, "big")
        + width.to_bytes(2, "big")
        + b"\x03\x01\x11\x00\x02\x11\x00\x03\x11\x00"
    )
    return (
        b"\xff\xd8\xff\xc0"
        + (len(sof_payload) + 2).to_bytes(2, "big")
        + sof_payload
        + b"\xff\xd9"
    )


@pytest.fixture
def fake_yolo():
    state = SimpleNamespace(mode="success", requests=[])

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/health":
                return self._json(200, {"ok": True, "api_version": "v1"})
            if self.path == "/classes":
                return self._json(
                    200,
                    {
                        "ok": True,
                        "classes": [
                            {
                                "class_id": 13,
                                "name": "Wanglaoji",
                                "supported_by_deployed_model": True,
                            }
                        ],
                    },
                )
            self._json(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            if self.path != "/predict":
                return self._json(404, {"ok": False, "error": "not found"})
            body = self.rfile.read(int(self.headers["Content-Length"]))
            request = json.loads(body)
            state.requests.append(request)
            payload = {
                "ok": True,
                "image": {"width": 640, "height": 480},
                "count": 1,
                "targets": [{"class_name": "Wanglaoji", "count": 1}],
                "detections": [
                    {
                        "class_name": "Wanglaoji",
                        "confidence": 0.9,
                        "bbox_xyxy": [1, 1, 20, 20],
                    }
                ],
                "latency_ms": 8.5,
                "overlay_jpeg_base64": base64.b64encode(
                    fake_jpeg(640, 480)
                ).decode("ascii"),
            }
            if state.mode == "http_error":
                return self._json(503, {"ok": False, "error": "busy"})
            if state.mode == "bad_json":
                return self._bytes(200, b"{not-json", "application/json")
            if state.mode == "not_ok":
                payload = {"ok": False, "error": "inference failed"}
            elif state.mode == "missing_overlay":
                payload.pop("overlay_jpeg_base64")
            elif state.mode == "bad_base64":
                payload["overlay_jpeg_base64"] = "%%%"
            elif state.mode == "missing_bbox":
                payload["detections"][0].pop("bbox_xyxy")
            elif state.mode == "invalid_bbox":
                payload["detections"][0]["bbox_xyxy"] = [20, 1, 1, 20]
            elif state.mode == "malformed_optional_mask":
                payload["detections"][0]["mask_polygon_xy"] = "ignored"
            elif state.mode == "temporal_recovered_without_polygon":
                payload["detections"][0].update(
                    {
                        "mask_polygon_xy": [],
                        "observed": False,
                        "temporal_recovered": True,
                    }
                )
            elif state.mode == "count_mismatch":
                payload["count"] = 2
            elif state.mode == "dimension_mismatch":
                payload["image"] = {"width": 320, "height": 240}
            self._json(200, payload)

        def _json(self, status, payload):
            self._bytes(
                status,
                json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                "application/json",
            )

        def _bytes(self, status, payload, content_type):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, _format, *_args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    fixture = SimpleNamespace(
        url=f"http://127.0.0.1:{server.server_port}",
        requests=state.requests,
        set_mode=lambda value: setattr(state, "mode", value),
    )
    try:
        yield fixture
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2.0)


def test_health_classes_and_predict_use_direct_json_api(fake_yolo, core):
    client = core.YoloHttpClient(fake_yolo.url, timeout=1.0)
    assert client.health() == {"ok": True, "api_version": "v1"}
    assert client.classes()["classes"][0]["name"] == "Wanglaoji"

    source = fake_jpeg(640, 480)
    frame = core.FrameSnapshot(source, 4, 9, 1.0)
    result = client.predict("left", frame, ("Wanglaoji", "Sprite"))

    assert core.jpeg_dimensions(result.overlay_jpeg) == (640, 480)
    assert result.count == 1
    assert result.class_counts == {"Wanglaoji": 1}
    assert result.service_latency_ms == 8.5
    request = fake_yolo.requests[-1]
    assert request["return_overlay"] is True
    assert request["targets"] == ["Wanglaoji", "Sprite"]
    assert request["request_id"].startswith("four-camera-left-")
    assert request["stream_id"] == "four-camera-left"
    assert request["temporal"] is True
    assert request["reset_temporal"] is True
    assert "mask" not in request
    assert base64.b64decode(request["image_base64"], validate=True) == source


def test_predict_generates_unique_request_ids(fake_yolo, core):
    client = core.YoloHttpClient(fake_yolo.url, timeout=1.0)
    frame = core.FrameSnapshot(fake_jpeg(640, 480), 4, 9, 1.0)
    client.predict("left", frame, ("Wanglaoji",))
    client.predict("left", frame, ("Wanglaoji",))
    assert fake_yolo.requests[-2]["request_id"] != fake_yolo.requests[-1]["request_id"]


def test_predict_resets_temporal_state_when_targets_change(fake_yolo, core):
    client = core.YoloHttpClient(fake_yolo.url, timeout=1.0)
    frame = core.FrameSnapshot(fake_jpeg(640, 480), 4, 9, 1.0)

    client.predict("left", frame, ("Wanglaoji",))
    client.predict("left", frame, ("Wanglaoji",))
    client.predict("left", frame, ("Sprite",))

    requests = fake_yolo.requests[-3:]
    assert [item["stream_id"] for item in requests] == [
        "four-camera-left",
        "four-camera-left",
        "four-camera-left",
    ]
    assert [item["temporal"] for item in requests] == [True, True, True]
    assert [item["reset_temporal"] for item in requests] == [True, False, True]


def test_predict_accepts_default_bbox_only_response(fake_yolo, core):
    result = core.YoloHttpClient(fake_yolo.url, 1.0).predict(
        "left",
        core.FrameSnapshot(fake_jpeg(640, 480), 1, 1, 1.0),
        ("Wanglaoji",),
    )
    assert result.count == 1


def test_predict_ignores_optional_mask_geometry(fake_yolo, core):
    fake_yolo.set_mode("malformed_optional_mask")
    result = core.YoloHttpClient(fake_yolo.url, 1.0).predict(
        "left",
        core.FrameSnapshot(fake_jpeg(640, 480), 1, 1, 1.0),
        ("Wanglaoji",),
    )
    assert result.count == 1


def test_client_ignores_broken_process_proxy(fake_yolo, core, monkeypatch):
    monkeypatch.setenv("HTTP_PROXY", "http://127.0.0.1:1")
    monkeypatch.setenv("HTTPS_PROXY", "http://127.0.0.1:1")
    monkeypatch.setenv("NO_PROXY", "")
    assert core.YoloHttpClient(fake_yolo.url, 1.0).health()["ok"] is True


@pytest.mark.parametrize(
    "mode, message",
    [
        ("not_ok", "inference failed"),
        ("bad_json", "JSON"),
        ("http_error", "HTTP 503"),
        ("missing_overlay", "overlay_jpeg_base64"),
        ("bad_base64", "base64"),
        ("missing_bbox", "bbox_xyxy"),
        ("invalid_bbox", "bbox_xyxy"),
        ("count_mismatch", "count"),
        ("dimension_mismatch", "dimensions"),
    ],
)
def test_predict_rejects_malformed_service_response(
    fake_yolo, core, mode, message
):
    fake_yolo.set_mode(mode)
    with pytest.raises(core.YoloProtocolError, match=message):
        core.YoloHttpClient(fake_yolo.url, 1.0).predict(
            "left",
            core.FrameSnapshot(fake_jpeg(640, 480), 1, 1, 1.0),
            ("Wanglaoji",),
        )


@pytest.mark.parametrize("payload", [b"", b"not-jpeg", b"\xff\xd8\xff\xc0\x00"])
def test_jpeg_dimensions_rejects_invalid_data(core, payload):
    with pytest.raises(core.YoloProtocolError, match="JPEG"):
        core.jpeg_dimensions(payload)
