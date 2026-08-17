# Four-Camera Live YOLO Web Console Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a port-7788 Web console that samples the newest frames from four ROS cameras in one globally timed concurrent round, sends a shared multi-target selection to the remote YOLO service, and shows each matching YOLO mask overlay with a source-image toggle.

**Architecture:** Keep ROS imports at the executable boundary and place deterministic state, validation, HTTP-client, and scheduling logic in a standard-library core module. A global scheduler offers one round to four independently guarded workers every configured interval; they share a lock-protected latest-frame store, while a `ThreadingHTTPServer` exposes compact JSON state and cache-busted JPEG endpoints to a responsive two-by-two browser UI.

**Tech Stack:** Python 3.10 standard library, ROS 2 Humble `rclpy`, `sensor_msgs/msg/CompressedImage`, browser HTML/CSS/JavaScript, Bash, pytest.

## Global Constraints

- Default Web bind is `0.0.0.0:7788`; an occupied port must fail closed without killing its listener.
- Default ROS domain is `99` and default YOLO URL is `http://192.168.4.121:7881`.
- Camera keys and compressed topics are exactly `left`, `front`, `right`, `head` mapped to `/camera_{l,f,r,h}/color/image_raw/compressed`.
- The global prompt is shared by all cameras and accepts comma- or newline-separated names.
- One global interval defaults to `3.0` seconds and must remain within `1.0` through `60.0` seconds.
- Each camera may have at most one YOLO request in flight; only the newest camera frame is retained.
- Source and overlay images are published atomically from the same inference request.
- The HTTP client must bypass process proxy variables explicitly.
- Do not add FastAPI, Uvicorn, WebSocket, or JavaScript build dependencies.
- Implement with test-driven development and commit after every task.

## File Structure

- Create `scripts/inference/four_camera_yolo_core.py`: camera/config dataclasses, target/interval validation, latest-frame state, YOLO HTTP client, prediction validation, global round scheduler, and health cache.
- Create `scripts/inference/four_camera_yolo_web.py`: standard-library HTTP handler, API routing, ROS adapter, CLI, lifecycle, and signal handling.
- Create `scripts/inference/four_camera_yolo_web.html`: responsive control bar, multi-target tags, and four mask-first camera cards with polling JavaScript.
- Create `scripts/inference/run_four_camera_yolo_web.sh`: ROS environment setup, defaults, port preflight, and executable launch.
- Create `tests/test_four_camera_yolo_core.py`: deterministic state, configuration, frame, scheduling, and prediction tests.
- Create `tests/test_four_camera_yolo_http.py`: fake YOLO service and direct-client transport/response tests.
- Create `tests/test_four_camera_yolo_web.py`: Web API, HTML, image, and fake four-worker integration tests.
- Modify `scripts/inference/README.md`: operator command, URL, prompt syntax, controls, environment overrides, and troubleshooting.

---

### Task 1: Deterministic Configuration and Latest-Frame State

**Files:**
- Create: `scripts/inference/four_camera_yolo_core.py`
- Create: `tests/test_four_camera_yolo_core.py`

**Interfaces:**
- Produces: `CAMERAS: tuple[CameraDefinition, ...]`, `parse_targets(value) -> tuple[str, ...]`, `validate_interval_sec(value) -> float`, `FrameSnapshot`, `InferenceResult`, and `StateStore`.
- `StateStore` methods used later: `update_frame`, `frame_snapshot`, `apply_config`, `config_snapshot`, `publish_result`, `record_error`, `image`, and `status`.

- [ ] **Step 1: Write failing prompt and interval tests**

```python
def test_parse_targets_accepts_commas_newlines_and_stable_deduplication(core):
    assert core.parse_targets(" Wanglaoji, Sprite\nWanglaoji ,, Daily C Grape Juice ") == (
        "Wanglaoji",
        "Sprite",
        "Daily C Grape Juice",
    )


@pytest.mark.parametrize("value, expected", [(1, 1.0), ("3", 3.0), (60, 60.0)])
def test_validate_interval_sec_accepts_closed_range(core, value, expected):
    assert core.validate_interval_sec(value) == expected


@pytest.mark.parametrize("value", [None, True, 0.99, 60.01, "nan", "bad"])
def test_validate_interval_sec_rejects_invalid_values(core, value):
    with pytest.raises(ValueError):
        core.validate_interval_sec(value)
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_core.py -k 'parse_targets or validate_interval'
```

Expected: FAIL because the partial core still exposes the old `validate_fps`
contract and has no `validate_interval_sec`.

- [ ] **Step 3: Implement camera constants and validation**

```python
@dataclass(frozen=True)
class CameraDefinition:
    key: str
    label: str
    topic: str


CAMERAS = (
    CameraDefinition("left", "左视角", "/camera_l/color/image_raw/compressed"),
    CameraDefinition("front", "前视角", "/camera_f/color/image_raw/compressed"),
    CameraDefinition("right", "右视角", "/camera_r/color/image_raw/compressed"),
    CameraDefinition("head", "头部视角", "/camera_h/color/image_raw/compressed"),
)
CAMERA_KEYS = tuple(camera.key for camera in CAMERAS)
MIN_INTERVAL_SEC = 1.0
MAX_INTERVAL_SEC = 60.0


def parse_targets(value: str | list[str] | tuple[str, ...]) -> tuple[str, ...]:
    pieces = re.split(r"[,\n]", value) if isinstance(value, str) else list(value)
    result: list[str] = []
    seen: set[str] = set()
    for piece in pieces:
        name = str(piece).strip()
        if name and name not in seen:
            seen.add(name)
            result.append(name)
    return tuple(result)


def validate_interval_sec(value: object) -> float:
    if isinstance(value, bool) or value is None:
        raise ValueError("interval_sec must be a number")
    try:
        interval_sec = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("interval_sec must be a number") from exc
    if not math.isfinite(interval_sec) or not MIN_INTERVAL_SEC <= interval_sec <= MAX_INTERVAL_SEC:
        raise ValueError(
            f"interval_sec must be between {MIN_INTERVAL_SEC} and {MAX_INTERVAL_SEC}"
        )
    return interval_sec
```

- [ ] **Step 4: Write failing latest-frame and generation tests**

```python
def test_latest_frame_replaces_previous_without_queue(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_frame("left", b"first", stamp_ns=10, received_at=1.0)
    state.update_frame("left", b"second", stamp_ns=20, received_at=2.0)
    frame = state.frame_snapshot("left")
    assert frame.jpeg == b"second"
    assert frame.sequence == 2
    assert frame.stamp_ns == 20


def test_result_from_old_config_generation_is_discarded(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.update_frame("left", b"source", stamp_ns=10, received_at=1.0)
    generation = state.apply_config(targets="Wanglaoji", interval_sec=3.0, paused=False, cameras={})
    frame = state.frame_snapshot("left")
    state.apply_config(targets="Sprite", interval_sec=3.0, paused=False, cameras={})
    accepted = state.publish_result(
        "left",
        generation,
        frame,
        core.InferenceResult(
            overlay_jpeg=b"overlay",
            count=1,
            class_counts={"Wanglaoji": 1},
            service_latency_ms=10.0,
            round_trip_ms=12.0,
            completed_at=3.0,
        ),
    )
    assert accepted is False
    assert state.image("left", "overlay") is None


def test_applying_new_prompt_clears_previous_prompt_images(core):
    state = state_with_published_left_result(core, targets="Wanglaoji")
    assert state.image("left", "overlay") is not None
    state.apply_config("Sprite", 3.0, False, {})
    assert state.image("left", "source") is None
    assert state.image("left", "overlay") is None
```

- [ ] **Step 5: Run the new state tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_core.py -k 'latest_frame or old_config'
```

Expected: FAIL because `StateStore`, `FrameSnapshot`, and `InferenceResult` are undefined.

- [ ] **Step 6: Implement atomic state and status snapshots**

```python
@dataclass(frozen=True)
class FrameSnapshot:
    jpeg: bytes
    sequence: int
    stamp_ns: int
    received_at: float


@dataclass(frozen=True)
class InferenceResult:
    overlay_jpeg: bytes
    count: int
    class_counts: dict[str, int]
    service_latency_ms: float
    round_trip_ms: float
    completed_at: float


@dataclass(frozen=True)
class CameraConfig:
    enabled: bool = True


@dataclass(frozen=True)
class WorkerConfigSnapshot:
    generation: int
    targets: tuple[str, ...]
    paused: bool
    enabled: bool
    interval_sec: float


@dataclass(frozen=True)
class GlobalConfigSnapshot:
    generation: int
    targets: tuple[str, ...]
    paused: bool
    interval_sec: float


class StateStore:
    def update_frame(self, camera: str, jpeg: bytes, stamp_ns: int, received_at: float) -> int:
        self._require_camera(camera)
        with self._lock:
            runtime = self._runtime[camera]
            runtime.input_sequence += 1
            runtime.latest_frame = FrameSnapshot(bytes(jpeg), runtime.input_sequence, stamp_ns, received_at)
            return runtime.input_sequence

    def publish_result(self, camera, generation, frame, result) -> bool:
        with self._lock:
            if generation != self._generation:
                return False
            runtime = self._runtime[camera]
            runtime.result_sequence += 1
            runtime.source_jpeg = frame.jpeg
            runtime.source_frame_sequence = frame.sequence
            runtime.overlay_jpeg = result.overlay_jpeg
            runtime.last_result = result
            runtime.error = ""
            return True
```

Implement `apply_config` as one locked update that validates every supplied
camera key/field before changing any state. Implement `status(now)` as a deep
JSON-safe copy containing config generation, prompt, pause state, health state,
and all camera sequences/ages/metrics without JPEG or polygon payloads.

```python
def apply_config(self, targets, interval_sec: object, paused: bool, cameras: dict[str, dict]) -> int:
    parsed_targets = parse_targets(targets)
    validated_interval = validate_interval_sec(interval_sec)
    if not isinstance(paused, bool):
        raise ValueError("paused must be boolean")
    updates = self._validated_camera_updates(cameras)
    with self._lock:
        targets_changed = parsed_targets != self._targets
        self._generation += 1
        self._targets = parsed_targets
        self._interval_sec = validated_interval
        self._paused = paused
        for key, values in updates.items():
            self._configs[key] = replace(self._configs[key], **values)
        if targets_changed:
            for runtime in self._runtime.values():
                runtime.source_jpeg = None
                runtime.overlay_jpeg = None
                runtime.last_result = None
                runtime.result_sequence = 0
                runtime.error = ""
        return self._generation


def apply_config_payload(self, payload: object) -> int:
    if not isinstance(payload, dict):
        raise ValueError("configuration must be a JSON object")
    unknown = set(payload) - {"targets", "interval_sec", "paused", "cameras"}
    if unknown:
        raise ValueError(f"unknown configuration field: {sorted(unknown)[0]}")
    cameras = payload.get("cameras", {})
    if not isinstance(cameras, dict):
        raise ValueError("cameras must be an object")
    return self.apply_config(
        payload.get("targets", ""), payload.get("interval_sec", self._interval_sec),
        payload.get("paused", False), cameras,
    )


def image(self, camera: str, kind: str) -> bytes | None:
    self._require_camera(camera)
    with self._lock:
        runtime = self._runtime[camera]
        value = {
            "latest": runtime.latest_frame.jpeg if runtime.latest_frame else None,
            "source": runtime.source_jpeg,
            "overlay": runtime.overlay_jpeg,
        }.get(kind)
        if kind not in {"latest", "source", "overlay"}:
            raise ValueError(f"unknown image kind: {kind}")
        return value


def config_snapshot(self, camera: str) -> WorkerConfigSnapshot:
    self._require_camera(camera)
    with self._lock:
        camera_config = self._configs[camera]
        return WorkerConfigSnapshot(
            self._generation, self._targets, self._paused,
            camera_config.enabled, self._interval_sec,
        )


def global_config_snapshot(self) -> GlobalConfigSnapshot:
    with self._lock:
        return GlobalConfigSnapshot(
            self._generation, self._targets, self._paused, self._interval_sec
        )


def record_error(self, camera: str, generation: int, message: str, when: float) -> bool:
    with self._lock:
        if generation != self._generation:
            return False
        runtime = self._runtime[camera]
        runtime.error = message
        runtime.error_at = when
        return True


def update_health(self, ok: bool, health: dict, classes: tuple[dict, ...], error: str, checked_at: float) -> None:
    with self._lock:
        self._health = {"ok": ok, "details": dict(health), "error": error, "checked_at": checked_at}
        if ok:
            self._classes = tuple(dict(item) for item in classes)


def classes_snapshot(self) -> list[dict]:
    with self._lock:
        return [dict(item) for item in self._classes]
```

`record_attempt(camera, started, completed)` stores a bounded deque of recent
completion timestamps and durations per camera:

```python
def record_attempt(self, camera: str, started: float, completed: float) -> None:
    with self._lock:
        runtime = self._runtime[camera]
        runtime.attempts.append((started, completed))
        cutoff = completed - 10.0
        while runtime.attempts and runtime.attempts[0][1] < cutoff:
            runtime.attempts.popleft()
```

`status(now)` calculates
`effective_rate_hz`, `topic_online`, `result_age_sec`, and `result_stale` from those
bounded values and returns this stable top-level shape:

```python
{
    "generation": 1,
    "targets": ["Wanglaoji"],
    "interval_sec": 3.0,
    "paused": False,
    "config_error": "",
    "yolo": {"ok": True, "error": "", "checked_at": 1.0},
    "cameras": {
        "left": {
            "enabled": True, "input_sequence": 4,
            "result_sequence": 2, "topic_online": True,
            "effective_rate_hz": 0.33, "service_latency_ms": 12.0,
            "round_trip_ms": 20.0, "result_age_sec": 0.2,
            "result_stale": False, "count": 1,
            "class_counts": {"Wanglaoji": 1}, "error": "",
        },
    },
}
```

- [ ] **Step 7: Run the complete core tests and verify GREEN**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_core.py
```

Expected: all Task 1 tests pass.

- [ ] **Step 8: Commit Task 1**

```bash
git add scripts/inference/four_camera_yolo_core.py tests/test_four_camera_yolo_core.py
git commit -m "feat: add four-camera yolo state core"
```

---

### Task 2: Direct YOLO HTTP Client and Response Validation

**Files:**
- Modify: `scripts/inference/four_camera_yolo_core.py`
- Create: `tests/test_four_camera_yolo_http.py`

**Interfaces:**
- Consumes: `FrameSnapshot` and `parse_targets` from Task 1.
- Produces: `YoloHttpClient(base_url, timeout)`, `YoloHttpClient.health()`, `classes()`, and `predict(camera, frame, targets) -> InferenceResult`.
- Produces: `YoloProtocolError`, used by workers and API diagnostics.

- [ ] **Step 1: Write a controllable fake YOLO server fixture**

```python
class FakeYoloHandler(BaseHTTPRequestHandler):
    response_mode = "success"
    received = []

    def do_GET(self):
        payload = {"ok": True, "api_version": "v1"} if self.path == "/health" else {
            "ok": True,
            "classes": [{"class_id": 13, "name": "Wanglaoji", "supported_by_deployed_model": True}],
        }
        self._json(payload)

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        type(self).received.append(request)
        overlay = base64.b64encode(fake_jpeg(640, 480)).decode("ascii")
        payload = {
            "ok": True,
            "image": {"width": 640, "height": 480},
            "count": 1,
            "targets": [{"class_name": "Wanglaoji", "count": 1}],
            "detections": [{"class_name": "Wanglaoji", "mask_polygon_xy": [[1, 1], [2, 1], [2, 2]]}],
            "latency_ms": 8.5,
            "overlay_jpeg_base64": overlay,
        }
        mode = type(self).response_mode
        if mode == "bad_json":
            return self._bytes(200, b"{not-json", "application/json")
        if mode == "not_ok":
            payload = {"ok": False, "error": "inference failed"}
        elif mode == "missing_overlay":
            payload.pop("overlay_jpeg_base64")
        elif mode == "bad_base64":
            payload["overlay_jpeg_base64"] = "%%%"
        elif mode == "missing_polygon":
            payload["detections"][0].pop("mask_polygon_xy")
        elif mode == "count_mismatch":
            payload["count"] = 2
        elif mode == "dimension_mismatch":
            payload["image"] = {"width": 320, "height": 240}
        self._json(payload)

    def _json(self, payload):
        self._bytes(200, json.dumps(payload).encode("utf-8"), "application/json")

    def _bytes(self, status, payload, content_type):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format, *_args):
        pass
```

Run the handler on `127.0.0.1:0` in a daemon thread and reset its class state in
the fixture teardown.

- [ ] **Step 2: Write failing client success and direct-proxy tests**

```python
def test_predict_posts_targets_and_decodes_overlay(fake_yolo, core):
    client = core.YoloHttpClient(fake_yolo.url, timeout=1.0)
    frame = core.FrameSnapshot(fake_jpeg(640, 480), 4, 9, 1.0)
    result = client.predict("left", frame, ("Wanglaoji",))
    assert core.jpeg_dimensions(result.overlay_jpeg) == (640, 480)
    assert result.class_counts == {"Wanglaoji": 1}
    assert fake_yolo.requests[-1]["return_overlay"] is True
    assert fake_yolo.requests[-1]["targets"] == ["Wanglaoji"]


def test_client_ignores_broken_process_proxy(fake_yolo, core, monkeypatch):
    monkeypatch.setenv("HTTP_PROXY", "http://127.0.0.1:1")
    monkeypatch.setenv("NO_PROXY", "")
    assert core.YoloHttpClient(fake_yolo.url, 1.0).health()["ok"] is True
```

- [ ] **Step 3: Run client tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_http.py -k 'predict or proxy'
```

Expected: FAIL because `YoloHttpClient` is undefined.

- [ ] **Step 4: Implement the direct client and strict decoder**

```python
class YoloHttpClient:
    def __init__(self, base_url: str, timeout: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _json(self, path: str, payload: dict | None = None) -> dict:
        body = None if payload is None else json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + path,
            data=body,
            headers={"Content-Type": "application/json"} if body else {},
            method="POST" if body else "GET",
        )
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                value = json.load(response)
        except (OSError, urllib.error.HTTPError, urllib.error.URLError, json.JSONDecodeError) as exc:
            raise YoloProtocolError(str(exc)) from exc
        if not isinstance(value, dict):
            raise YoloProtocolError("YOLO response must be a JSON object")
        return value


SOF_MARKERS = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}


def jpeg_dimensions(jpeg: bytes) -> tuple[int, int]:
    if len(jpeg) < 4 or jpeg[:2] != b"\xff\xd8":
        raise YoloProtocolError("image is not JPEG")
    offset = 2
    while offset + 4 <= len(jpeg):
        while offset < len(jpeg) and jpeg[offset] != 0xFF:
            offset += 1
        while offset < len(jpeg) and jpeg[offset] == 0xFF:
            offset += 1
        if offset >= len(jpeg):
            break
        marker = jpeg[offset]
        offset += 1
        if marker in {0x01, 0xD8, 0xD9} or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(jpeg):
            break
        segment_length = int.from_bytes(jpeg[offset:offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(jpeg):
            raise YoloProtocolError("invalid JPEG segment")
        if marker in SOF_MARKERS and segment_length >= 7:
            height = int.from_bytes(jpeg[offset + 3:offset + 5], "big")
            width = int.from_bytes(jpeg[offset + 5:offset + 7], "big")
            if width > 0 and height > 0:
                return width, height
        offset += segment_length
    raise YoloProtocolError("JPEG dimensions not found")
```

`predict` must set a unique request ID, include `return_overlay=True`, reject
`ok != true`, require `count == len(detections)`, require a non-empty polygon on
each detection, base64-decode with `validate=True`, and require both source and
overlay dimensions from `jpeg_dimensions` to equal the service `image` width and
height. Return numeric class counts and latency. Convert every validation
failure to `YoloProtocolError` with a field-specific message.

- [ ] **Step 5: Write and run malformed-response parameter tests**

```python
@pytest.mark.parametrize("mode, message", [
    ("not_ok", "inference failed"),
    ("bad_json", "JSON"),
    ("missing_overlay", "overlay_jpeg_base64"),
    ("bad_base64", "base64"),
    ("missing_polygon", "mask_polygon_xy"),
    ("count_mismatch", "count"),
    ("dimension_mismatch", "dimensions"),
])
def test_predict_rejects_malformed_service_response(fake_yolo, core, mode, message):
    fake_yolo.mode = mode
    with pytest.raises(core.YoloProtocolError, match=message):
        core.YoloHttpClient(fake_yolo.url, 1.0).predict(
            "left", core.FrameSnapshot(fake_jpeg(640, 480), 1, 1, 1.0), ("Wanglaoji",)
        )
```

Define the deterministic JPEG test helper with a valid SOF marker:

```python
def fake_jpeg(width: int, height: int) -> bytes:
    sof_payload = (
        b"\x08" + height.to_bytes(2, "big") + width.to_bytes(2, "big")
        + b"\x03\x01\x11\x00\x02\x11\x00\x03\x11\x00"
    )
    return b"\xff\xd8\xff\xc0" + (len(sof_payload) + 2).to_bytes(2, "big") + sof_payload + b"\xff\xd9"
```

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_http.py
```

Expected: all Task 2 tests pass.

- [ ] **Step 6: Commit Task 2**

```bash
git add scripts/inference/four_camera_yolo_core.py tests/test_four_camera_yolo_http.py
git commit -m "feat: add direct yolo http client"
```

---

### Task 3: Concurrent Global-Round Scheduling and Health Refresh

**Files:**
- Modify: `scripts/inference/four_camera_yolo_core.py`
- Modify: `tests/test_four_camera_yolo_core.py`
- Modify: `tests/test_four_camera_yolo_http.py`

**Interfaces:**
- Consumes: `StateStore` and `YoloHttpClient`.
- Produces: `InferenceWorker(camera, state, client_factory, clock)`, `InferenceWorker.run_once() -> bool`, `WorkerGroup.run_round()/start()/stop()`, and `HealthMonitor.run_once()`.

- [ ] **Step 1: Write failing latest-only and generation-race worker tests**

```python
def test_worker_uses_newest_frame_and_publishes_matching_pair(core):
    state = core.StateStore(default_interval_sec=3.0)
    state.apply_config("Wanglaoji", 3.0, False, {})
    state.update_frame("left", b"old", 1, 1.0)
    state.update_frame("left", b"new", 2, 2.0)
    client = FakeClient(overlay=b"mask-new")
    worker = core.InferenceWorker("left", state, lambda: client, time.monotonic)
    assert worker.run_once() is True
    assert client.frames == [b"new"]
    assert state.image("left", "source") == b"new"
    assert state.image("left", "overlay") == b"mask-new"


def test_worker_discards_response_when_prompt_changes_in_flight(core):
    client = BlockingFakeClient()
    worker_thread = start_one_run(client, state, worker)
    client.started.wait(1)
    state.apply_config("Sprite", 3.0, False, {})
    client.release.set()
    worker_thread.join(1)
    assert state.image("left", "overlay") is None
```

- [ ] **Step 2: Run worker tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_core.py -k worker
```

Expected: FAIL because worker classes are undefined.

- [ ] **Step 3: Implement one-step execution and globally timed rounds**

```python
class InferenceWorker:
    def run_once(self) -> bool:
        config = self.state.config_snapshot(self.camera)
        if config.paused or not config.enabled or not config.targets:
            return False
        frame = self.state.frame_snapshot(self.camera)
        if frame is None:
            return False
        started = self.clock()
        try:
            result = self._client().predict(self.camera, frame, config.targets)
            self.state.publish_result(self.camera, config.generation, frame, result)
        except Exception as exc:
            self.state.record_error(self.camera, config.generation, str(exc), self.clock())
        finally:
            self.state.record_attempt(self.camera, started, self.clock())
        return True

class WorkerGroup:
    def run_round(self) -> int:
        submitted = 0
        with self._futures_lock:
            for camera, worker in self._workers.items():
                previous = self._futures.get(camera)
                if previous is not None and not previous.done():
                    continue
                self._futures[camera] = self._executor.submit(worker.run_once)
                submitted += 1
        return submitted

    def _schedule(self) -> None:
        while not self.stop_event.is_set():
            self.run_round()
            interval = self.state.global_config_snapshot().interval_sec
            self.stop_event.wait(interval)
```

The group owns a four-thread `ThreadPoolExecutor` plus one scheduler thread.
`run_round()` submits at most one future per camera and skips a camera whose
previous future is incomplete. `start()` performs an immediate round and then
waits the current global interval; `stop()` sets the shared event, joins the
scheduler, and shuts down the executor. The worker must never hold the state
lock while calling the YOLO service.

- [ ] **Step 4: Write failing health isolation test**

```python
def test_health_monitor_retains_page_state_when_service_is_offline(core):
    state = core.StateStore(3.0)
    monitor = core.HealthMonitor(state, lambda: FailingClient("connection refused"), time.monotonic)
    assert monitor.run_once() is False
    status = state.status(time.monotonic())
    assert status["yolo"]["ok"] is False
    assert "connection refused" in status["yolo"]["error"]
    assert set(status["cameras"]) == {"left", "front", "right", "head"}
```

- [ ] **Step 5: Implement health/classes caching and verify GREEN**

`HealthMonitor.run_once()` calls `/health` and `/classes`, keeps only classes
with `supported_by_deployed_model`, records the check time and error, and never
clears camera frames or results on failure.

```python
class HealthMonitor:
    def run_once(self) -> bool:
        checked_at = self.clock()
        try:
            client = self.client_factory()
            health = client.health()
            classes = tuple(
                item for item in client.classes().get("classes", [])
                if item.get("supported_by_deployed_model")
            )
            self.state.update_health(True, health, classes, "", checked_at)
            return True
        except Exception as exc:
            self.state.update_health(False, {}, (), str(exc), checked_at)
            return False

    def run(self, stop_event: threading.Event, interval: float = 5.0) -> None:
        while not stop_event.is_set():
            self.run_once()
            stop_event.wait(interval)
```

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_http.py
```

Expected: all Task 1 through Task 3 tests pass.

- [ ] **Step 6: Commit Task 3**

```bash
git add scripts/inference/four_camera_yolo_core.py \
  tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_http.py
git commit -m "feat: schedule concurrent yolo rounds"
```

---

### Task 4: Web JSON and JPEG API

**Files:**
- Create: `scripts/inference/four_camera_yolo_web.py`
- Create: `tests/test_four_camera_yolo_web.py`

**Interfaces:**
- Consumes: `StateStore`, `CAMERA_KEYS`, and cached classes from the core module.
- Produces: `make_handler(state, html_bytes) -> type[BaseHTTPRequestHandler]`, `create_server(host, port, state, html_path) -> ThreadingHTTPServer`, and the approved HTTP routes.

- [ ] **Step 1: Write failing status/config/image endpoint tests**

```python
def test_status_and_config_round_trip(web_server):
    status = get_json(web_server.url + "/api/status")
    assert list(status["cameras"]) == ["left", "front", "right", "head"]
    classes = get_json(web_server.url + "/api/classes")
    assert classes["classes"][0]["name"] == "Wanglaoji"
    updated = post_json(web_server.url + "/api/config", {
        "targets": "Wanglaoji, Sprite",
        "interval_sec": 3.0,
        "paused": False,
        "cameras": {key: {"enabled": True} for key in status["cameras"]},
    })
    assert updated["ok"] is True
    assert get_json(web_server.url + "/api/status")["targets"] == ["Wanglaoji", "Sprite"]


def test_matching_source_and_overlay_jpeg_endpoints(web_server, published_pair):
    source = get_bytes(web_server.url + "/api/cameras/left/source.jpg?seq=1")
    overlay = get_bytes(web_server.url + "/api/cameras/left/overlay.jpg?seq=1")
    assert source.body == b"source-left"
    assert overlay.body == b"overlay-left"
    assert source.headers["Cache-Control"] == "no-store"
    assert overlay.headers["Content-Type"] == "image/jpeg"
```

- [ ] **Step 2: Run endpoint tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_web.py -k 'status or config or jpeg'
```

Expected: import failure because `four_camera_yolo_web.py` does not exist.

- [ ] **Step 3: Implement handler factory and explicit route parsing**

```python
def make_handler(state: StateStore, html_bytes: bytes):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            parsed = urllib.parse.urlsplit(self.path)
            if parsed.path == "/":
                return self._bytes(200, html_bytes, "text/html; charset=utf-8")
            if parsed.path == "/api/status":
                return self._json(200, state.status(time.monotonic()))
            if parsed.path == "/api/classes":
                return self._json(200, {"ok": True, "classes": state.classes_snapshot()})
            match = CAMERA_IMAGE_RE.fullmatch(parsed.path)
            if match:
                camera, kind = match.groups()
                payload = state.image(camera, kind)
                if payload is None:
                    return self._json(404, {"ok": False, "error": "image not available"})
                return self._bytes(200, payload, "image/jpeg", no_store=True)
            self._json(404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:
            if urllib.parse.urlsplit(self.path).path != "/api/config":
                return self._json(404, {"ok": False, "error": "not found"})
            try:
                payload = read_json_body(self, max_bytes=64 * 1024)
                generation = state.apply_config_payload(payload)
            except ValueError as exc:
                return self._json(400, {"ok": False, "error": str(exc)})
            self._json(200, {"ok": True, "generation": generation})
    return Handler


CAMERA_IMAGE_RE = re.compile(
    r"^/api/cameras/(left|front|right|head)/(latest|source|overlay)\.jpg$"
)


class ConsoleServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True
```

Set `protocol_version = "HTTP/1.1"`, include exact `Content-Length`, set
`Cache-Control: no-store` on JSON and image responses, reject request bodies
over 64 KiB, and suppress default access logs except errors.

- [ ] **Step 4: Add invalid-input tests**

```python
@pytest.mark.parametrize("payload, message", [
    ({"targets": "x", "interval_sec": 0.9, "paused": False, "cameras": {}}, "between 1.0 and 60.0"),
    ({"targets": "x", "interval_sec": 3, "paused": "no", "cameras": {}}, "paused must be boolean"),
    ({"targets": "x", "interval_sec": 3, "paused": False, "cameras": {"side": {"enabled": True}}}, "unknown camera"),
])
def test_config_rejects_invalid_payload_atomically(web_server, payload, message):
    before = get_json(web_server.url + "/api/status")
    error = post_json_error(web_server.url + "/api/config", payload, expected=400)
    assert message in error["error"]
    assert get_json(web_server.url + "/api/status")["generation"] == before["generation"]
```

- [ ] **Step 5: Run Web API tests and verify GREEN**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_web.py -k 'not html and not integration'
```

Expected: all API and image tests pass.

- [ ] **Step 6: Commit Task 4**

```bash
git add scripts/inference/four_camera_yolo_web.py tests/test_four_camera_yolo_web.py
git commit -m "feat: expose four-camera yolo web api"
```

---

### Task 5: Responsive Mask-First Grid and Tag Input UI

**Files:**
- Create: `scripts/inference/four_camera_yolo_web.html`
- Modify: `scripts/inference/four_camera_yolo_web.py`
- Modify: `tests/test_four_camera_yolo_web.py`

**Interfaces:**
- Consumes: `/api/status`, `/api/classes`, `/api/config`, and per-camera JPEG routes from Task 4.
- Produces: DOM IDs `target-input`, `target-tags`, `interval-sec`, `apply-config`, `pause-all`, `yolo-health`, `global-error`, and `<section data-camera="...">` for every camera.

- [ ] **Step 1: Write failing HTML contract test**

```python
def test_home_page_contains_global_controls_and_four_mask_first_cards(web_server):
    html = get_text(web_server.url + "/")
    assert 'id="target-input"' in html
    assert 'id="target-tags"' in html
    assert 'id="interval-sec"' in html
    assert 'min="1"' in html and 'max="60"' in html and 'value="3"' in html
    assert 'id="apply-config"' in html
    assert 'id="pause-all"' in html
    assert 'id="class-suggestions"' in html
    assert 'id="class-warning"' in html
    for camera in ("left", "front", "right", "head"):
        assert f'data-camera="{camera}"' in html
        assert f'id="{camera}-image"' in html
        assert f'id="{camera}-mask-view"' in html
        assert f'id="{camera}-source-view"' in html
    assert "setInterval(refreshStatus, 500)" in html
```

- [ ] **Step 2: Run HTML test and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_web.py -k home_page
```

Expected: FAIL because the HTML asset and required controls do not exist.

- [ ] **Step 3: Build the approved layout A markup and CSS**

Use one semantic template per camera:

```html
<section class="camera-card" data-camera="left" data-view="mask">
  <header><h2>左视角 <small>left</small></h2><span id="left-topic" class="badge">等待相机</span></header>
  <figure class="camera-frame"><img id="left-image" alt="左视角 YOLO mask 叠加图"></figure>
  <div class="camera-controls">
    <label><input id="left-enabled" type="checkbox" checked> 启用</label>
    <div class="view-toggle" role="group" aria-label="左视角显示模式">
      <button id="left-mask-view" type="button" aria-pressed="true">Mask</button>
      <button id="left-source-view" type="button" aria-pressed="false">原图</button>
    </div>
  </div>
  <dl id="left-metrics"></dl><p id="left-error" class="error" role="status"></p>
</section>
```

Add a sticky control bar, a desktop two-column card grid, one equal-aspect
primary image panel per card, clear online/stale/offline colors, loading placeholders, and a
`@media (max-width: 900px)` one-column layout. Do not use external fonts, image
CDNs, or JavaScript libraries.

- [ ] **Step 4: Implement polling, sequence-based images, and controls**

```javascript
const cameras = ["left", "front", "right", "head"];
const seen = Object.fromEntries(cameras.map(key => [key, {result: 0, input: 0, view: "mask"}]));
let lastStatus = {interval_sec: 3, cameras: {}};

async function refreshStatus() {
  const response = await fetch("/api/status", {cache: "no-store"});
  const status = await response.json();
  lastStatus = status;
  renderGlobal(status);
  for (const key of cameras) {
    const camera = status.cameras[key];
    renderCamera(key, camera);
    if (camera.result_sequence > 0 && camera.result_sequence !== seen[key].result) {
      seen[key].result = camera.result_sequence;
      updateCameraImage(key, camera);
    } else if (camera.result_sequence === 0) {
      if (seen[key].result !== 0) {
        seen[key].result = 0;
      }
      if (camera.input_sequence > 0 && camera.input_sequence !== seen[key].input) {
        seen[key].input = camera.input_sequence;
        document.getElementById(`${key}-image`).src = `/api/cameras/${key}/latest.jpg?seq=${camera.input_sequence}`;
      }
    }
  }
}

function updateCameraImage(key, camera) {
  const kind = seen[key].view === "source" ? "source" : "overlay";
  document.getElementById(`${key}-image`).src =
    `/api/cameras/${key}/${kind}.jpg?seq=${camera.result_sequence}`;
}

function setView(key, view) {
  seen[key].view = view;
  document.getElementById(`${key}-mask-view`).setAttribute("aria-pressed", String(view === "mask"));
  document.getElementById(`${key}-source-view`).setAttribute("aria-pressed", String(view === "source"));
  const camera = lastStatus.cameras[key];
  if (camera && camera.result_sequence > 0) updateCameraImage(key, camera);
}

setInterval(refreshStatus, 500);
refreshStatus();
```

Define the referenced rendering and configuration functions without injecting
service-provided text through `innerHTML`:

```javascript
function renderGlobal(status) {
  const health = document.getElementById("yolo-health");
  health.textContent = status.yolo.ok ? "YOLO 在线" : `YOLO 离线：${status.yolo.error || "等待检查"}`;
  health.className = `badge ${status.yolo.ok ? "online" : "offline"}`;
  document.getElementById("global-error").textContent = status.config_error || "";
}

function renderCamera(key, camera) {
  const topic = document.getElementById(`${key}-topic`);
  topic.textContent = camera.topic_online ? "相机在线" : (camera.input_sequence ? "相机超时" : "等待相机");
  topic.className = `badge ${camera.topic_online ? "online" : "offline"}`;
  const metrics = document.getElementById(`${key}-metrics`);
  const rows = [
    ["全局周期", `${lastStatus.interval_sec} s`], ["实际结果率", `${camera.effective_rate_hz} Hz`],
    ["服务延迟", `${camera.service_latency_ms ?? "-"} ms`],
    ["往返延迟", `${camera.round_trip_ms ?? "-"} ms`],
    ["检测数量", camera.count ?? 0], ["结果年龄", `${camera.result_age_sec ?? "-"} s`],
  ];
  metrics.replaceChildren(...rows.flatMap(([label, value]) => {
    const dt = document.createElement("dt"); dt.textContent = label;
    const dd = document.createElement("dd"); dd.textContent = String(value);
    return [dt, dd];
  }));
  document.querySelector(`[data-camera="${key}"]`).classList.toggle("stale", Boolean(camera.result_stale));
  document.getElementById(`${key}-error`).textContent = camera.error || "";
}

let selectedTargets = [];
function appendTarget(name) {
  const value = name.trim();
  if (value && !selectedTargets.includes(value)) selectedTargets.push(value);
  renderTargetTags();
}

function renderTargetTags() {
  const host = document.getElementById("target-tags");
  host.replaceChildren(...selectedTargets.map((name, index) => {
    const tag = document.createElement("span");
    tag.className = "target-tag";
    tag.append(document.createTextNode(name));
    const remove = document.createElement("button");
    remove.type = "button";
    remove.setAttribute("aria-label", `删除目标 ${name}`);
    remove.textContent = "×";
    remove.addEventListener("click", () => {
      selectedTargets.splice(index, 1);
      renderTargetTags();
    });
    tag.append(remove);
    return tag;
  }));
  renderClassWarning();
}

function renderClassWarning() {
  const unknown = selectedTargets.filter(name => !deployedClasses.has(name));
  document.getElementById("class-warning").textContent = unknown.length
    ? `非正式类别名，将交给服务尝试解析：${unknown.join(", ")}` : "";
}

async function postConfig(paused) {
  const camerasPayload = Object.fromEntries(cameras.map(key => [key, {
    enabled: document.getElementById(`${key}-enabled`).checked,
  }]));
  const response = await fetch("/api/config", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({
      targets: selectedTargets,
      interval_sec: Number(document.getElementById("interval-sec").value),
      paused,
      cameras: camerasPayload,
    }),
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || `HTTP ${response.status}`);
  return payload;
}
```

`Apply and start` serializes the selected tags, global interval, and four enabled
controls to `POST /api/config`; `Pause all` posts the same settings with
`paused=true`.
Disable buttons while a POST is in flight and display structured API errors.

Fetch `/api/classes` once at page startup, render clickable official-name chips
inside `class-suggestions`, and show names absent from that deployed-class set in
`class-warning` without blocking submission:

```javascript
let deployedClasses = new Set();
async function loadClasses() {
  const payload = await fetch("/api/classes", {cache: "no-store"}).then(response => response.json());
  deployedClasses = new Set(payload.classes.map(item => item.name));
  const host = document.getElementById("class-suggestions");
  host.replaceChildren(...payload.classes.map(item => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = item.name;
    button.addEventListener("click", () => appendTarget(item.name));
    return button;
  }));
}

async function runAction(paused) {
  const buttons = [document.getElementById("apply-config"), document.getElementById("pause-all")];
  buttons.forEach(button => { button.disabled = true; });
  try {
    await postConfig(paused);
    document.getElementById("global-error").textContent = "";
  } catch (error) {
    document.getElementById("global-error").textContent = error.message;
  } finally {
    buttons.forEach(button => { button.disabled = false; });
  }
}
document.getElementById("apply-config").addEventListener("click", () => runAction(false));
document.getElementById("pause-all").addEventListener("click", () => runAction(true));
document.getElementById("target-input").addEventListener("keydown", event => {
  if (event.key === "Enter" || event.key === ",") {
    event.preventDefault();
    appendTarget(event.currentTarget.value.replace(/,$/, ""));
    event.currentTarget.value = "";
  }
});
cameras.forEach(key => {
  document.getElementById(`${key}-mask-view`).addEventListener("click", () => setView(key, "mask"));
  document.getElementById(`${key}-source-view`).addEventListener("click", () => setView(key, "source"));
});
loadClasses().catch(error => {
  document.getElementById("global-error").textContent = `类别加载失败：${error.message}`;
});
```

- [ ] **Step 5: Add UI behavior contract assertions and verify GREEN**

Assert the HTML includes cache-busted sequence URLs, `role="status"`, range
attributes, the 500 ms polling interval, and no external `http://` or `https://`
asset references.

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_web.py -k html
```

Expected: all HTML contract tests pass.

- [ ] **Step 6: Commit Task 5**

```bash
git add scripts/inference/four_camera_yolo_web.html \
  scripts/inference/four_camera_yolo_web.py tests/test_four_camera_yolo_web.py
git commit -m "feat: add mask-first four-camera yolo dashboard"
```

---

### Task 6: ROS Adapter, CLI Lifecycle, and Port-Safe Launcher

**Files:**
- Modify: `scripts/inference/four_camera_yolo_web.py`
- Create: `scripts/inference/run_four_camera_yolo_web.sh`
- Modify: `tests/test_four_camera_yolo_web.py`

**Interfaces:**
- Consumes: `StateStore`, `WorkerGroup`, `HealthMonitor`, `create_server`, and `CAMERAS`.
- Produces: CLI options `--host`, `--port`, `--ros-domain-id`, `--yolo-url`, `--default-interval-sec`, `--request-timeout`; runtime exit code `0` on clean signal and nonzero on bind/startup failure.

- [ ] **Step 1: Write failing launcher and lazy-ROS-import tests**

```python
def test_web_module_imports_without_ros_installed(web_module):
    assert callable(web_module.parse_args)
    assert "rclpy" not in web_module.__dict__


def test_launcher_defaults_and_port_safety():
    text = LAUNCHER.read_text(encoding="utf-8")
    assert 'WEB_PORT="${WEB_PORT:-7788}"' in text
    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"' in text
    assert 'YOLO_URL="${YOLO_URL:-http://192.168.4.121:7881}"' in text
    assert "端口 ${WEB_PORT} 已被占用" in text
    assert "kill " not in text and "fuser -k" not in text


def test_create_server_rejects_an_occupied_port_without_closing_owner(web_module):
    owner = socket.socket()
    owner.bind(("127.0.0.1", 0))
    owner.listen()
    port = owner.getsockname()[1]
    with pytest.raises(OSError):
        web_module.create_server("127.0.0.1", port, FakeState(), HTML_PATH)
    assert owner.fileno() >= 0
    owner.close()
```

- [ ] **Step 2: Run lifecycle tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_web.py -k 'launcher or ros_installed'
```

Expected: FAIL because the launcher and runtime adapter do not exist.

- [ ] **Step 3: Implement the lazy ROS adapter**

```python
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=7788)
    parser.add_argument("--ros-domain-id", type=int, default=99)
    parser.add_argument("--yolo-url", default="http://192.168.4.121:7881")
    parser.add_argument("--default-interval-sec", type=validate_interval_sec, default=3.0)
    parser.add_argument("--request-timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.request_timeout <= 0:
        parser.error("--request-timeout must be positive")
    return args


def create_ros_node(state: StateStore):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    class FourCameraNode(Node):
        def __init__(self):
            super().__init__("four_camera_yolo_web")
            self._subscriptions = []
            for camera in CAMERAS:
                callback = lambda msg, key=camera.key: state.update_frame(
                    key,
                    bytes(msg.data),
                    msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
                    time.monotonic(),
                )
                self._subscriptions.append(
                    self.create_subscription(CompressedImage, camera.topic, callback, qos_profile_sensor_data)
                )
    return FourCameraNode()
```

Keep all ROS imports inside runtime functions so standard-library tests can load
the Web module under the Python 3.11 test environment.

- [ ] **Step 4: Implement coordinated startup and shutdown**

Parse and validate CLI arguments before `rclpy.init`. Bind the Web server before
starting worker threads so a port conflict cannot leave background workers.
Start the health monitor, four workers, and HTTP `serve_forever` thread; spin
ROS in the main thread. On SIGINT/SIGTERM set the shared stop event, call
`server.shutdown()`, join threads with bounded timeouts, destroy the node, and
call `rclpy.shutdown()`.

```python
def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    state = StateStore(args.default_interval_sec)
    stop_event = threading.Event()
    server = create_server(args.host, args.port, state, HTML_PATH)
    client_factory = lambda: YoloHttpClient(args.yolo_url, args.request_timeout)
    workers = WorkerGroup(state, client_factory, stop_event)
    health = HealthMonitor(state, client_factory, time.monotonic)
    import rclpy
    rclpy.init(args=None)
    node = create_ros_node(state)

    def request_stop(_signum=None, _frame=None):
        stop_event.set()
        try:
            import rclpy
            rclpy.try_shutdown()
        except RuntimeError:
            pass

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    web_thread = threading.Thread(target=server.serve_forever, name="yolo-web", daemon=True)
    health_thread = threading.Thread(target=health.run, args=(stop_event,), name="yolo-health", daemon=True)
    workers.start()
    web_thread.start()
    health_thread.start()
    try:
        rclpy.spin(node)
    finally:
        stop_event.set()
        server.shutdown()
        server.server_close()
        workers.stop(timeout=3.0)
        health_thread.join(3.0)
        web_thread.join(3.0)
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RuntimeError:
            pass
    return 0
```

- [ ] **Step 5: Add the launcher with a non-destructive preflight**

```bash
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-7788}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
YOLO_URL="${YOLO_URL:-http://192.168.4.121:7881}"
DEFAULT_INTERVAL_SEC="${DEFAULT_INTERVAL_SEC:-3.0}"

source /opt/ros/humble/setup.bash
[ -f /home/agilex/camera_ros/install/setup.bash ] && source /home/agilex/camera_ros/install/setup.bash

if ss -ltnH "sport = :${WEB_PORT}" | grep -q .; then
    echo "[错误] 端口 ${WEB_PORT} 已被占用；不会终止现有服务。" >&2
    exit 1
fi

export ROS_DOMAIN_ID
exec python3 -u "${SCRIPT_DIR}/four_camera_yolo_web.py" \
    --host "${WEB_HOST}" --port "${WEB_PORT}" --ros-domain-id "${ROS_DOMAIN_ID}" \
    --yolo-url "${YOLO_URL}" --default-interval-sec "${DEFAULT_INTERVAL_SEC}" "$@"
```

- [ ] **Step 6: Verify shell, Python, help, and unit suite**

Run:

```bash
bash -n scripts/inference/run_four_camera_yolo_web.sh
python3 -m py_compile scripts/inference/four_camera_yolo_core.py scripts/inference/four_camera_yolo_web.py
source /opt/ros/humble/setup.bash
source /home/agilex/camera_ros/install/setup.bash
python3 scripts/inference/four_camera_yolo_web.py --help
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest -q \
  tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_http.py tests/test_four_camera_yolo_web.py
```

Expected: shell and compilation commands exit 0, help lists every required
option, and all targeted tests pass.

- [ ] **Step 7: Commit Task 6**

```bash
git add scripts/inference/four_camera_yolo_web.py \
  scripts/inference/run_four_camera_yolo_web.sh tests/test_four_camera_yolo_web.py
git commit -m "feat: launch four-camera yolo web console"
```

---

### Task 7: Four-Worker Integration, Operator Documentation, and Live Acceptance

**Files:**
- Modify: `tests/test_four_camera_yolo_web.py`
- Modify: `scripts/inference/README.md`

**Interfaces:**
- Consumes: complete runtime from Tasks 1 through 6.
- Produces: documented launch workflow and final automated/live acceptance evidence.

- [ ] **Step 1: Write a failing four-worker concurrency/error-isolation test**

```python
def test_four_workers_run_concurrently_and_isolate_one_failure(core):
    state = core.StateStore(3.0)
    state.apply_config("Wanglaoji", 3.0, False, {})
    for index, key in enumerate(core.CAMERA_KEYS):
        state.update_frame(key, f"source-{key}".encode(), index, time.monotonic())
    client = BarrierFakeClient(parties=4, failing_camera="right")
    stop_event = threading.Event()
    group = core.WorkerGroup(state, lambda: client, stop_event)
    group.start()
    assert client.first_batch_complete.wait(2.0)
    stop_event.set()
    group.stop(timeout=2.0)
    status = state.status(time.monotonic())
    assert status["cameras"]["right"]["error"] == "forced right failure"
    for key in ("left", "front", "head"):
        assert state.image(key, "source") == f"source-{key}".encode()
        assert state.image(key, "overlay") == f"overlay-{key}".encode()
```

- [ ] **Step 2: Run the integration test and verify the production worker group**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest \
  -q tests/test_four_camera_yolo_web.py -k four_workers
```

Expected: PASS using the production `WorkerGroup.start()` and `stop()` methods;
the four-way barrier proves concurrent entry, and the status assertions prove
one camera error does not block the other three.

- [ ] **Step 3: Document operation and troubleshooting**

Add this concrete workflow to `scripts/inference/README.md`:

````markdown
## Four-camera live YOLO Web console

```bash
bash scripts/inference/run_four_camera_yolo_web.sh
```

Open `http://<robot-ip>:7788/`. Search deployed English class names, press Enter
or comma to create multiple tags, keep the default 3-second global interval,
and click **应用并开始**.

Overrides:

```bash
WEB_PORT=7788 ROS_DOMAIN_ID=99 DEFAULT_INTERVAL_SEC=3 \
YOLO_URL=http://192.168.4.121:7881 \
bash scripts/inference/run_four_camera_yolo_web.sh
```

If the YOLO health card reports 502 while `curl --noproxy '*'` works, verify the
launcher is using this console's direct HTTP client. If a camera is offline,
check its `/camera_[lfrh]/color/image_raw/compressed` topic.
````

- [ ] **Step 4: Run the complete automated verification**

Run:

```bash
git diff --check
bash -n scripts/inference/run_four_camera_yolo_web.sh
python3 -m py_compile scripts/inference/four_camera_yolo_core.py scripts/inference/four_camera_yolo_web.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/agilex/openpi/.venv/bin/pytest -q \
  tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_http.py tests/test_four_camera_yolo_web.py
```

Expected: no whitespace errors, shell/Python checks exit 0, and all targeted
tests pass with zero failures.

- [ ] **Step 5: Run live four-camera acceptance on port 7788**

First confirm the remote service and local topics:

```bash
curl --noproxy '*' -fsS http://192.168.4.121:7881/health
source /opt/ros/humble/setup.bash
source /home/agilex/camera_ros/install/setup.bash
export ROS_DOMAIN_ID=99
ros2 topic list | rg '^/camera_[lfrh]/color/image_raw/compressed$'
ss -ltnp | rg ':7788\b' || true
```

Start the console in a managed terminal session, then configure it:

```bash
bash scripts/inference/run_four_camera_yolo_web.sh
```

```bash
curl -fsS -X POST http://127.0.0.1:7788/api/config \
  -H 'Content-Type: application/json' \
  -d '{"targets":["Wanglaoji","Sprite"],"interval_sec":3,"paused":false,"cameras":{"left":{"enabled":true},"front":{"enabled":true},"right":{"enabled":true},"head":{"enabled":true}}}'
sleep 7
curl -fsS http://127.0.0.1:7788/api/status
```

Verify all four topic states are online, every enabled camera with detections has
a positive result sequence, source and overlay endpoints both return JPEG, and
each pair reports the same source sequence/dimensions. Open
`http://127.0.0.1:7788/` for visual confirmation, change the target tags once to
confirm old-generation results disappear, change the global interval, switch
each card between Mask and source, disable one card, and verify the other three
continue.

Stop with SIGINT and verify release:

```bash
ss -ltnp | rg ':7788\b' || true
```

Expected: no listener remains after shutdown.

- [ ] **Step 6: Commit Task 7**

```bash
git add tests/test_four_camera_yolo_web.py scripts/inference/README.md
git commit -m "test: verify four-camera yolo web console"
```

- [ ] **Step 7: Request final code review**

Use the `requesting-code-review` skill to compare the complete implementation
against `docs/superpowers/specs/2026-07-13-four-camera-yolo-web-design.md`, then
run the verification commands again after any accepted review fixes.
