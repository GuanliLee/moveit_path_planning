"""Deterministic configuration and state for four-camera YOLO inference."""

from __future__ import annotations

import base64
import binascii
import json
import math
import re
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor, wait
from dataclasses import dataclass, field, replace
from typing import Callable


@dataclass(frozen=True)
class CameraDefinition:
    key: str
    label: str
    topic: str


CAMERAS: tuple[CameraDefinition, ...] = (
    CameraDefinition("left", "左视角", "/camera_l/color/image_raw/compressed"),
    CameraDefinition("front", "前视角", "/camera_f/color/image_raw/compressed"),
    CameraDefinition("right", "右视角", "/camera_r/color/image_raw/compressed"),
    CameraDefinition("head", "头部视角", "/camera_h/color/image_raw/compressed"),
)
CAMERA_KEYS: tuple[str, ...] = tuple(camera.key for camera in CAMERAS)
MIN_INTERVAL_SEC = 0.2
MAX_INTERVAL_SEC = 60.0
ATTEMPT_WINDOW_SEC = 10.0
TOPIC_TIMEOUT_SEC = 2.0
RESULT_STALE_MIN_SEC = 2.0
SOF_MARKERS = {
    0xC0,
    0xC1,
    0xC2,
    0xC3,
    0xC5,
    0xC6,
    0xC7,
    0xC9,
    0xCA,
    0xCB,
    0xCD,
    0xCE,
    0xCF,
}


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
    if not (
        math.isfinite(interval_sec)
        and MIN_INTERVAL_SEC <= interval_sec <= MAX_INTERVAL_SEC
    ):
        raise ValueError(
            f"interval_sec must be between {MIN_INTERVAL_SEC} and {MAX_INTERVAL_SEC}"
        )
    return interval_sec


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


class YoloProtocolError(RuntimeError):
    """The remote service was unreachable or violated its response contract."""


def jpeg_dimensions(jpeg: bytes) -> tuple[int, int]:
    """Read JPEG dimensions without importing an image-processing dependency."""

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
        segment_length = int.from_bytes(jpeg[offset : offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(jpeg):
            raise YoloProtocolError("invalid JPEG segment")
        if marker in SOF_MARKERS and segment_length >= 7:
            height = int.from_bytes(jpeg[offset + 3 : offset + 5], "big")
            width = int.from_bytes(jpeg[offset + 5 : offset + 7], "big")
            if width > 0 and height > 0:
                return width, height
        offset += segment_length
    raise YoloProtocolError("JPEG dimensions not found")


class YoloHttpClient:
    """Small direct HTTP client for the deployed YOLO v1 API."""

    def __init__(self, base_url: str, timeout: float = 10.0) -> None:
        if not isinstance(base_url, str) or not base_url.startswith(("http://", "https://")):
            raise ValueError("base_url must start with http:// or https://")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.base_url = base_url.rstrip("/")
        self.timeout = float(timeout)
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        self._stream_targets: dict[str, tuple[str, ...]] = {}

    def _json(self, path: str, payload: dict | None = None) -> dict:
        body = (
            None
            if payload is None
            else json.dumps(payload, ensure_ascii=False).encode("utf-8")
        )
        request = urllib.request.Request(
            self.base_url + path,
            data=body,
            headers={"Content-Type": "application/json"} if body is not None else {},
            method="POST" if body is not None else "GET",
        )
        try:
            with self._opener.open(request, timeout=self.timeout) as response:
                raw = response.read()
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace").strip()
            suffix = f": {detail}" if detail else ""
            raise YoloProtocolError(f"HTTP {exc.code}{suffix}") from exc
        except (urllib.error.URLError, OSError) as exc:
            raise YoloProtocolError(f"YOLO connection failed: {exc}") from exc
        try:
            value = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise YoloProtocolError(f"invalid JSON response: {exc}") from exc
        if not isinstance(value, dict):
            raise YoloProtocolError("YOLO response must be a JSON object")
        return value

    @staticmethod
    def _require_ok(payload: dict, endpoint: str) -> dict:
        if payload.get("ok") is not True:
            error = payload.get("error")
            detail = error if isinstance(error, str) and error else f"{endpoint} failed"
            raise YoloProtocolError(detail)
        return payload

    def health(self) -> dict:
        return self._require_ok(self._json("/health"), "health")

    def classes(self) -> dict:
        return self._require_ok(self._json("/classes"), "classes")

    def predict(
        self,
        camera: str,
        frame: FrameSnapshot,
        targets: tuple[str, ...],
    ) -> InferenceResult:
        reset_temporal = self._stream_targets.get(camera) != targets
        request_payload = {
            "request_id": f"four-camera-{camera}-{uuid.uuid4()}",
            "stream_id": f"four-camera-{camera}",
            "temporal": True,
            "reset_temporal": reset_temporal,
            "image_base64": base64.b64encode(frame.jpeg).decode("ascii"),
            "targets": list(targets),
            "return_overlay": True,
        }
        started = time.monotonic()
        payload = self._require_ok(self._json("/predict", request_payload), "predict")
        self._stream_targets[camera] = targets
        completed = time.monotonic()

        image = payload.get("image")
        if not isinstance(image, dict):
            raise YoloProtocolError("image dimensions are missing")
        width = image.get("width")
        height = image.get("height")
        if (
            isinstance(width, bool)
            or isinstance(height, bool)
            or not isinstance(width, int)
            or not isinstance(height, int)
            or width <= 0
            or height <= 0
        ):
            raise YoloProtocolError("image dimensions are invalid")

        detections = payload.get("detections")
        count = payload.get("count")
        if not isinstance(detections, list):
            raise YoloProtocolError("detections must be a list")
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise YoloProtocolError("count must be a non-negative integer")
        if count != len(detections):
            raise YoloProtocolError("count does not match detections")
        for index, detection in enumerate(detections):
            if not isinstance(detection, dict):
                raise YoloProtocolError(f"detections[{index}] must be an object")
            bbox = detection.get("bbox_xyxy")
            bbox_is_valid = (
                isinstance(bbox, (list, tuple))
                and len(bbox) == 4
                and all(
                    isinstance(value, (int, float))
                    and not isinstance(value, bool)
                    and math.isfinite(value)
                    for value in bbox
                )
                and bbox[2] > bbox[0]
                and bbox[3] > bbox[1]
            )
            if not bbox_is_valid:
                raise YoloProtocolError(
                    f"detections[{index}].bbox_xyxy is missing or invalid"
                )

        overlay_value = payload.get("overlay_jpeg_base64")
        if not isinstance(overlay_value, str) or not overlay_value:
            raise YoloProtocolError("overlay_jpeg_base64 is missing")
        try:
            overlay_jpeg = base64.b64decode(overlay_value, validate=True)
        except (binascii.Error, ValueError) as exc:
            raise YoloProtocolError(
                "overlay_jpeg_base64 is invalid base64"
            ) from exc

        expected_dimensions = (width, height)
        if jpeg_dimensions(frame.jpeg) != expected_dimensions:
            raise YoloProtocolError("source dimensions do not match response image")
        if jpeg_dimensions(overlay_jpeg) != expected_dimensions:
            raise YoloProtocolError("overlay dimensions do not match response image")

        class_counts: dict[str, int] = {}
        target_stats = payload.get("targets", [])
        if not isinstance(target_stats, list):
            raise YoloProtocolError("targets must be a list")
        for index, item in enumerate(target_stats):
            if not isinstance(item, dict):
                raise YoloProtocolError(f"targets[{index}] must be an object")
            name = item.get("class_name")
            target_count = item.get("count")
            if (
                not isinstance(name, str)
                or not name
                or isinstance(target_count, bool)
                or not isinstance(target_count, int)
                or target_count < 0
            ):
                raise YoloProtocolError(f"targets[{index}] is invalid")
            class_counts[name] = target_count

        latency = payload.get("latency_ms", 0.0)
        if isinstance(latency, bool) or not isinstance(latency, (int, float)):
            raise YoloProtocolError("latency_ms must be numeric")
        return InferenceResult(
            overlay_jpeg=overlay_jpeg,
            count=count,
            class_counts=class_counts,
            service_latency_ms=float(latency),
            round_trip_ms=(completed - started) * 1000.0,
            completed_at=completed,
        )


@dataclass
class _CameraRuntime:
    input_sequence: int = 0
    latest_frame: FrameSnapshot | None = None
    result_sequence: int = 0
    source_jpeg: bytes | None = None
    source_frame_sequence: int = 0
    source_stamp_ns: int = 0
    overlay_jpeg: bytes | None = None
    last_result: InferenceResult | None = None
    error: str = ""
    error_at: float | None = None
    attempts: deque[tuple[float, float]] = field(default_factory=deque)


class StateStore:
    """Thread-safe latest-frame and configuration state shared by workers."""

    def __init__(self, default_interval_sec: object = 3.0) -> None:
        interval_sec = validate_interval_sec(default_interval_sec)
        self._lock = threading.Lock()
        self._config_condition = threading.Condition(self._lock)
        self._generation = 0
        self._targets: tuple[str, ...] = ()
        self._interval_sec = interval_sec
        self._paused = False
        self._config_error = ""
        self._configs = {key: CameraConfig() for key in CAMERA_KEYS}
        self._runtime = {key: _CameraRuntime() for key in CAMERA_KEYS}
        self._health = {
            "ok": False,
            "details": {},
            "error": "",
            "checked_at": 0.0,
        }
        self._classes: tuple[dict, ...] = ()

    def _require_camera(self, camera: str) -> None:
        if camera not in CAMERA_KEYS:
            raise ValueError(f"unknown camera: {camera}")

    def _validated_camera_updates(
        self, cameras: object
    ) -> dict[str, dict[str, object]]:
        if not isinstance(cameras, dict):
            raise ValueError("cameras must be an object")
        updates: dict[str, dict[str, object]] = {}
        for key, supplied in cameras.items():
            self._require_camera(key)
            if not isinstance(supplied, dict):
                raise ValueError(f"camera configuration must be an object: {key}")
            unknown = set(supplied) - {"enabled"}
            if unknown:
                name = sorted(unknown)[0]
                raise ValueError(f"unknown camera configuration field: {name}")
            values: dict[str, object] = {}
            if "enabled" in supplied:
                if not isinstance(supplied["enabled"], bool):
                    raise ValueError(f"enabled must be boolean: {key}")
                values["enabled"] = supplied["enabled"]
            updates[key] = values
        return updates

    def update_frame(
        self,
        camera: str,
        jpeg: bytes,
        stamp_ns: int,
        received_at: float,
    ) -> int:
        self._require_camera(camera)
        with self._lock:
            runtime = self._runtime[camera]
            runtime.input_sequence += 1
            runtime.latest_frame = FrameSnapshot(
                bytes(jpeg), runtime.input_sequence, stamp_ns, received_at
            )
            return runtime.input_sequence

    def frame_snapshot(self, camera: str) -> FrameSnapshot | None:
        self._require_camera(camera)
        with self._lock:
            return self._runtime[camera].latest_frame

    def apply_config(
        self,
        targets: str | list[str] | tuple[str, ...],
        interval_sec: object,
        paused: bool,
        cameras: dict[str, dict],
    ) -> int:
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
                    runtime.source_frame_sequence = 0
                    runtime.source_stamp_ns = 0
                    runtime.overlay_jpeg = None
                    runtime.last_result = None
                    runtime.result_sequence = 0
                    runtime.error = ""
                    runtime.error_at = None
            self._config_condition.notify_all()
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
        with self._lock:
            current_interval = self._interval_sec
        return self.apply_config(
            payload.get("targets", ""),
            payload.get("interval_sec", current_interval),
            payload.get("paused", False),
            cameras,
        )

    def config_snapshot(self, camera: str) -> WorkerConfigSnapshot:
        self._require_camera(camera)
        with self._lock:
            camera_config = self._configs[camera]
            return WorkerConfigSnapshot(
                self._generation,
                self._targets,
                self._paused,
                camera_config.enabled,
                self._interval_sec,
            )

    def global_config_snapshot(self) -> GlobalConfigSnapshot:
        with self._lock:
            return GlobalConfigSnapshot(
                self._generation,
                self._targets,
                self._paused,
                self._interval_sec,
            )

    def wait_for_config_change(
        self,
        generation: int,
        timeout: float,
        stop_event: threading.Event,
    ) -> bool:
        with self._config_condition:
            return self._config_condition.wait_for(
                lambda: self._generation != generation or stop_event.is_set(),
                timeout=max(0.0, timeout),
            )

    def wake_config_waiters(self) -> None:
        with self._config_condition:
            self._config_condition.notify_all()

    def publish_result(
        self,
        camera: str,
        generation: int,
        frame: FrameSnapshot,
        result: InferenceResult,
    ) -> bool:
        self._require_camera(camera)
        with self._lock:
            if generation != self._generation:
                return False
            runtime = self._runtime[camera]
            runtime.result_sequence += 1
            runtime.source_jpeg = frame.jpeg
            runtime.source_frame_sequence = frame.sequence
            runtime.source_stamp_ns = frame.stamp_ns
            runtime.overlay_jpeg = result.overlay_jpeg
            runtime.last_result = result
            runtime.error = ""
            runtime.error_at = None
            return True

    def record_error(
        self, camera: str, generation: int, message: str, when: float
    ) -> bool:
        self._require_camera(camera)
        with self._lock:
            if generation != self._generation:
                return False
            runtime = self._runtime[camera]
            runtime.error = message
            runtime.error_at = when
            return True

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

    def update_health(
        self,
        ok: bool,
        health: dict,
        classes: tuple[dict, ...],
        error: str,
        checked_at: float,
    ) -> None:
        with self._lock:
            self._health = {
                "ok": ok,
                "details": dict(health),
                "error": error,
                "checked_at": checked_at,
            }
            if ok:
                self._classes = tuple(dict(item) for item in classes)

    def classes_snapshot(self) -> list[dict]:
        with self._lock:
            return [dict(item) for item in self._classes]

    def record_attempt(self, camera: str, started: float, completed: float) -> None:
        self._require_camera(camera)
        with self._lock:
            runtime = self._runtime[camera]
            runtime.attempts.append((started, completed))
            cutoff = completed - ATTEMPT_WINDOW_SEC
            while runtime.attempts and runtime.attempts[0][1] < cutoff:
                runtime.attempts.popleft()

    def status(self, now: float) -> dict:
        with self._lock:
            cameras = {}
            for key in CAMERA_KEYS:
                config = self._configs[key]
                runtime = self._runtime[key]
                latest_frame = runtime.latest_frame
                topic_online = (
                    latest_frame is not None
                    and max(0.0, now - latest_frame.received_at) <= TOPIC_TIMEOUT_SEC
                )
                cutoff = now - ATTEMPT_WINDOW_SEC
                recent_attempts = sum(
                    completed >= cutoff for _, completed in runtime.attempts
                )
                effective_rate_hz = round(recent_attempts / ATTEMPT_WINDOW_SEC, 2)

                result = runtime.last_result
                result_age = (
                    None
                    if result is None
                    else max(0.0, now - result.completed_at)
                )
                stale_after = max(RESULT_STALE_MIN_SEC, 2.0 * self._interval_sec)
                result_stale = result_age is None or result_age > stale_after
                cameras[key] = {
                    "enabled": config.enabled,
                    "input_sequence": runtime.input_sequence,
                    "result_sequence": runtime.result_sequence,
                    "source_frame_sequence": runtime.source_frame_sequence,
                    "source_stamp_sec": (
                        None
                        if runtime.source_stamp_ns <= 0
                        else runtime.source_stamp_ns / 1_000_000_000.0
                    ),
                    "topic_online": topic_online,
                    "effective_rate_hz": effective_rate_hz,
                    "service_latency_ms": (
                        None if result is None else result.service_latency_ms
                    ),
                    "round_trip_ms": None if result is None else result.round_trip_ms,
                    "result_age_sec": result_age,
                    "result_stale": result_stale,
                    "count": 0 if result is None else result.count,
                    "class_counts": (
                        {} if result is None else dict(result.class_counts)
                    ),
                    "error": runtime.error,
                    "error_age_sec": (
                        None
                        if runtime.error_at is None
                        else max(0.0, now - runtime.error_at)
                    ),
                }

            return {
                "generation": self._generation,
                "targets": list(self._targets),
                "interval_sec": self._interval_sec,
                "paused": self._paused,
                "config_error": self._config_error,
                "yolo": {
                    "ok": self._health["ok"],
                    "error": self._health["error"],
                    "checked_at": self._health["checked_at"],
                    "check_age_sec": (
                        None
                        if self._health["checked_at"] <= 0
                        else max(0.0, now - self._health["checked_at"])
                    ),
                },
                "cameras": cameras,
            }


class InferenceWorker:
    """Run one camera request without owning scheduling or frame queues."""

    def __init__(
        self,
        camera: str,
        state: StateStore,
        client_factory: Callable[[], object],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        state._require_camera(camera)
        self.camera = camera
        self.state = state
        self.client_factory = client_factory
        self.clock = clock
        self._client_instance = None

    def _client(self):
        if self._client_instance is None:
            self._client_instance = self.client_factory()
        return self._client_instance

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
            self._client_instance = None
            message = str(exc).strip() or type(exc).__name__
            self.state.record_error(
                self.camera, config.generation, message, self.clock()
            )
        finally:
            self.state.record_attempt(self.camera, started, self.clock())
        return True


class WorkerGroup:
    """Offer synchronized rounds to four one-in-flight camera workers."""

    def __init__(
        self,
        state: StateStore,
        client_factory: Callable[[], object],
        stop_event: threading.Event,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.state = state
        self.stop_event = stop_event
        self._workers = {
            camera: InferenceWorker(camera, state, client_factory, clock)
            for camera in CAMERA_KEYS
        }
        self._executor = ThreadPoolExecutor(
            max_workers=len(CAMERA_KEYS), thread_name_prefix="yolo-camera"
        )
        self._futures: dict[str, Future] = {}
        self._futures_lock = threading.Lock()
        self._scheduler: threading.Thread | None = None
        self._shutdown = False

    def run_round(self) -> int:
        if self.stop_event.is_set() or self._shutdown:
            return 0
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
            config = self.state.global_config_snapshot()
            self.run_round()
            self.state.wait_for_config_change(
                config.generation, config.interval_sec, self.stop_event
            )

    def start(self) -> None:
        if self._scheduler is not None:
            raise RuntimeError("worker group already started")
        if self._shutdown:
            raise RuntimeError("worker group is shut down")
        self._scheduler = threading.Thread(
            target=self._schedule,
            name="yolo-round-scheduler",
            daemon=True,
        )
        self._scheduler.start()

    def wait_for_idle(self, timeout: float) -> bool:
        with self._futures_lock:
            pending = [future for future in self._futures.values() if not future.done()]
        if not pending:
            return True
        _, unfinished = wait(pending, timeout=max(0.0, timeout))
        return not unfinished

    def stop(self, timeout: float = 3.0) -> None:
        if self._shutdown:
            return
        self.stop_event.set()
        self.state.wake_config_waiters()
        if self._scheduler is not None:
            self._scheduler.join(max(0.0, timeout))
        idle = self.wait_for_idle(timeout)
        self._executor.shutdown(wait=idle, cancel_futures=True)
        self._shutdown = True


class HealthMonitor:
    """Refresh remote health and deployed-class suggestions independently."""

    def __init__(
        self,
        state: StateStore,
        client_factory: Callable[[], object],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.state = state
        self.client_factory = client_factory
        self.clock = clock

    def run_once(self) -> bool:
        checked_at = self.clock()
        try:
            client = self.client_factory()
            health = client.health()
            class_payload = client.classes()
            raw_classes = class_payload.get("classes")
            if not isinstance(raw_classes, list):
                raise YoloProtocolError("classes must be a list")
            classes = tuple(
                item
                for item in raw_classes
                if isinstance(item, dict)
                and item.get("supported_by_deployed_model") is True
            )
            self.state.update_health(True, health, classes, "", checked_at)
            return True
        except Exception as exc:
            message = str(exc).strip() or type(exc).__name__
            self.state.update_health(False, {}, (), message, checked_at)
            return False

    def run(
        self, stop_event: threading.Event, interval_sec: float = 5.0
    ) -> None:
        while not stop_event.is_set():
            self.run_once()
            if stop_event.wait(interval_sec):
                break
