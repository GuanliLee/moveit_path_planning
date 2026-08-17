#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import re
import signal
import shutil
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import PoseStamped
from playback import EpisodePlaybackLibrary

try:
    from graspnet_bridge_interfaces.srv import ExecuteNamedGraspTask
    from std_srvs.srv import Trigger
except ImportError:  # pragma: no cover - depends on sourced ROS overlay
    ExecuteNamedGraspTask = None  # type: ignore[assignment]
    Trigger = None  # type: ignore[assignment]


WEB_DIR = Path(__file__).resolve().parent
STATIC_DIR = WEB_DIR / "static"
PLACE_POSES_FILE = Path(os.environ.get("DRINK_GRASP_PLACE_POSES", WEB_DIR / "config" / "place_poses.json"))
PICK_PLACE_POSES_FILE = Path(
    os.environ.get("DRINK_GRASP_PICK_PLACE_POSES", WEB_DIR / "config" / "pick_place_poses.json")
)
RECORD_DIR = Path(os.environ.get("DRINK_GRASP_RECORD_DIR", WEB_DIR / "recordings"))

DEFAULT_RECORD_TOPICS = [
    "/cam_high/color/image_raw",
    "/cam_high/aligned_depth_to_color/image_raw",
    "/cam_high/color/camera_info",
    "/cam_left/color/image_raw",
    "/cam_left/aligned_depth_to_color/image_raw",
    "/cam_left/color/camera_info",
    "/cam_right/color/image_raw",
    "/cam_right/aligned_depth_to_color/image_raw",
    "/cam_right/color/camera_info",
    "/joint_states",
    "/joint_states_left",
    "/joint_states_right",
    "/joint_left",
    "/joint_right",
    "/arm_status_left",
    "/arm_status_right",
    "/end_pose_left",
    "/end_pose_right",
    "/end_pose_stamped_left",
    "/end_pose_stamped_right",
    "/pos_cmd_left",
    "/pos_cmd_right",
    "/joint_ctrl_cmd_left",
    "/joint_ctrl_cmd_right",
    "/joint_states_ctrl_left",
    "/joint_states_ctrl_right",
    "/left/enable_flag",
    "/right/enable_flag",
    "/tf",
    "/tf_static",
]

DRINKS = [
    {
        "id": "coca_cola",
        "name_zh": "可口可乐",
        "brand_zh": "Coca-Cola",
        "prompt": "Coca-Cola",
        "tone": "#d92d20",
    },
    {
        "id": "ad_calcium_milk",
        "name_zh": "AD钙奶",
        "brand_zh": "AD Calcium Milk",
        "prompt": "AD Calcium Milk",
        "tone": "#f2b705",
    },
    {
        "id": "daily_c_orange",
        "name_zh": "味全每日C橙汁",
        "brand_zh": "Daily C Orange Juice",
        "prompt": "Daily C Orange Juice",
        "tone": "#f97316",
    },
    {
        "id": "daily_c_grape",
        "name_zh": "味全每日C葡萄汁",
        "brand_zh": "Daily C Grape Juice",
        "prompt": "Daily C Grape Juice",
        "tone": "#7c3aed",
    },
    {
        "id": "wanglaoji",
        "name_zh": "王老吉",
        "brand_zh": "wanglaoji",
        "prompt": "wanglaoji",
        "tone": "#b42318",
    },
    {
        "id": "dahongpao_milk_tea",
        "name_zh": "果子熟了大红袍乌龙轻乳茶",
        "brand_zh": "Dahongpao Milk Tea",
        "prompt": "Dahongpao Milk Tea",
        "tone": "#9a3412",
    },
    {
        "id": "sprite",
        "name_zh": "雪碧",
        "brand_zh": "Sprite",
        "prompt": "Sprite",
        "tone": "#16a34a",
    },
    {
        "id": "hk_orange_fanta",
        "name_zh": "港版芬达橙子味",
        "brand_zh": "HK Orange Fanta",
        "prompt": "HK Orange Fanta",
        "tone": "#fb923c",
    },
    {
        "id": "aojiru",
        "name_zh": "伊藤园青汁",
        "brand_zh": "Aojiru",
        "prompt": "Aojiru",
        "tone": "#15803d",
    },
]

ARMS = [
    {"id": "right", "name_zh": "右臂", "camera_frame": "cam_right_color_optical_frame"},
    {"id": "left", "name_zh": "左臂", "camera_frame": "cam_left_color_optical_frame"},
]

SHELF = {
    "layers": [
        {"id": "1", "name_zh": "第一层"},
        {"id": "2", "name_zh": "第二层"},
        {"id": "3", "name_zh": "第三层"},
    ],
    "positions": [{"id": str(index), "name_zh": f"位置 {index}"} for index in range(1, 7)],
}


@dataclass
class Runtime:
    host: str
    port: int
    service_name: str
    pick_service_name: str
    cancel_service_name: str
    scene_id: int
    gripper_width_m: float
    service_wait_sec: float
    call_timeout_sec: float
    ros_node: Any | None
    client: Any | None
    pick_client: Any | None
    cancel_client: Any | None
    ros_error: str | None
    place_poses: dict[str, Any]
    pick_place_poses: dict[str, Any]
    recorder: Any
    playback: Any
    lock: threading.Lock
    busy: bool
    current_job: dict[str, Any] | None
    last_result: dict[str, Any] | None


@dataclass
class RecordingSession:
    episode_id: str
    task_name: str
    note: str
    group_name: str
    sequence: int
    output_name: str
    pending_dir: Path
    final_dir: Path
    log_file: Path
    metadata_file: Path
    started_at: float
    stopped_at: float | None
    status: str
    process: Any | None


class LocalRosbagRecorder:
    def __init__(self) -> None:
        self.enabled = env_bool("DRINK_GRASP_RECORDING_ENABLED", True)
        self.target_dir = RECORD_DIR
        self.storage = os.environ.get("DRINK_GRASP_RECORD_STORAGE", "mcap").strip() or "mcap"
        self.storage_preset = os.environ.get("DRINK_GRASP_RECORD_STORAGE_PRESET", "zstd_fast").strip()
        self.startup_wait_sec = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_STARTUP_WAIT_SEC"),
            0.8,
            0.0,
            5.0,
        )
        self.stop_timeout_sec = bounded_float(
            os.environ.get("DRINK_GRASP_RECORD_STOP_TIMEOUT_SEC"),
            30.0,
            1.0,
            180.0,
        )
        self.topics = configured_record_topics()
        self._lock = threading.RLock()
        self._session: RecordingSession | None = None
        self._last_saved: dict[str, Any] | None = None
        self._last_discarded: dict[str, Any] | None = None
        self._last_error: str | None = None

    def status(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_session_locked()
            return {
                "enabled": self.enabled,
                "status": self._session.status if self._session else "idle",
                "session": self._session_payload_locked(self._session),
                "target_dir": str(self.target_dir),
                "storage": self.storage,
                "storage_preset": self.storage_preset,
                "topics": list(self.topics),
                "last_saved": self._last_saved,
                "last_discarded": self._last_discarded,
                "last_error": self._last_error,
            }

    def start(self, task_name: str, note: str) -> dict[str, Any]:
        if not self.enabled:
            return {"ok": False, "error": "recording_disabled", "status": self.status()}
        if not self.topics:
            return {"ok": False, "error": "recording_topics_empty", "status": self.status()}
        with self._lock:
            self._refresh_session_locked()
            if self._session and self._session.status in {"recording", "stopped", "error"}:
                return {
                    "ok": False,
                    "error": "recording_busy",
                    "status": self._session.status,
                    "session": self._session_payload_locked(self._session),
                }
            session = self._allocate_session_locked(task_name, note)
            command = self._record_command(session)
            try:
                log_handle = session.log_file.open("ab")
                process = subprocess.Popen(
                    command,
                    cwd=str(WEB_DIR),
                    env=os.environ.copy(),
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                )
                log_handle.close()
            except Exception as exc:  # noqa: BLE001
                self._last_error = f"{exc.__class__.__name__}: {exc}"
                return {"ok": False, "error": "recording_start_failed", "message": self._last_error}
            session.process = process
            self._session = session

        if self.startup_wait_sec > 0.0:
            time.sleep(self.startup_wait_sec)

        with self._lock:
            self._refresh_session_locked()
            if self._session is session and session.status == "recording":
                self._wait_for_bag_dir_locked(session)
                if session.pending_dir.exists():
                    self._write_metadata_locked(session)
                return {"ok": True, "recording": self.status()}
            self._last_error = tail_text(session.log_file)
            return {
                "ok": False,
                "error": "recording_process_exited",
                "message": self._last_error,
                "recording": self.status(),
            }

    def stop(self) -> dict[str, Any]:
        with self._lock:
            result = self._stop_locked()
            return {**result, "recording": self.status()}

    def save(self, mark: str = "success") -> dict[str, Any]:
        with self._lock:
            self._refresh_session_locked()
            if not self._session:
                return {"ok": False, "error": "no_recording_session", "recording": self.status()}
            if self._session.status == "recording":
                stop_result = self._stop_locked()
                if not stop_result.get("ok"):
                    return {**stop_result, "recording": self.status()}
            session = self._session
            if session.status not in {"stopped", "error"}:
                return {
                    "ok": False,
                    "error": "recording_not_stopped",
                    "status": session.status,
                    "recording": self.status(),
                }
            if session.final_dir.exists():
                self._last_error = f"final recording already exists: {session.final_dir}"
                return {"ok": False, "error": "recording_exists", "message": self._last_error}
            session.final_dir.parent.mkdir(parents=True, exist_ok=True)
            session.pending_dir.rename(session.final_dir)
            moved_log = session.final_dir / "rosbag_record.log"
            with contextlib_suppress():
                shutil.move(str(session.log_file), str(moved_log))
            saved = {
                "episode_id": session.episode_id,
                "path": str(session.final_dir),
                "log_file": str(moved_log),
                "mark": clean_record_text(mark, "success", 40),
                "duration_sec": recording_duration(session),
                "saved_at": time.time(),
            }
            self._last_saved = saved
            self._session = None
            return {"ok": True, "saved": saved, "recording": self.status()}

    def discard(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_session_locked()
            if not self._session:
                return {"ok": False, "error": "no_recording_session", "recording": self.status()}
            if self._session.status == "recording":
                stop_result = self._stop_locked()
                if not stop_result.get("ok"):
                    return {**stop_result, "recording": self.status()}
            session = self._session
            shutil.rmtree(session.pending_dir, ignore_errors=True)
            with contextlib_suppress():
                session.log_file.unlink()
            discarded = {
                "episode_id": session.episode_id,
                "path": str(session.pending_dir),
                "duration_sec": recording_duration(session),
                "discarded_at": time.time(),
            }
            self._last_discarded = discarded
            self._session = None
            return {"ok": True, "discarded": discarded, "recording": self.status()}

    def _stop_locked(self) -> dict[str, Any]:
        self._refresh_session_locked()
        if not self._session:
            return {"ok": False, "error": "no_recording_session"}
        session = self._session
        if session.status in {"stopped", "error"}:
            return {"ok": True, "status": session.status, "session": self._session_payload_locked(session)}
        process = session.process
        if process is None:
            session.status = "error"
            session.stopped_at = time.time()
            self._last_error = "recording process is missing"
            return {"ok": False, "error": "recording_process_missing"}
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"{exc.__class__.__name__}: {exc}"
            return {"ok": False, "error": "recording_stop_signal_failed", "message": self._last_error}
        try:
            process.wait(timeout=self.stop_timeout_sec)
        except subprocess.TimeoutExpired:
            with contextlib_suppress():
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                with contextlib_suppress():
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
            session.status = "error"
            session.stopped_at = time.time()
            self._last_error = "ros2 bag record did not stop gracefully"
            return {"ok": False, "error": "recording_stop_timeout", "message": self._last_error}
        session.stopped_at = time.time()
        session.status = "stopped" if process.returncode in (0, -signal.SIGINT) else "error"
        if session.status == "error":
            self._last_error = tail_text(session.log_file)
        self._write_metadata_locked(session)
        return {"ok": True, "status": session.status, "session": self._session_payload_locked(session)}

    def _refresh_session_locked(self) -> None:
        session = self._session
        if not session or session.status != "recording" or session.process is None:
            return
        returncode = session.process.poll()
        if returncode is None:
            return
        session.stopped_at = time.time()
        session.status = "stopped" if returncode in (0, -signal.SIGINT) else "error"
        if session.status == "error":
            self._last_error = tail_text(session.log_file)
        self._write_metadata_locked(session)

    def _allocate_session_locked(self, task_name: str, note: str) -> RecordingSession:
        task = sanitize_component(task_name or "drink_grasp")
        now = datetime.now()
        group_name = f"{now:%Y%m%d}_{task}"
        group_dir = self.target_dir / group_name
        group_dir.mkdir(parents=True, exist_ok=True)
        sequence = 1
        while True:
            output_name = f"episode_{sequence:03d}"
            final_dir = group_dir / output_name
            pending_dir = group_dir / f".{output_name}.pending"
            if not final_dir.exists() and not pending_dir.exists():
                break
            sequence += 1
        return RecordingSession(
            episode_id=f"{group_name}/{output_name}",
            task_name=task,
            note=clean_record_text(note, "", 500),
            group_name=group_name,
            sequence=sequence,
            output_name=output_name,
            pending_dir=pending_dir,
            final_dir=final_dir,
            log_file=group_dir / f".{output_name}.rosbag_record.log",
            metadata_file=pending_dir / "drink_recording.json",
            started_at=time.time(),
            stopped_at=None,
            status="recording",
            process=None,
        )

    def _record_command(self, session: RecordingSession) -> list[str]:
        command = ["ros2", "bag", "record", "-s", self.storage]
        if self.storage_preset:
            command.extend(["--storage-preset-profile", self.storage_preset])
        command.extend(["-o", str(session.pending_dir)])
        command.append("--topics")
        command.extend(self.topics)
        return command

    def _wait_for_bag_dir_locked(self, session: RecordingSession) -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if session.pending_dir.exists():
                return
            if session.process is not None and session.process.poll() is not None:
                return
            time.sleep(0.1)

    def _write_metadata_locked(self, session: RecordingSession) -> None:
        if not session.pending_dir.exists():
            return
        payload = self._session_payload_locked(session)
        payload["topics"] = list(self.topics)
        session.metadata_file.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def _session_payload_locked(self, session: RecordingSession | None) -> dict[str, Any] | None:
        if session is None:
            return None
        return {
            "episode_id": session.episode_id,
            "task_name": session.task_name,
            "note": session.note,
            "group_name": session.group_name,
            "sequence": session.sequence,
            "output_name": session.output_name,
            "pending_path": str(session.pending_dir),
            "final_path": str(session.final_dir),
            "log_file": str(session.log_file),
            "started_at": session.started_at,
            "stopped_at": session.stopped_at,
            "elapsed_sec": recording_duration(session),
            "status": session.status,
            "pid": session.process.pid if session.process is not None else None,
        }


class DrinkGraspServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], runtime: Runtime) -> None:
        super().__init__(address, Handler)
        self.runtime = runtime


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "DrinkGraspWeb/0.1"

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)
        if path == "/":
            self._send_file(WEB_DIR / "index.html")
            return
        if path.startswith("/static/"):
            self._send_file(STATIC_DIR / path.removeprefix("/static/"))
            return
        if path == "/api/options":
            self._send_json(
                {
                    "ok": True,
                    "task_modes": [
                        {"id": "restock", "name_zh": "上架"},
                        {"id": "pick", "name_zh": "拣货"},
                    ],
                    "drinks": DRINKS,
                    "arms": ARMS,
                    "shelf": SHELF,
                    "defaults": {
                        "scene_id": self.server.runtime.scene_id,  # type: ignore[attr-defined]
                        "gripper_width_m": self.server.runtime.gripper_width_m,  # type: ignore[attr-defined]
                    },
                    "place_pose_config": {
                        "path": str(PLACE_POSES_FILE),
                        "configured_slots": configured_slot_keys(self.server.runtime.place_poses),  # type: ignore[attr-defined]
                    },
                    "pick_place_pose_config": {
                        "path": str(PICK_PLACE_POSES_FILE),
                        "configured_arms": configured_pick_place_arms(self.server.runtime.pick_place_poses),  # type: ignore[attr-defined]
                    },
                    "recording": {
                        "enabled": self.server.runtime.recorder.enabled,  # type: ignore[attr-defined]
                        "target_dir": str(self.server.runtime.recorder.target_dir),  # type: ignore[attr-defined]
                        "storage": self.server.runtime.recorder.storage,  # type: ignore[attr-defined]
                        "topics": list(self.server.runtime.recorder.topics),  # type: ignore[attr-defined]
                    },
                }
            )
            return
        if path == "/api/status":
            self._send_json(status_payload(self.server.runtime))  # type: ignore[attr-defined]
            return
        if path == "/api/recording/status":
            self._send_json({"ok": True, "recording": self.server.runtime.recorder.status()})  # type: ignore[attr-defined]
            return
        if path == "/api/playback/episodes":
            episodes = self.server.runtime.playback.catalog()  # type: ignore[attr-defined]
            self._send_json({"ok": True, "episodes": episodes, "count": len(episodes)})
            return
        if path == "/api/playback/manifest":
            episode_id = first_query_value(query, "episode")
            try:
                manifest = self.server.runtime.playback.manifest(episode_id)  # type: ignore[attr-defined]
            except (FileNotFoundError, ValueError) as exc:
                self._send_playback_error(exc)
                return
            except Exception as exc:  # noqa: BLE001
                self._send_playback_error(exc, internal=True)
                return
            self._send_json(manifest)
            return
        if path == "/api/playback/frame":
            episode_id = first_query_value(query, "episode")
            camera_id = first_query_value(query, "camera")
            try:
                frame_index = int(first_query_value(query, "index", "-1"))
                frame = self.server.runtime.playback.frame(  # type: ignore[attr-defined]
                    episode_id, camera_id, frame_index
                )
            except (FileNotFoundError, ValueError, KeyError, IndexError) as exc:
                self._send_playback_error(exc)
                return
            except Exception as exc:  # noqa: BLE001
                self._send_playback_error(exc, internal=True)
                return
            self._send_bytes(frame, "image/jpeg", cache_seconds=3600)
            return
        self._send_json({"ok": False, "error": "not_found", "path": path}, status=HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        body = self._read_json()
        if path == "/api/execute":
            result = start_execute_job(self.server.runtime, body)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        if path == "/api/stop":
            result = stop_execute_job(self.server.runtime)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        if path.startswith("/api/recording/"):
            action = path.removeprefix("/api/recording/")
            result = handle_recording_action(self.server.runtime, action, body)  # type: ignore[attr-defined]
            self._send_json(result, status=HTTPStatus.OK if result.get("ok") else HTTPStatus.CONFLICT)
            return
        self._send_json({"ok": False, "error": "not_found", "path": path}, status=HTTPStatus.NOT_FOUND)

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            value = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return {}
        return value if isinstance(value, dict) else {}

    def _send_file(self, path: Path) -> None:
        try:
            resolved = path.resolve()
            if not str(resolved).startswith(str(WEB_DIR.resolve())):
                raise FileNotFoundError
            data = resolved.read_bytes()
        except FileNotFoundError:
            self._send_json({"ok": False, "error": "not_found"}, status=HTTPStatus.NOT_FOUND)
            return
        content_type = "text/html; charset=utf-8"
        if resolved.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif resolved.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        self.send_response(HTTPStatus.OK)
        self._cors()
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_bytes(
        self,
        data: bytes,
        content_type: str,
        cache_seconds: int = 0,
    ) -> None:
        self.send_response(HTTPStatus.OK)
        self._cors()
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header(
            "Cache-Control",
            f"public, max-age={cache_seconds}" if cache_seconds else "no-store",
        )
        self.end_headers()
        self.wfile.write(data)

    def _send_playback_error(
        self,
        exc: Exception,
        internal: bool = False,
    ) -> None:
        if internal:
            status = HTTPStatus.INTERNAL_SERVER_ERROR
            error = "playback_decode_failed"
        elif isinstance(exc, ValueError):
            status = HTTPStatus.BAD_REQUEST
            error = "invalid_playback_request"
        else:
            status = HTTPStatus.NOT_FOUND
            error = "playback_data_not_found"
        self._send_json(
            {
                "ok": False,
                "error": error,
                "message": f"{exc.__class__.__name__}: {exc}",
            },
            status=status,
        )

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self._cors()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"{self.log_date_time_string()} {self.address_string()} {fmt % args}")


def first_query_value(
    query: dict[str, list[str]],
    name: str,
    fallback: str = "",
) -> str:
    values = query.get(name) or []
    return str(values[0]) if values else fallback


def start_execute_job(runtime: Runtime, body: dict[str, Any]) -> dict[str, Any]:
    task_mode = clean_choice(body.get("task_mode"), {"restock", "pick"}, "restock")
    drink = drink_by_id(str(body.get("drink_id") or ""))
    if drink is None:
        return {"ok": False, "error": "invalid_drink_id", "drink_id": body.get("drink_id")}
    arm = arm_by_id(str(body.get("arm") or "right"))
    if arm is None:
        return {"ok": False, "error": "invalid_arm", "arm": body.get("arm")}
    layer = clean_choice(body.get("shelf_layer"), {"1", "2", "3"}, "1")
    position = clean_choice(body.get("shelf_position"), {str(index) for index in range(1, 7)}, "1")
    scene_id = bounded_int(body.get("scene_id"), runtime.scene_id, 1, 999)
    gripper_width = bounded_float(body.get("gripper_width_m"), runtime.gripper_width_m, 0.0, 0.1)
    camera_frame = str(body.get("camera_frame") or arm["camera_frame"]).strip() or arm["camera_frame"]
    if task_mode == "pick":
        place_pose, place_pose_configured = pose_for_pick_place(runtime.pick_place_poses, arm["id"])
        service_name = runtime.pick_service_name
    else:
        place_pose, place_pose_configured = pose_for_slot(runtime.place_poses, arm["id"], layer, position)
        service_name = runtime.service_name

    with runtime.lock:
        if runtime.busy:
            return {
                "ok": False,
                "error": "task_already_running",
                "current_job": runtime.current_job,
            }
        job_id = uuid.uuid4().hex[:12]
        job = {
            "job_id": job_id,
            "status": "running",
            "started_at": time.time(),
            "finished_at": None,
            "task_mode": task_mode,
            "task_mode_name_zh": "拣货" if task_mode == "pick" else "上架",
            "drink_id": drink["id"],
            "drink_name_zh": drink["name_zh"],
            "prompt": drink["prompt"],
            "arm": arm["id"],
            "arm_name_zh": arm["name_zh"],
            "camera_frame": camera_frame,
            "scene_id": scene_id,
            "gripper_width_m": gripper_width,
            "shelf_layer": layer,
            "shelf_position": position,
            "place_pose_configured": place_pose_configured,
            "service": service_name,
        }
        runtime.busy = True
        runtime.current_job = job
        runtime.last_result = None

    thread = threading.Thread(
        target=execute_job,
        args=(runtime, job, place_pose),
        name=f"drink-grasp-{job_id}",
        daemon=True,
    )
    thread.start()
    return {"ok": True, "accepted": True, "job": job}


def stop_execute_job(runtime: Runtime) -> dict[str, Any]:
    with runtime.lock:
        if not runtime.busy or runtime.current_job is None:
            return {"ok": False, "error": "no_running_task", "message": "当前没有执行中的任务"}
        job = runtime.current_job
        if job.get("status") == "stopping":
            return {"ok": True, "accepted": True, "message": "停止请求已发送", "job": job}
        job["status"] = "stopping"
        job["stop_requested_at"] = time.time()

    client = runtime.cancel_client
    if runtime.ros_error or Trigger is None or client is None:
        with runtime.lock:
            if runtime.current_job is job:
                job["status"] = "running"
        return {"ok": False, "error": "cancel_service_unavailable", "message": "ROS 停止服务不可用"}
    if not client.wait_for_service(timeout_sec=min(runtime.service_wait_sec, 1.0)):
        with runtime.lock:
            if runtime.current_job is job:
                job["status"] = "running"
        return {
            "ok": False,
            "error": "cancel_service_unavailable",
            "message": f"停止服务不可用: {runtime.cancel_service_name}",
        }

    future = client.call_async(Trigger.Request())
    done = threading.Event()
    future.add_done_callback(lambda _future: done.set())
    if not done.wait(timeout=2.0):
        return {"ok": True, "accepted": True, "message": "停止请求已发送，等待执行链退出", "job": job}
    try:
        response = future.result()
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": exc.__class__.__name__, "message": str(exc), "job": job}
    return {
        "ok": bool(response and response.success),
        "accepted": bool(response and response.success),
        "message": str(response.message) if response else "停止服务未返回结果",
        "job": job,
    }


def handle_recording_action(runtime: Runtime, action: str, body: dict[str, Any]) -> dict[str, Any]:
    normalized = str(action or "").strip().lower().replace("-", "_")
    if normalized == "start":
        drink = drink_by_id(str(body.get("drink_id") or ""))
        task_mode = clean_choice(body.get("task_mode"), {"restock", "pick"}, "restock")
        task_name = clean_record_text(
            body.get("task_name")
            or f"{task_mode}_{drink['id'] if drink else 'drink'}",
            "drink_grasp",
            80,
        )
        note = clean_record_text(body.get("note"), "", 500)
        return runtime.recorder.start(task_name=task_name, note=note)
    if normalized == "stop":
        return runtime.recorder.stop()
    if normalized in {"save", "mark", "success"}:
        mark = "success" if normalized in {"mark", "success"} else body.get("mark", "success")
        return runtime.recorder.save(mark=str(mark or "success"))
    if normalized in {"discard", "delete"}:
        return runtime.recorder.discard()
    return {"ok": False, "error": "unknown_recording_action", "action": action}


def execute_job(runtime: Runtime, job: dict[str, Any], place_pose: PoseStamped) -> None:
    result: dict[str, Any]
    started = time.monotonic()
    print(
        "[TIMING] web phase=job_start "
        f"job_id={job.get('id')} task_mode={job.get('task_mode')} "
        f"target={job.get('prompt')!r} arm={job.get('arm')} service={job.get('service')}",
        flush=True,
    )
    try:
        if runtime.ros_error:
            raise RuntimeError(runtime.ros_error)
        if ExecuteNamedGraspTask is None:
            raise RuntimeError("graspnet_bridge_interfaces/srv/ExecuteNamedGraspTask is not available")
        task_mode = str(job.get("task_mode") or "restock")
        client = runtime.pick_client if task_mode == "pick" else runtime.client
        service_name = str(job.get("service") or runtime.service_name)
        if client is None:
            raise RuntimeError("ROS service client is not available")
        if not client.wait_for_service(timeout_sec=runtime.service_wait_sec):
            raise RuntimeError(f"service unavailable: {service_name}")

        request = ExecuteNamedGraspTask.Request()
        request.target_name = str(job["prompt"])
        request.arm_name = str(job["arm"])
        request.camera_frame = str(job["camera_frame"])
        request.scene_id = int(job["scene_id"])
        request.place_tcp_pose = place_pose
        request.gripper_width_m = float(job["gripper_width_m"])

        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(runtime.call_timeout_sec):
            with contextlib_suppress():
                future.cancel()
            raise TimeoutError(f"timeout waiting for {service_name}: {runtime.call_timeout_sec:.1f}s")
        response = future.result()
        if response is None:
            raise RuntimeError(f"{service_name} returned no response")
        ok = bool(response.success)
        message = str(response.message)
        stopped = job.get("status") == "stopping" or "cancel" in message.lower()
        result = {
            "ok": ok and not stopped,
            "success": ok and not stopped,
            "status": "stopped" if stopped else ("completed" if ok else "failed"),
            "message": "执行已停止" if stopped else message,
            "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
        }
        if stopped:
            result["error"] = "canceled"
        elif not ok:
            result["error"] = "state_machine_failed"
    except Exception as exc:  # noqa: BLE001
        stopped = job.get("status") == "stopping"
        result = {
            "ok": False,
            "success": False,
            "status": "stopped" if stopped else "error",
            "error": "canceled" if stopped else exc.__class__.__name__,
            "message": "执行已停止" if stopped else str(exc),
            "elapsed_ms": round((time.monotonic() - started) * 1000.0, 1),
        }

    print(
        "[TIMING] web phase=job_finish "
        f"job_id={job.get('id')} success={bool(result.get('success'))} "
        f"status={result.get('status')} elapsed_ms={result.get('elapsed_ms')} "
        f"message={str(result.get('message'))!r}",
        flush=True,
    )

    with runtime.lock:
        completed = {
            **job,
            **result,
            "finished_at": time.time(),
        }
        runtime.last_result = completed
        runtime.current_job = None
        runtime.busy = False


def status_payload(runtime: Runtime) -> dict[str, Any]:
    with runtime.lock:
        return {
            "ok": True,
            "busy": runtime.busy,
            "current_job": runtime.current_job,
            "last_result": runtime.last_result,
            "ros": {
                "available": (
                    runtime.ros_error is None
                    and runtime.client is not None
                    and runtime.pick_client is not None
                    and runtime.cancel_client is not None
                ),
                "error": runtime.ros_error,
                "service": runtime.service_name,
                "pick_service": runtime.pick_service_name,
                "cancel_service": runtime.cancel_service_name,
                "services": {
                    "restock": runtime.service_name,
                    "pick": runtime.pick_service_name,
                },
                "domain_id": os.environ.get("ROS_DOMAIN_ID"),
            },
            "recording": runtime.recorder.status(),
        }


def pose_for_slot(
    place_poses: dict[str, Any],
    arm: str,
    layer: str,
    position: str,
) -> tuple[PoseStamped, bool]:
    slot = (
        nested_get(place_poses, ["arms", arm, "layers", layer, "positions", position])
        or nested_get(place_poses, [arm, layer, position])
        or nested_get(place_poses, [layer, position])
    )
    return pose_from_config(slot)


def pose_for_pick_place(place_poses: dict[str, Any], arm: str) -> tuple[PoseStamped, bool]:
    slot = (
        nested_get(place_poses, ["arms", arm, "default"])
        or nested_get(place_poses, ["arms", arm, "place"])
        or nested_get(place_poses, [arm, "default"])
        or nested_get(place_poses, [arm, "place"])
        or nested_get(place_poses, ["default"])
        or nested_get(place_poses, ["place"])
    )
    return pose_from_config(slot)


def pose_from_config(slot: Any) -> tuple[PoseStamped, bool]:
    pose = PoseStamped()
    if not isinstance(slot, dict):
        return pose, False
    frame_id = str(slot.get("frame_id") or slot.get("frame") or "world").strip()
    xyz = slot.get("position") or slot.get("xyz") or [
        slot.get("x"),
        slot.get("y"),
        slot.get("z"),
    ]
    quat = slot.get("orientation_xyzw") or slot.get("xyzw") or [
        slot.get("qx"),
        slot.get("qy"),
        slot.get("qz"),
        slot.get("qw"),
    ]
    try:
        xyz_values = [float(value) for value in xyz]
        quat_values = [float(value) for value in quat]
    except (TypeError, ValueError):
        return pose, False
    if len(xyz_values) != 3 or len(quat_values) != 4:
        return pose, False
    pose.header.frame_id = frame_id or "world"
    pose.pose.position.x = xyz_values[0]
    pose.pose.position.y = xyz_values[1]
    pose.pose.position.z = xyz_values[2]
    pose.pose.orientation.x = quat_values[0]
    pose.pose.orientation.y = quat_values[1]
    pose.pose.orientation.z = quat_values[2]
    pose.pose.orientation.w = quat_values[3]
    return pose, True


def load_place_poses() -> dict[str, Any]:
    if not PLACE_POSES_FILE.exists():
        return {}
    try:
        value = json.loads(PLACE_POSES_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def load_pick_place_poses() -> dict[str, Any]:
    if not PICK_PLACE_POSES_FILE.exists():
        return {}
    try:
        value = json.loads(PICK_PLACE_POSES_FILE.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return {}
    return value if isinstance(value, dict) else {}


def configured_slot_keys(place_poses: dict[str, Any]) -> list[str]:
    keys: list[str] = []
    for arm in ("left", "right"):
        for layer in ("1", "2", "3"):
            for position in (str(index) for index in range(1, 7)):
                _, configured = pose_for_slot(place_poses, arm, layer, position)
                if configured:
                    keys.append(f"{arm}:{layer}:{position}")
    return keys


def configured_pick_place_arms(place_poses: dict[str, Any]) -> list[str]:
    arms: list[str] = []
    for arm in ("left", "right"):
        _, configured = pose_for_pick_place(place_poses, arm)
        if configured:
            arms.append(arm)
    return arms


def drink_by_id(value: str) -> dict[str, Any] | None:
    return next((drink for drink in DRINKS if drink["id"] == value), None)


def arm_by_id(value: str) -> dict[str, Any] | None:
    return next((arm for arm in ARMS if arm["id"] == value), None)


def clean_choice(value: Any, allowed: set[str], fallback: str) -> str:
    text = str(value or "").strip()
    return text if text in allowed else fallback


def bounded_int(value: Any, default: int, minimum: int, maximum: int) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError):
        number = default
    return max(minimum, min(maximum, number))


def bounded_float(value: Any, default: float, minimum: float, maximum: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = default
    if not minimum <= number <= maximum:
        return default
    return number


def env_bool(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off", "disabled"}


def configured_record_topics() -> list[str]:
    configured = os.environ.get("DRINK_GRASP_RECORD_TOPICS", "").strip()
    raw_topics = re.split(r"[\s,]+", configured) if configured else DEFAULT_RECORD_TOPICS
    topics: list[str] = []
    seen: set[str] = set()
    for raw in raw_topics:
        topic = str(raw or "").strip()
        if not topic:
            continue
        if not topic.startswith("/"):
            topic = f"/{topic}"
        if topic in seen:
            continue
        seen.add(topic)
        topics.append(topic)
    return topics


def sanitize_component(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value or "").strip())
    text = text.strip("._-")
    return text[:80] or "drink_grasp"


def clean_record_text(value: Any, fallback: str, limit: int) -> str:
    text = str(value if value is not None else fallback).strip()
    text = re.sub(r"[\x00-\x1f\x7f]+", " ", text)
    return (text or fallback)[:limit]


def recording_duration(session: RecordingSession) -> float:
    end = session.stopped_at if session.stopped_at is not None else time.time()
    return round(max(0.0, end - session.started_at), 3)


def tail_text(path: Path, max_bytes: int = 4000) -> str:
    try:
        data = path.read_bytes()
    except OSError:
        return ""
    return data[-max_bytes:].decode("utf-8", errors="replace").strip()


def nested_get(mapping: dict[str, Any], keys: list[str]) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


class contextlib_suppress:
    def __enter__(self) -> None:
        return None

    def __exit__(self, *_args: Any) -> bool:
        return True


def make_runtime(host: str, port: int) -> Runtime:
    ros_node = None
    client = None
    pick_client = None
    cancel_client = None
    ros_error = None
    service_name = os.environ.get("DRINK_GRASP_SERVICE", "/execute_named_grasp_task").strip()
    pick_service_name = os.environ.get("DRINK_GRASP_PICK_SERVICE", "/execute_named_pick_task").strip()
    cancel_service_name = os.environ.get(
        "DRINK_GRASP_CANCEL_SERVICE", "/cancel_named_grasp_task"
    ).strip()
    try:
        if ExecuteNamedGraspTask is None or Trigger is None:
            raise RuntimeError("required ROS service interfaces are unavailable; source the ROS overlays")
        rclpy.init(args=None)
        ros_node = rclpy.create_node("drink_grasp_web")
        client = ros_node.create_client(ExecuteNamedGraspTask, service_name)
        pick_client = ros_node.create_client(ExecuteNamedGraspTask, pick_service_name)
        cancel_client = ros_node.create_client(Trigger, cancel_service_name)
        threading.Thread(target=rclpy.spin, args=(ros_node,), name="drink-grasp-ros-spin", daemon=True).start()
    except Exception as exc:  # noqa: BLE001
        ros_error = f"{exc.__class__.__name__}: {exc}"
    return Runtime(
        host=host,
        port=port,
        service_name=service_name,
        pick_service_name=pick_service_name,
        cancel_service_name=cancel_service_name,
        scene_id=bounded_int(os.environ.get("DRINK_GRASP_SCENE_ID"), 1, 1, 999),
        gripper_width_m=bounded_float(os.environ.get("DRINK_GRASP_GRIPPER_WIDTH_M"), 0.08, 0.0, 0.1),
        service_wait_sec=bounded_float(os.environ.get("DRINK_GRASP_SERVICE_WAIT_SEC"), 5.0, 0.0, 60.0),
        call_timeout_sec=bounded_float(os.environ.get("DRINK_GRASP_CALL_TIMEOUT_SEC"), 180.0, 1.0, 900.0),
        ros_node=ros_node,
        client=client,
        pick_client=pick_client,
        cancel_client=cancel_client,
        ros_error=ros_error,
        place_poses=load_place_poses(),
        pick_place_poses=load_pick_place_poses(),
        recorder=LocalRosbagRecorder(),
        playback=EpisodePlaybackLibrary(RECORD_DIR),
        lock=threading.Lock(),
        busy=False,
        current_job=None,
        last_result=None,
    )


def main() -> int:
    host = os.environ.get("DRINK_GRASP_HOST", "0.0.0.0").strip() or "0.0.0.0"
    port = bounded_int(os.environ.get("DRINK_GRASP_PORT"), 8090, 1, 65535)
    runtime = make_runtime(host, port)
    server = DrinkGraspServer((host, port), runtime)

    def shutdown(signum: int, _frame: Any) -> None:
        print(f"received signal {signum}, shutting down")
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"Drink grasp web listening on http://{host}:{port}/")
    print(
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} "
        f"service={runtime.service_name} pick_service={runtime.pick_service_name}"
    )
    if runtime.ros_error:
        print(f"ROS unavailable: {runtime.ros_error}")
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
        if runtime.ros_node is not None:
            runtime.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
