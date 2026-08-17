#!/usr/bin/env python3
"""Four-camera ROS image gateway and YOLO mask Web console."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from four_camera_yolo_core import (
    CAMERAS,
    HealthMonitor,
    StateStore,
    WorkerGroup,
    YoloHttpClient,
    validate_interval_sec,
)


MAX_JSON_BODY_BYTES = 64 * 1024
HTML_PATH = Path(__file__).with_suffix(".html")
CAMERA_IMAGE_RE = re.compile(
    r"^/api/cameras/(left|front|right|head)/(latest|source|overlay)\.jpg$"
)


class RequestTooLarge(ValueError):
    pass


def read_json_body(
    handler: BaseHTTPRequestHandler,
    max_bytes: int = MAX_JSON_BODY_BYTES,
) -> object:
    value = handler.headers.get("Content-Length")
    if value is None:
        raise ValueError("Content-Length is required")
    try:
        length = int(value)
    except ValueError as exc:
        raise ValueError("Content-Length must be an integer") from exc
    if length < 0:
        raise ValueError("Content-Length must not be negative")
    if length > max_bytes:
        raise RequestTooLarge("request body is too large")
    raw = handler.rfile.read(length)
    try:
        text = raw.decode("utf-8")
        return json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON body: {exc}") from exc


def make_handler(state: StateStore, html_bytes: bytes):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _bytes(
            self,
            status: int,
            payload: bytes,
            content_type: str,
            *,
            no_store: bool = True,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            if no_store:
                self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def _json(self, status: int, payload: dict) -> None:
            encoded = json.dumps(
                payload, ensure_ascii=False, separators=(",", ":")
            ).encode("utf-8")
            self._bytes(
                status,
                encoded,
                "application/json; charset=utf-8",
                no_store=True,
            )

        def do_GET(self) -> None:
            parsed = urllib.parse.urlsplit(self.path)
            try:
                if parsed.path == "/":
                    self._bytes(
                        200, html_bytes, "text/html; charset=utf-8", no_store=True
                    )
                    return
                if parsed.path == "/api/status":
                    self._json(200, state.status(time.monotonic()))
                    return
                if parsed.path == "/api/classes":
                    self._json(
                        200,
                        {"ok": True, "classes": state.classes_snapshot()},
                    )
                    return
                match = CAMERA_IMAGE_RE.fullmatch(parsed.path)
                if match:
                    camera, kind = match.groups()
                    payload = state.image(camera, kind)
                    if payload is None:
                        self._json(
                            404,
                            {"ok": False, "error": "image not available"},
                        )
                        return
                    self._bytes(200, payload, "image/jpeg", no_store=True)
                    return
                self._json(404, {"ok": False, "error": "not found"})
            except (BrokenPipeError, ConnectionResetError):
                return
            except Exception:
                self._json(500, {"ok": False, "error": "internal server error"})

        def do_POST(self) -> None:
            if urllib.parse.urlsplit(self.path).path != "/api/config":
                self._json(404, {"ok": False, "error": "not found"})
                return
            try:
                content_type = self.headers.get_content_type()
                if content_type != "application/json":
                    self._json(
                        415,
                        {"ok": False, "error": "Content-Type must be application/json"},
                    )
                    return
                payload = read_json_body(self)
                generation = state.apply_config_payload(payload)
            except RequestTooLarge as exc:
                self._json(413, {"ok": False, "error": str(exc)})
                return
            except ValueError as exc:
                self._json(400, {"ok": False, "error": str(exc)})
                return
            except (BrokenPipeError, ConnectionResetError):
                return
            except Exception:
                self._json(500, {"ok": False, "error": "internal server error"})
                return
            self._json(200, {"ok": True, "generation": generation})

        def log_message(self, _format: str, *_args) -> None:
            pass

    return Handler


class ConsoleServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def create_server(
    host: str,
    port: int,
    state: StateStore,
    html_path: str | Path,
) -> ConsoleServer:
    html_bytes = Path(html_path).read_bytes()
    return ConsoleServer((host, port), make_handler(state, html_bytes))


def _port(value: str) -> int:
    try:
        port = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("port must be an integer") from exc
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def _positive_float(value: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be numeric") from exc
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="Web bind address")
    parser.add_argument("--port", type=_port, default=7788, help="Web TCP port")
    parser.add_argument("--ros-domain-id", type=int, default=99)
    parser.add_argument("--yolo-url", default="http://192.168.4.121:7881")
    parser.add_argument(
        "--default-interval-sec",
        type=validate_interval_sec,
        default=0.2,
        help="global inference interval, 0.2 through 60 seconds",
    )
    parser.add_argument("--request-timeout", type=_positive_float, default=10.0)
    args = parser.parse_args(argv)
    if args.ros_domain_id < 0:
        parser.error("--ros-domain-id must not be negative")
    return args


def create_ros_node(state: StateStore):
    """Create the ROS adapter lazily so HTTP unit tests need no ROS install."""

    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import CompressedImage

    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    class FourCameraNode(Node):
        def __init__(self) -> None:
            super().__init__("four_camera_yolo_web")
            self._camera_subscriptions = []
            for camera in CAMERAS:
                callback = lambda message, key=camera.key: state.update_frame(
                    key,
                    bytes(message.data),
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec,
                    time.monotonic(),
                )
                subscription = self.create_subscription(
                    CompressedImage,
                    camera.topic,
                    callback,
                    sensor_qos,
                )
                self._camera_subscriptions.append(subscription)

    return FourCameraNode()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    os.environ["ROS_DOMAIN_ID"] = str(args.ros_domain_id)
    state = StateStore(args.default_interval_sec)
    try:
        server = create_server(args.host, args.port, state, HTML_PATH)
    except OSError as exc:
        print(
            f"[错误] 无法监听 {args.host}:{args.port}：{exc}",
            file=sys.stderr,
        )
        return 2

    stop_event = threading.Event()
    node = None
    workers = None
    web_thread = None
    health_thread = None
    rclpy_started = False
    exit_code = 0
    old_handlers = {}
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException

        rclpy.init(args=None)
        rclpy_started = True
        node = create_ros_node(state)
        client_factory = lambda: YoloHttpClient(
            args.yolo_url, args.request_timeout
        )
        workers = WorkerGroup(state, client_factory, stop_event)
        health = HealthMonitor(state, client_factory, time.monotonic)

        def request_stop(_signum=None, _frame=None) -> None:
            stop_event.set()
            state.wake_config_waiters()
            try:
                rclpy.try_shutdown()
            except RuntimeError:
                pass

        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)

        web_thread = threading.Thread(
            target=server.serve_forever,
            name="yolo-web",
            daemon=True,
        )
        health_thread = threading.Thread(
            target=health.run,
            args=(stop_event,),
            name="yolo-health",
            daemon=True,
        )
        workers.start()
        web_thread.start()
        health_thread.start()
        print(
            f"[YOLO Web] http://{args.host}:{args.port} | "
            f"ROS_DOMAIN_ID={args.ros_domain_id} | YOLO={args.yolo_url}",
            flush=True,
        )
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    except Exception as exc:
        print(f"[错误] 启动或运行失败：{exc}", file=sys.stderr)
        exit_code = 1
    finally:
        stop_event.set()
        state.wake_config_waiters()
        if web_thread is not None:
            server.shutdown()
        server.server_close()
        if workers is not None:
            workers.stop(timeout=3.0)
        if health_thread is not None:
            health_thread.join(3.0)
        if web_thread is not None:
            web_thread.join(3.0)
        if node is not None:
            node.destroy_node()
        if rclpy_started:
            try:
                import rclpy

                rclpy.try_shutdown()
            except RuntimeError:
                pass
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
