#!/usr/bin/env python3
"""Single operator entry point for the existing collection workflows.

This module only supervises the existing shell entry points.  It intentionally
does not publish or subscribe to ROS topics and does not implement a second
collection state machine.
"""

from __future__ import annotations

import argparse
from html import escape
import json
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from collection_targets import TARGET_OPTIONS, validate_target


MODE_SCRIPTS = {
    "staged": SCRIPT_DIR / "collect_mobile_pipeline_web_staged.sh",
    "fixed_stage2": SCRIPT_DIR / "collect_mobile_pipeline_web_fixed_stage.sh",
    "grasp_place": SCRIPT_DIR / "collect_mobile_pipeline_web_grasp_place.sh",
    "inference": SCRIPT_DIR / "collect_mobile_pipeline_web_inference.sh",
}
MODE_NAMES = {
    "staged": "全阶段采集",
    "fixed_stage2": "第二阶段采集",
    "grasp_place": "自定义采集阶段",
    "inference": "推理采集",
}

TARGET_DISPLAY_NAMES = {
    "Coca-Cola": "可口可乐",
    "Daily C Grape Juice": "味全每日C葡萄汁",
    "Guangming Probiotic Milk": "光明益生菌风味发酵乳",
    "Daily C Orange Juice": "味全每日C橙汁",
    "AD Calcium Milk": "娃哈哈AD钙奶",
    "Robuk Velvet Latte": "罗伯克丝绒拿铁",
    "Aojiru": "青汁",
    "HK Orange Fanta": "港版橙味芬达",
    "Taro Milk": "芋泥牛乳",
    "Yili Peach Yogurt": "伊利桃味酸奶",
    "NEVER Coconut Latte": "NEVER生椰拿铁",
    "Yili Strawberry Yogurt": "伊利草莓味酸奶",
    "Wanglaoji": "王老吉",
    "Sprite": "雪碧",
    "Yakult": "养乐多",
    "Dahongpao Milk Tea": "大红袍奶茶",
}


def target_options_markup(empty_label: str, *, required: bool) -> str:
    empty_attrs = " disabled selected" if required else " selected"
    options = [f'<option value=""{empty_attrs}>{escape(empty_label)}</option>']
    for target in TARGET_OPTIONS:
        display_name = TARGET_DISPLAY_NAMES.get(target, target)
        label = f"{display_name} · {target}" if display_name != target else target
        options.append(
            f'<option value="{escape(target, quote=True)}">{escape(label)}</option>'
        )
    return "".join(options)


def json_bytes(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False).encode("utf-8")


def is_process_alive(process: subprocess.Popen[bytes] | None) -> bool:
    return process is not None and process.poll() is None


def process_identity(pid: int) -> tuple[int, int, str] | None:
    """Return (process group, start ticks, state) for one Linux process."""
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields = raw[raw.rfind(")") + 2 :].split()
        return int(fields[2]), int(fields[19]), fields[0]
    except (FileNotFoundError, PermissionError, IndexError, ValueError):
        return None


def descendant_identities(root_pid: int) -> dict[int, tuple[int, int]]:
    """Snapshot root and every currently visible descendant without psutil."""
    children: dict[int, list[int]] = {}
    identities: dict[int, tuple[int, int]] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            raw = (entry / "stat").read_text(encoding="utf-8")
            fields = raw[raw.rfind(")") + 2 :].split()
            parent_pid = int(fields[1])
            process_group = int(fields[2])
            start_ticks = int(fields[19])
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            continue
        children.setdefault(parent_pid, []).append(pid)
        identities[pid] = (process_group, start_ticks)

    selected: dict[int, tuple[int, int]] = {}
    pending = [root_pid]
    while pending:
        pid = pending.pop()
        if pid in selected:
            continue
        identity = identities.get(pid)
        if identity is not None:
            selected[pid] = identity
        pending.extend(children.get(pid, ()))
    return selected


def matching_live_pids(identities: dict[int, tuple[int, int]]) -> list[int]:
    live: list[int] = []
    for pid, (_process_group, expected_start) in identities.items():
        current = process_identity(pid)
        if current is None:
            continue
        _current_group, current_start, state = current
        if current_start == expected_start and state != "Z":
            live.append(pid)
    return live


def find_free_port(start: int, excluded: set[int], bind_host: str) -> int:
    for port in range(start, start + 500):
        if port in excluded:
            continue
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((bind_host, port))
            except OSError:
                continue
            return port
    raise RuntimeError(f"从端口 {start} 开始没有找到空闲端口")


def suggested_episode(data_dir: Path) -> int:
    highest = -1
    if data_dir.is_dir():
        for child in data_dir.iterdir():
            match = re.fullmatch(r"episode(\d+)", child.name)
            if match and child.is_dir():
                highest = max(highest, int(match.group(1)))
    return highest + 1


class CollectionSupervisor:
    def __init__(self, manager_host: str, manager_port: int, runtime_root: Path) -> None:
        self.manager_host = manager_host
        self.probe_host = "127.0.0.1" if manager_host in {"0.0.0.0", "::"} else manager_host
        self.manager_port = manager_port
        self.runtime_root = runtime_root
        self.runtime_root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.process: subprocess.Popen[bytes] | None = None
        self.log_handle: Any = None
        self.mode = ""
        self.mode_name = ""
        self.child_port = 0
        self.ready = False
        self.data_dir = ""
        self.start_index = 0
        self.started_at = ""
        self.finished_at = ""
        self.exit_code: int | None = None
        self.log_path = ""
        self.last_error = ""

    def status(self) -> dict[str, Any]:
        with self.lock:
            running = is_process_alive(self.process)
            if self.process is not None and not running and self.exit_code is None:
                self.exit_code = self.process.poll()
                self.finished_at = datetime.now().strftime("%F %T")
                self._close_log()
            return {
                "running": running,
                "mode": self.mode,
                "mode_name": self.mode_name,
                "pid": self.process.pid if running and self.process else None,
                "port": self.child_port if running else None,
                "ready": running and self.ready,
                "data_dir": self.data_dir,
                "start_index": self.start_index,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "exit_code": self.exit_code,
                "log_path": self.log_path,
                "last_error": self.last_error,
            }

    def _close_log(self) -> None:
        if self.log_handle is not None:
            try:
                self.log_handle.close()
            except OSError:
                pass
            self.log_handle = None

    def start(self, config: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if is_process_alive(self.process):
                raise ValueError("已有采集模式正在运行；请先在当前页面停止它")

            mode = str(config.get("mode", "")).strip()
            if mode not in MODE_SCRIPTS:
                raise ValueError("未知采集模式")
            script = MODE_SCRIPTS[mode]
            if not script.is_file():
                raise ValueError(f"找不到模式脚本: {script}")

            raw_data_dir = str(config.get("data_dir", "")).strip()
            if not raw_data_dir:
                raise ValueError("必须填写数据保存目录")
            data_dir = Path(raw_data_dir).expanduser()
            if not data_dir.is_absolute():
                raise ValueError("数据保存目录必须是绝对路径")
            data_dir.mkdir(parents=True, exist_ok=True)

            try:
                start_index = int(config.get("start_index", 0))
            except (TypeError, ValueError) as exc:
                raise ValueError("起始 episode 必须是非负整数") from exc
            if start_index < 0:
                raise ValueError("起始 episode 必须是非负整数")
            existing_episode = data_dir / f"episode{start_index}"
            if existing_episode.exists() or existing_episode.is_symlink():
                next_index = suggested_episode(data_dir)
                raise ValueError(
                    f"{existing_episode} 已存在；为防止原后端删除同名数据，门户不允许覆盖。"
                    f"请使用“自动计算下一个 episode”（当前建议 {next_index}）"
                )

            grade = str(config.get("grade", "")).strip().upper()
            if grade not in {"", "A", "B", "F"}:
                raise ValueError("自动等级只能是 A、B、F 或留空人工复核")
            left_target = str(config.get("left_target", "")).strip()
            right_target = str(config.get("right_target", "")).strip()
            if mode == "inference":
                try:
                    left_target = validate_target(left_target)
                    right_target = validate_target(right_target)
                except ValueError as exc:
                    raise ValueError("推理商品必须从 16 类下拉菜单中选择") from exc
                if not left_target:
                    raise ValueError("推理采集必须选择左手商品")
            else:
                left_target = ""
                right_target = ""

            child_port = find_free_port(
                max(8100, self.manager_port + 1),
                {self.manager_port},
                self.manager_host,
            )
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_dir = self.runtime_root / f"{stamp}_{mode}"
            session_dir.mkdir(parents=True, exist_ok=True)
            log_path = session_dir / "launcher.log"

            command = ["bash", str(script)]
            if mode == "fixed_stage2":
                command.extend(["--fixed-stage", "2", str(data_dir), str(start_index)])
            elif mode == "inference":
                command.extend([str(data_dir), str(start_index), left_target])
                if right_target:
                    command.append(right_target)
            else:
                command.extend([str(data_dir), str(start_index)])
            if grade and mode != "inference":
                command.extend(["--grade", grade])

            env = os.environ.copy()
            env.update(
                {
                    "WEB_HOST": self.manager_host,
                    "WEB_PORT": str(child_port),
                    "WEB_LOG_ROOT": str(session_dir / "outer"),
                    "WEB_RUNTIME_DIR": str(session_dir / "collector_runtime"),
                    "RAW_MCAP_ONLY": "1",
                    "RUN_CONVERT": "0",
                    "RUN_QC": "0",
                    "RUN_LEROBOT": "0",
                    "BACKGROUND_PROCESSING": "0",
                    "PIPELINE_ENABLE": "0",
                    "VISUALIZER_ENABLE": "0",
                    "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
                    "ROS_DOMAIN_ID": "99",
                }
            )
            if mode == "staged":
                env.update(
                    {
                        "COLLECTION_STAGED_CAPTURE_DEFAULT": "1",
                        "STATE_MACHINE_STAGED_DEFAULT": "1",
                    }
                )
            elif mode == "inference":
                env["COLLECTION_AUTO_GRADE"] = "A"
            self.log_handle = log_path.open("ab", buffering=0)
            try:
                process = subprocess.Popen(
                    command,
                    cwd=str(SCRIPT_DIR.parent.parent),
                    env=env,
                    stdin=subprocess.DEVNULL,
                    stdout=self.log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception:
                self._close_log()
                raise

            self.process = process
            self.mode = mode
            self.mode_name = MODE_NAMES[mode]
            self.child_port = child_port
            self.ready = False
            self.data_dir = str(data_dir)
            self.start_index = start_index
            self.started_at = datetime.now().strftime("%F %T")
            self.finished_at = ""
            self.exit_code = None
            self.log_path = str(log_path)
            self.last_error = ""
            threading.Thread(target=self._watch, args=(process,), daemon=True).start()
            threading.Thread(
                target=self._wait_until_ready,
                args=(process, child_port),
                daemon=True,
            ).start()
            return self.status()

    def _wait_until_ready(self, process: subprocess.Popen[bytes], port: int) -> None:
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline and is_process_alive(process):
            try:
                with socket.create_connection((self.probe_host, port), timeout=0.4):
                    with self.lock:
                        if self.process is process:
                            self.ready = True
                    return
            except OSError:
                time.sleep(0.25)
        with self.lock:
            if self.process is process and is_process_alive(process):
                self.last_error = "模式进程仍在运行，但操作台 45 秒内没有就绪；请查看日志"

    def _watch(self, process: subprocess.Popen[bytes]) -> None:
        exit_code = process.wait()
        with self.lock:
            if self.process is process:
                self.ready = False
                self.exit_code = exit_code
                self.finished_at = datetime.now().strftime("%F %T")
                if exit_code != 0 and not self.last_error:
                    self.last_error = f"模式进程退出，退出码 {exit_code}；请查看日志"
                self._close_log()

    def stop(self) -> dict[str, Any]:
        with self.lock:
            process = self.process
            if not is_process_alive(process):
                return self.status()
            assert process is not None
            process_group = process.pid
        owned = descendant_identities(process.pid)

        # SIGINT lets the existing scripts run their own save/cleanup traps.
        try:
            os.killpg(process_group, signal.SIGINT)
        except ProcessLookupError:
            pass

        graceful_deadline = time.monotonic() + 45.0
        while process.poll() is None and time.monotonic() < graceful_deadline:
            # Include nested setsid processes created while the outer script was
            # still entering its shutdown trap.
            owned.update(descendant_identities(process.pid))
            time.sleep(0.1)

        if process.poll() is None:
            try:
                os.killpg(process_group, signal.SIGTERM)
            except ProcessLookupError:
                pass
            term_deadline = time.monotonic() + 10.0
            while process.poll() is None and time.monotonic() < term_deadline:
                owned.update(descendant_identities(process.pid))
                time.sleep(0.1)

        if process.poll() is None:
            try:
                os.killpg(process_group, signal.SIGKILL)
            except ProcessLookupError:
                pass
        process.wait()

        # The outer scripts normally remove all nested process groups.  This is
        # a bounded fallback for a crashed cleanup trap; PID start times prevent
        # killing an unrelated process if Linux has already reused a PID.
        for sig, timeout in ((signal.SIGINT, 5.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 1.0)):
            live = matching_live_pids(owned)
            if not live:
                break
            for pid in live:
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + timeout
            while matching_live_pids(owned) and time.monotonic() < deadline:
                time.sleep(0.1)
        return self.status()


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>松灵数据采集统一入口</title>
  <style>
    :root {
      font-family: Inter, "PingFang SC", "Microsoft YaHei", system-ui, sans-serif;
      color: #172b3f;
      background: #edf3f7;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: linear-gradient(135deg, #f5f8fb 0%, #eaf1f6 100%);
    }
    .hidden { display: none !important; }
    #portalView { min-height: 100vh; padding: 24px 16px 38px; }
    main { width: min(1160px, 100%); margin: auto; display: grid; gap: 16px; }
    section {
      padding: 20px;
      border: 1px solid #d4e0e8;
      border-radius: 16px;
      background: white;
      box-shadow: 0 10px 28px rgba(28, 55, 77, .08);
    }
    .hero {
      padding: 28px;
      color: white;
      border: 0;
      background: linear-gradient(125deg, #12395b, #1769aa);
    }
    h1 { margin: 0 0 9px; font-size: 29px; }
    h2 { margin: 0 0 14px; font-size: 18px; }
    .note { color: #5c6f80; font-size: 13px; line-height: 1.55; }
    .hero .note { max-width: 850px; color: #dceaf5; }
    .grid { display: grid; grid-template-columns: 2fr .7fr 1fr; gap: 12px; }
    label { display: grid; gap: 6px; color: #4b6072; font-size: 13px; font-weight: 700; }
    input, select {
      width: 100%;
      height: 44px;
      padding: 0 11px;
      border: 1px solid #b8c8d5;
      border-radius: 9px;
      outline: none;
      background: white;
      font-size: 14px;
    }
    input:focus, select:focus {
      border-color: #2477c3;
      box-shadow: 0 0 0 3px rgba(36, 119, 195, .12);
    }
    .mode-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
      margin-top: 18px;
    }
    .mode-card {
      display: flex;
      flex-direction: column;
      min-height: 210px;
      gap: 9px;
      padding: 20px;
      border: 1px solid #d4e0e8;
      border-radius: 14px;
      background: #f7fafc;
    }
    .mode-card strong { font-size: 20px; }
    .mode-card button { width: 100%; margin-top: auto; }
    .mode-card.featured {
      min-height: 235px;
      color: white;
      border: 0;
      box-shadow: 0 14px 28px rgba(25, 75, 120, .18);
    }
    .mode-card.featured strong { font-size: 27px; }
    .mode-card.featured .note { color: rgba(255, 255, 255, .84); font-size: 14px; }
    .mode-card.staged { background: linear-gradient(135deg, #0d4e9b, #237fe0); }
    .mode-card.stage2 { background: linear-gradient(135deg, #08725a, #18a382); }
    .mode-card.custom { background: #f5f8fa; }
    .mode-card.inference { border-color: #d8cff5; background: #faf8ff; }
    .mode-badge {
      align-self: flex-start;
      padding: 5px 9px;
      border-radius: 999px;
      color: #496174;
      background: #e5edf3;
      font-size: 11px;
      font-weight: 800;
    }
    .featured .mode-badge { color: white; background: rgba(255, 255, 255, .17); }
    .target-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 9px; }
    .target-grid select { height: 41px; font-size: 12px; }
    .actions { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 14px; }
    button {
      min-height: 43px;
      padding: 0 17px;
      border: 0;
      border-radius: 9px;
      color: white;
      background: #1769aa;
      font-weight: 800;
      cursor: pointer;
    }
    button:hover:not(:disabled) { filter: brightness(1.06); transform: translateY(-1px); }
    .featured button { color: #164f86; background: white; }
    .stage2 button { color: #08725a; }
    button.stop { background: #b42318; }
    button.secondary { background: #526674; }
    button:disabled { opacity: .45; cursor: not-allowed; transform: none; }
    .status { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 9px; }
    .cell { min-height: 66px; padding: 11px; border-radius: 9px; background: #f2f6f8; }
    .cell b { display: block; margin-bottom: 5px; color: #6d7f8e; font-size: 11px; }
    .running { color: #137c4b; font-weight: 800; }
    .stopped { color: #64748b; font-weight: 800; }
    .error { min-height: 22px; margin-top: 10px; color: #b42318; white-space: pre-wrap; }
    #workbenchView { width: 100vw; height: 100vh; overflow: hidden; background: #dfe7ed; }
    .workbench-bar {
      height: 58px;
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 7px 12px;
      color: white;
      background: #162b3a;
    }
    .workbench-bar button { min-height: 40px; }
    .workbench-bar strong { flex: 1; font-size: 16px; }
    .workbench-bar span { color: #cbd5e1; font-size: 13px; }
    #console { display: block; width: 100%; height: calc(100vh - 58px); border: 0; background: white; }
    #workbenchLoading { height: calc(100vh - 58px); display: grid; place-items: center; color: #334155; font-weight: 700; }
    @media (max-width: 760px) {
      #portalView { padding: 8px; }
      .hero { padding: 21px 18px; }
      .grid, .mode-grid, .target-grid, .status { grid-template-columns: 1fr; }
      .mode-card, .mode-card.featured { min-height: 205px; }
      .workbench-bar span { display: none; }
      .workbench-bar button { padding: 0 10px; }
    }
  </style>
</head>
<body>
<div id="portalView">
  <main>
    <section class="hero">
      <h1>数据采集工作台</h1>
      <div class="note">先填写本次数据目录和 episode，再直接进入对应操作台。推理采集保存后自动标记为 A；其他模式仍使用所选的 A/B/F 等级或人工复核。</div>
    </section>
    <section id="configPanel">
      <h2>公共采集配置</h2>
      <div class="grid">
        <label>数据保存目录
          <input id="dataDir" placeholder="请输入绝对路径，例如 /home/agilex/data/stage2_new/scene13/20260729_scene13">
        </label>
        <label>起始 episode
          <input id="startIndex" type="number" min="0" value="0">
        </label>
        <label>保存后等级（非推理采集）
          <select id="grade"><option value="">人工选择 A / B / F（推荐）</option><option>A</option><option>B</option><option>F</option></select>
        </label>
      </div>
      <div class="actions">
        <button id="suggest" class="secondary">自动计算下一个 episode</button>
      </div>
      <div class="mode-grid">
        <div class="mode-card featured staged">
          <span class="mode-badge">核心入口</span>
          <strong>全阶段采集</strong>
          <div class="note">默认启用“分阶段”，沿用现有全阶段状态机。</div>
          <button class="modeStart" data-mode="staged">进入全阶段采集</button>
        </div>
        <div class="mode-card featured stage2">
          <span class="mode-badge">常用入口</span>
          <strong>第二阶段采集</strong>
          <div class="note">固定第二阶段快速入口。</div>
          <button class="modeStart" data-mode="fixed_stage2">进入第二阶段采集</button>
        </div>
        <div class="mode-card custom">
          <span class="mode-badge">专项入口</span>
          <strong>自定义采集阶段</strong>
          <div class="note">进入现有自定义双阶段采集流程，底层采集方式保持不变。</div>
          <button class="modeStart" data-mode="grasp_place">进入自定义采集阶段</button>
        </div>
        <div class="mode-card inference">
          <span class="mode-badge">16 类商品</span>
          <strong>推理采集</strong>
          <div class="target-grid">
            <label>左手商品（必选）
              <select id="leftTarget">__LEFT_TARGET_OPTIONS__</select>
            </label>
            <label>右手商品（可选）
              <select id="rightTarget">__RIGHT_TARGET_OPTIONS__</select>
            </label>
          </div>
          <div class="note">单手只选左侧；双手同时选择。每条推理录制保存后自动标记为 A，无需手动选择等级。</div>
          <button class="modeStart" data-mode="inference">进入推理采集</button>
        </div>
      </div>
      <div id="error" class="error"></div>
    </section>
    <section>
      <h2>运行状态</h2>
      <div class="status">
        <div class="cell"><b>状态</b><span id="runState">-</span></div>
        <div class="cell"><b>模式</b><span id="runMode">-</span></div>
        <div class="cell"><b>进程 / 端口</b><span id="runProcess">-</span></div>
        <div class="cell"><b>启动时间</b><span id="runTime">-</span></div>
      </div>
      <div class="note" id="runPaths"></div>
      <div class="actions">
        <button id="enterCurrent" class="hidden">进入当前操作台</button>
        <button id="stopPortal" class="stop" disabled>停止当前模式</button>
      </div>
    </section>
  </main>
</div>
<div id="workbenchView" class="hidden">
  <header class="workbench-bar">
    <button id="backPortal" class="secondary">← 返回门户</button>
    <strong id="workbenchMode">采集操作台</strong>
    <span>返回门户不会停止当前模式</span>
    <button id="stopWorkbench" class="stop">停止当前模式</button>
  </header>
  <div id="workbenchLoading">正在启动原采集后端和操作台…</div>
  <iframe id="console" class="hidden" title="采集操作台"></iframe>
</div>
<script>
const $ = id => document.getElementById(id);
const error = $('error');
let lastStatus = null;
let wasRunning = false;
let wantWorkbench = location.hash === '#workbench';

async function request(path, payload) {
  const options = payload === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)};
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function showPortal() {
  wantWorkbench = false;
  $('portalView').classList.remove('hidden');
  $('workbenchView').classList.add('hidden');
  history.replaceState(null, '', location.pathname + location.search + '#portal');
}

function showWorkbench(data) {
  wantWorkbench = true;
  $('portalView').classList.add('hidden');
  $('workbenchView').classList.remove('hidden');
  $('workbenchMode').textContent = (data?.mode_name || '当前模式') + ' · 操作台';
  history.replaceState(null, '', location.pathname + location.search + '#workbench');
}

function updateWorkbench(data) {
  const frame = $('console');
  const loading = $('workbenchLoading');
  if (data.running && data.ready && data.port) {
    const wanted = `${location.protocol}//${location.hostname}:${data.port}/`;
    if (frame.dataset.url !== wanted) {
      frame.src = wanted;
      frame.dataset.url = wanted;
    }
    loading.classList.add('hidden');
    frame.classList.remove('hidden');
  } else {
    frame.classList.add('hidden');
    loading.classList.remove('hidden');
    loading.textContent = data.running
      ? (data.last_error || '正在启动原采集后端和操作台…')
      : '当前模式已退出。';
  }
}

async function refresh() {
  try {
    const data = await request('/api/status');
    lastStatus = data;
    $('runState').textContent = data.running ? (data.ready ? '运行中' : '启动中') : (data.exit_code === null ? '未启动' : `已退出 (${data.exit_code})`);
    $('runState').className = data.running ? 'running' : 'stopped';
    $('runMode').textContent = data.mode_name || '-';
    $('runProcess').textContent = data.running ? `${data.pid} / ${data.port}` : '-';
    $('runTime').textContent = data.started_at || '-';
    $('runPaths').textContent = `数据目录: ${data.data_dir || '-'} | 日志: ${data.log_path || '-'}${data.last_error ? ' | ' + data.last_error : ''}`;
    document.querySelectorAll('#configPanel input,#configPanel select,.modeStart,#suggest').forEach(el => { el.disabled = data.running; });
    $('enterCurrent').classList.toggle('hidden', !data.running);
    $('stopPortal').disabled = !data.running;
    $('stopWorkbench').disabled = !data.running;
    updateWorkbench(data);
    if (data.running && wantWorkbench) {
      showWorkbench(data);
    } else if (!data.running) {
      if (wasRunning && data.exit_code !== 0 && data.last_error) error.textContent = data.last_error;
      if (wantWorkbench) showPortal();
      const frame = $('console');
      frame.removeAttribute('src');
      delete frame.dataset.url;
    }
    wasRunning = data.running;
  } catch (exc) {
    error.textContent = exc.message;
  }
}

async function startMode(selectedMode) {
  error.textContent = '';
  try {
    const data = await request('/api/start', {
      mode: selectedMode,
      data_dir: $('dataDir').value,
      start_index: $('startIndex').value,
      grade: selectedMode === 'inference' ? '' : $('grade').value,
      left_target: $('leftTarget').value,
      right_target: $('rightTarget').value,
    });
    showWorkbench(data);
    updateWorkbench(data);
    await refresh();
  } catch (exc) {
    showPortal();
    error.textContent = exc.message;
  }
}

async function stopCurrent() {
  if (!confirm('停止当前模式？若正在录制，原脚本会按既有退出逻辑处理当前 episode。')) return;
  error.textContent = '';
  try {
    await request('/api/stop', {});
    showPortal();
    await refresh();
  } catch (exc) {
    error.textContent = exc.message;
  }
}

document.querySelectorAll('.modeStart').forEach(button => button.addEventListener('click', () => startMode(button.dataset.mode)));
$('suggest').addEventListener('click', async () => {
  error.textContent = '';
  try {
    const q = encodeURIComponent($('dataDir').value);
    const data = await request('/api/suggest?data_dir=' + q);
    $('startIndex').value = data.start_index;
  } catch (exc) { error.textContent = exc.message; }
});
$('enterCurrent').addEventListener('click', () => { if (lastStatus?.running) showWorkbench(lastStatus); });
$('backPortal').addEventListener('click', showPortal);
$('stopPortal').addEventListener('click', stopCurrent);
$('stopWorkbench').addEventListener('click', stopCurrent);
window.addEventListener('hashchange', () => {
  if (location.hash === '#workbench' && lastStatus?.running) showWorkbench(lastStatus);
  else if (location.hash === '#portal') showPortal();
});
refresh();
setInterval(refresh, 1200);
</script>
</body>
</html>"""

HTML = HTML.replace(
    "__LEFT_TARGET_OPTIONS__",
    target_options_markup("请选择左手商品", required=True),
).replace(
    "__RIGHT_TARGET_OPTIONS__",
    target_options_markup("不使用右手（单目标）", required=False),
)


def build_handler(supervisor: CollectionSupervisor):
    class Handler(BaseHTTPRequestHandler):
        server_version = "CollectionLauncher/1.0"

        def send_body(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, status: int, payload: Any) -> None:
            self.send_body(status, json_bytes(payload), "application/json; charset=utf-8")

        def read_json(self) -> dict[str, Any]:
            length = min(int(self.headers.get("Content-Length", "0") or "0"), 65536)
            value = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
            if not isinstance(value, dict):
                raise ValueError("请求必须是 JSON 对象")
            return value

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            if parsed.path == "/":
                self.send_body(200, HTML.encode("utf-8"), "text/html; charset=utf-8")
            elif parsed.path == "/api/status":
                self.send_json(200, supervisor.status())
            elif parsed.path == "/api/suggest":
                raw = parse_qs(parsed.query).get("data_dir", [""])[0].strip()
                path = Path(raw).expanduser()
                if not raw or not path.is_absolute():
                    self.send_json(400, {"error": "数据目录必须是绝对路径"})
                else:
                    self.send_json(200, {"start_index": suggested_episode(path)})
            elif parsed.path == "/favicon.ico":
                self.send_body(204, b"", "image/x-icon")
            else:
                self.send_json(404, {"error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            try:
                payload = self.read_json()
                if self.path == "/api/start":
                    result = supervisor.start(payload)
                elif self.path == "/api/stop":
                    result = supervisor.stop()
                else:
                    self.send_json(404, {"error": "not found"})
                    return
                self.send_json(200, result)
            except (ValueError, json.JSONDecodeError) as exc:
                self.send_json(400, {"error": str(exc)})
            except Exception as exc:  # keep operator UI alive and show actionable failure
                supervisor.last_error = str(exc)
                self.send_json(500, {"error": str(exc)})

        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[{time.strftime('%F %T')}] {fmt % args}", flush=True)

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Unified collection Web launcher")
    parser.add_argument("--host", default="192.168.3.101")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--runtime-root",
        default="/home/agilex/data/logs/unified_collection_web",
    )
    args = parser.parse_args()
    supervisor = CollectionSupervisor(args.host, args.port, Path(args.runtime_root))
    server = ThreadingHTTPServer((args.host, args.port), build_handler(supervisor))
    shutdown_started = threading.Event()

    def shutdown(_signum: int, _frame: Any) -> None:
        if shutdown_started.is_set():
            return
        shutdown_started.set()
        print("收到 Ctrl+C，正在关闭本次数据采集的全部进程...", flush=True)
        supervisor.stop()
        print("本次数据采集的全部进程已关闭，统一入口退出。", flush=True)
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f"统一采集入口: http://{args.host}:{args.port}/", flush=True)
    try:
        server.serve_forever()
    finally:
        supervisor.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
